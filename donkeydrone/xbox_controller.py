"""
XboxDroneController - reads Xbox controller state from XboxBridge.app via
a Unix datagram socket, and emits the four user-input memory keys plus an
arm signal.

Why the bridge: Apple's `com.apple.gamecontroller.driver.XboxGamepad` dext
claims exclusive HID ownership of Xbox controllers on modern macOS, which
prevents pygame/SDL/hidapi from seeing inputs. The only supported path is
GameController.framework from a real .app bundle. `xbox_bridge/main.swift`
is that bundle; it sends 18-byte frames at 60Hz to /tmp/donkeydrone_xbox.sock
and this part binds the receiving end.

Frame format (18 bytes, little-endian, matches xbox_bridge/main.swift):
    float32 leftY, float32 rightX, float32 rightY, float32 rightTrigger,
    uint8 buttons (bit0=A, bit1=B), uint8 connected (1/0)

GameController.framework axis convention is **stick-up = +1** (opposite of
SDL/pygame), so neither lY nor rY is negated here.

Stick mapping:
    Right stick X → user/steering   (yaw)
    Right stick Y → user/throttle   (forward pitch, +1 = stick up = forward)
    Left  stick Y → user/altitude   (analog, +1 = stick up = climb)

Buttons / triggers:
    A → toggle `recording`
    B → cycle `user/mode` through user → local_angle → local
    RT → deadman arm switch — held past `arm_threshold` = armed,
         released = disarmed.

Wire this in alongside LocalWebController; because it is added after the web
controller in the Vehicle's part list, its outputs overwrite the web values
on each tick (last-writer-wins per memory key).
"""

import logging
import os
import socket
import struct

logger = logging.getLogger(__name__)

_DEFAULT_SOCK_PATH = os.environ.get(
    "DONKEYDRONE_XBOX_SOCK", "/tmp/donkeydrone_xbox.sock"
)
_FRAME_FMT = "<ffffBB"
_FRAME_SIZE = struct.calcsize(_FRAME_FMT)

# Tick-based stale-frame detection. The bridge sends at 60Hz; the vehicle
# loop runs at DRIVE_LOOP_HZ (typically 30Hz). After this many ticks with no
# frame the controller is considered offline and we zero the outputs so a
# crashed bridge can't leave the drone with the last stick command latched.
_STALE_TICK_THRESHOLD = 30  # ~1s at 30Hz


class XboxDroneController:
    """Non-threaded Vehicle part. Drains all queued UDS datagrams in run()
    and uses the most recent one. Socket is non-blocking so an empty queue
    just returns the previously latched values."""

    MODES = ["user", "local_angle", "local"]

    def __init__(
        self,
        deadzone=0.08,
        steering_scale=1.0,
        throttle_scale=1.0,
        altitude_scale=1.0,
        arm_threshold=0.5,
        socket_path=_DEFAULT_SOCK_PATH,
    ):
        self.deadzone = float(deadzone)
        self.steering_scale = float(steering_scale)
        self.throttle_scale = float(throttle_scale)
        self.altitude_scale = float(altitude_scale)
        self.arm_threshold = float(arm_threshold)
        self.socket_path = socket_path

        self.steering = 0.0
        self.throttle = 0.0
        self.altitude = 0.0
        self.mode = self.MODES[0]
        self.recording = False
        self.armed = False

        self._mode_idx = 0
        self._prev_a = False
        self._prev_b = False
        self._tick = 0
        self._ticks_since_frame = _STALE_TICK_THRESHOLD + 1
        self._frames_seen = 0
        self._bridge_connected = False

        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._sock.bind(self.socket_path)
        self._sock.setblocking(False)
        logger.info(
            "XboxDroneController bound %s; waiting for frames from XboxBridge.app",
            self.socket_path,
        )

    def _apply_deadzone(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _drain_latest(self):
        """Read all queued datagrams; return the last decoded one or None."""
        latest = None
        while True:
            try:
                data, _ = self._sock.recvfrom(64)
            except (BlockingIOError, OSError):
                break
            if len(data) >= _FRAME_SIZE:
                latest = struct.unpack(_FRAME_FMT, data[:_FRAME_SIZE])
        return latest

    def run(self):
        self._tick += 1
        frame = self._drain_latest()

        if frame is not None:
            self._frames_seen += 1
            self._ticks_since_frame = 0
            lY, rX, rY, rT, btns, conn = frame
            self._bridge_connected = bool(conn)
        else:
            self._ticks_since_frame += 1

        stale = self._ticks_since_frame > _STALE_TICK_THRESHOLD
        if stale or not self._bridge_connected:
            # Bridge is offline OR no controller plugged in — fail safe:
            # zero stick outputs and disarm. Mode/recording state is sticky.
            self.steering = 0.0
            self.throttle = 0.0
            self.altitude = 0.0
            self.armed = False
            self._prev_a = False
            self._prev_b = False
            if self._tick % 90 == 0:
                if stale:
                    logger.warning(
                        "XboxBridge no frames for %d ticks (~%.1fs); "
                        "is XboxBridge.app running?",
                        self._ticks_since_frame, self._ticks_since_frame / 30.0,
                    )
                else:
                    logger.info("XboxBridge connected but no controller")
            return (
                self.steering, self.throttle, self.altitude,
                self.mode, self.recording, self.armed,
            )

        # frame is guaranteed non-None below (we just returned otherwise).
        lY, rX, rY, rT, btns, _conn = frame  # type: ignore[misc]
        btn_a = bool(btns & 0x01)
        btn_b = bool(btns & 0x02)

        self.steering = max(
            -1.0,
            min(1.0, self._apply_deadzone(rX) * self.steering_scale),
        )
        # GameController convention: stick up = +1. No negation needed.
        self.throttle = max(
            -1.0,
            min(1.0, self._apply_deadzone(rY) * self.throttle_scale),
        )
        self.altitude = max(
            -1.0,
            min(1.0, self._apply_deadzone(lY) * self.altitude_scale),
        )

        # Right trigger: 0..1, deadman arm.
        self.armed = rT > self.arm_threshold

        if btn_a and not self._prev_a:
            self.recording = not self.recording
            logger.info("xbox: recording=%s", self.recording)
        self._prev_a = btn_a

        if btn_b and not self._prev_b:
            self._mode_idx = (self._mode_idx + 1) % len(self.MODES)
            self.mode = self.MODES[self._mode_idx]
            logger.info("xbox: mode=%s", self.mode)
        self._prev_b = btn_b

        if self._tick % 90 == 0:
            logger.info(
                "xbox raw: lY=%+.2f rX=%+.2f rY=%+.2f rT=%+.2f → "
                "steer=%+.2f thr=%+.2f alt=%+.2f armed=%s (frames=%d)",
                lY, rX, rY, rT,
                self.steering, self.throttle, self.altitude, self.armed,
                self._frames_seen,
            )

        return (
            self.steering, self.throttle, self.altitude,
            self.mode, self.recording, self.armed,
        )

    def shutdown(self):
        try:
            self._sock.close()
        except Exception:
            pass
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass
        except Exception:
            pass

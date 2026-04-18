"""
DroneGymEnv - DonkeyCar part wrapping BetaFlight SITL + Gazebo.

This part connects to a BetaFlight SITL instance via UDP RC channel packets
and provides the same interface as DonkeyGymEnv: it accepts (steering, throttle,
altitude) inputs and produces camera images + telemetry outputs.

The semantic mapping is (BetaFlight Angle mode):
    steering [-1, 1]  ->  yaw rate
    throttle [-1, 1]  ->  forward pitch (tilt angle)
    altitude [-1, 1]  ->  motor throttle (direct power, no PID)

Camera sources (set DRONE_CAMERA_SOURCE in your drone_config_XXmm.py):
    "gz_transport"  Native macOS: subscribe to Gazebo Harmonic camera topic
                    via gz-transport subprocess + shared memory.
    "rtsp"          Docker mode: read RTSP stream from Gazebo Classic in container.

Requires:
    - BetaFlight SITL + Gazebo running, UDP 9004 for RC input
    - opencv-python-headless
    - gz-transport mode: gz-python (brew install gz-harmonic)
"""

import collections
import logging
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path

import cv2
import numpy as np

logger = logging.getLogger(__name__)


_GZ_CAMERA_TOPIC_DEFAULT = (
    "/world/drone_course_65mm/model/betaloop_drone_cam_65mm"
    "/link/camera_link/sensor/camera/image"
)


class DroneGymEnv:
    """
    DonkeyCar part that interfaces with BetaFlight SITL + Gazebo via UDP RC.

    Supports two camera sources (controlled by the camera_source parameter):
        "gz_transport"  - native macOS: gz-transport subscription (Gazebo Harmonic)
        "rtsp"          - Docker mode: OpenCV VideoCapture on RTSP stream

    This is a threaded part: update() runs the BetaFlight control loop in a
    background thread, while run_threaded() is called by the Vehicle loop to
    exchange steering/throttle/altitude commands and camera images.

    Usage in manage.py:
        gym = DroneGymEnv(cfg)
        V.add(gym, inputs=['steering', 'throttle', 'altitude'],
              outputs=['cam/image_array', ...], threaded=True)
    """

    def __init__(self, rc_host="127.0.0.1", rc_port=9004,
                 camera_source="gz_transport",
                 gz_camera_topic=_GZ_CAMERA_TOPIC_DEFAULT,
                 rtsp_url="rtsp://127.0.0.1:8554/live",
                 max_pitch_angle=25.0, max_yaw_rate=90.0,
                 hover_throttle=1500, throttle_range=300,
                 arm_channel=4, mode_channel=5,
                 image_w=160, image_h=120,
                 simulated_delay_ms=0,
                 measure_loop_delay=False,
                 loop_delay_log_interval=100,
                 input_sensitivity=1.0,
                 yaw_pwm_cap=30,
                 record_position=False,
                 record_attitude=False,
                 record_velocity=False):

        self.rc_host = rc_host
        self.rc_port = rc_port
        self.camera_source = camera_source
        self.gz_camera_topic = gz_camera_topic
        self.rtsp_url = rtsp_url
        self.max_pitch_angle = max_pitch_angle
        self.max_yaw_rate = max_yaw_rate
        self.hover_throttle = hover_throttle
        self.throttle_range = throttle_range
        self.arm_channel = arm_channel
        self.mode_channel = mode_channel
        self.image_w = image_w
        self.image_h = image_h
        self.simulated_delay_ms = simulated_delay_ms
        self.measure_loop_delay = measure_loop_delay
        self.loop_delay_log_interval = loop_delay_log_interval
        self.input_sensitivity = float(max(0.0, min(1.0, input_sensitivity)))
        # Max CH4 (yaw) deflection in PWM microseconds from center (1500).
        # The motor mixer's ω²-asymmetry means yaw input at hover produces net
        # upward thrust — full ±500 deflection makes the drone rocket up. Cap
        # yaw small to keep the climb induced by turning manageable.
        self.yaw_pwm_cap = int(max(0, min(500, yaw_pwm_cap)))
        self.record_position = record_position
        self.record_attitude = record_attitude
        self.record_velocity = record_velocity

        # Shared state between threads
        self.steering = 0.0
        self.throttle = 0.0
        self.altitude = 0.0  # altitude control input [-1, 1] → motor throttle
        self.frame = np.zeros((image_h, image_w, 3), dtype=np.uint8)
        self.position = (0.0, 0.0, 0.0)
        self.attitude = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)

        self.running = True
        self._rc_sock = None
        self._cap = None            # used by RTSP mode
        self._shm = None            # shared memory for gz-transport subprocess
        self._camera_proc = None    # gz_camera_worker subprocess
        self._last_seq = 0          # sequence counter for new-frame detection
        self._frame_size = image_h * image_w * 3
        self._frame_skip = 0

        # Simulated camera delay buffer: deque of (timestamp_ms, frame)
        self._delay_buffer = collections.deque()

        # Loop delay measurement
        self._loop_delays = collections.deque(maxlen=loop_delay_log_interval)
        self._last_loop_time = None
        self._loop_count = 0

    def _start_camera(self):
        """Open RTSP stream from Gazebo (Docker mode)."""
        logger.info("Opening RTSP camera stream: %s", self.rtsp_url)

        self._cap = cv2.VideoCapture(self.rtsp_url)
        if self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            logger.info("RTSP stream opened successfully")
            return

        # Fallback: try GStreamer pipeline
        gst = (
            f"rtspsrc location={self.rtsp_url} latency=0 "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! videoconvert ! appsink drop=true max-buffers=1 sync=false"
        )
        logger.info("Trying GStreamer pipeline: %s", gst)
        self._cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if self._cap.isOpened():
            logger.info("GStreamer pipeline opened successfully")
            return

        logger.warning("Could not open RTSP stream. Images will be blank.")

    def _capture_frame(self):
        """Capture a single frame from the RTSP stream and resize (Docker mode)."""
        if self._cap is None or not self._cap.isOpened():
            return

        ret, raw_frame = self._cap.read()
        if ret and raw_frame is not None:
            rgb = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (self.image_w, self.image_h))
            self.frame = resized

    def _start_gz_camera(self):
        """Launch gz_camera_worker subprocess for gz-transport camera capture."""
        logger.info("Starting gz-transport camera subprocess for topic: %s",
                    self.gz_camera_topic)

        shm_size = 1 + self._frame_size
        self._shm = SharedMemory(create=True, size=shm_size)
        self._shm.buf[0] = 0
        logger.info("Created shared memory '%s' (%d bytes)",
                    self._shm.name, shm_size)

        worker_path = str(Path(__file__).parent / "gz_camera_worker.py")

        self._camera_proc = subprocess.Popen(
            ["uv", "run", "--env-file", ".env",
             "python", worker_path,
             self.gz_camera_topic,
             str(self.image_w), str(self.image_h),
             self._shm.name],
            env=os.environ.copy(),
        )
        logger.info("Camera worker started (PID %d)", self._camera_proc.pid)

    def _read_gz_frame(self):
        """Read the latest frame from shared memory if a new one is available."""
        if self._shm is None:
            return
        seq = self._shm.buf[0]
        if seq != self._last_seq:
            self._last_seq = seq
            frame_bytes = bytes(self._shm.buf[1:1 + self._frame_size])
            self.frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape(
                self.image_h, self.image_w, 3)
            if self._frame_skip == 0:
                logger.info("Frame updated (seq=%d, shape=%s, dtype=%s)", seq, self.frame.shape, self.frame.dtype)
            self._frame_skip = (self._frame_skip + 1) % 30

    def _send_rc(self, channels):
        """Send RC channel packet to BetaFlight SITL.

        Packet format: 8-byte timestamp (double) + 16 × 2-byte channels (uint16)
        = 40 bytes total.
        """
        timestamp = time.time()
        packet = struct.pack('<d', timestamp)
        for ch in channels:
            packet += struct.pack('<H', int(ch))
        try:
            self._rc_sock.sendto(packet, (self.rc_host, self.rc_port))
        except OSError as e:
            logger.warning("RC send failed: %s", e)

    def _map_controls_to_rc(self):
        """Convert steering/throttle/altitude [-1,1] to 16 RC PWM channels [1000-2000].

        BetaFlight Angle mode channel mapping:
            CH1 (roll):     1500 (centered, no lateral movement)
            CH2 (pitch):    1500 + throttle * 500 * sensitivity (forward tilt)
            CH3 (throttle): bipolar — altitude in [-1,1] → hover_throttle ± throttle_range.
                            altitude=0 → hover PWM (drone holds altitude in sim where
                            thrust is deterministic); altitude=+1 → hover+range (climb);
                            altitude=-1 → hover-range (descend).
            CH4 (yaw):      1500 + steering * yaw_pwm_cap (capped independently
                            of input_sensitivity to limit yaw-induced climb)
            CH5 (AUX1):     2000 = armed, 1000 = disarmed
            CH6 (AUX2):     2000 = angle mode active
            CH7-16:         1000 (unused)
        """
        channels = [1000] * 16

        # Sensitivity scales pitch/yaw stick deflection.
        deflection = 500 * self.input_sensitivity

        # CH1: roll (centered)
        channels[0] = 1500

        # CH2: pitch (forward tilt from throttle input)
        channels[1] = int(max(1000, min(2000, 1500 + self.throttle * deflection)))

        # CH3: motor throttle — bipolar around hover.
        alt = max(-1.0, min(1.0, self.altitude))
        channels[2] = int(max(1000, min(2000,
            self.hover_throttle + alt * self.throttle_range)))

        # CH4: yaw — capped separately from pitch sensitivity. At hover
        # throttle, yaw deflection adds net thrust via ω² mixer asymmetry;
        # a small cap keeps the yaw-induced climb manageable.
        steer = max(-1.0, min(1.0, self.steering))
        channels[3] = int(max(1000, min(2000, 1500 + steer * self.yaw_pwm_cap)))

        # CH5 (AUX1): armed
        channels[self.arm_channel] = 2000

        # CH6 (AUX2): angle mode
        channels[self.mode_channel] = 2000

        return channels

    def _betaflight_loop(self):
        """Main control loop: arm BetaFlight and send RC commands at 50Hz.

        Plain synchronous loop — no asyncio needed.
        """
        self._rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logger.info("RC UDP socket ready → %s:%d", self.rc_host, self.rc_port)

        # Phase 1: Disarm — send 1s of disarmed packets (throttle low, AUX1 low)
        logger.info("Sending disarm packets (1s)...")
        disarm_channels = [1000] * 16
        disarm_channels[0] = 1500  # roll centered
        disarm_channels[1] = 1500  # pitch centered
        disarm_channels[2] = 1000  # throttle low
        disarm_channels[3] = 1500  # yaw centered
        for _ in range(50):
            if not self.running:
                return
            self._send_rc(disarm_channels)
            time.sleep(0.02)

        # Phase 2: Arm — send 1s of armed packets (throttle low, AUX1 high, AUX2 high)
        logger.info("Arming BetaFlight (1s)...")
        arm_channels = list(disarm_channels)
        arm_channels[self.arm_channel] = 2000   # AUX1 armed
        arm_channels[self.mode_channel] = 2000  # AUX2 angle mode
        arm_channels[2] = 1000                  # throttle low during arm
        for _ in range(50):
            if not self.running:
                return
            self._send_rc(arm_channels)
            time.sleep(0.02)

        logger.info("Armed — entering control loop at 50Hz")

        # Phase 3: Control loop
        loop_count = 0
        while self.running:
            channels = self._map_controls_to_rc()
            self._send_rc(channels)

            # Log control values every ~2s
            loop_count += 1
            if loop_count % 100 == 0:
                logger.info(
                    "rc: steer=%.2f thr=%.2f alt=%.2f → "
                    "pitch=%d yaw=%d throttle=%d",
                    self.steering, self.throttle, self.altitude,
                    channels[1], channels[3], channels[2])

            # RTSP mode: poll for frames every other iteration
            if self.camera_source == 'rtsp':
                self._frame_skip += 1
                if self._frame_skip % 2 == 0:
                    self._capture_frame()

            time.sleep(0.02)  # 50 Hz

        # Phase 4: Disarm on exit
        logger.info("Disarming BetaFlight...")
        disarm_channels[self.arm_channel] = 1000
        disarm_channels[2] = 1000
        for _ in range(25):
            self._send_rc(disarm_channels)
            time.sleep(0.02)

        self._rc_sock.close()
        self._rc_sock = None
        logger.info("BetaFlight disarmed and socket closed")

    def update(self):
        """
        Background thread entry point.
        Starts camera, then runs the BetaFlight control loop.
        """
        if self.camera_source == 'rtsp':
            self._start_camera()
        else:
            self._start_gz_camera()

        try:
            self._betaflight_loop()
        except Exception as e:
            logger.error("BetaFlight loop error: %s", e)

    def run_threaded(self, steering, throttle, altitude):
        """
        Called by the DonkeyCar Vehicle loop each frame.

        :param steering: normalized steering [-1, 1], mapped to yaw rate
        :param throttle: normalized throttle [-1, 1], mapped to forward pitch
        :param altitude: normalized altitude [-1, 1], mapped to motor throttle
        :return: camera image array + optional telemetry values
        """
        # Loop delay measurement
        if self.measure_loop_delay:
            now = time.time()
            if self._last_loop_time is not None:
                delta_ms = (now - self._last_loop_time) * 1000.0
                self._loop_delays.append(delta_ms)
                self._loop_count += 1
                if self._loop_count % self.loop_delay_log_interval == 0:
                    delays = list(self._loop_delays)
                    if delays:
                        avg = sum(delays) / len(delays)
                        lo = min(delays)
                        hi = max(delays)
                        logger.info("loop delay: avg=%.1fms min=%.1fms max=%.1fms",
                                    avg, lo, hi)
            self._last_loop_time = now

        # Set control inputs
        if steering is None:
            steering = 0.0
        if throttle is None:
            throttle = 0.0
        if altitude is None:
            altitude = 0.0

        self.steering = float(steering)
        self.throttle = float(throttle)
        self.altitude = float(altitude)

        # Read camera frame
        if self.camera_source == 'gz_transport':
            self._read_gz_frame()

        # Simulated camera delay
        current_frame = self.frame
        if self.simulated_delay_ms > 0:
            now_ms = time.time() * 1000.0
            self._delay_buffer.append((now_ms, current_frame))
            # Find the frame that is delay_ms old
            target_time = now_ms - self.simulated_delay_ms
            output_frame = self._delay_buffer[0][1]  # oldest as fallback
            while len(self._delay_buffer) > 1 and self._delay_buffer[1][0] <= target_time:
                self._delay_buffer.popleft()
            output_frame = self._delay_buffer[0][1]
        else:
            output_frame = current_frame

        outputs = [output_frame]

        if self.record_position:
            outputs += [self.position[0], self.position[1], self.position[2]]
        if self.record_attitude:
            outputs += [self.attitude[0], self.attitude[1], self.attitude[2]]
        if self.record_velocity:
            outputs += [self.velocity[0], self.velocity[1], self.velocity[2]]

        if len(outputs) == 1:
            return output_frame
        return outputs

    def shutdown(self):
        """Stop the background thread and clean up."""
        logger.info("Shutting down DroneGymEnv...")
        self.running = False
        time.sleep(1.0)
        if self._cap is not None:
            self._cap.release()
        if self._camera_proc is not None:
            logger.info("Terminating camera worker (PID %d)...",
                        self._camera_proc.pid)
            self._camera_proc.terminate()
            try:
                self._camera_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._camera_proc.kill()
            self._camera_proc = None
        if self._shm is not None:
            self._shm.close()
            self._shm.unlink()
            self._shm = None

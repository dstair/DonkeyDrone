#!/usr/bin/env python3
"""
Thrust / yaw test: validate BF SITL + Gazebo behavior.

Modes (`--mode`):
    thrust   Ramp throttle 1000→2000 PWM, 0.5s per step. Used to find
             hover PWM and tune `motorConstant` in model.sdf.
    yaw      Hold throttle=1000 (motors should be idle) and apply a
             full yaw stick. Watch for altitude climb — that's the
             "yaw shoots drone into sky" bug. With AIRMODE disabled
             in BF, altitude should stay ~0.
    both     Run yaw test first, then thrust ramp (default).

Easiest way to run (brings up the stack, runs the test, tears it down):

    ./scripts/test_thrust.sh
    ./scripts/test_thrust.sh --mode=yaw

This test sends its OWN RC packets to BetaFlight SITL on UDP 9004, so
drone_manage.py must NOT be running — the two would fight over the port.
"""

import argparse
import logging
import socket
import struct
import time

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
logger = logging.getLogger(__name__)

RC_HOST = "127.0.0.1"
RC_PORT = 9004

POSE_TOPIC = "/world/drone_course/dynamic_pose/info"


def send_rc(rc_sock, channels):
    """Send 40-byte RC packet: timestamp + 16 x uint16."""
    timestamp = time.time()
    packet = struct.pack("<d", timestamp)
    for ch in channels:
        packet += struct.pack("<H", int(ch))
    rc_sock.sendto(packet, (RC_HOST, RC_PORT))


def make_channels(throttle_pwm, armed=True, yaw_pwm=1500):
    """Create 16-channel RC packet with throttle, yaw, and arm/disarm."""
    channels = [1000] * 16
    channels[0] = 1500  # CH1 roll (centered)
    channels[1] = 1500  # CH2 pitch (centered - no forward tilt)
    channels[2] = throttle_pwm  # CH3 throttle
    channels[3] = yaw_pwm  # CH4 yaw
    channels[4] = 2000 if armed else 1000  # CH5 AUX1 arm switch
    channels[5] = 2000  # CH6 AUX2 = angle mode
    return channels


def query_pose():
    """Query drone pose via gz-transport Pose_V topic."""
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V

        node = Node()
        pose_data = [None]

        def on_pose(msg):
            pose_data[0] = msg

        if not node.subscribe(Pose_V, POSE_TOPIC, on_pose):
            return None

        time.sleep(0.15)

        if pose_data[0] is not None and pose_data[0].pose:
            for p in pose_data[0].pose:
                if "betaloop" in p.name.lower():
                    return (p.position.x, p.position.y, p.position.z)
    except Exception as e:
        logger.debug("Pose query failed: %s", e)

    return None


def hold(rc_sock, channels, duration_s):
    """Send channels at 50Hz for duration_s seconds."""
    n = int(duration_s / 0.02)
    for _ in range(n):
        send_rc(rc_sock, channels)
        time.sleep(0.02)


def arm_sequence(rc_sock):
    """Disarm→arm boot sequence BF SITL requires to clear NOT_DISARMED flag."""
    logger.info("Sending disarm packets (1s)...")
    hold(rc_sock, make_channels(1000, armed=False), 1.0)
    logger.info("Arming and idling...")
    hold(rc_sock, make_channels(1000, armed=True), 1.0)


def run_yaw_test(rc_sock):
    """Test whether yaw input at CH3=1000 causes altitude to climb (airmode bug)."""
    logger.info("=== Yaw bleed test ===")
    logger.info("Baseline: throttle=1000, yaw centered, 1s")
    hold(rc_sock, make_channels(1000, armed=True, yaw_pwm=1500), 1.0)
    baseline = query_pose()
    baseline_alt = baseline[2] if baseline else None
    logger.info("Baseline altitude: %s",
                f"{baseline_alt:.3f}m" if baseline_alt is not None else "?")

    samples = []
    for direction, yaw_pwm in [("right", 2000), ("left", 1000)]:
        logger.info("Yaw %s (CH4=%d), throttle=1000, 3s — sampling altitude...",
                    direction, yaw_pwm)
        ch = make_channels(1000, armed=True, yaw_pwm=yaw_pwm)
        # 12 samples × 0.25s = 3s
        for i in range(12):
            hold(rc_sock, ch, 0.25)
            pos = query_pose()
            alt = pos[2] if pos else None
            samples.append((direction, (i + 1) * 0.25, alt))
            print(f"  yaw={direction} t={((i+1)*0.25):.2f}s alt={alt:.3f}m"
                  if alt is not None else
                  f"  yaw={direction} t={((i+1)*0.25):.2f}s alt=?")

        # Settle yaw centered briefly so the two directions don't interfere
        logger.info("Recentering yaw for 1s...")
        hold(rc_sock, make_channels(1000, armed=True, yaw_pwm=1500), 1.0)

    logger.info("--- Yaw test summary ---")
    if baseline_alt is None:
        logger.info("No baseline — can't evaluate.")
        return
    for direction in ("right", "left"):
        dir_alts = [a for d, _, a in samples if d == direction and a is not None]
        if not dir_alts:
            logger.info("  yaw %s: no altitude samples", direction)
            continue
        max_alt = max(dir_alts)
        delta = max_alt - baseline_alt
        verdict = "BUG PRESENT" if delta > 0.1 else "clean"
        logger.info("  yaw %s: max=%.3fm, delta=%+.3fm → %s",
                    direction, max_alt, delta, verdict)


def run_thrust_ramp(rc_sock):
    """Ramp throttle 1000→2000, measure altitude at each step."""
    logger.info("=== Thrust ramp test ===")
    logger.info("Ramping throttle from 1000 to 2000...")
    throttle_levels = list(range(1000, 2001, 50))

    results = []
    for pwm in throttle_levels:
        hold(rc_sock, make_channels(pwm, armed=True), 0.5)
        try:
            pos = query_pose()
            if pos:
                alt = pos[2]
                results.append((pwm, alt))
                print(f"throttle={pwm}, altitude={alt:.3f}m")
            else:
                results.append((pwm, None))
                print(f"throttle={pwm}, altitude=?")
        except Exception as e:
            results.append((pwm, None))
            print(f"throttle={pwm}, error: {e}")

    logger.info("--- Thrust ramp summary ---")
    hover_pwm = None
    for pwm, alt in results:
        if alt is not None and alt > 0.2:
            if hover_pwm is None:
                hover_pwm = pwm
            print(f"  {pwm} PWM -> {alt:.3f}m")
    if hover_pwm:
        logger.info("Approximate hover at ~%d PWM", hover_pwm)
    else:
        logger.info("No clear hover detected.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["thrust", "yaw", "both"], default="both",
                        help="Which test(s) to run (default: both)")
    args = parser.parse_args()

    logger.info("Opening RC socket...")
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    logger.info("Waiting for BetaFlight to come up...")
    time.sleep(2)

    arm_sequence(rc_sock)

    if args.mode in ("yaw", "both"):
        run_yaw_test(rc_sock)

    if args.mode in ("thrust", "both"):
        run_thrust_ramp(rc_sock)

    logger.info("Disarming...")
    hold(rc_sock, make_channels(1000, armed=False), 0.5)
    rc_sock.close()


if __name__ == "__main__":
    main()

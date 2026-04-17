#!/usr/bin/env python3
"""
Thrust test: automatically test motorConstant changes.

Usage:
    # First, start the full stack in one terminal:
    ./scripts/start.sh

    # Then in another terminal, run this test:
    uv run --env-file .env python donkeydrone/test_thrust.py

Output shows altitude at each throttle level, making it easy to
identify the hover point and tune motorConstant.
"""

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


def make_armed_channels(throttle_pwm):
    """Create 16-channel RC packet with throttle and arm/disarm."""
    channels = [1000] * 16
    channels[0] = 1500  # CH1 roll (centered)
    channels[1] = 1500  # CH2 pitch (centered - no forward tilt)
    channels[2] = throttle_pwm  # CH3 throttle
    channels[3] = 1500  # CH4 yaw (centered)
    channels[4] = 2000  # CH5 AUX1 = armed
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


def main():
    logger.info("Opening RC socket...")
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    logger.info("Waiting for BetaFlight to come up...")
    time.sleep(2)

    logger.info("Arming and idling...")
    for i in range(50):
        send_rc(rc_sock, make_armed_channels(1000))
        time.sleep(0.02)

    logger.info("Ramping throttle from 1000 to 2000...")
    throttle_levels = list(range(1000, 2001, 50))

    results = []

    for pwm in throttle_levels:
        send_rc(rc_sock, make_armed_channels(pwm))
        time.sleep(0.5)

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

    logger.info("Test complete.")
    logger.info("Summary:")

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

    logger.info("Disarming...")
    for _ in range(25):
        send_rc(rc_sock, make_armed_channels(1000))
        time.sleep(0.02)

    rc_sock.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Direct test of BetaFlight SITL FDM↔motor loop — NO Gazebo needed.

This script acts as a minimal BetaflightPlugin replacement:
  1. Sends fake FDM packets to BetaFlight on port 9003 (simulating a level drone)
  2. Sends RC packets to arm + apply throttle on port 9004
  3. Listens for motor commands from BetaFlight on port 9002

Run with BetaFlight SITL already running (no Gazebo):
    ~/dev/betaflight/obj/main/betaflight_SITL.elf &
    python3 scripts/test_bf_fdm.py

Expected result: motor values change from idle to non-zero when armed+throttle.
"""

import socket
import struct
import time
import math


def make_fdm_packet(timestamp):
    """Create a minimal FDM packet simulating a level drone at rest.

    BetaFlight fdm_packet layout (144 bytes):
      double timestamp
      double imu_angular_velocity_rpy[3]  (rad/s)
      double imu_linear_acceleration_xyz[3]  (m/s² body frame, NED)
      double imu_orientation_quat[4]  (w,x,y,z — identity = level)
      double velocity_xyz[3]  (m/s NED)
      double position_xyz[3]  (m NED)
      double pressure  (Pa)
    """
    pkt = struct.pack('<d', timestamp)                     # timestamp
    pkt += struct.pack('<3d', 0.0, 0.0, 0.0)              # angular velocity (no rotation)
    pkt += struct.pack('<3d', 0.0, 0.0, -9.81)            # linear acceleration (gravity in NED body frame: 0,0,-g)
    pkt += struct.pack('<4d', 1.0, 0.0, 0.0, 0.0)        # quaternion (identity = level)
    pkt += struct.pack('<3d', 0.0, 0.0, 0.0)              # velocity (stationary)
    pkt += struct.pack('<3d', 0.0, 0.0, 0.0)              # position (origin)
    pkt += struct.pack('<d', 101325.0)                     # pressure (sea level)
    return pkt


def make_rc_packet(timestamp, channels):
    """Create RC packet: 8-byte timestamp + 16 × uint16 channels."""
    pkt = struct.pack('<d', timestamp)
    for ch in channels:
        pkt += struct.pack('<H', int(ch))
    return pkt


def main():
    # FDM sender (us → BetaFlight port 9003)
    fdm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # RC sender (us → BetaFlight port 9004)
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Motor receiver (BetaFlight port 9002 → us)
    motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    motor_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    motor_sock.bind(('127.0.0.1', 9002))
    motor_sock.setblocking(False)

    print("=" * 60)
    print("BetaFlight FDM↔Motor Direct Test")
    print("=" * 60)
    print(f"FDM packet size: {len(make_fdm_packet(0.0))} bytes (BF expects 144)")
    print()

    # Phase 1: Send FDM + disarm RC for 2 seconds
    print("Phase 1: Sending FDM + disarm (2s)...")
    disarm_channels = [1500, 1500, 1000, 1500] + [1000] * 12  # AETR + AUX
    for i in range(100):
        t = time.time()
        fdm_sock.sendto(make_fdm_packet(t), ('127.0.0.1', 9003))
        rc_sock.sendto(make_rc_packet(t, disarm_channels), ('127.0.0.1', 9004))
        # Check for motor response
        try:
            data, addr = motor_sock.recvfrom(1024)
            if i == 0:
                print(f"  Motor packet received! ({len(data)} bytes from {addr})")
            if len(data) >= 16:
                motors = struct.unpack('<4f', data[:16])
                if i % 25 == 0:
                    print(f"  [{i:3d}] Motors (raw): {motors[0]:.3f} {motors[1]:.3f} "
                          f"{motors[2]:.3f} {motors[3]:.3f}")
        except BlockingIOError:
            if i % 25 == 0:
                print(f"  [{i:3d}] No motor packet")
        time.sleep(0.02)

    # Phase 2: Arm + angle mode + idle throttle for 2 seconds
    print("\nPhase 2: Arm + angle mode + idle throttle (2s)...")
    arm_channels = [1500, 1500, 1000, 1500, 2000, 2000] + [1000] * 10
    for i in range(100):
        t = time.time()
        fdm_sock.sendto(make_fdm_packet(t), ('127.0.0.1', 9003))
        rc_sock.sendto(make_rc_packet(t, arm_channels), ('127.0.0.1', 9004))
        try:
            data, _ = motor_sock.recvfrom(1024)
            if len(data) >= 16:
                motors = struct.unpack('<4f', data[:16])
                if i % 25 == 0:
                    print(f"  [{i:3d}] Motors (raw): {motors[0]:.3f} {motors[1]:.3f} "
                          f"{motors[2]:.3f} {motors[3]:.3f}")
                    print(f"         Adjusted (-1): {max(0,motors[0]-1):.3f} "
                          f"{max(0,motors[1]-1):.3f} "
                          f"{max(0,motors[2]-1):.3f} {max(0,motors[3]-1):.3f}")
        except BlockingIOError:
            if i % 25 == 0:
                print(f"  [{i:3d}] No motor packet")
        time.sleep(0.02)

    # Phase 3: Arm + hover throttle (1500 PWM) for 2 seconds
    print("\nPhase 3: Arm + hover throttle CH3=1500 (2s)...")
    hover_channels = [1500, 1500, 1500, 1500, 2000, 2000] + [1000] * 10
    for i in range(100):
        t = time.time()
        fdm_sock.sendto(make_fdm_packet(t), ('127.0.0.1', 9003))
        rc_sock.sendto(make_rc_packet(t, hover_channels), ('127.0.0.1', 9004))
        try:
            data, _ = motor_sock.recvfrom(1024)
            if len(data) >= 16:
                motors = struct.unpack('<4f', data[:16])
                if i % 25 == 0:
                    print(f"  [{i:3d}] Motors (raw): {motors[0]:.3f} {motors[1]:.3f} "
                          f"{motors[2]:.3f} {motors[3]:.3f}")
                    print(f"         Adjusted (-1): {max(0,motors[0]-1):.3f} "
                          f"{max(0,motors[1]-1):.3f} "
                          f"{max(0,motors[2]-1):.3f} {max(0,motors[3]-1):.3f}")
        except BlockingIOError:
            if i % 25 == 0:
                print(f"  [{i:3d}] No motor packet")
        time.sleep(0.02)

    # Phase 4: Disarm
    print("\nPhase 4: Disarm (1s)...")
    for i in range(50):
        t = time.time()
        fdm_sock.sendto(make_fdm_packet(t), ('127.0.0.1', 9003))
        rc_sock.sendto(make_rc_packet(t, disarm_channels), ('127.0.0.1', 9004))
        time.sleep(0.02)

    print("\nDone.")
    fdm_sock.close()
    rc_sock.close()
    motor_sock.close()


if __name__ == '__main__':
    main()

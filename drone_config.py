# """
# DRONE CONFIG
#
# Configuration overrides for flying a drone in PX4 SITL + Gazebo simulator.
# Use with: python drone_manage.py drive --myconfig=drone_config.py
#
# The drone uses the same DonkeyCar pipeline as a car, with a different
# semantic mapping:
#   steering [-1, 1]  ->  yaw rate
#   throttle [-1, 1]  ->  forward velocity
#   altitude           ->  held constant by PID controller
# """

# ---- Camera ----
# DroneGymEnv provides images directly, so use MOCK camera in DonkeyCar
CAMERA_TYPE = "MOCK"

# ---- Drive Train ----
# No physical actuators; DroneGymEnv sends commands via MAVSDK
DONKEY_GYM = False
DRIVE_TRAIN_TYPE = "MOCK"

# ---- Drone Simulator ----
USE_DRONE_SIM = True

# MAVSDK connection to PX4 SITL (Docker or native macOS)
DRONE_MAVSDK_ADDRESS = "udpin://0.0.0.0:14540"

# ---- Camera Source ----
# "gz_transport" - native macOS: Gazebo Harmonic via gz-transport (GPU-accelerated)
#                  Requires: pip install gz-python
#                  Discover topic: gz topic -l | grep camera
# "rtsp"         - Docker mode: RTSP stream from Gazebo Classic in container
DRONE_CAMERA_SOURCE = "gz_transport"

# gz-transport camera topic (native macOS mode).
# The default matches gz_x500_mono_cam in the "walls" world.
# Run `gz topic -l | grep camera` to confirm the topic on your setup.
DRONE_GZ_CAMERA_TOPIC = (
    "/world/walls/model/x500_mono_cam_0"
    "/link/camera_link/sensor/camera_sensor/image"
)

# RTSP camera stream URL (Docker mode only -- used when DRONE_CAMERA_SOURCE = "rtsp")
# DRONE_RTSP_URL = "rtsp://127.0.0.1:8554/live"

# ---- Flight Parameters ----
# Max forward velocity (m/s) when throttle = 1.0
DRONE_MAX_FORWARD_VEL = 2.0

# Max yaw rate (deg/s) when steering = 1.0 or -1.0
DRONE_MAX_YAW_RATE = 90.0

# Target altitude (meters) for altitude hold
DRONE_TARGET_ALTITUDE = 3.0     # Lower for emulated SITL (slow sim clock)

# Altitude hold PID gains (kp, ki, kd)
DRONE_ALTITUDE_KP = 0.5
DRONE_ALTITUDE_KI = 0.1
DRONE_ALTITUDE_KD = 0.2

# ---- Telemetry Recording ----
# Record additional drone telemetry in tubs alongside images
DRONE_RECORD_POSITION = True
DRONE_RECORD_ATTITUDE = True
DRONE_RECORD_VELOCITY = True

# ---- Vehicle Loop ----
DRIVE_LOOP_HZ = 10

# ---- Data Storage ----
# Always create a new tub (drone schema differs from car schema)
AUTO_CREATE_NEW_TUB = True

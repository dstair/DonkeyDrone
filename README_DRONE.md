# DonkeyDrone: CNN-Based Autonomous Drone Flight

Fly a drone using the same DonkeyCar workflow: 

1. drive manually
2. record data
3. train a CNN
4. fly autonomously

This adapts DonkeyCar's pipeline to control a simulated drone in PX4 + Gazebo.

The drone flies at a fixed altitude, and the CNN learns to control forward velocity
and yaw rate from camera images -- the same way DonkeyCar learns steering and throttle.

## How It Works

The drone uses the exact same DonkeyCar parts pipeline as a car:

```
Web Controller  -->  DriveMode  -->  DroneGymEnv  -->  PX4 SITL + Gazebo
 (steering,          (user vs         (maps to           (flies the
  throttle)           autopilot)       velocity            drone)
                                       commands)

Camera (RTSP)  <--  DroneGymEnv  <--  Gazebo
 (160x120 RGB)      (captures         (renders
                     frames)           scene)
```

**Semantic mapping:**
- `steering [-1, 1]` = yaw rate (turn left/right)
- `throttle [-1, 1]` = forward velocity (fly forward/backward)
- altitude is held constant by a PID controller

Since the Memory key names are identical to the car version, the web controller,
CNN model (KerasLinear), training pipeline, and data recording all work unchanged.



## Prerequisites

- **Python 3.11**
- **uv**
- **Docker Desktop** with Rosetta enabled (for Apple Silicon Macs):
  Docker Desktop -> Settings -> General -> "Use Rosetta for x86_64/amd64 emulation"

## Quick Start

### 1. Create a Python 3.11 environment and install dependencies

```bash
cd mycar
uv sync
# us sync should cover packages, but if you prefer: uv add donkeycar mavsdk opencv-python-headless "tornado>=6.2" tensorflow
```

### 2. Pull the PX4 simulator Docker image (one-time, ~2GB)

```bash
docker pull jonasvautherin/px4-gazebo-headless:latest --platform linux/arm64

# above arm64 build runs best for Mac, but for amd64 build: docker pull jonasvautherin/px4-gazebo-headless:1.16.1
```

### 3. Start the simulator

```bash
docker run --platform linux/arm64 --rm -d \
  --name px4_sitl \
  -p 14540:14540/udp \
  -p 14550:14550/udp \
  -p 8554:8554 \
  jonasvautherin/px4-gazebo-headless:latest \
  -v gz_x500_mono_cam
```

Wait ~20 seconds for PX4 to boot, then verify:

```bash
docker logs px4_sitl 2>&1 | grep "Startup script returned"
# Should show: INFO  [px4] Startup script returned successfully
```

### 4. Fly the drone

```bash
uv run python drone_manage.py drive --myconfig=drone_config.py
```

Then open http://localhost:8887 in your browser.

- Use the **steering slider** to yaw (turn left/right)
- Use the **throttle slider** to fly forward/backward
- The drone holds altitude automatically
- Click **Start Recording** to capture training data

### 5. Train a model

```bash
#.venv/bin/python train.py --tubs=data/<your_tub_folder> --model=models/drone_pilot.h5
uv run python train.py --tubs=data/<your_tub_folder> --model=models/drone_pilot.h5
```

### 6. Fly with autopilot

```bash
uv run python drone_manage.py drive --myconfig=drone_config.py --model=models/drone_pilot.h5
```

Switch to **Local** mode in the web UI to let the CNN fly.

## Stopping

```bash
# Stop the DonkeyCar process: Ctrl+C in the terminal

# Stop the simulator
docker stop px4_sitl
```

## Configuration

Edit `drone_config.py` to adjust flight parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRONE_MAX_FORWARD_VEL` | 2.0 | Max forward speed (m/s) at full throttle |
| `DRONE_MAX_YAW_RATE` | 90.0 | Max yaw rate (deg/s) at full steering |
| `DRONE_TARGET_ALTITUDE` | 3.0 | Altitude hold target (meters) |
| `DRONE_ALTITUDE_KP/KI/KD` | 0.5/0.1/0.2 | Altitude PID gains |
| `DRIVE_LOOP_HZ` | 20 | Vehicle loop frequency |

## Architecture

```
macOS Host                          Docker Container
+----------------------------------+  +---------------------------+
|                                  |  |                           |
| drone_manage.py                  |  | PX4 SITL (autopilot)     |
|   +-- DroneGymEnv                |  |   +-- Offboard mode      |
|   |     +-- MAVSDK-Python -------|->|   +-- Failsafes          |
|   |     |   (velocity cmds)      |  |                           |
|   |     +-- OpenCV (RTSP) <------|--|-- Gazebo (rendering)      |
|   |         (camera frames)      |  |     +-- gz_x500_mono_cam  |
|   +-- LocalWebController         |  |     +-- RTSP on :8554     |
|   +-- DriveMode                  |  |                           |
|   +-- KerasLinear (CNN)          |  | UDP 14540: MAVLink        |
|   +-- TubWriter                  |  | TCP 8554:  camera stream  |
+----------------------------------+  +---------------------------+
```

## Files

| File | Purpose |
|------|---------|
| `drone_manage.py` | Main entry point (replaces `manage.py` for drone use) |
| `drone_gym.py` | DroneGymEnv part: MAVSDK + RTSP bridge to DonkeyCar |
| `drone_config.py` | Drone-specific configuration |
| `config.py` | Base DonkeyCar config (shared, not modified) |
| `manage.py` | Original car entry point (unchanged) |



## Troubleshooting

**"heartbeats timed out" warnings**: Normal when running PX4 SITL under Rosetta
emulation on Apple Silicon. The sim clock is slower than realtime. The drone still
flies correctly.

**RTSP stream not opening**: Make sure port 8554 is mapped in the Docker run command.
Verify with: `curl -v rtsp://127.0.0.1:8554/live` or test in Python:
```python
import cv2
cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/live")
print(cap.isOpened())  # Should be True
```

**"Address already in use" on port 14540**: A previous MAVSDK session didn't close
cleanly. Wait a few seconds or restart the Docker container.

**Docker container exits immediately**: Check RAM usage. PX4 + Gazebo under Rosetta
needs ~4GB. Close other applications if needed.

**Drone doesn't reach target altitude**: Under Rosetta emulation the sim runs slowly.
Lower `DRONE_TARGET_ALTITUDE` in `drone_config.py` (e.g., to 3.0m).

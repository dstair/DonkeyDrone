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

Camera frames  <--  DroneGymEnv  <--  Gazebo
 (160x120 RGB)      (gz-transport      (renders
                     or RTSP)          scene)
```

Two camera modes (set `DRONE_CAMERA_SOURCE` in `drone_config.py`):
- **`gz_transport`** (default) — native macOS: Gazebo Harmonic publishes images
  on a gz-transport topic. GPU-accelerated via Metal/OpenGL. Requires `gz-python`.
- **`rtsp`** — Docker mode: Gazebo Classic streams RTSP on port 8554. No GPU.

**Semantic mapping:**
- `steering [-1, 1]` = yaw rate (turn left/right)
- `throttle [-1, 1]` = forward velocity (fly forward/backward)
- altitude is held constant by a PID controller

Since the Memory key names are identical to the car version, the web controller,
CNN model (KerasLinear), training pipeline, and data recording all work unchanged.



## Prerequisites

- **Python 3.11**
- **uv**
- **One of:**
  - **Native macOS** (recommended — GPU-accelerated): Gazebo Harmonic + PX4 SITL under Rosetta. See [Native macOS Setup](#native-macos-setup-gpu-accelerated) below.
  - **Docker Desktop** with Rosetta enabled: Docker Desktop → Settings → General → "Use Rosetta for x86_64/amd64 emulation". See [Docker Setup](#docker-setup-no-gpu) below.

## Native macOS Setup (GPU-Accelerated)

> **⚠️ WARNING: Native macOS setup on Apple Silicon is EXTREMELY COMPLEX and may not work.**
>
> **Known Issues:**
> - PX4 SITL requires x86_64 (Rosetta), but many dependencies exist in ARM64 Homebrew
> - CMake cannot easily isolate x86_64 libraries when both ARM64 and x86_64 Homebrew exist
> - Anaconda Qt libraries conflict with x86_64 gz-harmonic
> - Requires extensive manual dependency management and dual Homebrew installations
>
> **Recommendation:** Use Docker mode (below) unless you have a specific need for GPU acceleration
> and are willing to troubleshoot complex architecture conflicts.

Runs Gazebo Harmonic natively with Metal/OpenGL rendering. Significantly lower
CPU usage than Docker since the M1 GPU handles scene rendering.

This all needs to be done under Rosetta because PX4 Doesn't Support ARM64 Software In The Loop (SITL).

> **Known risk:** The Metal camera sensor crash ([gz-sim #2877](https://github.com/gazebosim/gz-sim/issues/2877))
> may affect `gz_x500_mono_cam`. If you hit it, add
> `--render-engine-gui-api-backend opengl` to the `gz sim` invocations below.

### Phase 1: Install Gazebo Harmonic (one-time)

```bash
arch -x86_64 /usr/local/bin/brew tap osrf/simulation
arch -x86_64 /usr/local/bin/brew install gz-harmonic
gz sim --version  # verify install
```

### Phase 2: Build PX4 SITL under Rosetta (one-time)

PX4 does not support native ARM64 SITL. Use a Rosetta x86 terminal:

1. Run terminal in Rosetta/emulation mode: `arch -x86_64 zsh -l` (depending on shell, replace `zsh` with `bash` etc)

I ended up installing x86_64 homebrew - need to avoid an OpenCV error.
```bash
arch -x86_64 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
/usr/local/bin/brew install opencv

#/usr/local/Cellar/opencv/4.13.0_3
```

```bash
brew tap PX4/px4
brew install px4-dev
arch -x86_64 brew install opencv
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/dev/PX4-Autopilot
cd ~/dev/PX4-Autopilot

# Create x86_64 Python virtual environment (CRITICAL - must be x86_64, not ARM64!)
/usr/local/bin/python3 -m venv ~/px4-venv-x86
source ~/px4-venv-x86/bin/activate
pip install -r /Users/Dan/dev/PX4-Autopilot/Tools/setup/requirements.txt

# Build PX4 in full Rosetta x86_64 environment
# Note: Comment out -Wdouble-promotion in cmake/px4_add_common_flags.cmake:76
# if you encounter Abseil compatibility errors
arch -x86_64 /bin/bash -c "
cd ~/dev/PX4-Autopilot
source ~/px4-venv-x86/bin/activate
PYTHON_EXECUTABLE=\$(which python3) make px4_sitl gz_x500_mono_cam
"
```

### Phase 3: Start PX4 + Gazebo (3 terminals each session)

```bash
# Terminal 1 (native): Gazebo server
gz sim -s -r ~/dev/px4-gazebo-models/worlds/walls.sdf

# Terminal 2 (native): Gazebo GUI (GPU-rendered)
gz sim -g

# Terminal 3 (Rosetta x86): PX4 SITL
cd ~/dev/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mono_cam \
  ./build/px4_sitl_default/bin/px4
```

Wait ~15 seconds for PX4 to connect, then verify the camera topic:

```bash
gz topic -l | grep camera
# Should show: /world/walls/model/x500_mono_cam_0/link/camera_link/sensor/camera_sensor/image
```

If the topic path differs (e.g., model index `_1` instead of `_0`), update
`DRONE_GZ_CAMERA_TOPIC` in `drone_config.py`.

### Phase 4: Install Python dependencies and gz-python

```bash
uv sync
uv add gz-python   # gz-transport Python bindings
```

> If `gz-python` is not available on PyPI for your platform, see the
> [gz-python build-from-source guide](https://github.com/srmainwaring/gz-python).

### Phase 5: Fly the drone

`DRONE_CAMERA_SOURCE = "gz_transport"` is already the default in `drone_config.py`.

```bash
uv run python drone_manage.py drive --myconfig=drone_config.py
```

Then open http://localhost:8887.

---

## Docker Setup (No GPU)

### 1. Create a Python 3.11 environment and install dependencies

```bash
cd mycar
uv sync
# us sync should cover packages, but if you prefer: uv add donkeycar mavsdk opencv-python-headless "tornado>=6.2" tensorflow
```

Set `DRONE_CAMERA_SOURCE = "rtsp"` in `drone_config.py` and uncomment
`DRONE_RTSP_URL = "rtsp://127.0.0.1:8554/live"`.

### 2. Pull the PX4 simulator Docker image (one-time, ~2GB)

```bash
docker pull jonasvautherin/px4-gazebo-headless:latest --platform linux/arm64

# above arm64 build runs best for Mac, but for amd64 build: docker pull jonasvautherin/px4-gazebo-headless:1.16.1
```


### 2a download Gazebo worlds of interest

git clone https://github.com/PX4/PX4-gazebo-models.git ~/dev/px4-gazebo-models

### 3. Start the simulator

# choose your Gazebo 8.10 world - walls, baylands, forest
```bash
  docker run --platform linux/arm64 --rm -d \
    --name px4_sitl \
    --cpus=3 \
    --volume ~/dev/px4-gazebo-models/worlds:/root/px4/Tools/simulation/gz/worlds \
    --volume ~/dev/px4-gazebo-models/models:/root/px4/Tools/simulation/gz/models \
    -p 14540:14540/udp \
    -p 14550:14550/udp \
    -p 8554:8554 \
    jonasvautherin/px4-gazebo-headless:latest \
    -v gz_x500_mono_cam \
    -w walls
```

Wait ~20 seconds for PX4 to boot, then verify:

```bash
docker logs px4_sitl 2>&1 | grep "Startup script returned"
# Should show: INFO  [px4] Startup script returned successfully
```

### 4. Fly the drone (Docker mode)

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

Edit `drone_config.py` to adjust parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRONE_CAMERA_SOURCE` | `"gz_transport"` | `"gz_transport"` (native macOS) or `"rtsp"` (Docker) |
| `DRONE_GZ_CAMERA_TOPIC` | (x500_mono_cam default) | gz-transport topic for camera images (native mode) |
| `DRONE_RTSP_URL` | `"rtsp://127.0.0.1:8554/live"` | RTSP stream URL (Docker mode only) |
| `DRONE_MAX_FORWARD_VEL` | 2.0 | Max forward speed (m/s) at full throttle |
| `DRONE_MAX_YAW_RATE` | 90.0 | Max yaw rate (deg/s) at full steering |
| `DRONE_TARGET_ALTITUDE` | 3.0 | Altitude hold target (meters) |
| `DRONE_ALTITUDE_KP/KI/KD` | 0.5/0.1/0.2 | Altitude PID gains |
| `DRIVE_LOOP_HZ` | 10 | Vehicle loop frequency |

## Architecture

**Native macOS (gz-transport mode):**
```
macOS Host
+----------------------------------------------+
|                                              |
| gz sim (Gazebo Harmonic, GPU-accelerated)    |
|   +-- gz_x500_mono_cam (drone + camera)      |
|   +-- gz-transport: publishes /world/.../image|
|                                              |
| PX4 SITL (Rosetta x86 terminal)             |
|   +-- gz_bridge (connects to Gazebo)         |
|   +-- MAVLink on UDP 14540                   |
|                                              |
| drone_manage.py                              |
|   +-- DroneGymEnv                            |
|   |     +-- MAVSDK-Python -> PX4 (UDP 14540) |
|   |     +-- gz.transport13 <- Gazebo (images) |
|   +-- LocalWebController                     |
|   +-- DriveMode                              |
|   +-- KerasLinear (CNN)                      |
|   +-- TubWriter                              |
+----------------------------------------------+
```

**Docker mode (RTSP):**
```
macOS Host                          Docker Container
+----------------------------------+  +---------------------------+
|                                  |  |                           |
| drone_manage.py                  |  | PX4 SITL (autopilot)     |
|   +-- DroneGymEnv                |  |   +-- Offboard mode      |
|   |     +-- MAVSDK-Python -------|->|   +-- Failsafes          |
|   |     |   (velocity cmds)      |  |                           |
|   |     +-- OpenCV (RTSP) <------|--|-- Gazebo Classic          |
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
| `drone_manage.py` | Main entry point (replaces `manage.py` for drone use)
| `drone_gym.py`    | DroneGymEnv part: MAVSDK + gz-transport/RTSP bridge to DonkeyCar
| `drone_config.py` | Drone-specific configuration (camera source, flight params)
| `config.py`       | Base DonkeyCar config (shared, not modified)
| `manage.py`       | Original car entry point (unchanged)



## Troubleshooting

### Native macOS (gz-transport mode)

**`gz-python` not available via pip**: Build from source following the
[gz-python guide](https://github.com/srmainwaring/gz-python). Alternatively,
switch to Docker mode: set `DRONE_CAMERA_SOURCE = "rtsp"` in `drone_config.py`.

**Camera topic not found / blank frames**: Run `gz topic -l | grep camera` while
Gazebo is running to confirm the exact topic name. Update `DRONE_GZ_CAMERA_TOPIC`
in `drone_config.py` if the model index differs (e.g., `_1` instead of `_0`).

**Metal crash with camera sensor** ([gz-sim #2877](https://github.com/gazebosim/gz-sim/issues/2877)):
Fall back to OpenGL by adding `--render-engine-gui-api-backend opengl` to the
`gz sim` commands in Phase 3.

**PX4 SITL build fails on macOS 15.x**: Try pinning a specific PX4 version. See
[PX4 macOS dev setup docs](https://docs.px4.io/main/en/dev_setup/dev_env_mac).

### Docker mode (RTSP)

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

**Docker container exits immediately**: Check RAM usage. PX4 + Gazebo under Rosetta
needs ~4GB. Close other applications if needed.

### General

**"Address already in use" on port 14540**: A previous MAVSDK session didn't close
cleanly. Wait a few seconds or restart the simulator.

**Drone doesn't reach target altitude**: Under Rosetta emulation the sim runs slowly.
Lower `DRONE_TARGET_ALTITUDE` in `drone_config.py` (e.g., to 3.0m).

###### CPU usage too high on M1 Mac (Docker mode)

Docker does not have access to the Mac's GPU. Native macOS setup resolves this.
For Docker, these mitigations help:

  | Change | CPU Savings | Trade-off |
  |--------|-------------|-----------|
  | `--cpus=3` on Docker | Caps container at 3 cores | Sim runs slower |
  | `PX4_SIM_SPEED_FACTOR=0.5` | ~halves Gazebo load | Sim time 2x slower |
  | `DRIVE_LOOP_HZ = 10` | Less host-side work | Fine for emulated SITL |
  | RTSP buffer=1 + frame skip | Less OpenCV decode overhead | ~5 FPS camera |
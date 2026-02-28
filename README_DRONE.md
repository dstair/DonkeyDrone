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
  - **Native macOS** (recommended — GPU-accelerated): PX4 SITL + Gazebo Harmonic running natively on ARM64. See [Native macOS Setup](#native-macos-setup-gpu-accelerated) below.
  - **Docker Desktop**: Gazebo Classic + PX4 in a Linux container. No GPU access. See [Docker Setup](#docker-setup-no-gpu) below.

## Native macOS Setup (GPU-Accelerated)

Runs PX4 SITL and Gazebo Harmonic natively on Apple Silicon (ARM64). No Rosetta
required. Uses the M1/M2 GPU for scene rendering — significantly lower CPU usage
than Docker.

> **Known risk:** Metal camera sensor crash ([gz-sim #2877](https://github.com/gazebosim/gz-sim/issues/2877))
> may affect `gz_x500_mono_cam`. If you hit it, set `PX4_GZ_SIM_RENDER_ENGINE=ogre`
> before launching PX4.

### Phase 1: Install dependencies (one-time)

All dependencies must come from the **ARM64 Homebrew** at `/opt/homebrew`. Do not
use Rosetta or `/usr/local/bin/brew` for any of these.

```bash
# PX4 build toolchain
brew tap PX4/px4
brew install px4-dev
brew link --overwrite --force arm-gcc-bin@13

# Gazebo Harmonic (ARM64) — installs gz-sim8, gz-transport13, etc.
brew tap osrf/simulation
brew install gz-harmonic

# OpenCV (needed for the optical flow Gazebo plugin)
brew install opencv

# Verify Gazebo installed correctly
gz sim --version   # should print: Gazebo Sim, version 8.x.x
```

### Phase 2: Build PX4 SITL (one-time)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/dev/PX4-Autopilot
cd ~/dev/PX4-Autopilot
```

Apply a required source patch — `optical_flow.cmake` hardcodes `.so` (Linux
convention) which breaks linking on macOS. In
`src/modules/simulation/gz_plugins/optical_flow/optical_flow.cmake`, replace
both occurrences of `libOpticalFlow.so` with
`libOpticalFlow${CMAKE_SHARED_LIBRARY_SUFFIX}`.

Then build:

```bash
cd ~/dev/PX4-Autopilot
make px4_sitl gz_x500_mono_cam
```

The build ends with `ERROR [init] Timed out waiting for Gazebo world` — this is
**expected** (the post-build self-test tries to connect to a Gazebo that isn't
running). The binary is complete. Verify:

```bash
file build/px4_sitl_default/bin/px4
# → Mach-O 64-bit executable arm64  ✓
```

After building, create one symlink needed at runtime (the OpticalFlow plugin's
rpath points to a different directory than where the build places the library):

```bash
mkdir -p ~/dev/PX4-Autopilot/build/px4_sitl_default/external/Install/lib
ln -sf ~/dev/PX4-Autopilot/build/px4_sitl_default/OpticalFlow/install/lib/libOpticalFlow.dylib \
       ~/dev/PX4-Autopilot/build/px4_sitl_default/external/Install/lib/libOpticalFlow.dylib
```

Also comment out the GstCameraSystem plugin in
`src/modules/simulation/gz_bridge/server.config` (GStreamer is not built; we use
gz-transport for camera frames instead):

```xml
<!-- <plugin entity_name="*" entity_type="world" filename="libGstCameraSystem.so" name="custom::GstCameraSystem"/> -->
```

### Phase 3: Download Gazebo world models (one-time)

```bash
git clone https://github.com/PX4/PX4-gazebo-models.git ~/dev/px4-gazebo-models
```

### Phase 4: Start PX4 + Gazebo (each session)

Save the following as a launch script (e.g. `~/start_px4.sh`) and run it in a
dedicated terminal:

```bash
#!/bin/bash
# ARM64 Ruby must come first — the `gz` wrapper is a Ruby script and macOS has
# an x86_64 Ruby at /usr/local/bin/ruby that would load the wrong arch dylibs.
export PATH=/opt/homebrew/opt/ruby/bin:/opt/homebrew/bin:$PATH
export HEADLESS=1                    # skip Gazebo GUI (add a separate `gz sim -g` if you want it)
export PX4_SYS_AUTOSTART=4001       # x500 quadrotor airframe
export PX4_SIM_MODEL=gz_x500_mono_cam  # camera-equipped model (overrides autostart default of x500)
export PX4_GZ_WORLD=default         # use 'default' world (fully local, no downloads)
# All paths must be set explicitly — gz_env.sh is not sourced reliably at runtime
export PX4_GZ_WORLDS=~/dev/PX4-Autopilot/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=~/dev/PX4-Autopilot/Tools/simulation/gz/models
export PX4_GZ_PLUGINS=~/dev/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins
export GZ_SIM_RESOURCE_PATH=$PX4_GZ_MODELS:$PX4_GZ_WORLDS
export GZ_IP=127.0.0.1              # suppress multicast "No route to host" warnings on macOS
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=~/dev/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config
cd ~/dev/PX4-Autopilot/build/px4_sitl_default
./bin/px4 -s etc/init.d-posix/rcS
```

Wait ~15 seconds. Success looks like:

```
INFO  [init] Gazebo world is ready
INFO  [init] Spawning Gazebo model
INFO  [gz_bridge] world: default, model: x500_mono_cam_0
INFO  [px4] Startup script returned successfully
pxh>
```

Expected (non-fatal) warnings: `ekf2 missing data` (no sensor lock yet),
`No connection to the GCS` (no ground station connected yet), Qt5 duplicate
class warnings (two Qt5 installs coexist from Homebrew and Anaconda).
`GZ_IP=127.0.0.1` suppresses the repeated `Exception sending a multicast message: No route to host`
warnings (macOS loopback has no multicast route by default).


PX4 SITL spawns both a px4 process and a gz sim server. Ctrl+C typically only kills the shell foreground process, leaving Gazebo orphaned. Kill both:                                                                                   

```bash
pkill -f "bin/px4" && pkill -f "gz sim"    
```

(optional) Verify the camera topic is publishing. The first command lists ALL topics getting published, the second dumps an image to the terminal.

```bash
GZ_IP=127.0.0.1 PATH=/opt/homebrew/opt/ruby/bin:$PATH gz topic -l
GZ_IP=127.0.0.1 PATH=/opt/homebrew/opt/ruby/bin:$PATH gz topic -e -n 1 -t /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
```

The above command should show: "/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image". If the topic path differs (e.g., model index `_1`), update
`DRONE_GZ_CAMERA_TOPIC` in `drone_config.py`.

### Phase 5: Install Python dependencies

```bash
uv sync
```

`gz-python` (the gz-transport Python bindings) is **not on PyPI**. It is installed
automatically by `brew install gz-harmonic` into Homebrew's Python site-packages.
The project uses Python 3.12, matching the Homebrew-installed bindings.

Create a `.env` file in the project root to tell `uv run` where to find the bindings:

```bash
# .env  (edit paths if your Homebrew prefix differs)
PYTHONPATH=/opt/homebrew/lib/python3.12/site-packages
DYLD_LIBRARY_PATH=/opt/homebrew/lib
GZ_IP=127.0.0.1   # suppresses "No route to host" multicast warnings on macOS loopback
```

Verify the bindings are importable:

```bash
uv run --env-file .env python -c "import gz.transport13; print('OK')"
```

### Phase 6: Fly the drone

`DRONE_CAMERA_SOURCE = "gz_transport"` is already the default in `drone_config.py`.

```bash
uv run --env-file .env python drone_manage.py drive --myconfig=drone_config.py
```

Then open http://localhost:8887.

---

## Configuration

Edit `drone_config.py` to adjust parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRONE_CAMERA_SOURCE` | `"gz_transport"` | `"gz_transport"` (native macOS) or `"rtsp"` (Docker) |
| `DRONE_GZ_CAMERA_TOPIC` | (x500_mono_cam default) | gz-transport topic for camera images (native mode) |
| `DRONE_MAX_FORWARD_VEL` | 2.0 | Max forward speed (m/s) at full throttle |
| `DRONE_MAX_YAW_RATE` | 90.0 | Max yaw rate (deg/s) at full steering |
| `DRONE_TARGET_ALTITUDE` | 3.0 | Altitude hold target (meters) |
| `DRONE_ALTITUDE_KP/KI/KD` | 0.5/0.1/0.2 | Altitude PID gains |
| `DRIVE_LOOP_HZ` | 10 | Vehicle loop frequency |

## Architecture

**Native macOS (gz-transport mode):**
```
macOS Host (Apple Silicon, ARM64)
+----------------------------------------------+
|                                              |
| gz sim (Gazebo Harmonic, GPU-accelerated)    |
|   +-- x500_mono_cam_0 (drone + camera)       |
|   +-- gz-transport: publishes /world/.../image|
|                                              |
| PX4 SITL (native ARM64 binary)              |
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

**Camera topic not found / blank frames**: Run
`PATH=/opt/homebrew/opt/ruby/bin:$PATH gz topic -l | grep camera` while Gazebo
is running to confirm the exact topic name. Update `DRONE_GZ_CAMERA_TOPIC` in
`drone_config.py` if the model index differs (e.g., `_1` instead of `_0`).

**Metal crash with camera sensor** ([gz-sim #2877](https://github.com/gazebosim/gz-sim/issues/2877)):
Set `PX4_GZ_SIM_RENDER_ENGINE=ogre` in the launch script before starting PX4.

**`gz sim` fails to load plugins / "incompatible architecture"**: The `gz` Ruby
wrapper must run under ARM64 Ruby. Ensure
`PATH=/opt/homebrew/opt/ruby/bin:/opt/homebrew/bin:$PATH` is set before running
any `gz` command. The x86_64 Ruby at `/usr/local/bin/ruby` will fail to dlopen
ARM64 Gazebo libraries.

**`ERROR [init] Gazebo gz sim not found`**: PX4's `px4-rc.gzsim` runs under `/bin/sh` which may
not inherit PATH correctly on macOS, so it can't find `gz`. The source patch in
`ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim` adds
`PATH="/opt/homebrew/opt/ruby/bin:/opt/homebrew/bin:$PATH"` at the top. After patching,
copy the file to `build/px4_sitl_default/etc/init.d-posix/px4-rc.gzsim` (no rebuild needed).

**PX4 "Timed out waiting for Gazebo world" at build time**: Expected — the
post-build self-test tries to connect to a running Gazebo. The binary is built
correctly regardless.

**Rebuild needed after `make` changes**: If you need to clean and rebuild:
```bash
rm -rf ~/dev/PX4-Autopilot/build/px4_sitl_default
cd ~/dev/PX4-Autopilot && make px4_sitl gz_x500_mono_cam
# Then recreate the libOpticalFlow.dylib symlink (see Phase 2)
```

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

###### Resource utilization too high

These settings can be adjusted to let the simulation run more slowly, consuming fewer resources.

| Change | CPU Savings | Trade-off |
|--------|-------------|-----------|
| `PX4_SIM_SPEED_FACTOR=0.5` | ~halves Gazebo load | Sim time 2x slower |
| `DRIVE_LOOP_HZ = 10` | Less host-side work | Fine for emulated SITL |
| RTSP buffer=1 + frame skip | Less OpenCV decode overhead | ~5 FPS camera |


## TODO:

- DonkeyDrone runs Walls world, but can really just see gray and white - figure out why. Also consider Warehouse world which seems better for CNN training.
- Also render speed seems slow, try to speed up for flyability (double check the above sections)

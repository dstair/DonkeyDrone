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

- **Python 3.12**
- **uv**
- **One of:**
  - **Native macOS** (recommended — GPU-accelerated): PX4 SITL + Gazebo Harmonic running natively on ARM64. See [Native macOS Setup](#native-macos-setup-gpu-accelerated) below.
  - **Docker Desktop**: Gazebo Classic + PX4 in a Linux container. No GPU access. See [Docker Setup](#docker-setup-no-gpu) below.

## Native macOS Setup (GPU-Accelerated)

Runs PX4 SITL and Gazebo Harmonic natively on Apple Silicon (ARM64). Tested on an M1 GPU for scene rendering. GPU (not supported by docker) was needed for acceptable performance.


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

Symlink the DonkeyDrone course world into PX4's worlds directory (one-time):

```bash
ln -sf ~/dev/DonkeyDrone/worlds/drone_course.sdf \
       ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/drone_course.sdf
```

[RUN COMMAND] Run the start script in a dedicated terminal:

```bash
bash ./px4_gazebo_start.sh
```

Wait ~15 seconds. Success looks like:

```
INFO  [init] Gazebo world is ready
INFO  [init] Spawning Gazebo model
INFO  [gz_bridge] world: walls, model: x500_mono_cam_0
INFO  [px4] Startup script returned successfully
pxh>
```

Expected (non-fatal) warnings: 
- `ekf2 missing data` (no sensor lock yet),
- `No connection to the GCS` (no ground station connected yet), 
- Qt5 duplicate class warnings (two Qt5 installs coexist from Homebrew and Anaconda).
- `GZ_IP=127.0.0.1` solves noisy `Exception sending a multicast message: No route to host` warnings



(optional) Verify the camera topic is publishing. The first command lists ALL topics getting published, the second dumps an image to the terminal.

```bash
GZ_IP=127.0.0.1  gz topic -l
GZ_IP=127.0.0.1 gz topic -e -n 1 -t /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image
```

The above command should show: "/world/drone_course/model/x500_mono_cam_0/link/camera_link/sensor/camera/image". If the topic path differs (e.g., model index `_1` or a different world name), update
`DRONE_GZ_CAMERA_TOPIC` in `drone_config.py`.


[RUN COMMAND] Optional - open the Gazebo GUI:

```bash
export PATH="/opt/homebrew/opt/ruby/bin:/opt/homebrew/bin:$PATH" # add to ~/.zshrc 
GZ_IP=127.0.0.1 gz sim -g &
```

#### Stopping everything

This spans a number of process that Ctrl + C will not cleanly kill. To stop via a series of pkill commands:                                                                              

```bash
bash ./stop_all.sh
```

### Phase 5: Install Python dependencies

```bash
uv sync
```

`gz-python` (the gz-transport Python bindings) is **not on PyPI**. Install via `brew install gz-harmonic`.

Create a `.env` file in the project root to tell `uv run` where to find the bindings:

```bash
# .env  (edit paths if your Homebrew prefix differs)
PYTHONPATH=/opt/homebrew/lib/python3.12/site-packages
DYLD_LIBRARY_PATH=/opt/homebrew/lib
GZ_IP=127.0.0.1   # suppresses noisy warning
```

Verify the bindings are importable:

```bash
uv run --env-file .env python -c "import gz.transport13; print('OK')"
```

### Run DonkeyDrone

[RUN COMMAND] run DonkeyDrone, the Web UI. This allows you to see the camera, provide inputs (fly the drone around) and record training data which will be used in the next step of the pipeline.

```bash
uv run --env-file .env python drone_manage.py drive --myconfig=drone_config.py
```

Then open http://127.0.0.1:8887

---

## Running the training pipeline


```bash
uv run python train.py --tubs=data/tub_16_26-03-01 --model=models/pilot.h5
```

Once training completes, note the model outputs - models/\*.h5 and models/\*.tflite - needed for next step.

If interested, you can optionally look at:
- number of epochs. Did early stopping kick in?
- val_loss and what epoch minimum loss occurred at.
- number of samples it training and and validation datasets.


## Testing the trained autopilot

To test the autopilot model created in the previous step:

```bash
uv run --env-file .env \
  python drone_manage.py \
  drive --model=models/pilot.h5 \
  --myconfig=drone_config.py
```

In the Web UI, switch to "local_angle" or "local" mode.

## Configuration

Edit `drone_config.py` to adjust parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRONE_CAMERA_SOURCE` | `"gz_transport"` | `"gz_transport"` (native macOS) or `"rtsp"` (Docker) |
| `DRONE_GZ_CAMERA_TOPIC` | (drone_course world default) | gz-transport topic for camera images (native mode) |
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

### General notes & troubleshooting.

#### Steering zeroed at low throttle (can't yaw in place)

DonkeyCar's web controller has car-specific logic that zeros steering when throttle
is near zero (a car can't steer while stationary). This prevents the drone from
yawing in place and makes it feel like turning barely works when you aren't also
pushing forward.

**Fix:** Comment out the deadzone check in the nipple.js `move` handler in
`.venv/lib/python3.12/site-packages/donkeycar/parts/web_controller/templates/static/main.js`:

```javascript
// ~line 215 — comment out these three lines:
// if (state.tele.user.throttle < .001) {
//   state.tele.user.angle = 0
// }
```

After patching, restart `drone_manage.py` and hard-refresh the web UI (Cmd+Shift+R).
This patch lives inside `.venv/` and will be lost if you recreate the virtual
environment.

#### Resource utilization too high

These settings can be adjusted to let the simulation run more slowly, consuming fewer resources.

| Change | CPU Savings | Trade-off |
|--------|-------------|-----------|
| `PX4_SIM_SPEED_FACTOR=0.5` | ~halves Gazebo load | Sim time 2x slower |
| `DRIVE_LOOP_HZ = 10` | Less host-side work | Fine for emulated SITL |
| RTSP buffer=1 + frame skip | Less OpenCV decode overhead | ~5 FPS camera |


#### Worlds

The default world is **`drone_course`** (`worlds/drone_course.sdf` in this repo).
It provides high-contrast colored surfaces that a CNN can actually learn from:

| Feature | Color | Purpose |
|---------|-------|---------|
| Ground | Dark green | Distinct from walls and sky |
| Sky | Blue with clouds | Horizon reference |
| Left wall | Red | Turn-left cue |
| Back wall | Yellow | End-of-corridor cue |
| Right walls | Blue | Turn-right cue |
| Top wall | Orange | Boundary marker |
| Pillars | White, purple | Interior landmarks |
| Landing pad | Dark gray circle | Origin reference |

The walls form an L-shaped corridor the drone can fly through, with each wall a
different color so the CNN learns directional associations.

To switch worlds, change `PX4_GZ_WORLD` in the launch script **and** update
`DRONE_GZ_CAMERA_TOPIC` in `drone_config.py` to match. Other bundled PX4 worlds:
`walls` (gray boxes), `lawn` (green ground, no obstacles), `default` (empty gray).


#### Render Performance

The mono_cam sensor renders at **1280x960 @ 30 Hz** natively, downscaled to
160x120 in `drone_gym.py`.  If Gazebo CPU usage is too high:

| Change | Where | Effect |
|--------|-------|--------|
| Lower `update_rate` to 15 | `PX4-Autopilot/.../mono_cam/model.sdf` | Halves render load; still >DRIVE_LOOP_HZ |
| `PX4_SIM_SPEED_FACTOR=0.5` | Launch script env var | Halves Gazebo physics + render load |
| `DRIVE_LOOP_HZ = 5` | `drone_config.py` | Fewer MAVSDK commands per second |
| `HEADLESS=1` | Launch script (already set) | No GUI window rendering |

### CNN Training size

KerasLinear is fully size-agnostic. The CNN uses Flatten() after convolutions, so it adapts to any input resolution. The only change needed is setting IMAGE_W and IMAGE_H in drone_config.py

At 640x480 the Dense(100) layer alone would have ~28M weights. At 720p it'd be ~85M. Training on a laptop would be noticeably slower.

Trade-offs by resolution:                                                                                                       
  ┌───────────────────┬────────┬──────────────────┬───────────────┬───────────────────────────────┐                                           
  │    Resolution     │ Pixels │ CNN Flatten size │ Training cost │             Notes             │                                           
  ├───────────────────┼────────┼──────────────────┼───────────────┼───────────────────────────────┤
  │ 160x120 (default) │ 19K    │ ~18K params      │ Baseline      │ DonkeyCar car default         │                                           
  ├───────────────────┼────────┼──────────────────┼───────────────┼───────────────────────────────┤                                           
  │ 320x240           │ 77K    │ ~73K params      │ ~4x           │ Good middle ground            │                                           
  ├───────────────────┼────────┼──────────────────┼───────────────┼───────────────────────────────┤                                           
  │ 640x480           │ 307K   │ ~277K params     │ ~15x          │ Rich detail, heavier training │                                           
  ├───────────────────┼────────┼──────────────────┼───────────────┼───────────────────────────────┤                                           
  │ 1280x720          │ 922K   │ ~850K params     │ ~47x          │ Native 720p, very heavy       │                                           
  └───────────────────┴────────┴──────────────────┴───────────────┴───────────────────────────────┘

#### Cleaning Up Data

Several locations accumulate data over time:

| Location | Grows from | Typical size |
|----------|-----------|--------------|
| `data/` | DonkeyCar tub recordings (images + JSON per driving session) | ~1 MB per tub |
| `~/.gz/fuel/` | Gazebo Fuel model cache (downloaded meshes/textures) | ~400 MB |
| `~/.gz/auto_default.log` | Gazebo server log (appended each run) | grows over time |
| `~/dev/PX4-Autopilot/build/` | PX4 build artifacts (not runtime, but large) | ~800 MB |

```bash
# Check sizes
du -sh data/ ~/.gz/fuel/ ~/.gz/auto_default.log ~/dev/PX4-Autopilot/build/

# Delete old tub recordings (keeps the data/ directory)
rm -rf data/tub_*

# Clear Gazebo Fuel model cache (will re-download if needed)
rm -rf ~/.gz/fuel/

# Truncate Gazebo log
> ~/.gz/auto_default.log
```

**Safe to delete anytime:** `data/tub_*` (training data you've already used or don't need),
`~/.gz/fuel/` (re-downloads on next run), `~/.gz/auto_default.log`.

**Don't delete unless rebuilding:** `~/dev/PX4-Autopilot/build/` — takes ~10 min to rebuild.



#### Known risk: Metal camera sensor crash 

([gz-sim #2877](https://github.com/gazebosim/gz-sim/issues/2877))
may affect `gz_x500_mono_cam`. If you hit it, set `PX4_GZ_SIM_RENDER_ENGINE=ogre`
before launching PX4.


## TODO:

must have:
X get drone actually moving and test.
- test training CNN.
- Test flying using the new autopilot

nice to have:
- Add randomization of worlds (wall locations, colors) for better CNN training
- Add looping to train CNN on a variety of worlds
- research other tasks that would be interesting to implement (CNN to scan/build a 3D model of an object, for example.)
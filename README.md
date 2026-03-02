# DonkeyDrone: CNN-Based Autonomous Drone Flight

Fly a drone using the same workflow as DonkeyCar:

1. fly manually
2. record data
3. train a CNN
4. fly autonomously

This adapts DonkeyCar's pipeline to control a simulated drone in PX4 (flight controller firmware) + Gazebo (simulator).

The drone flies at a fixed altitude, and the CNN learns to control forward velocity
and yaw rate from camera images -- the same way DonkeyCar learns steering and throttle.

## How It Works

The drone uses the exact same DonkeyCar parts pipeline as a car:

```
Web Controller  -->  DriveMode  -->  DroneGymEnv  -->  PX4 SITL + Gazebo
 (steering,          (user or         (maps to           (flies the
  throttle)           autopilot)       velocity            drone)
                                       commands)

Camera frames  <--  DroneGymEnv  <--  Gazebo
 (320x240 RGB)      (gz-transport)    (renders scene)
```

Two camera modes (set `DRONE_CAMERA_SOURCE` in `donkeydrone/drone_config.py`):
- **`gz_transport`** (default) — native macOS: Gazebo Harmonic publishes images
  on a gz-transport topic. GPU-accelerated via Metal/OpenGL. Requires `gz-python`.

**Semantic mapping:**
- `steering [-1, 1]` = yaw rate (turn left/right)
- `throttle [-1, 1]` = forward velocity (fly forward/backward)
- altitude is held constant by a PID controller for simplicity; could be changed in the future.

Since Memory key names are identical to the car version, the web controller,
CNN model (KerasLinear), training pipeline, and data recording all work unchanged.

## Prerequisites

- **Python 3.12** - gz harmonic only supports Python 3.12, 3.13, or 3.14
- **uv**
- OS: This "should" run on any system that can run Gazebo, all required python packages, and PX4. However, it's only been tested on OSX Tahoe 26.3.

Side note: the setup was very, very involved. I would recommend you use a coding agent to work through the various build failures you will invariably hit. Gazebo and PX4 were both quite tricky to get up and running, largely because they do so much and require so many dependences. Also - installing Gazebo/PX4 on Ubuntu was much faster and smoother than on OSX, FWIW. But the convenience of running everything on my regular laptop made this worthwhile.

## Native macOS Setup (GPU-Accelerated)

Runs PX4 SITL and Gazebo Harmonic natively on Apple Silicon (ARM64). Tested on an M1 GPU for scene rendering. GPU (not supported by docker) was needed for acceptable performance.

### Step 1: Install dependencies (one-time)

Dependencies must come from the **ARM64 Homebrew** at `/opt/homebrew`. Do not
use Rosetta based HomeBrew (`/usr/local/bin/brew`).

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

### Step 2: Build PX4 SITL (one-time)

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
gz-transport for camera frames instead; this avoids a dependency issue):

```xml
<!-- <plugin entity_name="*" entity_type="world" filename="libGstCameraSystem.so" name="custom::GstCameraSystem"/> -->
```

### Step 3: Download Gazebo world models for PX4 (one-time)

```bash
git clone https://github.com/PX4/PX4-gazebo-models.git ~/dev/px4-gazebo-models
```

### Step 4: One-time world setup

Symlink the DonkeyDrone course world into PX4's worlds directory:

```bash
ln -sf ~/dev/DonkeyDrone/worlds/drone_course.sdf \
       ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/drone_course.sdf
```

### Step 5: Install Python dependencies

```bash
uv sync
```

Create a `.env` file in the project root to tell `uv run` where to find the bindings. The below links assume you installed Python 3.12 via HomeBrew:

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

A single `scripts/start.sh` script launches PX4 + Gazebo in the background, waits for readiness, then runs `donkeydrone/drone_manage.py` in the foreground. Ctrl+C stops everything cleanly (no orphan processes).

```bash
./scripts/start.sh                              # manual drive mode to collect training data
./scripts/start.sh --model=models/pilot.h5      # once you have an autopilot, launch with autopilot
```

PX4 SITL output is logged to `logs/px4_sitl.log`.

Then open http://127.0.0.1:8887

**What `scripts/start.sh` does:**
1. Launches PX4 SITL in the background
2. Waits for PX4 SITL drone to be ready
4. Runs `donkeydrone/drone_manage.py` which starts the Web UI
5. On exit or Ctrl + C, calls `scripts/stop_all.sh` to kill all processes cleanly.

(optional) Open the Gazebo GUI in a separate terminal while the sim is running; useful for seeing the whole world in higher resolution:

```bash
GZ_IP=127.0.0.1 gz sim -g &
```

#### Stopping everything

Ctrl+C in the `scripts/start.sh` terminal stops everything automatically. If processes are
still running for any reason:

```bash
bash ./scripts/stop_all.sh
```

---

## Running the training pipeline

Once you have flown the drone for a bit, there will be a new "tub" in the `data/` folder. You can then use that data to train a CNN. Note, this could take a while. A "hello world" test on 300 images took 5 minutes on an M1 Mac:

```bash
uv run python donkeydrone/torch_train.py --tubs=data/tub_[number]_[yy-mm-dd] --model=models/pilot.pth
```

Once training completes, note the model output: models/pilot.pth. It is needed for next step.

If interested, you can optionally look at:
- number of epochs. Did early stopping kick in?
- val_loss and when minimum loss occurred at.
- number of samples in training and and validation datasets.


## Testing the trained autopilot

To test the autopilot model created in the previous step:

```bash
./scripts/start.sh --model=models/pilot.pth
```

In the Web UI, switch to "local_angle" or "local" mode.

## Configuration

Edit `donkeydrone/drone_config.py` to adjust parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRONE_CAMERA_SOURCE` | `"gz_transport"` | `"gz_transport"` (native macOS) |
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
| `scripts/start.sh`        | Single-command launcher: starts PX4+Gazebo, runs drone_manage, cleans up on exit |
| `scripts/stop_all.sh`     | Kills all PX4/Gazebo/MAVSDK processes |
| `scripts/px4_gazebo_start.sh` | Standalone PX4+Gazebo launcher (used internally by `start.sh`) |
| `donkeydrone/drone_manage.py` | Main entry point (replaces `manage.py` for drone use) |
| `donkeydrone/drone_gym.py`    | DroneGymEnv part: MAVSDK + gz-transport/RTSP bridge to DonkeyCar |
| `donkeydrone/drone_config.py` | Drone-specific configuration (camera source, flight params) |
| `donkeydrone/config.py`       | Base DonkeyCar config (shared, not modified) |
| `donkeydrone/manage.py`       | Original car entry point (unchanged) |

## Troubleshooting

### Native macOS (gz-transport mode)

**`gz-python` not available via pip**: Build from source following the
[gz-python guide](https://github.com/srmainwaring/gz-python). Alternatively,
switch to Docker mode: set `DRONE_CAMERA_SOURCE = "rtsp"` in `donkeydrone/drone_config.py`.

**Camera topic not found / blank frames**: Run
`PATH=/opt/homebrew/opt/ruby/bin:$PATH gz topic -l | grep camera` while Gazebo
is running to confirm the exact topic name. Update `DRONE_GZ_CAMERA_TOPIC` in
`donkeydrone/drone_config.py` if the model index differs (e.g., `_1` instead of `_0`).

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
# Then recreate the libOpticalFlow.dylib symlink (see Step 2)
```

### General notes & troubleshooting.

#### Resource utilization

These settings can be adjusted to let the simulation run more slowly, consuming fewer resources.

| Change | CPU Savings | Trade-off |
|--------|-------------|-----------|
| `PX4_SIM_SPEED_FACTOR=0.5` | ~halves Gazebo load | Sim time 2x slower |
| `DRIVE_LOOP_HZ = 10` | Less host-side work | Fine for emulated SITL |
| Lower `update_rate` to 15 in `PX4-Autopilot/.../mono_cam/model.sdf` | Halves render load; still >DRIVE_LOOP_HZ | |
| `HEADLESS=1` | Launch script (already set) | No GUI window rendering |

#### Worlds

The default world is **`drone_course`** (`worlds/drone_course.sdf` in this repo).
It provides high-contrast colored surfaces.

To switch worlds, change `PX4_GZ_WORLD` in the launch script **and** update
`DRONE_GZ_CAMERA_TOPIC` in `donkeydrone/drone_config.py` to match.

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
| `~/dev/PX4-Autopilot/build/px4_sitl_default/log/*` | flight log data | 100s of MB to GBs | 


```bash
# Check sizes
du -sh data/ ~/.gz/fuel/ ~/.gz/auto_default.log ~/dev/PX4-Autopilot/build/

# Delete old tub recordings (keeps the data/ directory)
rm -rf data/tub_*

# Truncate Gazebo log
> ~/.gz/auto_default.log
```

**Safe to delete anytime:** `data/tub_*` (training data you've already used or don't need),
`~/.gz/fuel/` (re-downloads on next run), `~/.gz/auto_default.log`.

**Don't delete unless rebuilding:** `~/dev/PX4-Autopilot/build/` — takes ~10 min to rebuild.


## TODO:

of interest :
X performance acceleration for training on M1 mac. 
- add xbox controller support.
- swap out quadcopter type - in my planned build, can't see the rotors.
- research improvements to CNN, though it already is quite impressive.


lower priority :
- try a different world.
- Add randomization of worlds (wall locations, colors) for better CNN training
- Add looping to train CNN on a variety of worlds
- research other tasks that would be interesting to implement (CNN to scan/build a 3D model of an object, for example; object scanning CNN)

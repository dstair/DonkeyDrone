# DonkeyDrone: CNN-Based Autonomous Drone Flight

Fly a drone using the same workflow as DonkeyCar:

1. fly manually
2. record data
3. train a CNN
4. fly autonomously

This adapts DonkeyCar's pipeline to control a simulated drone in BetaFlight SITL (flight controller firmware) + Gazebo (simulator).

The CNN learns to control forward pitch, yaw rate, and motor throttle from camera
images -- the same way DonkeyCar learns steering and throttle.

## How It Works

The drone uses the exact same DonkeyCar parts pipeline as a car:

```
Web Controller  -->  DriveMode  -->  DroneGymEnv  -->  BetaFlight SITL + Gazebo
 (steering,          (user or         (RC channel        (flies the
  throttle,           autopilot)       packets via         drone)
  altitude)                            UDP 9004)

Camera frames  <--  DroneGymEnv  <--  Gazebo
 (320x240 RGB)      (gz-transport)    (renders scene)
```

Camera mode (set `DRONE_CAMERA_SOURCE` in your `donkeydrone/drone_config_XXmm.py`):
- **`gz_transport`** (default) — native macOS: Gazebo Harmonic publishes images
  on a gz-transport topic. GPU-accelerated via Metal/OpenGL. Requires `gz-python`.

**Semantic mapping (BetaFlight Angle mode):**
- `steering [-1, 1]` = yaw rate (turn left/right)
- `throttle [-1, 1]` = forward pitch (tilt forward/backward)
- `altitude [-1, 1]` = motor throttle (Arrow Up/Down keys). Direct power control — no PID, matching real BetaFlight behavior for CNN transferability.

Since memory key names are identical to the car version, the web controller,
CNN model (LinearModel), training pipeline, and data recording all work unchanged.

## Prerequisites

- **Python 3.12** — gz-harmonic only supports Python 3.12, 3.13, or 3.14
- **uv** — Python package manager
- **macOS on Apple Silicon** — tested on M1, macOS Tahoe 26.3. Should work on any ARM64 Mac.

Side note: the setup was very intricate on macOS. I would recommend you use a coding agent to work through the various build failures you will likely hit. Gazebo was tricky to get up and running, largely because it is highly capable and requires many dependencies.

## Native macOS Setup (GPU-Accelerated)

Runs BetaFlight SITL and Gazebo Harmonic natively on Apple Silicon (ARM64). GPU rendering (not supported by Docker) was needed for acceptable performance.

### Step 1: Install dependencies (one-time)

Dependencies must come from the **ARM64 Homebrew** at `/opt/homebrew`. Do not
use Rosetta-based Homebrew (`/usr/local/bin/brew`).

```bash
# Gazebo Harmonic (ARM64) — installs gz-sim8, gz-transport13, etc.
brew tap osrf/simulation
brew install gz-harmonic

# Verify Gazebo installed correctly
gz sim --version   # should print: Gazebo Sim, version 8.x.x
```

### Step 2: Build BetaFlight SITL (one-time)

```bash
git clone https://github.com/betaflight/betaflight.git ~/dev/betaflight
cd ~/dev/betaflight
```

BetaFlight's Makefile checks for the ARM cross-compiler before loading the SITL
target config that disables it. Work around this by creating a dummy SDK directory:

```bash
# Hydrate the config submodule
make configs

# Create dummy ARM SDK dir to pass the toolchain check (SITL overrides this to use native clang)
mkdir -p tools/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi/bin

# Build the SITL target
make TARGET=SITL
```

The build compiles with native clang (not ARM cross-compiler) and produces a
native ARM64 executable. Verify:

```bash
file obj/main/betaflight_SITL.elf
# should print: Mach-O 64-bit executable arm64
```

The `-Ofast` deprecation warnings from clang are harmless.

### Step 3: Build the Gazebo-BetaFlight bridge plugin (one-time)

The [aeroloop_gazebo](https://github.com/aeroloop/aeroloop_gazebo) plugin
connects BetaFlight SITL to Gazebo by relaying motor commands and sensor data
over UDP. Follow its build instructions for ARM64 macOS. Set `AEROLOOP_GAZEBO_DIR`
to point to the install location.

### Step 4: Install Python dependencies

```bash
cd ~/dev/DonkeyDrone
uv sync
```

Create a `.env` file in the project root to tell `uv run` where to find the gz-python bindings:

```bash
# .env  (edit paths if your Homebrew prefix differs)
PYTHONPATH=/opt/homebrew/lib/python3.12/site-packages
DYLD_LIBRARY_PATH=/opt/homebrew/lib
GZ_IP=127.0.0.1   # suppresses noisy multicast warning
```

Verify the bindings are importable:

```bash
uv run --env-file .env python -c "import gz.transport13; print('OK')"
```

### Run DonkeyDrone

`scripts/start.sh` launches Gazebo and BetaFlight SITL in the background, waits
for readiness, then runs `donkeydrone/drone_manage.py` in the foreground. Ctrl+C
stops everything cleanly (no orphan processes).

```bash
./scripts/start.sh                              # manual drive mode to collect training data
./scripts/start.sh --model=models/pilot.pth     # launch with autopilot
```

Logs are written to `logs/gazebo.log` and `logs/betaflight.log`.

Then open http://127.0.0.1:8887

**What `scripts/start.sh` does:**
1. Launches Gazebo with the drone course world
2. Waits for the camera topic to appear
3. Launches BetaFlight SITL
4. Runs `donkeydrone/drone_manage.py` which starts the Web UI and sends RC packets
5. On exit or Ctrl+C, calls `scripts/stop_all.sh` to kill all processes cleanly

**Environment variables** for customization:
- `GZ_WORLD` — Gazebo world name (default: `drone_course_65mm`; derived from `--airframe`)
- `BETAFLIGHT_SITL_BIN` — path to BetaFlight SITL binary (default: `~/dev/betaflight/obj/main/betaflight_SITL.elf`)
- `AEROLOOP_GAZEBO_DIR` — path to aeroloop_gazebo install (default: `~/dev/aeroloop_gazebo`)

(Optional) Open the Gazebo GUI in a separate terminal while the sim is running:

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

Two airframe configs are maintained side-by-side. Pick one with `--airframe=65mm|85mm` on `start.sh` (default `65mm`, the BetaFPV Air65). Edit the matching `donkeydrone/drone_config_XXmm.py` to adjust parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DRONE_CAMERA_SOURCE` | `"gz_transport"` | Camera mode: `"gz_transport"` (native macOS) or `"rtsp"` (Docker) |
| `DRONE_GZ_CAMERA_TOPIC` | (drone_course default) | gz-transport topic for camera images |
| `BETAFLIGHT_RC_HOST/PORT` | `127.0.0.1:9004` | BetaFlight SITL RC input endpoint |
| `DRONE_HOVER_THROTTLE` | 1500 | PWM midpoint for hover |
| `DRONE_THROTTLE_RANGE` | 300 | altitude [-1,1] maps to hover ± this PWM range |
| `DRONE_MAX_PITCH_ANGLE` | 25.0 | Max forward pitch angle (degrees) |
| `DRONE_MAX_YAW_RATE` | 90.0 | Max yaw rate (deg/s) at full steering |
| `SIMULATED_DELAY_MS` | 0 | Simulated camera delay in ms (0=off) |
| `MEASURE_LOOP_DELAY` | True | Log vehicle loop timing stats |
| `DRIVE_LOOP_HZ` | 30 | Vehicle loop frequency |

## Architecture

```
macOS Host (Apple Silicon, ARM64)
+----------------------------------------------+
|                                              |
| gz sim (Gazebo Harmonic, GPU-accelerated)    |
|   +-- drone model (camera sensor attached)   |
|   +-- gz-transport: publishes /world/.../image|
|   +-- aeroloop_gazebo: BF bridge plugin      |
|   |     UDP 9002 ← motor speeds from BF      |
|   |     UDP 9003 → FDM state to BF           |
|                                              |
| BetaFlight SITL (native ARM64 binary)        |
|   +-- Angle mode flight controller           |
|   +-- UDP 9004 ← RC channels from us         |
|                                              |
| drone_manage.py                              |
|   +-- DroneGymEnv                            |
|   |     +-- UDP RC packets → BF (port 9004)  |
|   |     +-- gz_camera_worker ← Gazebo images |
|   +-- LocalWebController (Web UI)            |
|   +-- DriveMode (user/autopilot switch)      |
|   +-- LinearModel (PyTorch CNN)              |
|   +-- TubWriter (records to data/)           |
+----------------------------------------------+
```

## Files

| File | Purpose |
|------|---------|
| `scripts/start.sh`        | Single-command launcher: starts Gazebo + BetaFlight SITL, runs drone_manage, cleans up on exit |
| `scripts/stop_all.sh`     | Kills all BetaFlight/Gazebo processes |
| `donkeydrone/drone_manage.py` | Main entry point (replaces `manage.py` for drone use) |
| `donkeydrone/drone_gym.py`    | DroneGymEnv part: BetaFlight RC UDP + gz-transport camera bridge |
| `donkeydrone/drone_config_65mm.py` | BetaFPV Air65 config (default airframe) |
| `donkeydrone/drone_config_85mm.py` | FlyWoo Flylens 85mm config (alternate airframe) |
| `donkeydrone/config.py`       | Base DonkeyCar config (shared, not modified) |
| `donkeydrone/torch_model.py`  | CNN architecture (LinearModel, PyTorch) |
| `donkeydrone/torch_pilot.py`  | Inference wrapper for vehicle loop |
| `donkeydrone/torch_train.py`  | Training script |
| `worlds/drone_course_65mm.sdf` | 65mm Air65 world (colored walls + Air65 model) |
| `worlds/drone_course_85mm.sdf` | 85mm Flylens world (same course, 85mm model) |

## Worlds for autopilot training

All these .sdf files are in the worlds/ folder. They are provided to allow the CNN to train on different scenarios, but feel free to make your own.

- forest_65mm: 
- landing_target_65mm:
- slalom_gates_65mm:
- baylands: from px4

GZ_WORLD=forest_65mm ./scripts/start.sh --airframe=65mm


## Troubleshooting

### BetaFlight SITL build

**`arm-none-eabi-gcc not in the PATH`**: The Makefile checks for the ARM toolchain before
the SITL target overrides it. Create the dummy SDK directory as shown in Step 2.

**Rebuild from clean**: If you need to rebuild:
```bash
cd ~/dev/betaflight
make clean TARGET=SITL
mkdir -p tools/arm-gnu-toolchain-13.3.rel1-darwin-arm64-arm-none-eabi/bin
make TARGET=SITL
```

### Gazebo / gz-transport

**`gz-python` not available via pip**: gz-python is installed by `brew install gz-harmonic`
into Homebrew's site-packages. It is NOT on PyPI. The `.env` file's `PYTHONPATH` makes it
importable.

**Camera topic not found / blank frames**: Run
`PATH=/opt/homebrew/opt/ruby/bin:$PATH gz topic -l | grep camera` while Gazebo
is running to confirm the exact topic name. Update `DRONE_GZ_CAMERA_TOPIC` in
`donkeydrone/drone_config.py` if the model index differs (e.g., `_1` instead of `_0`).

**`gz sim` fails to load plugins / "incompatible architecture"**: The `gz` Ruby
wrapper must run under ARM64 Ruby. Ensure
`PATH=/opt/homebrew/opt/ruby/bin:/opt/homebrew/bin:$PATH` is set before running
any `gz` command. The x86_64 Ruby at `/usr/local/bin/ruby` will fail to dlopen
ARM64 Gazebo libraries.

### General notes

#### Resource utilization

| Change | CPU Savings | Trade-off |
|--------|-------------|-----------|
| `DRIVE_LOOP_HZ = 10` | Less host-side work | Lower control rate |
| Lower camera `update_rate` in world SDF | Halves render load | Fewer training frames per second |

#### Worlds

The default world is **`drone_course`** (`worlds/drone_course.sdf` in this repo).
It provides high-contrast colored surfaces.

To switch worlds, set `GZ_WORLD=<world>` when running `start.sh` **and** update
`DRONE_GZ_CAMERA_TOPIC` in `donkeydrone/drone_config.py` to match.

### CNN Training size

LinearModel is fully size-agnostic. The CNN uses Flatten() after convolutions, so it adapts to any input resolution. The only change needed is setting IMAGE_W and IMAGE_H in drone_config.py

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
| `~/dev/betaflight/obj/` | BetaFlight build artifacts | ~100 MB |


```bash
# Check sizes
du -sh data/ ~/.gz/fuel/ ~/.gz/auto_default.log ~/dev/betaflight/obj/

# Delete old tub recordings (keeps the data/ directory)
rm -rf data/tub_*

# Truncate Gazebo log
> ~/.gz/auto_default.log
```

**Safe to delete anytime:** `data/tub_*` (training data you've already used or don't need),
`~/.gz/fuel/` (re-downloads on next run), `~/.gz/auto_default.log`.

**Don't delete unless rebuilding:** `~/dev/betaflight/obj/` — takes a few minutes to rebuild.


## TODO:

X performance acceleration for training on M1 mac.
X Switch drone flying from 2D to 3D. CNN predicts 3 outputs [steering, throttle, altitude].
X Switch from PX4/MAVSDK to BetaFlight SITL for real-world tiny whoop transferability.
X Build aeroloop_gazebo bridge plugin for ARM64 macOS
X Create Gazebo drone model compatible with BetaFlight bridge (camera sensor attached)
X swap out quadcopter type - in my planned build, can't see the rotors.
X research improvements to CNN. multimodal with IMU and control inputs; cross encoding; GLU
X add/test input controller support.


lower priority:
- try a different world.
- Add randomization of worlds (wall locations, colors) for better CNN training
- Add looping to train model on a variety of worlds
- research other tasks that would be interesting to implement (CNN to scan/build a 3D model of an object, for example)

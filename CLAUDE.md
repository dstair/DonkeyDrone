# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

DonkeyDrone adapts the DonkeyCar pipeline to fly a simulated quadrotor drone using a CNN trained from camera images. Workflow: manually fly → record data → train CNN → fly autonomously. Runs on BetaFlight SITL + Gazebo Harmonic, native Apple Silicon (ARM64).

**Semantic mapping** (identical key names to DonkeyCar, BetaFlight Angle mode):
- `steering [-1, 1]` = yaw rate
- `throttle [-1, 1]` = forward pitch (tilt angle)
- `altitude [-1, 1]` = motor throttle (direct power, no PID)

## Commands

```bash
# Install dependencies
uv sync

# Launch (manual drive)
./scripts/start.sh

# Launch (autopilot)
./scripts/start.sh --model=models/pilot.pth

# Run without start.sh (BetaFlight SITL + Gazebo must already be running)
uv run --env-file .env python -W ignore::SyntaxWarning donkeydrone/drone_manage.py drive --myconfig=drone_config.py

# Bring up the sim stack only (no drone_manage) — useful for tools that talk to
# BetaFlight's RC port directly, like test_thrust.py
./scripts/start.sh --no-manage

# One-command thrust/hover test (starts stack, runs test, tears down)
./scripts/test_thrust.sh

# Train CNN
uv run python donkeydrone/torch_train.py --tubs=data/tub_NN_YY-MM-DD --model=models/pilot.pth

# Multiple tubs (comma-separated)
uv run python donkeydrone/torch_train.py --tubs=data/tub_1_26-03-01,data/tub_2_26-03-01 --model=models/pilot.pth

# Stop everything
bash ./scripts/stop_all.sh

# Force kill
pkill -9 -f betaflight_SITL; pkill -9 -f "gz sim"; pkill -9 -f "ruby.*gz"

# Verify gz-python works
uv run --env-file .env python -c "import gz.transport13; print('OK')"
```

Web UI: http://127.0.0.1:8887

## Python Environment

- **Python 3.12 exactly** (`requires-python = "==3.12.*"`)
- Package manager: `uv` (not pip)
- `.env` file at project root is required (sets PYTHONPATH, DYLD_LIBRARY_PATH, GZ_IP)
- gz-python is NOT on PyPI — installed by `brew install gz-harmonic` into Homebrew site-packages
- No tests, no linting/formatting configuration

## Architecture

```
Web Browser (http://127.0.0.1:8887)
    ↓
LocalWebController (tornado) → user/steering, user/throttle, user/altitude, user/mode
    ↓
DriveMode (selects user vs autopilot)
    ↓
DroneGymEnv (threaded DonkeyCar part)
  ├── update(): background thread with BetaFlight RC loop (50Hz UDP)
  │     → BetaFlight SITL (UDP 9004): RC channel packets (arm, pitch, yaw, throttle)
  │     → Direct throttle control (no PID), Angle mode stabilization by BetaFlight
  ├── gz_camera_worker.py: separate subprocess for camera frames
  │     → subscribes to gz-transport topic
  │     → writes frames to POSIX shared memory (1-byte seq + RGB pixels)
  └── run_threaded(): reads shared memory, applies simulated delay, returns cam/image_array
    ↓ (autopilot mode)
TorchPilot (LinearModel CNN inference)
    ↓
TubWriter (records to data/)
```

### BetaFlight SITL Protocol

| Port | Direction | Purpose |
|------|-----------|---------|
| 9002 | SITL → Gazebo | Motor speeds [0-1] (Gazebo plugin) |
| 9003 | Gazebo → SITL | FDM state: IMU, position, orientation |
| **9004** | **Us → SITL** | **RC channels (16x uint16, 1000-2000 μs PWM)** |
| 5761 | TCP | Configurator (setup only) |

RC packet: `struct.pack('<d', timestamp)` + 16 × `struct.pack('<H', channel)` = 40 bytes

### Key design decisions

- **gz_camera_worker runs as a subprocess** (not thread) to avoid libprotobuf version conflicts between gz-python and TensorFlow/PyTorch
- **Shared memory IPC**: parent creates POSIX SharedMemory, worker writes frames with a sequence counter, parent polls counter in `run_threaded()` for zero-copy reads
- **Direct throttle**: no altitude PID — `altitude [-1,1]` maps to motor power, matching real BetaFlight Angle mode behavior for CNN transferability
- **Config system** (DonkeyCar pattern): `dk.load_config(config_path='config.py', myconfig='drone_config.py')` — edit `drone_config.py`, never `config.py`

## Key Files

| File | Purpose |
|------|---------|
| `donkeydrone/drone_manage.py` | Main entry point |
| `donkeydrone/drone_gym.py` | DroneGymEnv: BetaFlight RC UDP + camera bridge |
| `donkeydrone/drone_config.py` | Drone config overrides (**edit this one**) |
| `donkeydrone/config.py` | Base DonkeyCar config (**do not modify**) |
| `donkeydrone/gz_camera_worker.py` | Subprocess: gz-transport camera → shared memory |
| `donkeydrone/torch_model.py` | CNN architecture (LinearModel, PyTorch) |
| `donkeydrone/torch_pilot.py` | Inference wrapper for vehicle loop |
| `donkeydrone/torch_train.py` | Training script |
| `scripts/start.sh` | One-command launcher (Gazebo + BetaFlight + drone_manage). Accepts `--no-manage` to bring up the sim stack only. |
| `scripts/stop_all.sh` | Force-kill all processes |
| `scripts/test_thrust.sh` | Wrapper: `start.sh --no-manage` + `test_thrust.py` + teardown |
| `donkeydrone/test_thrust.py` | Ramps throttle 1000→2000 PWM and reports altitude at each step; used for tuning `motorConstant` and finding hover PWM |
| `worlds/drone_course.sdf` | Custom Gazebo world with colored walls + drone model |

### External Files (outside this repo)

| File | Purpose |
|------|---------|
| `~/dev/aeroloop_gazebo/` | BetaFlight-Gazebo bridge plugin repo (gz branch) |
| `~/dev/aeroloop_gazebo/plugins/BetaflightPlugin.cc` | Bridge plugin source: UDP 9002/9003 between BetaFlight ↔ Gazebo |
| `~/dev/aeroloop_gazebo/plugins/build/libBetaflightPlugin.dylib` | Compiled plugin loaded by Gazebo at runtime |
| `~/dev/aeroloop_gazebo/models/betaloop_drone_cam/` | Quadrotor model: iris body + 4 rotors + LiftDrag + IMU + forward camera |
| `~/dev/aeroloop_gazebo/models/betaloop_drone_cam/model.sdf` | Model definition (BetaflightPlugin config, rotor mapping, camera sensor) |
| `~/dev/betaflight/` | BetaFlight firmware source (SITL target) |
| `~/dev/betaflight/obj/main/betaflight_SITL.elf` | Compiled BetaFlight SITL binary |
| `~/.gz/sim/8/server.config` | Gazebo default server plugins (Physics, UserCommands, SceneBroadcaster) |

## Camera Modes

Controlled by `DRONE_CAMERA_SOURCE` in `donkeydrone/drone_config.py`:
- `"gz_transport"` (default): native macOS, Gazebo Harmonic, gz-python bindings
- `"rtsp"`: Docker legacy mode, Gazebo Classic, OpenCV VideoCapture

## CNN Model (LinearModel)

- 5× Conv2d (stride 2 or 1, ReLU, Dropout 0.2) → Flatten → Dense(100) → Dense(50) → Linear(3) [steering, throttle, altitude]
- Input: `(B, 3, H, W)` float32 [0,1]. Fully size-agnostic (adapts to IMAGE_W/IMAGE_H)
- Training uses MPS (Apple Silicon GPU) automatically if available, then CUDA, then CPU

## Important Config Parameters (`donkeydrone/drone_config.py`)

- `DRONE_GZ_CAMERA_TOPIC`: must match world name — update when switching worlds
- `GZ_WORLD` env var in `scripts/start.sh`: must also be updated when switching worlds (default: `drone_course`)
- `BETAFLIGHT_RC_HOST`/`BETAFLIGHT_RC_PORT`: BetaFlight SITL RC endpoint (default 127.0.0.1:9004)
- `DRONE_HOVER_THROTTLE`: PWM midpoint for hover (default 1500)
- `DRONE_THROTTLE_RANGE`: altitude [-1,1] maps to ±this around hover (default 300)
- `DRONE_MAX_PITCH_ANGLE`: max pitch degrees for forward tilt (default 25.0)
- `SIMULATED_DELAY_MS`: simulated camera delay in ms (0=off)
- `MEASURE_LOOP_DELAY`: log vehicle loop timing stats
- `IMAGE_W`/`IMAGE_H`: camera resolution for CNN pipeline (default 320×240)
- `DRIVE_LOOP_HZ`: vehicle loop frequency

## Flight Tuning: Throttle, PWM, Hover

### RC channels and PWM

BetaFlight SITL is fed 16-channel RC packets over UDP 9004 at 50Hz. Each channel is a `uint16` PWM value in microseconds, range `1000`–`2000` (1500 = centered stick). Defaults follow the BetaFlight AETR rxmap:

| Channel | Meaning (Angle mode) | How it's driven |
|---------|---------------------|-----------------|
| CH1 | Roll  | held at 1500 (no lateral input in the current mapping) |
| CH2 | Pitch | `1500 + throttle × 500 × DRONE_INPUT_SENSITIVITY` — forward tilt |
| CH3 | **Motor throttle** | bipolar around hover: `clamp(HOVER_THROTTLE + altitude × THROTTLE_RANGE, 1000, 2000)` |
| CH4 | Yaw   | `1500 + steering × 500 × DRONE_INPUT_SENSITIVITY` |
| CH5 (AUX1) | Arm | 2000 armed, 1000 disarmed |
| CH6 (AUX2) | Angle mode | 2000 active |

CH3 is **bipolar**: `altitude=0` → hover PWM (drone holds altitude — sim thrust is deterministic), `altitude=+1` → `HOVER_THROTTLE + THROTTLE_RANGE` (climb), `altitude=-1` → `HOVER_THROTTLE - THROTTLE_RANGE` (descend). Arrow-key UI increments by `DRONE_THROTTLE_STEP_SIZE` per keydown and snaps altitude to 0 on keyup, giving an analog-stick feel. See `drone_gym.py:_map_controls_to_rc`.

### Hover PWM and the thrust-to-weight envelope

The hover PWM depends on: drone mass, `motorConstant` in `model.sdf`, `maxRpm`, and (secondarily) the LiftDrag plugin tuning on each rotor. At hover, total thrust must equal weight: `4 × motorConstant × ω² = m × g`.

For the current 85mm-style model (total mass ≈ 0.125 kg, `maxRpm=2094`):

- `motorConstant = 2.8e-7` → hover at **PWM ≈ 1500** (50% throttle), TWR ≈ 4× — matches a real 85mm FlyWoo Flylens
- `motorConstant = 8.0e-7` → hover at PWM ≈ 1290, TWR ≈ 11× — way too hot; tiny stick movements launch the drone

If the drone launches at a PWM well below 1500, lower `motorConstant`. If it can't lift off at all, raise it. A halving of `motorConstant` roughly shifts the hover PWM up by `~150` (since thrust ∝ ω² and ω scales linearly with PWM).

### Finding hover PWM: `test_thrust.py`

`./scripts/test_thrust.sh` is the fastest way to re-tune after any change to `model.sdf` or drone mass. It:
1. Brings up BetaFlight SITL + Gazebo (via `start.sh --no-manage`)
2. Arms BetaFlight with a proper **disarm → arm** sequence (BF sets a `NOT_DISARMED | ARM_SWITCH` flag on boot if it sees AUX1 HIGH before ever seeing it LOW — any test that skips the disarm phase will sit with motors at 0)
3. Ramps throttle 1000→2000 in 50 PWM steps (0.5s per step) and logs altitude at each
4. Tears everything down

Interpret the output like this: hover is the lowest PWM where altitude climbs above ~0.04m within 0.5s. Rows showing `0.010m` are "still on the ground" (the spawn altitude). A well-tuned drone should show hover near PWM 1500, with the altitude growing smoothly — not exponentially — for the next 2–3 steps.

### Why BetaFlight needs a disarm phase

BetaFlight's SITL boot state includes an "arm switch was high at boot" safety flag (`NOT_DISARMED | ARM_SWITCH` in MSP_STATUS_EX's arming disable flags). The flag clears only after BF observes AUX1 go LOW. `drone_gym.py` already does this in phase 1 of `_betaflight_loop`; any ad-hoc test or tool that sends RC packets to port 9004 must do the same or motors will stay at 0 regardless of the throttle channel.

### `DRONE_HOVER_THROTTLE` vs. `DRONE_THROTTLE_RANGE`

`DRONE_HOVER_THROTTLE` is the PWM that produces hover thrust (altitude=0). `DRONE_THROTTLE_RANGE` is the symmetric deflection around it. The result is clamped to `[1000, 2000]`. With defaults (`HOVER_THROTTLE=1500`, `THROTTLE_RANGE=300`):

- `altitude = -1.0` → PWM 1200 (descend)
- `altitude =  0.0` → PWM 1500 (hover)
- `altitude = +1.0` → PWM 1800 (climb)

The drone takes off on arm because CH3 starts at hover PWM as soon as the arm sequence (which holds CH3=1000 for 2s) completes. To land, hold Down to bring altitude toward -1. If you change the real hover PWM (e.g. by editing `motorConstant`), update `DRONE_HOVER_THROTTLE` so `altitude=0` still corresponds to hover — otherwise the CNN will learn a skewed altitude distribution.

### Planned: vertical-velocity damper (not yet implemented)

Problem: "hover PWM" means thrust = weight → zero *acceleration*. It does not zero existing vertical *velocity*. In sim (minimal air drag) a drone that was climbing and has its stick released keeps coasting upward. Users expect "release = stop in mid-air," which needs an active damper.

Design — proportional altitude-hold in `drone_gym.py`:
1. Subscribe to `/world/drone_course/dynamic_pose/info` (Pose_V) in a gz-transport thread; keep latest position and compute vz by differencing consecutive poses (or subscribe to a velocity topic if one exists). `test_thrust.py` already has the subscribe-once-keep-latest pattern — lift it into `drone_gym.py` as `_PoseTracker`.
2. In `_map_controls_to_rc`, when `abs(altitude) < hold_deadband` (e.g. 0.05), bias CH3 by `-k × vz`. `k` in PWM-per-(m/s) — start with `k = 30` (i.e. a 1 m/s climb gets countered by -30 PWM, roughly -1.4 m/s² in current physics).
3. When the user gives altitude input (stick out of deadband), bypass the damper so the climb command dominates.
4. Add config knobs: `DRONE_ALTITUDE_HOLD_K = 30`, `DRONE_ALTITUDE_HOLD_DEADBAND = 0.05`, `DRONE_ALTITUDE_HOLD_ENABLED = True`.

Why proportional only: we specifically *don't* want integral because it would fight the user's altitude commands. We also don't want target-altitude-based hold (too complex, requires latching a target when stick releases).

Validation: add `--mode=damper` to `test_thrust.py`. Steps: (a) fly to ~2m with CH3=1600 for 2s, (b) cut to CH3=hover, sample altitude for 5s at 0.25s, (c) assert `|vz| < 0.1 m/s` within 1s and altitude doesn't drift more than 0.5m over the following 4s.

Risks: oscillation if `k` is too high. Start low and tune up. The Gazebo pose topic is ~30Hz so the damper loop won't be tight — expect smooth but not snappy settling.

## External Dependencies (not in pyproject.toml)

- BetaFlight SITL binary: `~/dev/betaflight/obj/main/betaflight_SITL.elf` (override with `BETAFLIGHT_SITL_BIN` env var)
  - Source: `~/dev/betaflight/` — build with `make TARGET=SITL` (needs dummy ARM SDK dir, see README)
- aeroloop_gazebo: BetaFlight-Gazebo bridge plugin — set `AEROLOOP_GAZEBO_DIR` env var
  - Source: `~/dev/aeroloop_gazebo/` (gz branch) — build: `cd plugins && mkdir build && cd build && cmake .. -DCMAKE_PREFIX_PATH="/opt/homebrew;/opt/homebrew/opt/qt@5" && make`
  - Plugin: `libBetaflightPlugin.dylib` — loaded via `GZ_SIM_SYSTEM_PLUGIN_PATH` (set by start.sh)
- Gazebo Harmonic: `brew install gz-harmonic` (ARM64 Homebrew only)
- ARM64 Ruby required for gz CLI wrapper: `/opt/homebrew/opt/ruby/bin/ruby`

## Gazebo World & Drone Model

`worlds/drone_course.sdf` includes:
- Colored wall course (red, yellow, blue, orange) with landmark pillars
- `<include>` for `betaloop_drone_cam` model (resolved via `GZ_SIM_RESOURCE_PATH`)
- All 5 required world plugins: `Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` (ogre2), `Imu`

The `betaloop_drone_cam` model (`~/dev/aeroloop_gazebo/models/betaloop_drone_cam/model.sdf`) contains:
- 65mm tiny whoop (0.022kg body, ~0.034kg AUW) with 4 rotors + LiftDrag aerodynamics
- 31mm (1.2") props, 0802-class motors (vel_cmd_max=2094 rad/s ≈ 20k RPM)
- Forward-facing camera (640×480, 30Hz, 80° FOV) on `camera_link`
- IMU sensor on `iris/imu_link` (1000Hz, NED-rotated)
- BetaflightPlugin with rotor-to-joint mapping (BF QUADX motor order)
- Visual: scaled Iris mesh (placeholder — replace with whoop mesh later)
- Camera topic: `/world/drone_course/model/betaloop_drone_cam/link/camera_link/sensor/camera/image`

### BetaflightPlugin Rotor Mapping

**Critical gotcha:** BF SITL's `pwmCompleteMotorUpdate()` shuffles motors before the UDP send to Gazebo:

```
pkt[0] = BF motor 1   (Front-Right, CCW)
pkt[1] = BF motor 2   (Rear-Left,   CCW)
pkt[2] = BF motor 3   (Front-Left,  CW)
pkt[3] = BF motor 0   (Rear-Right,  CW)
```

BetaflightPlugin's `<rotor id="N">` indexes the **packet slot**, not BF's internal motor number. The model's rotor layout and the `<rotor>` blocks in `model.sdf` are intentionally ordered to match the post-remap packet, so the final `rotor id → joint` mapping is 1:1:

| `<rotor id>` | Receives | Position | Direction | Joint |
|--------------|----------|----------|-----------|-------|
| 0 | BF motor 1 | Front-Right | CCW | `rotor_0_joint` |
| 1 | BF motor 2 | Rear-Left   | CCW | `rotor_1_joint` |
| 2 | BF motor 3 | Front-Left  | CW  | `rotor_2_joint` |
| 3 | BF motor 0 | Rear-Right  | CW  | `rotor_3_joint` |

Do **not** re-map `<jointName>` using a "BF motor index → joint" table. That double-applies the remap: the drone inverts thrust at arm (measured: total thrust ≈ −2.3 N, motors wildly asymmetric) and flips upside-down immediately. The source of truth is the comment block at the top of the `<plugin name="...BetaflightPlugin">` element in `model.sdf`.

## Gotchas & Troubleshooting

### Gazebo on macOS requires separate server and GUI processes
`gz sim` cannot run server + GUI in one process on macOS ([gz-sim#44](https://github.com/gazebosim/gz-sim/issues/44)). `start.sh` launches `gz sim -s` (server) then `gz sim -g` (GUI) separately. Set `GZ_HEADLESS=1` to skip the GUI.

### Stale gz-transport topics
After killing Gazebo, camera topics can persist in gz-transport's multicast discovery cache for several minutes. The readiness check in `start.sh` matches the specific `betaloop_drone_cam` topic to avoid false positives from stale topics.

### World-level plugins override server.config defaults
Adding ANY `<plugin>` to the world SDF causes Gazebo to skip loading `~/.gz/sim/8/server.config` plugins (Physics, UserCommands, SceneBroadcaster). This means `drone_course.sdf` must explicitly include ALL five plugins: Physics, UserCommands, SceneBroadcaster, Sensors, and Imu. Without Physics, nothing moves — no gravity, no forces, no joint actuation.

### aeroloop_gazebo CMake needs Qt5 path
The BetaflightPlugin links against gz-sim8 which transitively depends on gz-gui8 (Qt5). On macOS Homebrew, Qt5 is keg-only, so CMake needs: `cmake .. -DCMAKE_PREFIX_PATH="/opt/homebrew;/opt/homebrew/opt/qt@5"`

### BetaflightPlugin FDM bootstrap (deadlock fix)
The upstream BetaflightPlugin only sends FDM state after receiving motor commands, but BetaFlight SITL blocks on `recv(9003)` waiting for FDM before sending motors — a deadlock. Our fix (in `BetaflightPlugin.cc`) moves `SendState()` to run unconditionally every sim tick, before `ReceiveMotorCommand()`, so FDM flows immediately and BetaFlight unblocks. If you re-clone or update aeroloop_gazebo, this fix must be reapplied.

### BetaFlight SITL creates eeprom.bin in cwd
The SITL binary writes `eeprom.bin` (32KB BetaFlight config storage) in whatever directory it's launched from. `start.sh` runs it from the project root, so `eeprom.bin` appears there. It's gitignored. Delete it to reset BetaFlight configuration to defaults.

### Metal rendering crash (gz-sim #2877)
Gazebo camera sensors can crash with the Metal rendering backend. The world uses `ogre2` render engine explicitly. If crashes occur, try: `export GZ_SIM_RENDER_ENGINE=ogre2`

### Shared memory leak warning on shutdown
The `resource_tracker` may warn about leaked shared memory on shutdown — this is cosmetic, caused by a race between `stop_all.sh` killing processes and Python's resource tracker cleanup.

### Switching Gazebo worlds
Two things must be updated in sync:
1. `GZ_WORLD` env var (or default in `start.sh`)
2. `DRONE_GZ_CAMERA_TOPIC` in `donkeydrone/drone_config.py` — the topic path includes the world name

## Current Status (2026-04-16)

Drone model switched from Iris (~510mm, 525g) to 65mm tiny whoop (~34g AUW). Physics, LiftDrag, and BetaflightPlugin parameters all updated in model.sdf. Hover tuned for ~42% throttle. Visual uses scaled Iris mesh as placeholder. Delete eeprom.bin before first launch to reset BetaFlight PIDs for the new lighter airframe. May need PID re-tuning via BetaFlight Configurator if hover is unstable.

Full stack working end-to-end prior to whoop conversion: `start.sh` → Gazebo + BetaFlight SITL + drone_manage.py all launch, RC packets flow at 50Hz, camera frames arrive via shared memory, Web UI at :8887.

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

DonkeyDrone adapts the DonkeyCar pipeline to fly a simulated quadrotor drone using a CNN trained from camera images. Workflow: manually fly → record data → train CNN → fly autonomously. Runs on PX4 SITL + Gazebo Harmonic, native Apple Silicon (ARM64).

**Semantic mapping** (identical key names to DonkeyCar):
- `steering [-1, 1]` = yaw rate
- `throttle [-1, 1]` = forward velocity
- `altitude [-1, 1]` = altitude change rate (PID-stabilized)

## Commands

```bash
# Install dependencies
uv sync

# Launch (manual drive)
./scripts/start.sh

# Launch (autopilot)
./scripts/start.sh --model=models/pilot.pth

# Run without start.sh
uv run --env-file .env python -W ignore::SyntaxWarning donkeydrone/drone_manage.py drive --myconfig=drone_config.py

# Train CNN
uv run python donkeydrone/torch_train.py --tubs=data/tub_NN_YY-MM-DD --model=models/pilot.pth

# Multiple tubs (comma-separated)
uv run python donkeydrone/torch_train.py --tubs=data/tub_1_26-03-01,data/tub_2_26-03-01 --model=models/pilot.pth

# Stop everything
bash ./scripts/stop_all.sh

# Force kill
pkill -9 -f "bin/px4"; pkill -9 -f "gz sim"; pkill -9 -f "ruby.*gz"; pkill -f mavsdk_server

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
  ├── update(): background thread with asyncio MAVSDK loop
  │     → PX4 SITL (UDP 14540): arm, takeoff, offboard velocity cmds
  │     → AltitudePID: tracks dynamic target altitude
  ├── gz_camera_worker.py: separate subprocess for camera frames
  │     → subscribes to gz-transport topic
  │     → writes frames to POSIX shared memory (1-byte seq + RGB pixels)
  └── run_threaded(): reads shared memory, returns cam/image_array
    ↓ (autopilot mode)
TorchPilot (LinearModel CNN inference)
    ↓
TubWriter (records to data/)
```

### Key design decisions

- **gz_camera_worker runs as a subprocess** (not thread) to avoid libprotobuf version conflicts between gz-python and TensorFlow/PyTorch
- **Shared memory IPC**: parent creates POSIX SharedMemory, worker writes frames with a sequence counter, parent polls counter in `run_threaded()` for zero-copy reads
- **Config system** (DonkeyCar pattern): `dk.load_config(config_path='config.py', myconfig='drone_config.py')` — edit `drone_config.py`, never `config.py`

## Key Files

| File | Purpose |
|------|---------|
| `donkeydrone/drone_manage.py` | Main entry point |
| `donkeydrone/drone_gym.py` | DroneGymEnv: MAVSDK + camera bridge |
| `donkeydrone/drone_config.py` | Drone config overrides (**edit this one**) |
| `donkeydrone/config.py` | Base DonkeyCar config (**do not modify**) |
| `donkeydrone/gz_camera_worker.py` | Subprocess: gz-transport camera → shared memory |
| `donkeydrone/torch_model.py` | CNN architecture (LinearModel, PyTorch) |
| `donkeydrone/torch_pilot.py` | Inference wrapper for vehicle loop |
| `donkeydrone/torch_train.py` | Training script |
| `scripts/start.sh` | One-command launcher |
| `scripts/stop_all.sh` | Force-kill all processes |
| `worlds/drone_course.sdf` | Custom Gazebo world with colored walls |

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
- `PX4_GZ_WORLD` in `scripts/start.sh`: must also be updated when switching worlds
- `IMAGE_W`/`IMAGE_H`: camera resolution for CNN pipeline (default 320×240)
- `DRIVE_LOOP_HZ`: vehicle loop frequency

## External Dependencies (not in pyproject.toml)

- PX4 SITL binary: `~/dev/PX4-Autopilot/build/px4_sitl_default/bin/px4`
- Gazebo Harmonic: `brew install gz-harmonic` (ARM64 Homebrew only)
- ARM64 Ruby required for gz CLI wrapper: `/opt/homebrew/opt/ruby/bin/ruby`

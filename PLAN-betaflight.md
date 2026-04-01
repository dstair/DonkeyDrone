# Plan: Replace PX4/MAVSDK with BetaFlight SITL + Add Delay Features

## Context

DonkeyDrone currently uses PX4 SITL + MAVSDK (MAVLink over UDP 14540) for flight control. The goal is to replace PX4 entirely with BetaFlight SITL so the CNN-trained autopilot can fly an inexpensive tiny whoop drone running BetaFlight in Angle mode. Direct throttle control (no altitude PID) ensures the CNN learns real-world-transferable behavior.

Additionally: measure the vehicle loop delay in milliseconds, and add configurable simulated camera delay to test latency tolerance.

## Design Decisions

- **Fully replace PX4** — no dual-backend, remove MAVSDK entirely
- **Direct throttle** — no PID altitude hold; `altitude [-1,1]` maps to motor power, matching real BetaFlight behavior
- **Camera-only delay** — simulated delay buffers camera frames; commands go out instantly
- **Keep DonkeyCar naming** — `steering`=yaw, `throttle`=pitch (forward tilt), `altitude`=motor throttle

## BetaFlight SITL Protocol

| Port | Direction | Purpose |
|------|-----------|---------|
| 9002 | SITL → Gazebo | Motor speeds [0-1] (handled by Gazebo plugin) |
| 9003 | Gazebo → SITL | FDM state: IMU, position, orientation |
| **9004** | **Us → SITL** | **RC channels (16x uint16, 1000-2000 μs PWM)** |
| 5761 | TCP | Configurator (setup only, not runtime) |

**RC packet format**: `struct.pack('<d', timestamp)` + `struct.pack('<H', ch) × 16` = 40 bytes

**Channel mapping** (BetaFlight Angle mode):
- CH1 (roll): always 1500 (centered, no lateral)
- CH2 (pitch): `1500 + throttle_input * 500` (forward tilt)
- CH3 (throttle): `HOVER_THROTTLE + altitude_input * THROTTLE_RANGE` (motor power)
- CH4 (yaw): `1500 + steering_input * 500` (yaw rate)
- CH5 (AUX1): 2000 = armed, 1000 = disarmed
- CH6 (AUX2): 2000 = angle mode active
- CH7-16: 1000 (unused)

## Files to Modify

### 1. `pyproject.toml` — Remove mavsdk dependency
Remove `"mavsdk>=3.15.3"` line. No new dependencies needed (uses stdlib `socket`, `struct`, `collections`).

### 2. `donkeydrone/drone_config.py` — Replace PX4 config with BetaFlight config

**Remove**: `DRONE_MAVSDK_ADDRESS`, `DRONE_TARGET_ALTITUDE`, `DRONE_ALTITUDE_KP/KI/KD`, `DRONE_ALTITUDE_CHANGE_RATE`, `DRONE_ALTITUDE_STEP`, `DRONE_MIN/MAX_ALTITUDE`

**Add**:
```python
# BetaFlight SITL
BETAFLIGHT_RC_HOST = "127.0.0.1"
BETAFLIGHT_RC_PORT = 9004
BETAFLIGHT_ARM_CHANNEL = 4      # AUX1 (0-indexed)
BETAFLIGHT_MODE_CHANNEL = 5     # AUX2

# Flight control mapping (Angle mode)
DRONE_MAX_PITCH_ANGLE = 25.0    # max pitch degrees
DRONE_HOVER_THROTTLE = 1500     # PWM midpoint for hover
DRONE_THROTTLE_RANGE = 300      # altitude [-1,1] → [1200, 1800] PWM

# Simulated camera delay
SIMULATED_DELAY_MS = 0          # 0=off; e.g. 150 for 150ms lag

# Loop timing
MEASURE_LOOP_DELAY = True
LOOP_DELAY_LOG_INTERVAL = 100   # log stats every N iterations
```

Update semantic mapping comment to match BetaFlight Angle mode.

### 3. `donkeydrone/drone_gym.py` — Major rewrite (core change)

**Remove entirely**:
- `AltitudePID` class
- `from mavsdk import System` / `from mavsdk.offboard import ...`
- `_mavsdk_loop()` async method
- `_telemetry_position()` / `_telemetry_attitude()` async methods
- `import asyncio` (no longer needed)

**Add imports**: `socket`, `struct`, `collections`

**Rewrite `__init__`**: Replace mavsdk_address, altitude PID params with:
- `rc_host`, `rc_port`, `arm_channel`, `mode_channel`
- `max_pitch_angle`, `hover_throttle`, `throttle_range`
- `simulated_delay_ms`, `measure_loop_delay`, `loop_delay_log_interval`
- `self._delay_buffer = collections.deque()` for camera delay
- `self._loop_delays = collections.deque(maxlen=N)` for measurement
- `self._rc_sock = None` (UDP socket, created in update thread)

**New `_send_rc(channels)`**: Pack and send 40-byte UDP packet to BetaFlight SITL.

**New `_map_controls_to_rc()`**: Convert steering/throttle/altitude [-1,1] to 16 RC PWM channels [1000-2000].

**New `_betaflight_loop()`** (replaces `_mavsdk_loop`): Plain synchronous loop (no asyncio):
1. Disarm phase: 1s of disarmed packets (throttle low, AUX1 low)
2. Arm phase: 1s of armed packets (throttle low, AUX1 high, AUX2 high for angle mode)
3. Control loop at 50Hz: `_map_controls_to_rc()` → `_send_rc()` → `time.sleep(0.02)`
4. On exit: disarm and close socket

**Rewrite `update()`**: Start camera, then call `self._betaflight_loop()` directly (no asyncio event loop).

**Rewrite `run_threaded(steering, throttle, altitude)`**:
- Loop delay measurement: compute delta from last call, log avg/min/max every N iterations
- Set control inputs (same as before)
- Read gz camera frame (same as before)
- Simulated delay: maintain deque of `(timestamp_ms, frame)` pairs; return frame that is `SIMULATED_DELAY_MS` old
- Build and return outputs

**Telemetry note**: Position/attitude/velocity will output zeros since MAVSDK telemetry is removed. Gazebo pose subscription can be added later by extending `gz_camera_worker.py`. The CNN only uses camera images, so this doesn't affect core functionality.

### 4. `donkeydrone/drone_manage.py` — Update DroneGymEnv wiring

**`add_drone_sim()`**: Replace constructor args:
- Remove: `mavsdk_address`, `target_altitude`, `altitude_change_rate`, `min/max_altitude`, `altitude_pid`
- Add: `rc_host`, `rc_port`, `max_pitch_angle`, `hover_throttle`, `throttle_range`, `arm_channel`, `mode_channel`, `simulated_delay_ms`, `measure_loop_delay`, `loop_delay_log_interval`

**Startup banner**: Replace PX4 info with BetaFlight info (RC host:port, hover throttle, max pitch, delay if set).

No changes needed to: DriveMode, TubWriter, model outputs, web controller wiring (all already support 3D from previous work).

### 5. `scripts/start.sh` — Replace PX4 launch with BetaFlight SITL

**Remove**: All PX4 env vars (`PX4_SYS_AUTOSTART`, `PX4_SIM_MODEL`, `PX4_GZ_WORLD`, etc.), PX4 binary launch, PX4 readiness check.

**New structure**:
1. Set Gazebo env vars (GZ_IP, GZ_SIM_RESOURCE_PATH with project worlds + aeroloop_gazebo paths)
2. Launch Gazebo standalone: `gz sim -r <world>.sdf &`
3. Wait for Gazebo readiness (check for camera topic via `gz topic -l`)
4. Launch BetaFlight SITL: `$BETAFLIGHT_SITL_BIN &` (configurable via env var)
5. Wait for BetaFlight init (~2s)
6. Run `drone_manage.py` in foreground
7. Trap EXIT → `stop_all.sh`

### 6. `scripts/stop_all.sh` — Update process kill list

Replace `pkill -9 -f "bin/px4"` with `pkill -9 -f betaflight_SITL`. Remove `pkill -f mavsdk_server`. Keep gz sim and ruby kills.

### 7. `CLAUDE.md` — Update documentation

- Replace all PX4/MAVSDK references with BetaFlight SITL
- Update architecture diagram (BetaFlight RC UDP 9004 replaces MAVSDK offboard)
- Remove AltitudePID from diagram
- Update semantic mapping: altitude = motor throttle (direct control)
- Update external dependencies: remove PX4-Autopilot, add BetaFlight SITL binary + aeroloop_gazebo
- Document new config parameters
- Update kill commands

## Gazebo World (not part of this code change)

The existing `worlds/drone_course.sdf` uses PX4's `x500_mono_cam` model. For BetaFlight, the SDF needs a drone model compatible with the aeroloop_gazebo BetaFlight bridge plugin, with a camera sensor attached. This is a separate Gazebo/SDF task that depends on having the aeroloop_gazebo plugin compiled for ARM64 macOS.

## Verification

1. **Packet test**: Run a UDP listener on port 9004, launch drone_manage.py, verify 40-byte RC packets arrive at 50Hz with correct channel values
2. **Channel mapping**: With BetaFlight SITL + Gazebo running, verify: disarmed = no motor spin; armed + throttle low = idle; pitch forward = drone tilts forward; yaw = drone rotates
3. **Camera delay**: Set `SIMULATED_DELAY_MS=150`, verify visible lag in web UI video vs simulator movement
4. **Loop delay**: Check console logs for `loop delay: avg=Xms min=Xms max=Xms` messages
5. **Clean shutdown**: `Ctrl+C` triggers disarm sequence → `stop_all.sh` kills all processes
6. **No MAVSDK residue**: `grep -r "mavsdk" donkeydrone/` returns zero hits
7. **Full flight test**: Manual fly via web UI, record tub data, train CNN, test autopilot

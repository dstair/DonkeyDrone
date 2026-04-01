# Plan: Switch Drone from 2D to 3D Flight Control

## Context
Currently the drone flies at a fixed altitude held by a PID controller. The CNN predicts 2 outputs (steering, throttle). This change adds a third control axis — altitude — controlled by Arrow Up/Down keys during manual flight and predicted by the CNN during autopilot. The PID controller remains active for stability, but its target altitude is now dynamic.

## Design
- **New control channel**: `altitude [-1, 1]` where +1 = go up, -1 = go down, 0 = hold current altitude
- **Altitude mapping**: Each frame, `target_altitude += altitude * DRONE_ALTITUDE_CHANGE_RATE / DRIVE_LOOP_HZ`
- **PID stays active**: always tracks `target_altitude` for smooth, stable vertical control
- **Arrow Up/Down**: step-based like IJKL keys (press = increment, hold not needed initially)
- **CNN output**: changes from 2 to 3 values: `[steering, throttle, altitude]`
- **Backward compatibility**: old 2-output models won't load (new architecture); old tub data without altitude can't train 3-output models

## Files to Modify

### 1. `donkeydrone/drone_config.py`
- Add `DRONE_ALTITUDE_CHANGE_RATE = 0.5` (meters/second at altitude=1.0)
- Add `DRONE_ALTITUDE_STEP = 0.1` (step size per arrow key press, in normalized [-1,1] units)
- Update docstring to reflect 3D control

### 2. `donkeydrone/drone_gym.py` — DroneGymEnv
- `__init__`: add `altitude_change_rate` param, store `self.altitude = 0.0`
- `run_threaded(self, steering, throttle, altitude)`: accept third input, update `self.target_altitude += altitude * self.altitude_change_rate * dt`
- `_mavsdk_loop`: uses existing PID with the now-dynamic `self.target_altitude` (no change needed in loop itself)
- Clamp `self.target_altitude` to reasonable bounds (e.g., 1.0 to 20.0 meters)
- Log altitude target changes

### 3. `donkeydrone/drone_manage.py`
- **`add_drone_sim()`**: add `altitude_change_rate` to DroneGymEnv constructor; add `'altitude'` to inputs list
- **`DriveMode.run()`**: accept and return 3 values (steering, throttle, altitude); handle all 3 modes (user, local_angle, local)
- **DriveMode wiring** (line 285): add `user/altitude`, `pilot/altitude` inputs and `altitude` output
- **Model outputs** (line 266): add `'pilot/altitude'`
- **TubWriter inputs** (line 314): add `'user/altitude'` with type `'float'`
- **RECORD_DURING_AI** (line 336): add `'pilot/altitude'`
- **Pipe for user/altitude**: add `V.add(Pipe(), inputs=['user/altitude_raw'], outputs=['user/altitude'])` or similar alias

### 4. `donkeydrone/torch_model.py` — LinearModel
- Change `nn.Linear(50, 2)` → `nn.Linear(50, 3)` on line 49
- Update docstring: Output: `(B, 3) — [steering, throttle, altitude]`

### 5. `donkeydrone/torch_pilot.py` — TorchPilot
- `run()`: extract third output `altitude = float(output[0, 2])`
- Return 3-tuple: `return steering, throttle, altitude`
- Update default return: `return 0.0, 0.0, 0.0`

### 6. `donkeydrone/torch_train.py` — TubDataset
- `__getitem__`: add altitude to label: `torch.tensor([rec['angle'], rec['throttle'], rec['altitude']], dtype=torch.float32)`
- Record loading (line 58-62): extract `rec['user/altitude']` from catalog data, default to 0.0 if missing

### 7. DonkeyCar web controller (forked package in `.venv/`)
These files are in `.venv/lib/python3.12/site-packages/donkeycar/parts/web_controller/`:

**`web.py`** — LocalWebController + WebSocketDriveAPI:
- `LocalWebController.__init__`: add `self.altitude = 0.0`
- `LocalWebController.run_threaded()`: return 6-tuple (add altitude)
- `WebSocketDriveAPI.on_message()`: extract `data.get('altitude', ...)`

**`templates/static/main.js`**:
- Add `'altitude': 0` to `state.tele.user`
- Add Arrow Up (keyCode 38) → `altitudeUp()` and Arrow Down (keyCode 40) → `altitudeDown()` handlers
- Add `altitudeUp()`/`altitudeDown()` functions (similar to `throttleUp()`/`throttleDown()`)
- Add `'altitude'` to `ALL_POST_FIELDS` and `postDrive()` switch
- **Note**: these changes should be committed to the `dstair/donkeycar` fork (`feature/donkeydrone` branch)

### 8. `donkeydrone/drone_manage.py` — `add_user_controller()`
- Update LocalWebController outputs to include `'user/altitude'` (6th output)

## Changes to Web UI Outputs
Current: `['user/steering', 'user/throttle', 'user/mode', 'recording', 'web/buttons']` (5 outputs)
New: `['user/steering', 'user/throttle', 'user/altitude', 'user/mode', 'recording', 'web/buttons']` (6 outputs)

## Verification
1. Start PX4+Gazebo: `./scripts/start.sh`
2. Open http://127.0.0.1:8887
3. Verify Arrow Up/Down change altitude (visible in console logs)
4. Record a tub with altitude changes
5. Check tub catalog contains `user/altitude` values
6. Train: `uv run python donkeydrone/torch_train.py --tubs=data/tub_XX --model=models/pilot3d.pth`
7. Test autopilot: `./scripts/start.sh --model=models/pilot3d.pth` — verify CNN predicts altitude changes

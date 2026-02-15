# Native macOS Install Plan for DonkeyDrone (PX4 + Gazebo on Apple Silicon)

## Current Setup (Docker)
- PX4 SITL + Gazebo Classic running inside Docker (linux/arm64 via Rosetta)
- No GPU access -- rendering is software-only, hence high CPU usage
- Camera frames via RTSP on port 8554
- MAVSDK commands via UDP 14540

## What Changes Going Native

| Component | Docker (now) | Native macOS |
|-----------|-------------|--------------|
| PX4 SITL | Runs in container | Built under Rosetta x86 terminal (no native ARM64 support) |
| Gazebo | Gazebo Classic in container | **Gazebo Harmonic** via Homebrew (GPU-accelerated via Metal/OpenGL) |
| Camera | RTSP stream (`rtsp://127.0.0.1:8554/live`) | **gz-transport topic** (RTSP plugin doesn't exist for new Gazebo) |
| Rendering | Software-only (no GPU in Docker) | Metal or OpenGL on M1 GPU |
| PX4 <-> Gazebo | Internal to container | `gz_bridge` via Gazebo Transport (shared memory) |

## The Plan

### Phase 1: Install Gazebo Harmonic

```bash
brew tap osrf/simulation
brew install gz-harmonic
```

Verify with `gz sim --version`. This gives you GPU-accelerated rendering natively.

### Phase 2: Build PX4 SITL under Rosetta

PX4 does not support native ARM64 SITL builds. You need a Rosetta x86 terminal:

1. Duplicate `Terminal.app`, rename to "x86 Terminal"
2. Get Info -> check "Open using Rosetta"
3. In that terminal:

```bash
brew tap PX4/px4
brew install px4-dev
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/dev/PX4-Autopilot
cd ~/dev/PX4-Autopilot
make px4_sitl gz_x500_mono_cam
```

### Phase 3: Run PX4 + Gazebo in standalone mode

Three terminals:

```bash
# Terminal 1 (native): Gazebo server with your world
gz sim -s -r ~/dev/px4-gazebo-models/worlds/walls.sdf

# Terminal 2 (native): Gazebo GUI (GPU-rendered)
gz sim -g

# Terminal 3 (Rosetta x86): PX4 SITL
cd ~/dev/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_mono_cam \
  ./build/px4_sitl_default/bin/px4
```

### Phase 4: Update `drone_gym.py` camera to use gz-transport

This is the biggest code change. The RTSP camera plugin only existed in Gazebo Classic. New Gazebo publishes camera images on a gz-transport topic like:

```
/world/walls/model/x500_mono_cam_0/link/camera_link/sensor/camera_sensor/image
```

Replace the RTSP-based `_start_camera()` / `_capture_frame()` with a `gz.transport` subscriber that decodes protobuf Image messages into numpy arrays. The `gz-python` package provides Python bindings.

You can discover the exact topic name with:
```bash
gz topic -l | grep camera
```

Python subscription example using gz-transport bindings:
```python
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

def image_cb(msg: Image):
    # msg.width, msg.height, msg.data contain raw pixels
    # Convert to numpy array for OpenCV processing
    pass

node = Node()
node.subscribe(Image, "/world/walls/model/x500_mono_cam_0/.../image", image_cb)
```

### Phase 5: Update config and docs

- Remove RTSP config (`DRONE_RTSP_URL`)
- Add gz-transport topic config (`DRONE_GZ_CAMERA_TOPIC`)
- MAVSDK address stays the same (`udpin://0.0.0.0:14540`)
- Update `drone_config.py` with a new `DRONE_CAMERA_SOURCE` setting (e.g., `"gz_transport"` vs `"rtsp"`)
- Update `README_DRONE.md` with native install instructions

## Known Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| **Metal threading crash with camera sensors** ([gz-sim #2877](https://github.com/gazebosim/gz-sim/issues/2877)) | High | May need to fall back to OpenGL backend, or use a workaround. This directly affects `gz_x500_mono_cam`. |
| **PX4 SITL build fragility on macOS** | Medium | Sequoia 15.x has reported issues. May need to pin specific PX4 versions. |
| **gz-python bindings availability** | Medium | If not available via pip, may need to build from source or use a subprocess bridge (`gz topic -e`) |
| **Homebrew gz-harmonic cmake issues on macOS 15.x** | Medium | May need dependency pinning |

## Recommendation

The GPU benefit is real -- Gazebo Harmonic on native macOS will use your M1's GPU for scene rendering instead of software rasterization in Docker. However, the **camera sensor Metal crash** (gz-sim #2877) is a significant blocker since your project depends on camera frames.

Suggested approach: install Gazebo Harmonic first and verify the camera sensor works in the GUI before investing in the code changes. If the camera crash affects you, the fallback is to use the OpenGL backend (`--render-engine ogre2 --render-engine-gui-api-backend opengl`) until the Metal issue is fixed.

## Key References

- [PX4 macOS Development Environment](https://docs.px4.io/main/en/dev_setup/dev_env_mac)
- [PX4 Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [Gazebo Harmonic macOS Install](https://gazebosim.org/docs/harmonic/install_osx/)
- [gz-python (Python bindings for gz-transport)](https://github.com/srmainwaring/gz-python)
- [Metal camera crash issue (gz-sim #2877)](https://github.com/gazebosim/gz-sim/issues/2877)
- [GStreamer plugin request for new Gazebo (PX4 #22563)](https://github.com/PX4/PX4-Autopilot/issues/22563)

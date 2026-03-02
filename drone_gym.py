"""
DroneGymEnv - DonkeyCar part wrapping PX4 SITL + Gazebo via MAVSDK.

This part connects to a PX4 SITL instance (Docker or native macOS) and provides
the same interface as DonkeyGymEnv: it accepts (steering, throttle) inputs and
produces camera images + telemetry outputs.

The semantic mapping is:
    steering [-1, 1]  ->  yaw rate (deg/s)
    throttle [-1, 1]  ->  forward velocity (m/s)

Altitude is held constant by a simple PID controller.

Camera sources (set DRONE_CAMERA_SOURCE in drone_config.py):
    "gz_transport"  Native macOS: subscribe to Gazebo Harmonic camera topic
                    via gz-transport. Requires gz-python:
                        pip install gz-python
                    Run: gz topic -l | grep camera  to find the exact topic name.
    "rtsp"          Docker mode: read RTSP stream from Gazebo Classic in container.
                    Requires: port 8554 mapped in docker run command.

Requires:
    - PX4 SITL + Gazebo running (Docker or native), UDP 14540 for MAVSDK
    - pip packages: mavsdk, opencv-python-headless
    - gz-transport mode: gz-python (pip install gz-python)
    - rtsp mode: opencv with GStreamer support optional
"""

import asyncio
import logging
import os
import subprocess
import sys
import threading
import time
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path

import cv2
import numpy as np

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

logger = logging.getLogger(__name__)


class AltitudePID:
    """Simple PID controller for altitude hold."""

    def __init__(self, kp=0.5, ki=0.1, kd=0.2, max_output=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, target_alt, current_alt):
        now = time.time()
        error = target_alt - current_alt

        if self._prev_time is None:
            dt = 0.05
        else:
            dt = now - self._prev_time
            dt = max(dt, 0.001)

        self._integral += error * dt
        # Anti-windup clamp
        self._integral = max(-self.max_output, min(self.max_output, self._integral))

        derivative = (error - self._prev_error) / dt

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        output = max(-self.max_output, min(self.max_output, output))

        self._prev_error = error
        self._prev_time = now

        # Return negative because PX4 uses NED (down is positive)
        return -output


_GZ_CAMERA_TOPIC_DEFAULT = (
    "/world/drone_course/model/x500_mono_cam_0"
    "/link/camera_link/sensor/camera/image"
)


class DroneGymEnv:
    """
    DonkeyCar part that interfaces with PX4 SITL + Gazebo via MAVSDK.

    Supports two camera sources (controlled by the camera_source parameter):
        "gz_transport"  - native macOS: gz-transport subscription (Gazebo Harmonic)
        "rtsp"          - Docker mode: OpenCV VideoCapture on RTSP stream

    This is a threaded part: update() runs the async MAVSDK event loop in a
    background thread, while run_threaded() is called by the Vehicle loop to
    exchange steering/throttle commands and camera images.

    Usage in manage.py:
        gym = DroneGymEnv(cfg)
        V.add(gym, inputs=['steering', 'throttle'],
              outputs=['cam/image_array', ...], threaded=True)
    """

    def __init__(self, mavsdk_address="udpin://0.0.0.0:14540",
                 camera_source="gz_transport",
                 gz_camera_topic=_GZ_CAMERA_TOPIC_DEFAULT,
                 rtsp_url="rtsp://127.0.0.1:8554/live",
                 max_forward_vel=2.0, max_yaw_rate=90.0,
                 target_altitude=10.0,
                 image_w=160, image_h=120,
                 altitude_pid=(0.5, 0.1, 0.2),
                 record_position=False,
                 record_attitude=False,
                 record_velocity=False):

        self.mavsdk_address = mavsdk_address
        self.camera_source = camera_source
        self.gz_camera_topic = gz_camera_topic
        self.rtsp_url = rtsp_url
        self.max_forward_vel = max_forward_vel
        self.max_yaw_rate = max_yaw_rate
        self.target_altitude = target_altitude
        self.image_w = image_w
        self.image_h = image_h
        self.record_position = record_position
        self.record_attitude = record_attitude
        self.record_velocity = record_velocity

        self.alt_pid = AltitudePID(kp=altitude_pid[0],
                                   ki=altitude_pid[1],
                                   kd=altitude_pid[2])

        # Shared state between threads
        self.steering = 0.0
        self.throttle = 0.0
        self.frame = np.zeros((image_h, image_w, 3), dtype=np.uint8)
        self.position = (0.0, 0.0, 0.0)
        self.attitude = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.current_altitude = 0.0

        self.running = True
        self._connected = False
        self._offboard_started = False
        self._cap = None            # used by RTSP mode
        self._shm = None            # shared memory for gz-transport subprocess
        self._camera_proc = None    # gz_camera_worker subprocess
        self._last_seq = 0          # sequence counter for new-frame detection
        self._frame_size = image_h * image_w * 3
        self._loop = None
        self._frame_skip = 0

    def _start_camera(self):
        """Open RTSP stream from Gazebo (Docker mode)."""
        logger.info("Opening RTSP camera stream: %s", self.rtsp_url)

        # Try direct RTSP first
        self._cap = cv2.VideoCapture(self.rtsp_url)
        if self._cap.isOpened():
            # Minimize frame buffer to reduce decode overhead
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            logger.info("RTSP stream opened successfully")
            return

        # Fallback: try GStreamer pipeline
        gst = (
            f"rtspsrc location={self.rtsp_url} latency=0 "
            "! rtph264depay ! h264parse ! avdec_h264 "
            "! videoconvert ! appsink drop=true max-buffers=1 sync=false"
        )
        logger.info("Trying GStreamer pipeline: %s", gst)
        self._cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if self._cap.isOpened():
            logger.info("GStreamer pipeline opened successfully")
            return

        logger.warning("Could not open RTSP stream. Images will be blank.")

    def _capture_frame(self):
        """Capture a single frame from the RTSP stream and resize (Docker mode)."""
        if self._cap is None or not self._cap.isOpened():
            return

        ret, raw_frame = self._cap.read()
        if ret and raw_frame is not None:
            # Convert BGR -> RGB and resize to DonkeyCar dimensions
            rgb = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
            resized = cv2.resize(rgb, (self.image_w, self.image_h))
            self.frame = resized

    def _start_gz_camera(self):
        """Launch gz_camera_worker subprocess for gz-transport camera capture.

        The worker runs in a separate process to avoid libprotobuf conflicts
        with TensorFlow. Frames are passed via shared memory.

        Find your exact topic name with:
            gz topic -l | grep camera
        """
        logger.info("Starting gz-transport camera subprocess for topic: %s",
                    self.gz_camera_topic)

        # Create shared memory: 1 byte seq counter + frame data
        shm_size = 1 + self._frame_size
        self._shm = SharedMemory(create=True, size=shm_size)
        self._shm.buf[0] = 0  # initial sequence counter
        logger.info("Created shared memory '%s' (%d bytes)",
                    self._shm.name, shm_size)

        # Locate the worker script next to this file
        worker_path = str(Path(__file__).parent / "gz_camera_worker.py")

        self._camera_proc = subprocess.Popen(
            ["uv", "run", "--env-file", ".env",
             "python", worker_path,
             self.gz_camera_topic,
             str(self.image_w), str(self.image_h),
             self._shm.name],
            env=os.environ.copy(),
        )
        logger.info("Camera worker started (PID %d)", self._camera_proc.pid)

    def _read_gz_frame(self):
        """Read the latest frame from shared memory if a new one is available."""
        if self._shm is None:
            return
        seq = self._shm.buf[0]
        if seq != self._last_seq:
            self._last_seq = seq
            frame_bytes = bytes(self._shm.buf[1:1 + self._frame_size])
            self.frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape(
                self.image_h, self.image_w, 3)

    async def _mavsdk_loop(self):
        """Main async loop: connect, arm, takeoff, then send offboard commands."""
        drone = System()
        logger.info("Connecting to PX4 SITL at %s...", self.mavsdk_address)
        await drone.connect(system_address=self.mavsdk_address)

        # Wait for connection
        async for state in drone.core.connection_state():
            if state.is_connected:
                logger.info("Connected to PX4 SITL")
                self._connected = True
                break

        # Wait for GPS lock
        logger.info("Waiting for global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                logger.info("Global position OK")
                break

        # Arm and takeoff
        logger.info("Arming...")
        await drone.action.arm()
        logger.info("Taking off to %.1fm...", self.target_altitude)
        await drone.action.set_takeoff_altitude(self.target_altitude)
        await drone.action.takeoff()

        # Wait to reach approximate target altitude
        logger.info("Waiting to reach target altitude...")
        while self.running:
            async for position in drone.telemetry.position():
                self.current_altitude = position.relative_altitude_m
                if position.relative_altitude_m >= self.target_altitude * 0.9:
                    logger.info("Reached %.1fm altitude", position.relative_altitude_m)
                    break
                break
            if self.current_altitude >= self.target_altitude * 0.9:
                break
            await asyncio.sleep(0.5)

        # Start telemetry subscription tasks
        asyncio.ensure_future(self._telemetry_position(drone))
        asyncio.ensure_future(self._telemetry_attitude(drone))

        # PX4 requires offboard setpoints streaming at >2 Hz for ≥0.5 s
        # before it accepts an OFFBOARD mode switch.  Send a warm-up burst.
        logger.info("Sending offboard setpoint warm-up...")
        for _ in range(20):                    # 1 s at 20 Hz
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.05)

        try:
            await drone.offboard.start()
            self._offboard_started = True
            logger.info("Offboard mode started — drone is now accepting commands")
        except OffboardError as e:
            logger.error("Failed to start offboard mode: %s", e)
            return

        # Main control loop: send velocity commands based on steering/throttle
        loop_count = 0
        while self.running:
            forward_vel = self.throttle * self.max_forward_vel
            yaw_rate = self.steering * self.max_yaw_rate
            down_vel = self.alt_pid.compute(self.target_altitude,
                                            self.current_altitude)

            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward_vel, 0.0, down_vel, yaw_rate))

            # Log control values every ~2 s so operator can verify inputs
            loop_count += 1
            if loop_count % 40 == 0:
                logger.info(
                    "ctrl: steer=%.2f thr=%.2f → fwd=%.1f m/s yaw=%.1f°/s "
                    "alt=%.1f/%.1fm down_vel=%.2f",
                    self.steering, self.throttle,
                    forward_vel, yaw_rate,
                    self.current_altitude, self.target_altitude, down_vel)

            # RTSP mode: poll for frames every other iteration to reduce CPU.
            # gz-transport mode: frames arrive via push callback; no polling needed.
            if self.camera_source == 'rtsp':
                self._frame_skip += 1
                if self._frame_skip % 2 == 0:
                    self._capture_frame()

            await asyncio.sleep(0.05)  # ~20 Hz control loop

        # Cleanup: stop offboard and land
        try:
            await drone.offboard.stop()
        except OffboardError:
            pass
        await drone.action.land()

    async def _telemetry_position(self, drone):
        """Subscribe to position telemetry."""
        async for pos in drone.telemetry.position():
            if not self.running:
                break
            self.current_altitude = pos.relative_altitude_m
            self.position = (pos.latitude_deg, pos.longitude_deg,
                             pos.relative_altitude_m)

    async def _telemetry_attitude(self, drone):
        """Subscribe to attitude (Euler angles) telemetry."""
        async for att in drone.telemetry.attitude_euler():
            if not self.running:
                break
            self.attitude = (att.roll_deg, att.pitch_deg, att.yaw_deg)

    def update(self):
        """
        Background thread entry point.
        Runs the asyncio event loop for MAVSDK communication.
        """
        if self.camera_source == 'rtsp':
            self._start_camera()
        else:
            self._start_gz_camera()

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._mavsdk_loop())
        except Exception as e:
            logger.error("MAVSDK loop error: %s", e)
        finally:
            self._loop.close()

    def run_threaded(self, steering, throttle):
        """
        Called by the DonkeyCar Vehicle loop each frame.

        :param steering: normalized steering [-1, 1], mapped to yaw rate
        :param throttle: normalized throttle [-1, 1], mapped to forward velocity
        :return: camera image array + optional telemetry values
        """
        if steering is None:
            steering = 0.0
        if throttle is None:
            throttle = 0.0

        self.steering = float(steering)
        self.throttle = float(throttle)

        if self.camera_source == 'gz_transport':
            self._read_gz_frame()

        outputs = [self.frame]

        if self.record_position:
            outputs += [self.position[0], self.position[1], self.position[2]]
        if self.record_attitude:
            outputs += [self.attitude[0], self.attitude[1], self.attitude[2]]
        if self.record_velocity:
            outputs += [self.velocity[0], self.velocity[1], self.velocity[2]]

        if len(outputs) == 1:
            return self.frame
        return outputs

    def shutdown(self):
        """Stop the background thread and clean up."""
        logger.info("Shutting down DroneGymEnv...")
        self.running = False
        time.sleep(1.0)
        if self._cap is not None:
            self._cap.release()
        # Terminate gz-transport camera subprocess
        if self._camera_proc is not None:
            logger.info("Terminating camera worker (PID %d)...",
                        self._camera_proc.pid)
            self._camera_proc.terminate()
            try:
                self._camera_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._camera_proc.kill()
            self._camera_proc = None
        if self._shm is not None:
            self._shm.close()
            self._shm.unlink()
            self._shm = None

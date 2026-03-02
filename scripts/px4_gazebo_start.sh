#!/bin/bash

export HEADLESS=1                    # skip Gazebo GUI (add a separate `gz sim -g` if you want it)
export PX4_SYS_AUTOSTART=4001       # x500 quadrotor airframe
export PX4_SIM_MODEL=gz_x500_mono_cam  # camera-equipped model (overrides autostart default of x500)
export PX4_GZ_WORLD=drone_course    # colorful course for CNN training (from DonkeyDrone/worlds/)
# All paths must be set explicitly — gz_env.sh is not sourced reliably at runtime
export PX4_GZ_WORLDS=~/dev/PX4-Autopilot/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=~/dev/PX4-Autopilot/Tools/simulation/gz/models
export PX4_GZ_PLUGINS=~/dev/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins
# Include DonkeyDrone's worlds/ dir so Gazebo can find drone_course.sdf
export DONKEYDRONE_DIR=~/dev/DonkeyDrone
export GZ_SIM_RESOURCE_PATH=$PX4_GZ_MODELS:$PX4_GZ_WORLDS:$DONKEYDRONE_DIR/worlds
export GZ_IP=127.0.0.1              # suppress multicast "No route to host" warnings on macOS
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=~/dev/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config
cd ~/dev/PX4-Autopilot/build/px4_sitl_default
./bin/px4 -s etc/init.d-posix/rcS
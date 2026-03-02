#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── PX4 SITL + Gazebo environment ──────────────────────────────────
export PATH="/opt/homebrew/opt/ruby/bin:/opt/homebrew/bin:$PATH"
export HEADLESS=1
export PX4_SYS_AUTOSTART=4001
export PX4_SIM_MODEL=gz_x500_mono_cam
export PX4_GZ_WORLD=drone_course
export PX4_GZ_WORLDS=~/dev/PX4-Autopilot/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=~/dev/PX4-Autopilot/Tools/simulation/gz/models
export PX4_GZ_PLUGINS=~/dev/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins
export GZ_SIM_RESOURCE_PATH="$PX4_GZ_MODELS:$PX4_GZ_WORLDS:$PROJECT_DIR/worlds"
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_GZ_PLUGINS"
export GZ_SIM_SERVER_CONFIG_PATH=~/dev/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config
export PX4_PARAM_SDLOG_MODE=-1   # disable ULog flight data recording (saves GB of disk)

# ── Cleanup on exit ────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Stopping all processes..."
    "$SCRIPT_DIR/stop_all.sh"
    echo "Done."
}
trap cleanup EXIT

# ── Launch PX4 SITL in background ──────────────────────────────────
PX4_DIR=~/dev/PX4-Autopilot/build/px4_sitl_default
LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$LOG_DIR"
PX4_LOG="$LOG_DIR/px4_sitl.log"

echo "Starting PX4 SITL + Gazebo (log: $PX4_LOG)..."
> "$PX4_LOG"
cd "$PX4_DIR"
./bin/px4 -s etc/init.d-posix/rcS > "$PX4_LOG" 2>&1 &
PX4_PID=$!

# ── Wait for PX4 + Gazebo readiness ──────────────────────────────
TIMEOUT=60
echo "Waiting for PX4 SITL ready (timeout ${TIMEOUT}s)..."
elapsed=0
while ! grep -q "remote port 14540" "$PX4_LOG" 2>/dev/null; do
    if ! kill -0 "$PX4_PID" 2>/dev/null; then
        echo "ERROR: PX4 process died. Check $PX4_LOG"
        exit 1
    fi
    if [ "$elapsed" -ge "$TIMEOUT" ]; then
        echo "ERROR: Timed out waiting for PX4 readiness. Check $PX4_LOG"
        exit 1
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done
echo "PX4 ready (after ${elapsed}s)."

# ── Run drone_manage.py in foreground ──────────────────────────────
cd "$PROJECT_DIR"
echo "Starting drone_manage.py..."
uv run --env-file .env python -W ignore::SyntaxWarning \
    donkeydrone/drone_manage.py drive --myconfig=drone_config.py "$@"

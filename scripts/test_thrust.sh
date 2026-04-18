#!/bin/bash
# One-command wrapper: bring up Gazebo + BetaFlight SITL (no drone_manage),
# run donkeydrone/test_thrust.py, then tear down.
#
# Usage:
#     ./scripts/test_thrust.sh
#
# Any extra args are forwarded to test_thrust.py.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$LOG_DIR"

STACK_LOG="$LOG_DIR/start_no_manage.log"
> "$STACK_LOG"

echo "Bringing up sim stack (Gazebo + BetaFlight SITL)..."
# Run start.sh --no-manage headless in the background so this script keeps
# control. Its EXIT/INT trap will clean up when we kill it.
GZ_HEADLESS=1 "$SCRIPT_DIR/start.sh" --no-manage > "$STACK_LOG" 2>&1 &
STACK_PID=$!

cleanup() {
    echo ""
    echo "Stopping sim stack..."
    # Kill start.sh; its EXIT trap runs stop_all.sh, but call it ourselves too
    # as a belt-and-suspenders in case start.sh died before setting its trap.
    kill -TERM "$STACK_PID" 2>/dev/null || true
    bash "$SCRIPT_DIR/stop_all.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# Wait for the STACK_READY sentinel start.sh prints when setup is complete.
TIMEOUT=90
echo "Waiting for stack readiness (timeout ${TIMEOUT}s)..."
elapsed=0
while true; do
    if ! kill -0 "$STACK_PID" 2>/dev/null; then
        echo "ERROR: start.sh exited before ready. Log tail:"
        tail -40 "$STACK_LOG"
        exit 1
    fi
    if grep -q STACK_READY "$STACK_LOG" 2>/dev/null; then
        echo "Stack ready after ${elapsed}s."
        break
    fi
    if [ "$elapsed" -ge "$TIMEOUT" ]; then
        echo "ERROR: Timed out waiting for stack. Log tail:"
        tail -40 "$STACK_LOG"
        exit 1
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done

echo "Running test_thrust.py..."
cd "$PROJECT_DIR"
uv run --env-file .env python donkeydrone/test_thrust.py "$@"

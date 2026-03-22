#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Gazebo environment ────────────────────────────────────────────
export PATH="/opt/homebrew/opt/ruby/bin:/opt/homebrew/bin:$PATH"
export GZ_IP=127.0.0.1

# World to load (override with GZ_WORLD env var)
GZ_WORLD="${GZ_WORLD:-drone_course}"

# Resource paths: project worlds + aeroloop_gazebo models (if present)
AEROLOOP_DIR="${AEROLOOP_GAZEBO_DIR:-$HOME/dev/aeroloop_gazebo}"
export GZ_SIM_RESOURCE_PATH="$PROJECT_DIR/worlds"
if [ -d "$AEROLOOP_DIR/models" ]; then
    export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$AEROLOOP_DIR/models"
fi
if [ -d "$AEROLOOP_DIR/worlds" ]; then
    export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$AEROLOOP_DIR/worlds"
fi

# BetaflightPlugin shared library path (built from aeroloop_gazebo)
if [ -d "$AEROLOOP_DIR/plugins/build" ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$AEROLOOP_DIR/plugins/build"
fi

# BetaFlight SITL binary (override with BETAFLIGHT_SITL_BIN env var)
BETAFLIGHT_BIN="${BETAFLIGHT_SITL_BIN:-$HOME/dev/betaflight/obj/main/betaflight_SITL.elf}"

# ── Cleanup on exit ──────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Stopping all processes..."
    "$SCRIPT_DIR/stop_all.sh"
    echo "Done."
}
trap cleanup EXIT

LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$LOG_DIR"

# ── Launch BetaFlight SITL (before Gazebo to avoid FDM-during-init crash) ──
BF_LOG="$LOG_DIR/betaflight.log"
# Remove stale eeprom.bin — BetaFlight SITL's save corrupts it on macOS ARM64.
# Modes are configured at runtime via MSP after launch.
rm -f "$PROJECT_DIR/eeprom.bin"
echo "Starting BetaFlight SITL: $BETAFLIGHT_BIN (log: $BF_LOG)..."
> "$BF_LOG"
"$BETAFLIGHT_BIN" > "$BF_LOG" 2>&1 &
BF_PID=$!

# Give BetaFlight time to initialize
sleep 2
if ! kill -0 "$BF_PID" 2>/dev/null; then
    echo "ERROR: BetaFlight SITL died. Check $BF_LOG"
    exit 1
fi
echo "BetaFlight SITL running (PID $BF_PID)."

# ── Configure BetaFlight via MSP ─────────────────────────────────
# Sets ARM on AUX1, ANGLE on AUX2, changes failsafe to AUTO_LANDING if
# needed. Queries arming status for diagnostics.
python3 -c "
import socket, time, struct, sys

def msp_send(sock, cmd, payload=b''):
    size = len(payload)
    checksum = size ^ cmd
    for b in payload:
        checksum ^= b
    frame = b'\$M<' + bytes([size, cmd]) + payload + bytes([checksum])
    sock.send(frame)
    time.sleep(0.1)
    # Read response — may need multiple recv() calls
    resp = b''
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            resp += chunk
            # Check if we have a complete MSP frame
            if len(resp) >= 4 and resp[:2] == b'\$M':
                if len(resp) >= 5:
                    pkt_size = resp[3]
                    expected = 6 + pkt_size  # header(3) + size(1) + cmd(1) + payload + checksum(1)
                    if len(resp) >= expected:
                        break
    except socket.timeout:
        pass
    if len(resp) >= 5 and resp[:3] == b'\$M>':
        return resp[5:5+resp[3]]
    if len(resp) >= 5 and resp[:3] == b'\$M!':
        print('  MSP cmd %d: ERROR response' % cmd)
        return b''
    if resp:
        print('  MSP cmd %d: unexpected response (%d bytes): %s' % (cmd, len(resp), resp[:20].hex()))
    return b''

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 5761))
s.settimeout(1)
time.sleep(0.5)

# Drain any initial banner/data
try:
    s.recv(4096)
except socket.timeout:
    pass

# MSP_SET_MODE_RANGE (35): ARM on AUX1, ANGLE on AUX2
msp_send(s, 35, struct.pack('BBBBB', 0, 0, 0, 32, 48))  # ARM on AUX1
msp_send(s, 35, struct.pack('BBBBB', 1, 1, 1, 32, 48))  # ANGLE on AUX2
print('  Modes set: ARM on AUX1, ANGLE on AUX2')

# Change failsafe from GPS_RESCUE(2) to AUTO_LANDING(0) if needed
fs = msp_send(s, 75)  # MSP_FAILSAFE_CONFIG
if len(fs) >= 8:
    procedure = fs[7]
    if procedure == 2:  # GPS_RESCUE
        msp_send(s, 76, fs[:7] + bytes([0]))  # procedure=AUTO_LANDING
        print('  Failsafe: GPS_RESCUE -> AUTO_LANDING')
    else:
        print('  Failsafe: procedure=%d (OK)' % procedure)

# Query arming status
time.sleep(0.3)  # Let BetaFlight process config changes
status = msp_send(s, 150)  # MSP_STATUS_EX
if len(status) >= 20:
    # Parse arming disable flags (variable offset due to flight mode flags)
    flags_byte_count_raw = status[15]
    flags_byte_count = flags_byte_count_raw & 0x0F
    offset = 16 + flags_byte_count
    if len(status) >= offset + 5:
        arming_count = status[offset]
        arming_flags = struct.unpack('<I', status[offset+1:offset+5])[0]
        flag_names = ['NO_GYRO','FAILSAFE','RX_FAILSAFE','NOT_DISARMED',
            'BOXFAILSAFE','RUNAWAY','CRASH','THROTTLE','ANGLE','BOOT_GRACE',
            'NOPREARM','LOAD','CALIBRATING','CLI','CMS_MENU','BST','MSP',
            'PARALYZE','GPS','RESC','DSHOT_TELEM','REBOOT_REQ','DSHOT_BB',
            'ACC_CAL','MOTOR_PROTO','CRASHFLIP','ALTHOLD','POSHOLD','ARM_SWITCH']
        active = [flag_names[i] for i in range(min(len(flag_names), 29)) if arming_flags & (1 << i)]
        print('  Arming flags: 0x%08x %s' % (arming_flags, active if active else '(none)'))
    else:
        print('  Status: payload too short for arming flags (got %d bytes)' % len(status))
else:
    print('  Status: MSP_STATUS_EX returned %d bytes' % len(status))

print('BetaFlight MSP configuration complete.')
s.close()
" 2>&1

# ── Launch Gazebo standalone ─────────────────────────────────────
GZ_LOG="$LOG_DIR/gazebo.log"

echo "Starting Gazebo with world: $GZ_WORLD (log: $GZ_LOG)..."
> "$GZ_LOG"
# macOS requires separate server (-s) and GUI (-g) processes for gz sim
gz sim -s -r "${GZ_WORLD}.sdf" >> "$GZ_LOG" 2>&1 &
GZ_SERVER_PID=$!
GZ_PID=$GZ_SERVER_PID

# Wait for Gazebo readiness — use world service response (immune to stale topics)
TIMEOUT=60
echo "Waiting for Gazebo readiness (timeout ${TIMEOUT}s)..."
elapsed=0
while true; do
    if ! kill -0 "$GZ_PID" 2>/dev/null; then
        echo "ERROR: Gazebo process died. Check $GZ_LOG"
        exit 1
    fi
    if [ "$elapsed" -ge "$TIMEOUT" ]; then
        echo "ERROR: Timed out waiting for Gazebo. Check $GZ_LOG"
        exit 1
    fi
    # Check if Gazebo world service is responding (immune to stale topics).
    # gz service returns exit 0 even on timeout, so check output for "timed out".
    SVC_OUT=$(gz service -s /world/${GZ_WORLD}/gui/info --reqtype gz.msgs.Empty \
         --reptype gz.msgs.GUI --timeout 500 --req "" 2>&1 || true)
    if ! echo "$SVC_OUT" | grep -qi "timed out"; then
        # Also verify camera topic exists
        if gz topic -l 2>/dev/null | grep -q "betaloop_drone_cam.*camera/image"; then
            break
        fi
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done
echo "Gazebo server ready (after ${elapsed}s)."

# Launch Gazebo GUI (optional — skip with GZ_HEADLESS=1)
if [ "${GZ_HEADLESS:-0}" != "1" ]; then
    GZ_GUI_LOG="$LOG_DIR/gazebo_gui.log"
    gz sim -g >> "$GZ_GUI_LOG" 2>&1 &
    echo "Gazebo GUI launched."
fi

# ── Run drone_manage.py in foreground ────────────────────────────
cd "$PROJECT_DIR"
echo "Starting drone_manage.py..."
uv run --env-file .env python -W ignore::SyntaxWarning \
    donkeydrone/drone_manage.py drive --myconfig=drone_config.py "$@"

#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Arg parsing ───────────────────────────────────────────────────
# --no-manage: bring up Gazebo + BetaFlight SITL but skip drone_manage.py.
# Useful for test_thrust.py or any tool that needs the sim running without
# drone_manage fighting over RC UDP port 9004.
SKIP_MANAGE=0
MANAGE_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--no-manage" ]; then
        SKIP_MANAGE=1
    else
        MANAGE_ARGS+=("$arg")
    fi
done

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
    # Kill all background children of this script
    jobs -p | xargs -r kill -9 2>/dev/null
    # Fall back to pkill for any stragglers
    "$SCRIPT_DIR/stop_all.sh"
    echo "Done."
}
trap cleanup EXIT INT TERM

LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$LOG_DIR"

# ── Launch BetaFlight SITL (before Gazebo to avoid FDM-during-init crash) ──
BF_LOG="$LOG_DIR/betaflight.log"

# BetaFlight SITL runs best with existing eeprom.bin (deletion causes SIGTRAP on first run)

echo "Starting BetaFlight SITL: $BETAFLIGHT_BIN (log: $BF_LOG)..."

# BetaFlight SITL on macOS ARM64 sometimes crashes on first launch (SIGTRAP), but works on retry
for attempt in 1 2; do
    > "$BF_LOG"
    # Removed gstdbuf to avoid DYLD_INSERT_LIBRARIES SIGTRAP on Apple Silicon
    "$BETAFLIGHT_BIN" > "$BF_LOG" 2>&1 &
    BF_PID=$!

    # Give BetaFlight time to initialize
    sleep 2
    if kill -0 "$BF_PID" 2>/dev/null; then
        echo "BetaFlight SITL running (PID $BF_PID)."
        break
    else
        if [ $attempt -eq 1 ]; then
            echo "BetaFlight crashed on attempt 1, retrying..."
            sleep 1
        else
            echo "ERROR: BetaFlight SITL died on both attempts. Check $BF_LOG"
            exit 1
        fi
    fi
done

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

def connect_msp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for _ in range(30):
        try:
            sock.connect(('127.0.0.1', 5761))
            break
        except (ConnectionRefusedError, OSError):
            time.sleep(0.3)
    else:
        raise RuntimeError('Could not reconnect to BetaFlight MSP after reboot')
    sock.settimeout(1)
    time.sleep(0.5)
    try:
        sock.recv(4096)
    except socket.timeout:
        pass
    return sock

s = connect_msp()

# Disable AIRMODE feature if enabled. AIRMODE re-scales the motor mix so
# yaw/attitude commands retain authority at low throttle — which on our
# 125g airframe causes the drone to climb on yaw input even at CH3=1000.
# Feature changes require eeprom save + reboot to take effect.
FEATURE_AIRMODE = 1 << 22
feat = msp_send(s, 36)  # MSP_FEATURE_CONFIG
if len(feat) >= 4:
    current = struct.unpack('<I', feat[:4])[0]
    if current & FEATURE_AIRMODE:
        new_feat = current & ~FEATURE_AIRMODE
        msp_send(s, 37, struct.pack('<I', new_feat))  # MSP_SET_FEATURE_CONFIG
        msp_send(s, 250)  # MSP_EEPROM_WRITE
        print('  Features: 0x%08x -> 0x%08x (AIRMODE disabled), saved to eeprom' % (current, new_feat))
        # MSP_REBOOT (68) — BF restarts internally, socket dies
        try:
            frame = b'\$M<' + bytes([0, 68, 68])
            s.send(frame)
        except OSError:
            pass
        s.close()
        time.sleep(1.5)
        s = connect_msp()
        print('  Reconnected after reboot')
    else:
        print('  Features: 0x%08x (AIRMODE already disabled)' % current)
else:
    print('  Features: MSP_FEATURE_CONFIG returned %d bytes — skipping airmode check' % len(feat))

# Set mixer_type = LINEAR via CLI. LEGACY (the default) clips motors at PWM
# limits when yaw PID saturates — two motors slam to 2000, two to 1000, and
# total thrust doubles → drone rockets up on any yaw input. LINEAR scales
# throttle down instead of clipping, preserving motor headroom.
def cli_drain(sock, timeout=0.5, match=None):
    # Read from sock until timeout; stop early if match bytes appear.
    sock.settimeout(timeout)
    buf = b''
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            if match is not None and match in buf:
                break
    except socket.timeout:
        pass
    sock.settimeout(1)
    return buf

# Entering CLI: BF's serial parser accepts '#' to switch from MSP to CLI mode
s.sendall(b'#\r\n')
banner = cli_drain(s, timeout=0.8, match=b'# ')
# Query current mixer_type
s.sendall(b'get mixer_type\r\n')
resp = cli_drain(s, timeout=0.5, match=b'# ')
if b'LINEAR' in resp:
    print('  mixer_type: already LINEAR')
    s.sendall(b'exit\r\n')
    cli_drain(s, timeout=0.3)
else:
    s.sendall(b'set mixer_type = LINEAR\r\n')
    cli_drain(s, timeout=0.5, match=b'# ')
    s.sendall(b'save\r\n')
    # save commits eeprom and reboots — socket dies
    cli_drain(s, timeout=0.5)
    s.close()
    time.sleep(1.8)
    s = connect_msp()
    print('  mixer_type: set to LINEAR, saved (reboot)')

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
    GZ_GUI_PID=$!
    echo "Gazebo GUI launched (PID $GZ_GUI_PID)."
fi

# ── Run drone_manage.py in foreground (unless --no-manage) ───────
cd "$PROJECT_DIR"
if [ "$SKIP_MANAGE" = "1" ]; then
    echo "STACK_READY"  # sentinel for wrapper scripts (e.g. test_thrust.sh)
    echo "Sim stack up. Press Ctrl+C to stop."
    # Park the script on `wait` so INT/TERM interrupt promptly and the EXIT
    # trap fires. `tail -f /dev/null` is a cheap way to get a backgrounded
    # child to wait on.
    tail -f /dev/null &
    wait $!
else
    echo "Starting drone_manage.py..."
    uv run --env-file .env python -W ignore::SyntaxWarning \
        donkeydrone/drone_manage.py drive --myconfig=drone_config.py ${MANAGE_ARGS[@]+"${MANAGE_ARGS[@]}"}
fi

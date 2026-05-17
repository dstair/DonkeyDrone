import struct
import socket
import time
import logging

logger = logging.getLogger(__name__)

class MSPClient:
    """Helper to handle BetaFlight MultiWii Serial Protocol over TCP."""
    
    ARMING_FLAG_NAMES = [
        "NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "NOT_DISARMED",
        "BOXFAILSAFE", "RUNAWAY", "CRASH", "THROTTLE", "ANGLE",
        "BOOT_GRACE", "NOPREARM", "LOAD", "CALIBRATING", "CLI",
        "CMS_MENU", "BST", "MSP", "PARALYZE", "GPS", "RESC",
        "DSHOT_TELEM", "REBOOT_REQ", "DSHOT_BB", "ACC_CAL",
        "MOTOR_PROTO", "CRASHFLIP", "ALTHOLD", "POSHOLD", "ARM_SWITCH",
    ]

    MODE_FLAG_NAMES = [
        "ARM", "ANGLE", "HORIZON", "ANTIGRAVITY", "MAG", "HEADFREE",
        "HEADADJ", "CAMSTAB", "PASSTHRU", "BEEPER", "LEDLOW",
        "CALIB", "OSD", "TELEMETRY", "SERVO1", "SERVO2", "SERVO3",
        "BLACKBOX", "FAILSAFE_MODE", "AIRMODE",
    ]

    def __init__(self, host="127.0.0.1", port=5761):
        self.host = host
        self.port = port

    def query_status(self, timeout=0.1):
        """Returns (arming_flags, arming_names, mode_bits, mode_names) or Nones."""
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((self.host, self.port))
            
            cmd = 150  # MSP_STATUS_EX
            frame = b"$M<" + bytes([0, cmd, 0 ^ cmd])
            sock.send(frame)
            
            resp = b""
            deadline = time.time() + timeout
            while time.time() < deadline:
                chunk = sock.recv(4096)
                if not chunk: break
                resp += chunk
                if len(resp) >= 6 and resp[:3] == b"$M>" and len(resp) >= 6 + resp[3]:
                    break
            
            if len(resp) < 6 or resp[:3] != b"$M>":
                return None, [], None, []

            payload = resp[5 : 5 + resp[3]]
            if len(payload) < 20:
                return None, [], None, []

            mode_bits = struct.unpack("<I", payload[6:10])[0]
            active_modes = [
                self.MODE_FLAG_NAMES[i] for i in range(min(len(self.MODE_FLAG_NAMES), 32))
                if mode_bits & (1 << i)
            ]
            
            flags_byte_count = payload[15] & 0x0F
            offset = 16 + flags_byte_count
            if len(payload) < offset + 5:
                return 0, [], mode_bits, active_modes
                
            flags = struct.unpack("<I", payload[offset + 1 : offset + 5])[0]
            active_arming = [
                self.ARMING_FLAG_NAMES[i] for i in range(min(len(self.ARMING_FLAG_NAMES), 29))
                if flags & (1 << i)
            ]
            return flags, active_arming, mode_bits, active_modes
            
        except (OSError, socket.timeout):
            return None, [], None, []
        finally:
            if sock: sock.close()

def pack_rc_frame(channels):
    """Packs 16 PWM channels into the BetaFlight SITL 40-byte format."""
    packet = struct.pack("<d", time.time())
    for ch in channels[:16]:
        packet += struct.pack("<H", int(ch))
    # Pad if fewer than 16 channels provided
    for _ in range(16 - len(channels)):
        packet += struct.pack("<H", 1000)
    return packet
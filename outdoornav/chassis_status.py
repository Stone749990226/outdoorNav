"""Read-only S10 usage-mode check and motion eligibility (no posture commands)."""

import json
import math
import socket
import struct
import time


def usage_mode(host='10.21.33.103', port=30004, timeout=1.5):
    payload = json.dumps({'PatrolDevice': {'Type': 0x00100064, 'Command': 5,
        'Time': time.strftime('%Y-%m-%d %H:%M:%S'), 'Items': {}}}).encode()
    frame = b'\xeb\x91\xeb\x90' + struct.pack('<HH', len(payload), 0) + bytes([1,0,1]) + b'\0'*5 + payload
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((host, port))
        sock.settimeout(.3)
        sock.send(frame)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            try:
                raw = sock.recv(8192)
            except socket.timeout:
                continue
            if len(raw) < 16 or raw[:4] != frame[:4]:
                continue
            size = struct.unpack_from('<H', raw, 4)[0]
            if len(raw) < 16+size:
                continue
            try:
                pd = json.loads(raw[16:16+size]).get('PatrolDevice', {})
                mode = pd.get('Items', {}).get('BasicStatus', {}).get('ControlUsageMode')
            except (ValueError, TypeError, AttributeError):
                continue
            if type(mode) is int and mode in (0, 1, 2):
                return mode
    return None


def chassis_blockers(state, gait, motion_age, mode, mode_age):
    blockers = []
    if not math.isfinite(motion_age) or not 0 <= motion_age < 1.:
        blockers.append('motion_feedback_stale')
    if state != 17:
        blockers.append('not_rl_control')
    if gait not in (0x3002, 0x3003):
        blockers.append('not_navigation_gait')
    if mode != 1 or not math.isfinite(mode_age) or not 0 <= mode_age < 8.:
        blockers.append('navigation_usage_mode_unverified')
    return blockers

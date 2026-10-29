"""Read-only S10 device-status subscription, without ROS or motion commands."""

import json
import math
import socket
import struct
import threading
import time

SYNC = b'\xeb\x91\xeb\x90'


def heartbeat():
    payload = json.dumps({'PatrolDevice': {
        'Type': 0x00100064, 'Command': 5,
        'Time': time.strftime('%Y-%m-%d %H:%M:%S'), 'Items': {},
    }}).encode()
    return SYNC + struct.pack('<HH', len(payload), 0) + bytes([1, 0, 1]) + b'\0'*5 + payload


def battery_status(packet):
    if len(packet) < 16 or packet[:4] != SYNC:
        return None
    size = struct.unpack_from('<H', packet, 4)[0]
    if len(packet) < 16 + size:
        return None
    message = json.loads(packet[16:16+size]).get('PatrolDevice', {})
    if message.get('Type') != 0x00100002 or message.get('Command') != 0x00f00000:
        return None
    batteries = message.get('Items', {}).get('BatteryList', [])
    if not batteries or not isinstance(batteries[0], dict):
        return None
    entry = batteries[0]
    percent = entry.get('BatteryLevel')  # SDK documents percent, not 0..1.
    if (isinstance(percent, bool) or not isinstance(percent, (int, float))
            or not math.isfinite(percent) or not 0 <= percent <= 100):
        return None
    voltage = entry.get('Voltage')
    if (isinstance(voltage, bool) or not isinstance(voltage, (int, float))
            or not math.isfinite(voltage) or voltage < 0):
        voltage = None
    return {'percent': percent, 'voltage_v': voltage,
            'charging': entry.get('charge') if type(entry.get('charge')) is bool else None}


class BatteryMonitor:
    def __init__(self, host='10.21.33.103', port=30004):
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.value, self.received = None, None
        self.error = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, name='battery-status', daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            age = time.monotonic()-self.received if self.received is not None else None
            available = age is not None and age < 15
            return {**(self.value or {}), 'available': available, 'age_s': age,
                    'error': self.error, 'source': 'S10 BatteryList[0]'}

    def run(self):
        while not self.stopping.is_set():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    # Connected UDP only accepts replies from the configured robot.
                    sock.connect((self.host, self.port))
                    sock.settimeout(.4)
                    sock.send(heartbeat())
                    end = time.monotonic()+2
                    while not self.stopping.is_set() and time.monotonic() < end:
                        try:
                            packet = sock.recv(8192)
                        except socket.timeout:
                            continue
                        try:
                            value = battery_status(packet)
                        except (ValueError, TypeError, AttributeError):
                            continue
                        if value is not None:
                            with self.lock:
                                self.value, self.received = value, time.monotonic()
                                self.error = None
                            break
                    else:
                        with self.lock:
                            self.error = 'No fresh battery status received'
            except OSError as error:
                with self.lock:
                    self.error = str(error)
            self.stopping.wait(5)

    def stop(self):
        self.stopping.set()
        self.thread.join(timeout=1)

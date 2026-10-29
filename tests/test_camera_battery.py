import json
import struct
import threading
import time

import pytest
from tools.camera_view.battery import BatteryMonitor, battery_status, heartbeat, SYNC


def packet(value):
    body = json.dumps({'PatrolDevice': {'Type': 0x00100002, 'Command': 0x00f00000,
                      'Items': {'BatteryList': [value]}}}).encode()
    return SYNC + struct.pack('<HH', len(body), 0) + bytes([1, 0, 1]) + b'\0'*5 + body


def test_percent_voltage_and_charge_are_not_confused():
    assert battery_status(packet({'BatteryLevel': 100, 'Voltage': 82.79, 'charge': True})) == {
        'percent': 100, 'voltage_v': 82.79, 'charging': True}
    assert battery_status(packet({'BatteryLevel': 0}))['percent'] == 0


@pytest.mark.parametrize('bad', [-1, 101, True, '50', None])
def test_invalid_values_do_not_become_zero_percent(bad):
    assert battery_status(packet({'BatteryLevel': bad})) is None


def test_header_and_heartbeat_are_read_only():
    assert battery_status(b'bad') is None
    assert battery_status(packet({'BatteryLevel': 50})[:-1]) is None
    request = json.loads(heartbeat()[16:])['PatrolDevice']
    assert request['Type'] == 0x00100064
    assert request['Command'] == 5 and request['Items'] == {}


def test_stale_data_is_labeled_unavailable():
    monitor = object.__new__(BatteryMonitor)
    monitor.lock = threading.Lock()
    monitor.value = {'percent': 50}
    monitor.received = time.monotonic()-20
    monitor.error = None
    assert monitor.snapshot()['available'] is False
    assert monitor.snapshot()['percent'] == 50
    monitor.received = None
    monitor.value = None
    assert 'percent' not in monitor.snapshot()

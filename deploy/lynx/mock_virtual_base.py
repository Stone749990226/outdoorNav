#!/usr/bin/env python3
"""Mock RTK + IMU-independent virtual base for indoor full-flow testing.

Same binary path as the real receiver: emits UBX NAV-PVT and NMEA GGA with the
8-byte host-monotonic prefix the collector uses, so the localization code is
unchanged. The pose integrates only the command topic it is told to listen to;
no real robot motion is produced and no real /cmd_vel is touched.
"""

import argparse
import json
import math
import os
import socket
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from outdoornav.bundle import load_bundle
from outdoornav.coordinates import local_radii


def nmea(body):
    checksum = 0
    for value in body.encode():
        checksum ^= value
    return ('$%s*%02X\r\n' % (body, checksum)).encode()


def nav_pvt(tow, latitude, longitude, height, hacc, vel_n, vel_e, sacc, satellites):
    payload = bytearray(92)
    struct.pack_into('<I', payload, 0, tow)
    now = time.gmtime()
    struct.pack_into('<HBBBBBB', payload, 4, now.tm_year, now.tm_mon, now.tm_mday,
                     now.tm_hour, now.tm_min, now.tm_sec, 0x07)
    struct.pack_into('<I', payload, 12, 20)
    struct.pack_into('<i', payload, 16, 0)
    payload[20] = 3          # fixType: 3D
    payload[21] = 0x03 | 0x80  # gnssFixOk + diffSoln + carrSoln=2 (fixed)
    payload[22] = 0
    payload[23] = satellites
    struct.pack_into('<ii', payload, 24, int(round(longitude * 1e7)), int(round(latitude * 1e7)))
    struct.pack_into('<ii', payload, 32, int(round(height * 1000)), int(round(height * 1000)))
    struct.pack_into('<II', payload, 40, int(hacc * 1000), int(hacc * 1000))
    struct.pack_into('<iii', payload, 48, int(round(vel_n * 1000)), int(round(vel_e * 1000)), 0)
    struct.pack_into('<i', payload, 60, int(round(math.hypot(vel_n, vel_e) * 1000)))
    struct.pack_into('<I', payload, 68, int(sacc * 1000))
    struct.pack_into('<H', payload, 76, 80)  # pDOP 0.80
    payload[78] = 0
    frame = bytearray(b'\xb5\x62\x01\x07' + struct.pack('<H', len(payload)) + payload)
    first = second = 0
    for value in frame[2:]:
        first = (first + value) & 255
        second = (second + first) & 255
    return bytes(frame) + bytes((first, second))


def wit_frame(kind, words):
    frame = bytes([0x55, kind]) + struct.pack('<hhhh', *words)
    return frame + bytes((sum(frame) & 255,))


def wit_packets(yaw_sensor, gyro_z):
    # 11-byte frames: three measurements plus a fourth word (temperature/
    # version), all signed int16 little-endian.
    euler = [0, 0, int(round(math.degrees(yaw_sensor) / 180 * 32768)), 0]
    gyro = [0, 0, int(round(math.degrees(gyro_z) / 2000 * 32768)), 0]
    accel = [0, 0, 2048, 0]
    return (wit_frame(0x51, accel) + wit_frame(0x52, gyro) +
            wit_frame(0x53, euler) + wit_frame(0x54, [0, 0, 0, 0]))


class MockBase(Node):
    def __init__(self, bundle, domain_port, imu_port, start, speed_limit, turn_limit):
        super().__init__('mock_virtual_base')
        self.port = domain_port
        self.imu_port = imu_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.generation = 'mock-' + str(os.getpid())
        self.x, self.y = start
        self.yaw = 0.0
        self.v = self.w = 0.0
        self.command_at = -math.inf
        self.tow = 1000
        self.anchor = bundle['origin_wgs84']
        self.east_radius, self.north_radius = local_radii(self.anchor['latitude'])
        self.speed_limit, self.turn_limit = speed_limit, turn_limit
        self.last = time.monotonic()
        self.create_subscription(Twist, '/mock_base/cmd_vel', self.command, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.2, self.emit)
        self.create_timer(0.02, self.emit_imu)

    def command(self, message):
        self.v = max(-self.speed_limit, min(self.speed_limit, float(message.linear.x)))
        self.w = max(-self.turn_limit, min(self.turn_limit, float(message.angular.z)))
        self.command_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self.last))
        self.last = now
        if now - self.command_at > 0.5:
            self.v = self.w = 0.0
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

    def emit(self):
        self.tow = (self.tow + 200) % 604800000
        latitude = self.anchor['latitude'] + math.degrees(self.y / self.north_radius)
        longitude = self.anchor['longitude'] + math.degrees(self.x / self.east_radius)
        vel_e = self.v * math.cos(self.yaw)
        vel_n = self.v * math.sin(self.yaw)
        stamp = struct.pack('!d', time.monotonic())
        gga_time = time.strftime('%H%M%S', time.gmtime()) + '.00'
        gga = nmea('GNGGA,%s,%.8f,N,%.8f,E,4,12,0.65,%.2f,M,7.5,M,1.0,656' % (
            gga_time, latitude, longitude, 8.0))
        self.sock.sendto(stamp + gga, ('127.0.0.1', self.port))
        self.sock.sendto(stamp + nav_pvt(self.tow, latitude, longitude, 8.0, 0.010,
                                         vel_n, vel_e, 0.060, 30), ('127.0.0.1', self.port))

    def emit_imu(self):
        # Sensor axes: front = -Y, left = +X, up = +Z, so the body yaw is the
        # module yaw minus 90 degrees; feed the inverse so the estimator sees
        # exactly the virtual body attitude.
        raw = wit_packets(self.yaw + math.pi / 2, self.w)
        packet = {'schema_version': 1, 'source': 'wit_imu', 'generation': self.generation,
                  'axis6_register': 0, 'received': time.monotonic(), 'raw_hex': raw.hex()}
        self.sock.sendto(json.dumps(packet).encode(), ('127.0.0.1', self.imu_port))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', required=True)
    parser.add_argument('--port', type=int, default=18765)
    parser.add_argument('--imu-port', type=int, default=18767)
    parser.add_argument('--speed', type=float, default=0.15)
    parser.add_argument('--turn', type=float, default=0.4)
    args = parser.parse_args()
    bundle = load_bundle(Path(args.bundle))
    start = bundle['network']['roads'][0]['points'][0]
    rclpy.init()
    node = MockBase(bundle, args.port, args.imu_port, (start['x'], start['y']), args.speed, args.turn)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

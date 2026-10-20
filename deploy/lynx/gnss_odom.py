"""Publish odometry and TF from RTK position + IMU yaw (no SLAM/ICP).

map  = surveyed ENU frame of the scene bundle (x east, y north)
odom = same origin as map; map->odom is identity
base = odom->base_link from GNSS position and integrated IMU yaw

Heading needs a start value: an explicit --initial-heading-rad, or it is taken
from GNSS course over ground once the robot has moved >= 0.5 m.
"""

import argparse
import json
import math
import socket
import struct
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from outdoornav.bundle import load_bundle
from outdoornav.coordinates import enu
from outdoornav.gnss import FixDecoder, usable_fix


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GnssOdom(Node):
    def __init__(self, args):
        super().__init__('gnss_odom')
        self.args = args
        bundle = load_bundle()
        self.anchor = bundle['origin_wgs84']
        self.decoder = FixDecoder()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', args.udp_port))
        self.socket.setblocking(False)
        self.yaw = None
        self.gyro_bias = None
        self.bias_samples = []
        self.last_imu = None
        self.position = None
        self.heading_source = 'none'
        self.track = []
        self.tf = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node_self() if False else self)
        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.create_subscription(Imu, args.imu_topic, self.imu, qos_profile_sensor_data)
        self.create_timer(0.02, self.receive)
        self.create_timer(0.05, self.publish)
        self.imu_count = 0
        self.fix_count = 0
        self.create_timer(3.0, self.report)

    def node_self(self):
        return self

    def imu(self, msg):
        now = time.monotonic()
        self.imu_count += 1
        # Rotate the measured angular velocity from imu_link into base_link.
        w = msg.angular_velocity
        try:
            t = self.buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
            q = t.transform.rotation
            # rotate vector by quaternion
            x, y, z = w.x, w.y, w.z
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            ix = qw * x + qy * z - qz * y
            iy = qw * y + qz * x - qx * z
            iz = qw * z + qx * y - qy * x
            iw = -qx * x - qy * y - qz * z
            yaw_rate = ix * -qw + iw * qx + iy * -qz - iz * -qy
        except Exception:
            yaw_rate = w.z
        if self.gyro_bias is None:
            if abs(yaw_rate) < 0.3 and len(self.bias_samples) < 200:
                self.bias_samples.append(yaw_rate)
                self.last_imu = now
                return
            if self.bias_samples:
                self.gyro_bias = sum(self.bias_samples) / len(self.bias_samples)
            else:
                self.gyro_bias = 0.0
        if self.last_imu is not None and self.yaw is not None:
            dt = min(now - self.last_imu, 0.2)
            self.yaw = wrap(self.yaw + (yaw_rate - self.gyro_bias) * dt)
        self.last_imu = now

    def receive(self):
        for _ in range(100):
            try:
                raw = self.socket.recv(65535)
            except BlockingIOError:
                break
            if len(raw) < 8:
                continue
            received = struct.unpack('!d', raw[:8])[0]
            if not 0 <= time.monotonic() - received < 1:
                continue
            fix = self.decoder.feed(raw[8:], received)
            if fix is None or not fix.get('valid'):
                continue
            x, y = enu(fix['latitude'], fix['longitude'], self.anchor)
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            self.fix_count += 1
            self.position = (x, y, fix['hacc_m'], fix['tow_ms'])
            if self.yaw is None:
                self.track.append((received, x, y))
                self.track = [s for s in self.track if received - s[0] <= 8]
                if self.args.initial_heading_rad is not None and not self.heading_source == 'config':
                    self.yaw = wrap(self.args.initial_heading_rad)
                    self.heading_source = 'config'
                else:
                    # Latch heading from motion: 1 m over 2 s with at least a
                    # differential fix is enough; a float solution is 1 m
                    # accurate, so a fixed solution is not required here.
                    first, last = self.track[0], self.track[-1]
                    dx, dy = last[1] - first[1], last[2] - first[2]
                    if (math.hypot(dx, dy) >= 1.0 and last[0] - first[0] >= 2.0
                            and fix.get('differential') and fix['hacc_m'] <= 2.0):
                        self.yaw = math.atan2(dy, dx)
                        self.heading_source = 'gnss_course'

    def report(self):
        self.get_logger().info(
            'imu=%d fix=%d pos=%s yaw=%s source=%s bias=%s' % (
                self.imu_count, self.fix_count,
                None if self.position is None else [round(v, 2) for v in self.position[:2]],
                None if self.yaw is None else round(self.yaw, 3),
                self.heading_source, self.gyro_bias))

    def publish(self):
        if self.position is None or self.yaw is None:
            return
        x, y, hacc, epoch = self.position
        now = self.get_clock().now().to_msg()
        yaw = self.yaw
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        variance = max(hacc or 0.0, 0.02) ** 2
        odom.pose.covariance[0] = variance
        odom.pose.covariance[7] = variance
        odom.pose.covariance[35] = 0.02 ** 2
        self.odom_pub.publish(odom)
        identity = TransformStamped()
        identity.header.stamp = now
        identity.header.frame_id = 'map'
        identity.child_frame_id = 'odom'
        identity.transform.rotation.w = 1.0
        self.tf.sendTransform(identity)
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--udp-port', type=int, default=18766)
    parser.add_argument('--imu-topic', default='/IMU/framed')
    parser.add_argument('--odom-topic', default='/outdoor/odom')
    parser.add_argument('--initial-heading-rad', type=float, default=None)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = GnssOdom(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

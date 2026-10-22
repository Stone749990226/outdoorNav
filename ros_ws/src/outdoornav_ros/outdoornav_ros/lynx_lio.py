"""Guarded FAST-LIO sensor adapter. Owns no TF and never publishes motion."""

import copy
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from outdoornav.config import load_robot_config
from outdoornav.lio import LioPose, canonical_points, imu_si, quaternion


class LynxLio(Node):
    def __init__(self):
        super().__init__('lynx_lio_adapter')
        self.declare_parameter('robot_config', '')
        self.config = load_robot_config(self.get_parameter('robot_config').value)
        self.lio = self.config['lio']
        if not self.lio['enabled']:
            raise ValueError('LIO is disabled')
        self.pose = LioPose(self.lio['base_from_imu'])
        self.fault = None
        self.received, self.stamps = {}, {}
        self.scan_end = None
        self.odom_at = -math.inf
        self.stable_since = None
        self.imu_pub = self.create_publisher(Imu, '/outdoor/lio/imu', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/outdoor/lio/points', 10)
        self.odom_pub = self.create_publisher(Odometry, '/outdoor/lio/odom', 10)
        self.status_pub = self.create_publisher(String, '/outdoor/lio/status', 10)
        self.create_subscription(Imu, self.lio['imu_topic'], self.imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.config['cloud_topic'], self.cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/outdoor/lio/raw_odom', self.odom, 10)
        self.create_timer(0.1, self.status)

    def fail(self, error):
        if self.fault is None:
            self.fault = str(error)
            self.get_logger().error(self.fault)

    def observe(self, kind, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if not -0.1 <= age < self.lio['input_timeout_s']:
            raise ValueError(kind + ' stale/future source timestamp')
        previous = self.stamps.get(kind)
        if previous is not None and not 0 < stamp-previous < self.lio['input_timeout_s']:
            raise ValueError(kind + ' timestamp reversed, repeated or interrupted; restart required')
        self.stamps[kind], self.received[kind] = stamp, time.monotonic()
        return stamp

    def inputs_ready(self):
        now = time.monotonic()
        return not self.fault and all(
            now-self.received.get(key, -math.inf) < self.lio['input_timeout_s']
            for key in ('imu', 'lidar'))

    def imu(self, msg):
        if self.fault:
            return
        try:
            self.observe('imu', msg)
            if ((not msg.header.frame_id and not self.lio['allow_empty_imu_frame']) or
                    (msg.header.frame_id and msg.header.frame_id != self.lio['imu_source_frame'])):
                raise ValueError('IMU frame differs from measured extrinsics')
            gyro, accel = imu_si(
                [getattr(msg.angular_velocity, key) for key in ('x', 'y', 'z')],
                [getattr(msg.linear_acceleration, key) for key in ('x', 'y', 'z')], self.lio)
            if msg.angular_velocity_covariance[0] < 0 or msg.linear_acceleration_covariance[0] < 0:
                raise ValueError('IMU has no gyro/accelerometer measurement')
            # Noise comes from explicit configuration; zero covariance is not treated as zero noise.
            value = copy.deepcopy(msg)
            value.header.frame_id = self.lio['imu_frame']
            value.angular_velocity.x, value.angular_velocity.y, value.angular_velocity.z = gyro.tolist()
            value.linear_acceleration.x, value.linear_acceleration.y, value.linear_acceleration.z = accel.tolist()
            # FAST-LIO consumes only gyro and acceleration; do not advertise an
            # orientation in a newly identified physical IMU frame.
            value.orientation_covariance[0] = -1.
            gyro_factor = 1. if self.lio['gyro_unit'] == 'rad/s' else math.pi/180
            accel_factor = 1. if self.lio['accel_unit'] == 'm/s2' else 9.80665
            value.angular_velocity_covariance = [v*gyro_factor**2 for v in msg.angular_velocity_covariance]
            value.linear_acceleration_covariance = [v*accel_factor**2 for v in msg.linear_acceleration_covariance]
            self.imu_pub.publish(value)
        except (ValueError, TypeError) as error:
            self.fail(error)

    def cloud(self, msg):
        if self.fault:
            return
        try:
            stamp = self.observe('lidar', msg)
            if msg.header.frame_id != self.lio['lidar_frame']:
                raise ValueError('Lidar frame differs from measured extrinsics')
            names = ('x', 'y', 'z', 'intensity', self.lio['ring_field'], self.lio['time_field'])
            if not set(names) <= {field.name for field in msg.fields}:
                raise ValueError('Lidar requires x/y/z/intensity, measured ring and per-point time fields')
            points, scan_start = canonical_points([tuple(p) for p in point_cloud2.read_points(
                msg, field_names=names, skip_nans=False)], stamp, self.lio)
            self.scan_end = scan_start + float(points[-1, 5])
            fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                      for name, offset, datatype in (
                          ('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7),
                          ('ring', 16, 4), ('time', 20, 7))]
            rows = [(float(x), float(y), float(z), float(i), int(r), float(t)) for x, y, z, i, r, t in points]
            header = copy.deepcopy(msg.header)
            stamp_ns = round(scan_start*1e9)
            header.stamp.sec, header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            self.cloud_pub.publish(point_cloud2.create_cloud(header, fields, rows))
        except (ValueError, TypeError, KeyError, AssertionError) as error:
            self.fail(error)

    def odom(self, msg):
        if not self.inputs_ready():
            return
        try:
            stamp = self.observe('odom', msg)
            if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
                raise ValueError('Unexpected FAST-LIO odometry frames')
            if not self.scan_end-self.lio['input_timeout_s'] < stamp <= self.scan_end+0.01:
                raise ValueError('LIO output does not match lidar epoch')
            if self.stamps['imu'] + 0.01 < stamp:
                raise ValueError('LIO output is ahead of received IMU')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            matrix, _ = self.pose.update([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], stamp)
            result = Odometry()
            result.header.stamp = msg.header.stamp
            result.header.frame_id, result.child_frame_id = 'odom', 'base_link'
            p, q = result.pose.pose.position, result.pose.pose.orientation
            p.x, p.y, p.z = matrix[:3, 3].tolist()
            q.x, q.y, q.z, q.w = quaternion(matrix[:3, :3]).tolist()
            # Upstream covariance is published one iteration late and is in IMU coordinates.
            # Do not relabel it as base covariance. The localizer measures twist from poses.
            self.odom_pub.publish(result)
            self.odom_at = time.monotonic()
            if self.stable_since is None:
                self.stable_since = self.odom_at
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            self.fail(error)

    def status(self):
        for topic in ('/outdoor/lio/odom', '/outdoor/lio/raw_odom', '/outdoor/lio/imu', '/outdoor/lio/points'):
            if self.count_publishers(topic) > 1:
                self.fail('Multiple LIO publishers on ' + topic)
        now = time.monotonic()
        fresh = self.inputs_ready() and now-self.odom_at < self.lio['input_timeout_s']
        if not fresh:
            self.stable_since = None
        ready = bool(fresh and self.stable_since is not None
                     and now-self.stable_since >= self.config['gnss_stable_s'])
        self.status_pub.publish(String(data=json.dumps({
            'ready': ready, 'backend': 'fast_lio', 'fault': self.fault,
            'inputs_fresh': bool(self.inputs_ready()), 'odometry_fresh': now-self.odom_at < self.lio['input_timeout_s'],
            'motion_allowed': False,  # adapter itself never authorizes movement
        })))


def main():
    rclpy.init()
    node = LynxLio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

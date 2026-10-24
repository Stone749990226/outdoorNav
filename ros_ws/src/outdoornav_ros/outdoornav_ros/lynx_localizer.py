"""Read-only native odometry + receiver callback bridge, owning outdoor TF."""

import json
import math
import socket
import struct
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from outdoornav.config import load_robot_config
from outdoornav.diagnostics import StaticFrames, json_safe

from outdoornav.bundle import load_bundle
from outdoornav.coordinates import enu
from outdoornav.dual_rtk import DualRtkDecoder
from outdoornav.gnss import FixDecoder
from outdoornav.localization import GeoAlignment


def transform_key(transform):
    p, q, stamp = transform.transform.translation, transform.transform.rotation, transform.header.stamp
    return (transform.header.frame_id, transform.child_frame_id, stamp.sec, stamp.nanosec,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class LynxLocalizer(Node):
    def __init__(self):
        super().__init__("lynx_geo_localization")
        for key in ("bundle", "robot_config"):
            self.declare_parameter(key, "")
        self.config = load_robot_config(self.get_parameter("robot_config").value)
        bundle = load_bundle(self.get_parameter("bundle").value)
        self.alignment = GeoAlignment(bundle["origin_wgs84"], self.config)
        self.decoder = FixDecoder(self.config)
        self.heading_decoder = None
        self.heading_socket = None
        self.heading_packets = self.heading_bytes = 0
        self.heading_rx_at = -math.inf
        self.heading_applied_at = None
        if self.config['heading_source'] == 'dual_antenna':
            heading = self.config['dual_heading']
            self.heading_decoder = DualRtkDecoder(
                timeout_s=heading['timeout_s'],
                baseline_to_body_yaw_rad=(heading['baseline_to_body_yaw_rad']
                                         if heading['mounting_confirmed'] else None))
            self.heading_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.heading_socket.bind(('127.0.0.1', heading['udp_port']))
            self.heading_socket.setblocking(False)
        self.rx_packets, self.rx_bytes, self.rx_pvt = 0, 0, 0
        self.rx_at = -math.inf
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", self.config["gnss_udp_port"]))
        self.socket.setblocking(False)
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/outdoor/odom", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/outdoor/gnss/fix", 10)
        self.status_pub = self.create_publisher(String, "/outdoor/localization", 10)
        self.heading_pub = self.create_publisher(String, "/outdoor/heading", 10)
        self.lio_status, self.lio_at = {}, -math.inf
        if self.config.get('lio', {}).get('enabled'):
            self.create_subscription(String, '/outdoor/lio/status', self.on_lio, 10)
        if self.config['odom_topic']:
            self.create_subscription(Odometry, self.config['odom_topic'], self.odom, qos_profile_sensor_data)
        self.static_frames = StaticFrames()
        self.cloud_frame = None
        static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self.static_frames.receive, static_qos)
        self.create_subscription(PointCloud2, self.config['cloud_topic'], self.on_cloud, qos_profile_sensor_data)
        self.ownership_conflict = False
        self.sent_transforms = deque(maxlen=200)
        self.dynamic_tf_sub = self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_timer(1.0, self.check_tf_owners)
        self.create_timer(0.02, self.receive)
        if self.heading_socket is not None:
            self.create_timer(0.02, self.receive_heading)
        self.create_timer(0.1, self.status)

    def on_cloud(self, msg):
        self.cloud_frame = msg.header.frame_id

    def on_lio(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('LIO status must be an object')
            self.lio_status, self.lio_at = value, time.monotonic()
        except (ValueError, TypeError):
            self.lio_status = {}

    def lio_ready(self, now):
        return (not self.config.get('lio', {}).get('enabled') or
                (self.lio_status.get('ready') is True and
                 now-self.lio_at < self.config['lio']['input_timeout_s']))

    def extrinsics(self):
        return {'ready': self.static_frames.matrix(self.cloud_frame) is not None,
                'cloud_frame': self.cloud_frame, 'faults': sorted(self.static_frames.faults)}

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if (
            not all(math.isfinite(v) for v in (norm, p.x, p.y, p.z))
            or abs(norm - 1) > 0.05
            or not -0.1 <= age < self.config["sensor_timeout_s"]
        ):
            self.alignment.fault = "odometry_stamp_or_quaternion_invalid"
            return
        magnitude = math.sqrt(norm)
        q.x, q.y, q.z, q.w = (v / magnitude for v in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        previous = self.alignment.history[-1] if self.alignment.history else None
        if not self.alignment.odom(p.x, p.y, yaw, stamp, time.monotonic()):
            return
        if previous:
            from outdoornav.localization import wrap

            dt = stamp - previous[3]
            vx, vy = (p.x - previous[0]) / dt, (p.y - previous[1]) / dt
            msg.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
            msg.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
            msg.twist.twist.angular.z = wrap(yaw - previous[2]) / dt
        else:
            # One pose cannot establish a measured velocity or standstill.
            return
        # Preserve the source clock; no restamping of stale samples.
        msg.header.frame_id, msg.child_frame_id = "odom", "base_link"
        self.odom_pub.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "base_link"
        (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ) = p.x, p.y, p.z
        tf.transform.rotation = q
        self.send_tf(tf)
        a = self.alignment
        if a.ready(time.monotonic()) and self.lio_ready(time.monotonic()) and not self.ownership_conflict:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id, tf.child_frame_id = "map", "odom"
            tf.transform.translation.x, tf.transform.translation.y = a.translation
            tf.transform.rotation.z = math.sin(a.rotation / 2)
            tf.transform.rotation.w = math.cos(a.rotation / 2)
            self.send_tf(tf)

    def send_tf(self, transform):
        # Humble callbacks do not expose publisher IDs. Remember our exact
        # messages so a competing publisher of these frames cannot be hidden.
        self.sent_transforms.append(transform_key(transform))
        self.tf.sendTransform(transform)

    def check_tf_owners(self):
        if self.count_publishers(self.dynamic_tf_sub.topic_name) > 1:
            self.ownership_conflict = True

    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id in ('odom', 'base_link') and transform_key(transform) not in self.sent_transforms:
                self.ownership_conflict = True
                self.alignment.fault = 'conflicting_tf_owner; stop other localization and restart'

    def receive_heading(self):
        for _ in range(100):
            try:
                raw = self.heading_socket.recv(65535)
            except BlockingIOError:
                break
            if len(raw) < 8:
                continue
            received = struct.unpack('!d', raw[:8])[0]
            if not 0 <= time.monotonic()-received < self.config['dual_heading']['timeout_s']:
                continue
            self.heading_packets += 1
            self.heading_bytes += len(raw)-8
            self.heading_rx_at = received
            self.heading_decoder.feed(raw[8:], received)
            snapshot = self.heading_decoder.snapshot(time.monotonic())
            # HDT/GGA packets must not extend the freshness of an old THS.
            if self.heading_decoder.heading_at != self.heading_applied_at:
                self.alignment.update_heading(snapshot, self.heading_decoder.heading_at)
                self.heading_applied_at = self.heading_decoder.heading_at

    def heading_status(self, now):
        if self.heading_decoder is None:
            return {'source': 'configured', 'ready': True,
                    'body_yaw_enu_rad': self.config['initial_heading_rad'],
                    'time_basis': 'configuration, not a measured absolute heading'}
        value = self.heading_decoder.snapshot(now)
        heading = value['heading'] or {}
        age = now-self.heading_decoder.heading_at
        rx_age = now-self.heading_rx_at
        return {'source': 'dual_antenna', 'ready': self.alignment.heading_ready(now),
                'true_heading_deg': heading.get('true_heading_deg'),
                'heading_fresh': value['heading_fresh'],
                'status': heading.get('status'),
                'body_yaw_enu_rad': value['body_yaw_enu_rad'] if value['heading_fresh'] else None,
                'mounting_confirmed': self.config['dual_heading']['mounting_confirmed'],
                'quality_confirmed': self.config['dual_heading']['quality_confirmed'],
                'age_s': age if math.isfinite(age) else None,
                'receive_age_s': rx_age if math.isfinite(rx_age) else None,
                'odom_receive_skew_s': self.alignment.heading_skew_s,
                'time_basis': 'host_monotonic_receive; THS has no acquisition timestamp',
                'packets': self.heading_packets, 'bytes': self.heading_bytes,
                'counts': value['counts'], 'bad_checksums': value['bad_checksums'],
                'malformed': value['malformed'], 'fault': value['fault'],
                'unverified_hdt': value['unverified_hdt']}

    def receive(self):
        for _ in range(100):
            try:
                raw = self.socket.recv(65535)
            except BlockingIOError:
                break
            if len(raw) < 8:
                continue
            received = struct.unpack("!d", raw[:8])[0]
            if not 0 <= time.monotonic() - received < self.config["sensor_timeout_s"]:
                continue
            self.rx_packets += 1
            self.rx_bytes += len(raw) - 8
            self.rx_at = received
            fix = self.decoder.feed(raw[8:], received)
            if fix is None:
                continue
            self.rx_pvt += 1
            self.alignment.update_fix(fix, received)
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gnss_antenna"
            msg.latitude, msg.longitude, msg.altitude = (
                fix["latitude"],
                fix["longitude"],
                fix["altitude_m"],
            )
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.status.status = (
                NavSatStatus.STATUS_FIX if fix["valid"] else NavSatStatus.STATUS_NO_FIX
            )
            msg.position_covariance = [
                fix["hacc_m"] ** 2,
                0.0,
                0.0,
                0.0,
                fix["hacc_m"] ** 2,
                0.0,
                0.0,
                0.0,
                fix["vacc_m"] ** 2,
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.fix_pub.publish(msg)

    def status(self):
        a, now = self.alignment, time.monotonic()
        pose = a.pose()
        heading = self.heading_status(now)
        if (pose is None and a.fix and a.fix["valid"] and
                (self.config["heading_source"] == "configured" or
                 (heading.get("heading_fresh") and heading.get("body_yaw_enu_rad") is not None))):
            x, y = enu(a.fix["latitude"], a.fix["longitude"], a.anchor)
            yaw = (heading.get("body_yaw_enu_rad") if self.config["heading_source"] == "dual_antenna"
                   else self.config["initial_heading_rad"])
            pose = {"x": x, "y": y, "yaw": yaw}
        age = now - a.fix_at
        payload = {
            "ready": a.ready(now) and self.lio_ready(now) and not self.ownership_conflict,
            "lio": ({**self.lio_status, 'ready': self.lio_ready(now)}
                    if self.config.get('lio', {}).get('enabled') else None),
            "grade": a.grade,
            "heading": heading,
            "position_source": "ublox_nav_pvt",
            "position_time_basis": "receiver_iTOW sequence; host_monotonic_receive freshness",
            "ownership_conflict": self.ownership_conflict or bool(self.static_frames.faults),
            "extrinsics": self.extrinsics(),
            "tf_conflicts": {"dynamic": self.ownership_conflict, "static": sorted(self.static_frames.faults)},
            "fault": a.fault or (None if self.config["odom_topic"] else "odometry_source_unconfigured"),
            "receiver": {"packets": self.rx_packets, "bytes": self.rx_bytes,
                         "pvt_messages": self.rx_pvt, "last_packet_age_s": now - self.rx_at},
            "fix": a.fix,
            "fix_age_s": age if math.isfinite(age) else None,
            "pose": pose,
            "pose_source": "aligned_odom" if a.pose() else "gnss_antenna",
        }
        self.heading_pub.publish(String(data=json.dumps(json_safe(heading), allow_nan=False)))
        self.status_pub.publish(String(data=json.dumps(json_safe(payload), allow_nan=False)))

    def destroy_node(self):
        self.socket.close()
        if self.heading_socket is not None:
            self.heading_socket.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LynxLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

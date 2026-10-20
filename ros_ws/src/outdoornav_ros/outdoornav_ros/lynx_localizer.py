"""Read-only native odometry + receiver callback bridge, owning outdoor TF."""

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
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from outdoornav.bundle import load_bundle
from outdoornav.coordinates import enu
from outdoornav.gnss import FixDecoder
from outdoornav.localization import GeoAlignment


class LynxLocalizer(Node):
    def __init__(self):
        super().__init__("lynx_geo_localization")
        for key in ("bundle", "robot_config"):
            self.declare_parameter(key, "")
        self.config = json.loads(
            Path(self.get_parameter("robot_config").value).read_text()
        )
        bundle = load_bundle(self.get_parameter("bundle").value)
        self.alignment = GeoAlignment(bundle["origin_wgs84"], self.config)
        self.decoder = FixDecoder()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", self.config["gnss_udp_port"]))
        self.socket.setblocking(False)
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/outdoor/odom", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/outdoor/gnss/fix", 10)
        self.status_pub = self.create_publisher(String, "/outdoor/localization", 10)
        self.create_subscription(
            Odometry, self.config["odom_topic"], self.odom, qos_profile_sensor_data
        )
        self.create_timer(0.02, self.receive)
        self.create_timer(0.1, self.status)

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if (
            not all(math.isfinite(v) for v in (norm, p.x, p.y, p.z))
            or abs(norm - 1) > 0.05
            or not -0.1 <= age < 1
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
        self.tf.sendTransform(tf)
        a = self.alignment
        if a.ready(time.monotonic()):
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id, tf.child_frame_id = "map", "odom"
            tf.transform.translation.x, tf.transform.translation.y = a.translation
            tf.transform.rotation.z = math.sin(a.rotation / 2)
            tf.transform.rotation.w = math.cos(a.rotation / 2)
            self.tf.sendTransform(tf)

    def receive(self):
        for _ in range(100):
            try:
                raw = self.socket.recv(65535)
            except BlockingIOError:
                break
            if len(raw) < 8:
                continue
            received = struct.unpack("!d", raw[:8])[0]
            if not 0 <= time.monotonic() - received < 1:
                continue
            fix = self.decoder.feed(raw[8:], received)
            if fix is None:
                continue
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
        if pose is None and a.fix and a.fix["valid"]:
            x, y = enu(a.fix["latitude"], a.fix["longitude"], a.anchor)
            pose = {"x": x, "y": y, "yaw": self.config["initial_heading_rad"]}
        age = now - a.fix_at
        payload = {
            "ready": a.ready(now),
            "fault": a.fault,
            "fix": a.fix,
            "fix_age_s": age if math.isfinite(age) else None,
            "pose": pose,
            "pose_source": "aligned_odom" if a.pose() else "gnss_antenna",
        }
        self.status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LynxLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

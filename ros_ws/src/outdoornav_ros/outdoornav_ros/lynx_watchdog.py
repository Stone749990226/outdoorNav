"""Final command timeout independent of the navigation worker process."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Watchdog(Node):
    def __init__(self):
        super().__init__("lynx_command_watchdog")
        self.command, self.command_at, self.scan_at, self.geo_at = (
            Twist(),
            0.0,
            0.0,
            0.0,
        )
        self.ready = False
        self.conflict = False
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/outdoor/safe_command", self.receive, 10)
        self.create_subscription(String, "/outdoor/localization", self.geo, 10)
        self.create_subscription(
            LaserScan, "/outdoor/scan", self.scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.check_owners)

    def check_owners(self):
        if self.count_publishers("/cmd_vel") > 1:
            self.conflict = True
            self.get_logger().error(
                "Another /cmd_vel publisher detected; restart after removing it"
            )

    def receive(self, msg):
        self.command, self.command_at = msg, time.monotonic()

    def geo(self, msg):
        try:
            self.ready = bool(json.loads(msg.data)["ready"])
            self.geo_at = time.monotonic()
        except (ValueError, KeyError):
            self.ready = False

    def scan(self, msg):
        age = (
            self.get_clock().now().nanoseconds / 1e9
            - msg.header.stamp.sec
            - msg.header.stamp.nanosec / 1e9
        )
        if -0.1 <= age < 1 and any(
            math.isfinite(r) and msg.range_min < r < msg.range_max for r in msg.ranges
        ):
            self.scan_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        output = Twist()
        if (
            not self.conflict
            and self.ready
            and now - self.geo_at < 1
            and now - self.scan_at < 1
            and now - self.command_at < 0.3
        ):
            x, y, z = (
                self.command.linear.x,
                self.command.linear.y,
                self.command.angular.z,
            )
            if all(math.isfinite(v) for v in (x, y, z)):
                # No reverse or lateral motion until rear coverage is validated.
                output.linear.x = max(0.0, min(0.15, x))
                output.angular.z = max(-0.4, min(0.4, z))
        self.pub.publish(output)

    def destroy_node(self):
        self.pub.publish(Twist())
        super().destroy_node()


def main():
    rclpy.init()
    node = Watchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

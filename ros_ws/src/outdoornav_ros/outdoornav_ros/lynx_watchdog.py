"""Final command timeout independent of the navigation worker process."""

import json
import math
import time
import signal
from rclpy.signals import SignalHandlerOptions
from outdoornav.config import load_robot_config

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Watchdog(Node):
    def __init__(self):
        super().__init__("lynx_command_watchdog")
        self.declare_parameter('robot_config', '')
        self.declare_parameter('diagnostic', False)
        self.diagnostic = self.get_parameter('diagnostic').value
        self.config = load_robot_config(self.get_parameter('robot_config').value)
        self.command, self.command_at, self.scan_at, self.geo_at = (
            Twist(),
            0.0,
            0.0,
            0.0,
        )
        self.ready = False
        self.conflict = False
        self.pub = self.create_publisher(Twist, "/outdoor/diagnostic_cmd_vel" if self.diagnostic else "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/outdoor/command_status", 10)
        self.create_subscription(Twist, "/outdoor/safe_command", self.receive, 10)
        self.create_subscription(String, "/outdoor/localization", self.geo, 10)
        self.create_subscription(
            LaserScan, "/outdoor/scan", self.scan, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.check_owners)

    def check_owners(self):
        if self.count_publishers(self.pub.topic_name) > 1:
            self.conflict = True
            self.get_logger().error(
                "Another command output publisher detected; restart after removing it"
            )

    def receive(self, msg):
        self.command, self.command_at = msg, time.monotonic()

    def geo(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("Localization status must be an object")
            self.ready = bool(data["ready"]) and not data.get("ownership_conflict", False) and data.get("extrinsics", {}).get("ready", False)
            self.geo_at = time.monotonic()
        except (ValueError, KeyError):
            self.ready = False

    def scan(self, msg):
        age = (
            self.get_clock().now().nanoseconds / 1e9
            - msg.header.stamp.sec
            - msg.header.stamp.nanosec / 1e9
        )
        if -0.1 <= age < self.config["sensor_timeout_s"] and any(
            math.isfinite(r) and msg.range_min < r < msg.range_max for r in msg.ranges
        ):
            self.scan_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        output = Twist()
        if (
            not self.conflict
            and not self.diagnostic
            and self.config["motion_enabled"]
            and self.ready
            and now - self.geo_at < self.config["sensor_timeout_s"]
            and now - self.scan_at < self.config["sensor_timeout_s"]
            and now - self.command_at < self.config["command_timeout_s"]
        ):
            x, y, z = (
                self.command.linear.x,
                self.command.linear.y,
                self.command.angular.z,
            )
            if all(math.isfinite(v) for v in (x, y, z)):
                # No reverse or lateral motion until rear coverage is validated.
                output.linear.x = max(0.0, min(self.config["max_speed_mps"], x))
                output.angular.z = max(-self.config["max_angular_speed_rps"], min(self.config["max_angular_speed_rps"], z))
        self.pub.publish(output)
        self.status_pub.publish(String(data=json.dumps({'conflict': self.conflict,
            'diagnostic': self.diagnostic, 'output_topic': self.pub.topic_name})))

    def destroy_node(self):
        if rclpy.ok():
            self.pub.publish(Twist())
        super().destroy_node()


def main():
    # Publish the final zero before shutting down the ROS context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Watchdog()
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

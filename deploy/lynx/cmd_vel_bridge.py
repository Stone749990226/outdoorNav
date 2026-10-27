#!/usr/bin/env python3
"""Bridge our /cmd_vel (Twist) to the S10 native /NAV_CMD; zeros when stale.

Used only for the field demo where the Robonix chassis service is not running.
The robot must already be in RL navigation mode (state 17 + nav gait + usage
mode 1); this node neither changes posture nor reads the robot odometry.
"""

import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from drdds.msg import NavCmd


class Bridge(Node):
    def __init__(self):
        super().__init__('outdoor_cmd_vel_bridge')
        self.stamp = -1e9
        self.v = self.w = 0.0
        self.count = 0
        self.commands = 0
        self.publisher = self.create_publisher(NavCmd, '/NAV_CMD', 10)
        self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.create_timer(0.05, self.tick)

    def receive(self, message):
        self.v = float(message.linear.x)
        self.w = float(message.angular.z)
        self.stamp = time.monotonic()
        self.count += 1

    def tick(self):
        fresh = time.monotonic() - self.stamp < 0.35
        command = NavCmd()
        if fresh:
            command.data.x_vel = self.v
            command.data.y_vel = 0.0
            command.data.yaw_vel = self.w
            if abs(self.v) + abs(self.w) > 1e-6:
                self.commands += 1
        self.publisher.publish(command)
        if self.count % 100 == 0 and self.count:
            self.get_logger().info(
                'twist=%d nonzero=%d fresh=%s v=%.2f w=%.2f' %
                (self.count, self.commands, fresh, self.v, self.w))


def main():
    rclpy.init()
    node = Bridge()
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for _ in range(20):
            node.publisher.publish(NavCmd())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

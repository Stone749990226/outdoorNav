#!/usr/bin/env python3
"""Bridge our /cmd_vel (Twist) to the S10 native /NAV_CMD; zeros when stale.

Used only for the field demo where the Robonix chassis service is not running.
The robot must already be in RL navigation mode (state 17 + nav gait + usage
mode 1); this node neither changes posture nor reads the robot odometry.
"""

import signal
import time
import json
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from drdds.msg import NavCmd, MotionInfo
from std_msgs.msg import String
from outdoornav.chassis_status import chassis_blockers, usage_mode


class Bridge(Node):
    def __init__(self):
        super().__init__('outdoor_cmd_vel_bridge')
        self.motion_at = self.mode_at = -math.inf
        self.state = self.gait = self.mode = None
        self.last_log_count = -1
        self.closing = threading.Event()
        self.create_subscription(MotionInfo, '/MOTION_INFO', self.motion, 10)
        self.status_pub = self.create_publisher(String, '/outdoor/chassis_status', 10)
        self.mode_thread = threading.Thread(target=self.poll_mode, daemon=True)
        self.mode_thread.start()
        self.stamp = -1e9
        self.v = self.w = 0.0
        self.count = 0
        self.commands = 0
        self.publisher = self.create_publisher(NavCmd, '/NAV_CMD', 10)
        self.create_subscription(Twist, '/cmd_vel', self.receive, 10)
        self.create_timer(0.05, self.tick)

    def motion(self, message):
        self.state = int(message.data.motion_state.state)
        self.gait = int(message.data.gait_state.gait)
        self.motion_at = time.monotonic()

    def poll_mode(self):
        while not self.closing.is_set():
            try:
                mode = usage_mode()
                if mode is not None:
                    self.mode, self.mode_at = mode, time.monotonic()
            except OSError:
                pass
            self.closing.wait(3)

    def receive(self, message):
        self.v = float(message.linear.x)
        self.w = float(message.angular.z)
        self.stamp = time.monotonic()
        self.count += 1

    def tick(self):
        fresh = time.monotonic() - self.stamp < 0.35
        command = NavCmd()
        now = time.monotonic()
        blockers = chassis_blockers(self.state, self.gait, now-self.motion_at, self.mode, now-self.mode_at)
        if fresh and not blockers and all(math.isfinite(v) for v in (self.v, self.w)):
            command.data.x_vel = max(0., min(.4, self.v))
            command.data.y_vel = 0.0
            command.data.yaw_vel = max(-.4, min(.4, self.w))
            if abs(self.v) + abs(self.w) > 1e-6:
                self.commands += 1
        self.publisher.publish(command)
        self.status_pub.publish(String(data=json.dumps({'ready': not blockers,
            'state': self.state, 'gait': self.gait, 'usage_mode': self.mode,
            'blocked_reasons': blockers})))
        if self.count % 100 == 0 and self.count and self.count != self.last_log_count:
            self.last_log_count = self.count
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
        node.closing.set()
        node.mode_thread.join(timeout=2)
        for _ in range(20):
            node.publisher.publish(NavCmd())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

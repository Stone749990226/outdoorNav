#!/usr/bin/env python3
"""Offline ground-truth and estimated trajectory recorder."""

import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__("rtk_trajectory_logger")

        self.declare_parameter("ground_truth_topic", "/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/odometry/global")
        self.declare_parameter("output_csv", "/tmp/trajectory.csv")
        self.declare_parameter("flush_every", 20)

        self.output_csv = self.get_parameter("output_csv").value
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        self._handle = open(self.output_csv, "w", encoding="utf-8")
        self._handle.write("t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw\n")
        self._handle.flush()

        self.gt = None
        self.est = None
        self.count = 0

        self.create_subscription(
            Odometry, self.get_parameter("ground_truth_topic").value, self.on_gt, 20
        )
        self.create_subscription(
            Odometry, self.get_parameter("estimate_topic").value, self.on_est, 20
        )
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "轨迹记录：真值 %s / 估计 %s -> %s"
            % (
                self.get_parameter("ground_truth_topic").value,
                self.get_parameter("estimate_topic").value,
                self.output_csv,
            )
        )

    @staticmethod
    def pose_of(msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return (p.x, p.y, yaw)

    def on_gt(self, msg: Odometry):
        self.gt = self.pose_of(msg)

    def on_est(self, msg: Odometry):
        self.est = self.pose_of(msg)

    def tick(self):
        if self.gt is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        if self.est is None:
            self._handle.write("%.4f,%.4f,%.4f,%.4f,,,\n" % (t, *self.gt))
        else:
            self._handle.write(
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (t, *self.gt, *self.est)
            )
        self.count += 1
        # 立即落盘：演示可能被强杀，缓冲没刷就丢整段轨迹
        if self.count % int(self.get_parameter("flush_every").value) == 0:
            self._handle.flush()

    def destroy_node(self):
        try:
            self._handle.flush()
            self._handle.close()
        except Exception:
            pass
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

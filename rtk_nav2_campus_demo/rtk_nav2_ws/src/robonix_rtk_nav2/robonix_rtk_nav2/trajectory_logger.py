#!/usr/bin/env python3
"""轨迹记录：真值 + RTK/EKF 估计，写 CSV，供**离线**误差评估。

真值（/ground_truth/odom）在这里的角色是"事后评估的尺子"：
它只被本记录器与 rosbag 订阅，不进 TF、不进定位、不进控制、不进代价地图——
验收会按这个口径做来源审计。

CSV 列：t, gt_x, gt_y, gt_yaw, est_x, est_y, est_yaw
  gt_*  ：Gazebo 真值（世界坐标，m / rad）
  est_* ：定位链路给出的绝对位姿（map 坐标）
本场景里 map 系与世界系原点重合（RTK 基准点就是世界原点），所以两者可直接比。
"""

import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class TrajectoryLogger(Node):

    def __init__(self):
        super().__init__('rtk_trajectory_logger')

        self.declare_parameter('ground_truth_topic', '/ground_truth/odom')
        self.declare_parameter('estimate_topic', '/odometry/global')
        self.declare_parameter('output_csv', '/tmp/trajectory.csv')
        self.declare_parameter('flush_every', 20)

        self.output_csv = self.get_parameter('output_csv').value
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        self._handle = open(self.output_csv, 'w', encoding='utf-8')
        self._handle.write('t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw\n')
        self._handle.flush()

        self.gt = None
        self.est = None
        self.count = 0

        self.create_subscription(Odometry, self.get_parameter('ground_truth_topic').value,
                                 self.on_gt, 20)
        self.create_subscription(Odometry, self.get_parameter('estimate_topic').value,
                                 self.on_est, 20)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('轨迹记录：真值 %s / 估计 %s -> %s'
                               % (self.get_parameter('ground_truth_topic').value,
                                  self.get_parameter('estimate_topic').value,
                                  self.output_csv))

    @staticmethod
    def pose_of(msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
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
            self._handle.write('%.4f,%.4f,%.4f,%.4f,,,\n' % (t, *self.gt))
        else:
            self._handle.write('%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n'
                               % (t, *self.gt, *self.est))
        self.count += 1
        # 立即落盘：演示可能被强杀，缓冲没刷就丢整段轨迹
        if self.count % int(self.get_parameter('flush_every').value) == 0:
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


if __name__ == '__main__':
    raise SystemExit(main())

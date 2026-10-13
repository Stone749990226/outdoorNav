#!/usr/bin/env python3
"""实车位姿与**姿态**记录（阶段七验收用）。

为什么要单记姿态：验收要求报"实车位姿与姿态"，而已知一个异常——车停稳后
Gazebo 里的车体俯仰实测到 21.6°（坡面本身只有 8.5°/10.6°）。二维定位不发布俯仰，
所以姿态只能从 IMU 拿：本脚本同时记 `/ground_truth/odom`（真值 x,y,yaw）与
`/rtk/imu` 的 orientation（世界系 roll/pitch/yaw），10 Hz 落 CSV。
到站/停在坡上时，这几列就是"车到底以什么姿态待着"的直接证据。
"""
import argparse
import csv
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


class Log(Node):
    def __init__(self, path):
        super().__init__('pose_attitude_log')
        self.f = open(path, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        self.w.writerow(['wall', 'gt_x', 'gt_y', 'gt_yaw_deg',
                         'imu_roll_deg', 'imu_pitch_deg', 'imu_yaw_deg'])
        self.gt = None
        self.imu = None
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 10)
        self.create_subscription(Imu, '/rtk/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, math.degrees(rpy(p.orientation)[2]))

    def on_imu(self, m):
        self.imu = rpy(m.orientation)

    def tick(self):
        if self.gt is None or self.imu is None:
            return
        self.w.writerow([round(time.time(), 3), round(self.gt[0], 4),
                         round(self.gt[1], 4), round(self.gt[2], 2),
                         round(self.imu[0], 2), round(self.imu[1], 2),
                         round(self.imu[2], 2)])
        self.f.flush()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--csv', required=True)
    p.add_argument('--seconds', type=float, default=420)
    a = p.parse_args()
    rclpy.init()
    n = Log(a.csv)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

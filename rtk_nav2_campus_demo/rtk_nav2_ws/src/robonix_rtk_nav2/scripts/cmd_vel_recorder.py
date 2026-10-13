#!/usr/bin/env python3
"""记录 `/cmd_vel`（wall 时间 + linear.x/angular.z），用于 card ③：
卡死前"控制器有没有把速度收到 0"。只订阅、不发布。"""
import argparse
import csv
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--seconds', type=float, default=1500)
    a = ap.parse_args()
    rclpy.init()
    n = Node('cmd_vel_recorder')
    f = open(a.csv, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['wall', 'linear_x', 'angular_z'])
    f.flush()

    def cb(m):
        w.writerow([round(time.time(), 3), round(m.linear.x, 4), round(m.angular.z, 4)])
        f.flush()
    n.create_subscription(Twist, '/cmd_vel', cb, qos_profile_sensor_data)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

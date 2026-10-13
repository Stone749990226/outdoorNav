#!/usr/bin/env python3
"""量"goal_checker 眼里的距离"：`/controller_server/local_plan` 末端 vs 机器人 odom 位姿。

reviewer 的定案方案（`rev7-ramp-e3` 之后）：同刻 map 系与 odom 系**距离相同**（刚体变换），
所以"容差落在 odom"本身解释不了 0.33 m；剩下的可能是**时间错配**——
checker 手里的"目标在 odom 的位置"是路径被折算进代价地图系（=odom）那一刻的 `map→odom`，
而机器人 odom 位姿是另一刻的 ⇒ 两者之差 = 错配量。
做法（不改任何配置）：
  1. 订阅 `/controller_server/local_plan`（RPP 发出的、已折算到代价地图系的路径）记其**末端点**；
  2. 同刻 `lookup_transform("odom","base_footprint",Time())` 与 `("map","odom",Time())`；
  3. 打印 checker 距离 = |local_plan 末点 − 机器人 odom| 与 map 系真值距离并排。
按 20 Hz 落 CSV。
"""
import argparse
import csv
import math
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# nav2 的 local_plan 用 SystemDefaultsQoS（volatile/reliable）发布
PLAN_Q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)


class Probe(Node):
    def __init__(self, goal, csv_path):
        super().__init__('local_plan_probe')
        self.goal = goal
        self.path = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Path, '/controller_server/local_plan', self.on_path, PLAN_Q)
        self.f = open(csv_path, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        self.w.writerow(['wall', 'plan_frame', 'plan_end_x', 'plan_end_y', 'plan_n',
                         'map_robot_x', 'map_robot_y', 'odom_robot_x', 'odom_robot_y',
                         'checker_d', 'map_d'])
        self.f.flush()
        self.create_timer(0.05, self.tick)

    def on_path(self, m):
        self.path = m

    def tick(self):
        try:
            ob = self.tf.lookup_transform('odom', 'base_footprint', Time())
            mo = self.tf.lookup_transform('map', 'odom', Time())
            mb = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return
        ox, oy = ob.transform.translation.x, ob.transform.translation.y
        mx, my = mb.transform.translation.x, mb.transform.translation.y
        pe = None
        frame = ''
        if self.path is not None and self.path.poses:
            frame = self.path.header.frame_id
            p = self.path.poses[-1].pose.position
            pe = (p.x, p.y)
        checker = None
        if pe is not None and frame == 'odom':
            checker = math.hypot(pe[0] - ox, pe[1] - oy)
        self.w.writerow([round(time.time(), 3), frame,
                         None if pe is None else round(pe[0], 4),
                         None if pe is None else round(pe[1], 4),
                         None if self.path is None else len(self.path.poses),
                         round(mx, 4), round(my, 4), round(ox, 4), round(oy, 4),
                         None if checker is None else round(checker, 4),
                         round(math.hypot(mx - self.goal[0], my - self.goal[1]), 4)])
        self.f.flush()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--goal-x', type=float, default=2.0)
    p.add_argument('--goal-y', type=float, default=2.0)
    p.add_argument('--csv', required=True)
    p.add_argument('--seconds', type=float, default=420)
    a = p.parse_args()
    rclpy.init()
    n = Probe((a.goal_x, a.goal_y), a.csv)
    # 先空转 8 s 让 TF buffer 填满（reviewer 提醒：lookup 前必须真 spin 够）
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

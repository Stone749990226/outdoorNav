#!/usr/bin/env python3
"""到达判定用的是哪个坐标系？——同刻记 map 系与 odom 系两组距离（阶段七 ① 用）。

reviewer 的怀疑（`rev7-ramp-e3`）：goal_checker 的容差是在**它所在代价地图的坐标系**
（本配置里局部图的 `global_frame: odom`）里判的，而 odom 在坡上有轮式幻走 ⇒
"odom 系距离"可能小于"map 系真距离"，于是容差内达标、真值仍在 0.3 m 外。
本探针每秒落一行：机器人（map/odom 两系）、目标 (2,2)（map/odom 两系）、当前路径末端（两系），
以及两系下的距离。到站那几行就是判据。
"""
import argparse
import csv
import math
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def apply(tf, x, y):
    r, t = rot(tf.transform.rotation), tf.transform.translation
    return (r[0][0]*x + r[0][1]*y + t.x, r[1][0]*x + r[1][1]*y + t.y)


class Probe(Node):
    def __init__(self, goal, csv_path, period):
        super().__init__('goal_frame_probe')
        self.goal = goal
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.odom = None
        self.path = None
        self.state = ''
        # 目标 = 机器人里程计系里的"目标点"会随 map→odom 漂移；checker 眼里的距离用的是
        # **路径被折算那一刻**的 map→odom。所以除了位姿，还要记路径的 header 时间戳，
        # 才能算 reviewer 要的"错配量 (b)"。
        self.create_subscription(String, '/rtk/goal_navigator_state',
                                 lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(Odometry, '/odometry/local',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Path, '/rtk/plan_path',
                                 lambda m: setattr(self, 'path', m), LATCHED)
        self.f = open(csv_path, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        self.w.writerow(['wall', 'sim', 'map_robot_x', 'map_robot_y', 'odom_robot_x',
                         'odom_robot_y', 'map_to_odom_dx', 'map_to_odom_dy',
                         'goal_map_d', 'goal_odom_d', 'path_stamp', 'path_end_map_x',
                         'path_end_map_y', 'state'])
        self.f.flush()
        self.warned = False
        self.create_timer(period, self.tick)

    def look(self, target, source):
        try:
            return self.tf.lookup_transform(target, source, Time())
        except Exception:
            return None

    def tick(self):
        sim = self.get_clock().now().nanoseconds * 1e-9
        mb = self.look('map', 'base_footprint')
        ob = self.look('odom', 'base_footprint')
        mo = self.look('map', 'odom')
        if mb is None or ob is None or mo is None:
            if not self.warned:
                self.get_logger().warn('TF 还没齐（map/base=%s odom/base=%s map/odom=%s）'
                                       % (mb is not None, ob is not None, mo is not None))
                self.warned = True
            return
        mx, my = mb.transform.translation.x, mb.transform.translation.y
        ox, oy = ob.transform.translation.x, ob.transform.translation.y
        gx, gy = self.goal
        g_odom = apply(mo, gx, gy)              # 目标在 odom 系的位置
        pe_map = pe_odom = (None, None)
        if self.path is not None and self.path.poses:
            p = self.path.poses[-1].pose.position
            if self.path.header.frame_id == 'map':
                pe_map = (p.x, p.y)
                pe_odom = apply(mo, p.x, p.y)
            else:
                pe_odom = (p.x, p.y)
        d = lambda a, b: None if None in a else round(math.hypot(a[0]-b[0], a[1]-b[1]), 4)
        # map→odom 的平移（刚体+小角度，用位姿差近似已足够看漂移方向/大小）
        mo_t = mo.transform.translation
        stamp = None
        if self.path is not None:
            stamp = self.path.header.stamp.sec + self.path.header.stamp.nanosec * 1e-9
        self.w.writerow([round(time.time(), 3), round(sim, 3), round(mx, 4), round(my, 4),
                         round(ox, 4), round(oy, 4), round(mo_t.x, 5), round(mo_t.y, 5),
                         d((mx, my), (gx, gy)), d((ox, oy), g_odom),
                         None if stamp is None else round(stamp, 3),
                         None if pe_map[0] is None else round(pe_map[0], 4),
                         None if pe_map[1] is None else round(pe_map[1], 4), self.state])
        self.f.flush()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--goal-x', type=float, default=2.0)
    p.add_argument('--goal-y', type=float, default=2.0)
    p.add_argument('--csv', required=True)
    p.add_argument('--seconds', type=float, default=420)
    p.add_argument('--period', type=float, default=1.0)
    a = p.parse_args()
    rclpy.init()
    n = Probe((a.goal_x, a.goal_y), a.csv, a.period)
    # TF buffer 要先真 spin 够（上一轮就是这里栽的）
    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

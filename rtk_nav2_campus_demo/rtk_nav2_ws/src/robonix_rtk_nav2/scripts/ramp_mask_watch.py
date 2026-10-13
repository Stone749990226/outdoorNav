#!/usr/bin/env python3
"""按区域统计 mask / 两张真实 costmap 的致命格数（阶段七 ramp 复现用）。

reviewer 的独立复跑（rev7-ramp-e1）报了"车停在坡口时 mask 在**坡面矩形内**出现 29 格"。
本工具把同一时刻的三张图**分区**统计，避免只看一张图或只看全场总数：
  · cliff  带：x∈[-0.25, 0.25]、y∈[-3.0, 0.5]（断崖应当有格）
  · ramp   带：x∈[ 0.00, 1.65]、y∈[ 0.5, 3.0]（坡面应当**始终 0 格**）
  · upper  带：x∈[-4.0,-0.20]、y∈[-3.0, 3.0]（高台平地应当 0 格）
odom 帧的图先经 TF 换到 map 再判区。每 --interval 秒打一行 CSV。
"""
import argparse
import csv
import time

import numpy as np
import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE)
TOPICS = [('/rtk/drop/mask_map', 'mask', OccupancyGrid),
          ('/global_costmap/costmap', 'global_occ', OccupancyGrid),
          ('/global_costmap/costmap_raw', 'global_raw', Costmap),
          ('/local_costmap/costmap', 'local_occ', OccupancyGrid),
          ('/local_costmap/costmap_raw', 'local_raw', Costmap)]
ZONES = {'cliff': (-0.25, 0.25, -3.0, 0.5),
         'ramp': (0.0, 1.65, 0.5, 3.0),
         'upper': (-4.0, -0.20, -3.0, 3.0)}


def meta(m):
    return m.metadata if hasattr(m, 'metadata') else m.info


def grid(m):
    i = meta(m)
    h = int(getattr(i, 'size_y', getattr(i, 'height', 0)))
    w = int(getattr(i, 'size_x', getattr(i, 'width', 0)))
    return np.asarray(m.data, dtype=np.int16).reshape(h, w), i


def rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Watch(Node):
    def __init__(self):
        super().__init__('ramp_mask_watch')
        self.d = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        for topic, key, typ in TOPICS:
            self.create_subscription(typ, topic,
                                     lambda m, k=key: self.d.__setitem__(k, m), Q)

    def counts(self, key):
        m = self.d.get(key)
        if m is None:
            return None
        a, i = grid(m)
        res = float(i.resolution)
        lethal = 254 if key.endswith('raw') else 100
        to_map = m.header.frame_id != 'map'
        R = T = None
        if to_map:
            try:
                tr = self.tf.lookup_transform('map', m.header.frame_id, Time())
            except Exception:
                return {'tf': 'fail'}
            R = rot(tr.transform.rotation)
            T = np.array([tr.transform.translation.x, tr.transform.translation.y])
        yy, xx = np.where(a >= lethal)
        px = i.origin.position.x + (xx + 0.5) * res
        py = i.origin.position.y + (yy + 0.5) * res
        if to_map and len(px):
            T3 = np.array([T[0], T[1], 0.0])
            p = (R @ np.c_[px, py, np.zeros(len(px))].T).T + T3
            px, py = p[:, 0], p[:, 1]
        out = {}
        for zone, (x0, x1, y0, y1) in ZONES.items():
            out[zone] = int(np.count_nonzero(
                (px >= x0) & (px <= x1) & (py >= y0) & (py <= y1))) if len(px) else 0
        return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--csv', required=True)
    p.add_argument('--seconds', type=float, default=300)
    p.add_argument('--interval', type=float, default=2.0)
    a = p.parse_args()
    rclpy.init()
    n = Watch()
    f = open(a.csv, 'w', newline='', encoding='utf-8')
    w = csv.writer(f)
    cols = [k for _, k, _ in TOPICS]
    w.writerow(['wall'] + ['%s_%s' % (k, z) for k in cols for z in ZONES])
    end = time.time() + a.seconds
    last = 0.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - last >= a.interval and n.d:
            last = time.time()
            row = [round(last, 3)]
            for k in cols:
                c = n.counts(k) or {}
                row.extend([c.get(z, -1) if isinstance(c, dict) else -1 for z in ZONES])
            w.writerow(row)
            f.flush()
    f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

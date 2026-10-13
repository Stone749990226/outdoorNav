#!/usr/bin/env python3
"""断崖 hazard 的两图格值探针（阶段七验收用）。

对给定的 y 列表，在断崖线 x≈0 附近的一个小窗口里取**最大值**，分别报：
  · mask_map / mask_odom（检测器发布的 Keepout 掩膜，100 = 禁行）
  · /global_costmap/costmap（真实 OccupancyGrid，map 帧）
  · /global_costmap/costmap_raw（真实 Costmap，254 = LETHAL）
  · /local_costmap/costmap、/local_costmap/costmap_raw（odom 帧；先经 TF 换到 map 再取样）

窗口而不是单点：掩膜格心可能落在 x=-0.007 或 0.043，逐点取样会漏。
输出 JSON 以便和别处的证据对表；用法：
    cliff_probe.py --label before --json /work/artifacts/rtk-nav2/<run>/cliff_before.json
"""
import argparse
import json
import math
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
TOPICS = [('/rtk/drop/mask_map', 'mask_map', OccupancyGrid),
          ('/rtk/drop/mask_odom', 'mask_odom', OccupancyGrid),
          ('/global_costmap/costmap', 'global_occ', OccupancyGrid),
          ('/local_costmap/costmap', 'local_occ', OccupancyGrid),
          ('/global_costmap/costmap_raw', 'global_raw', Costmap),
          ('/local_costmap/costmap_raw', 'local_raw', Costmap)]


def meta(msg):
    return msg.metadata if hasattr(msg, 'metadata') else msg.info


def grid(msg):
    i = meta(msg)
    h = int(getattr(i, 'size_y', getattr(i, 'height', 0)))
    w = int(getattr(i, 'size_x', getattr(i, 'width', 0)))
    return np.asarray(msg.data, dtype=np.int16).reshape(h, w), i


def quat_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Probe(Node):
    def __init__(self):
        super().__init__('cliff_probe')
        self.d = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        for topic, key, typ in TOPICS:
            self.create_subscription(typ, topic,
                                     lambda m, k=key: self.d.__setitem__(k, m), Q)

    def window_max(self, key, y, tf_map):
        """在 map 帧的 (x∈[-0.25,0.25], y±0.15) 窗口里取最大值。"""
        msg = self.d.get(key)
        if msg is None:
            return None
        a, i = grid(msg)
        res = float(i.resolution)
        lethal = 254 if key.endswith('raw') else 100
        xs, ys, vals = [], [], []
        for row in range(a.shape[0]):
            for col in range(a.shape[1]):
                v = int(a[row, col])
                if v <= 0:
                    continue
                px = i.origin.position.x + (col + 0.5) * res
                py = i.origin.position.y + (row + 0.5) * res
                if msg.header.frame_id != 'map':
                    if tf_map is None:
                        continue
                    px, py = tf_map(px, py)
                if abs(px) <= 0.25 and abs(py - y) <= 0.15:
                    xs.append(px); ys.append(py); vals.append(v)
        if not vals:
            return {'cells': 0, 'max': 0, 'max_at': None,
                    'lethal_cells': 0, 'frame': msg.header.frame_id}
        k = int(np.argmax(vals))
        return {'cells': len(vals), 'max': int(vals[k]),
                'max_at': [round(xs[k], 3), round(ys[k], 3)],
                'lethal_cells': int(sum(1 for v in vals if v >= lethal)),
                'frame': msg.header.frame_id}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--json', required=True)
    p.add_argument('--label', default='')
    p.add_argument('--ys', default='-2.0,-1.5,-1.0,-0.5,0.0,0.5,0.9')
    p.add_argument('--seconds', type=float, default=20.0)
    args = p.parse_args()
    ys = [float(v) for v in args.ys.split(',')]
    rclpy.init()
    node = Probe()
    end = time.time() + args.seconds
    while time.time() < end and len(node.d) < len(TOPICS):
        rclpy.spin_once(node, timeout_sec=0.1)

    def tf_map(px, py):
        try:
            tr = node.tf.lookup_transform('map', 'odom', Time())
        except Exception:
            return px, py
        r = quat_rot(tr.transform.rotation)
        t = tr.transform.translation
        out = r @ np.array([px, py, 0.0]) + np.array([t.x, t.y, t.z])
        return float(out[0]), float(out[1])

    out = {'label': args.label, 'wall': time.time(), 'cliff_y': {},
           'note': '窗口 x∈[-0.25,0.25]、y±0.15 内取最大值；ld 为 254(raw)/100(occ)'}
    for y in ys:
        out['cliff_y'][str(y)] = {key: node.window_max(key, y, tf_map)
                                 for _, key, _ in TOPICS}
    with open(args.json, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""按框数 mask 的致命格（实验 P 用）：`/rtk/drop/mask_map` 在给定 x/y 区间内的 >=100 格数。

用法：mask_zone.py --json out.json --boxes "0.1,1.5,0.40,0.60" --boxes "0.1,1.5,0.60,0.90"
每个框打印 致命格数 + 该框内致命格的实际 y 范围（判"标到哪了"）。
只读，不改任何节点。
"""
import argparse
import json
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--boxes', action='append', required=True,
                    help='x0,x1,y0,y1')
    ap.add_argument('--seconds', type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    n = Node('mask_zone')
    d = {}
    n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map',
                          lambda m: d.__setitem__('m', m), Q)
    t = time.time()
    while time.time() - t < a.seconds and 'm' not in d:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = d.get('m')
    out = {'has_mask': m is not None, 'stamp': round(time.time(), 3)}
    if m is not None:
        i = m.info
        res = i.resolution
        g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
        X = i.origin.position.x + (np.arange(i.width) + 0.5) * res
        Y = i.origin.position.y + (np.arange(i.height) + 0.5) * res
        leth = g >= 100
        out['lethal_total'] = int(leth.sum())
        out['boxes'] = {}
        for spec in a.boxes:
            x0, x1, y0, y1 = [float(v) for v in spec.split(',')]
            mx = (X >= x0) & (X <= x1)
            my = (Y >= y0) & (Y <= y1)
            sub = leth[np.ix_(my, mx)]
            rr, cc = np.nonzero(sub)
            out['boxes'][spec] = {
                'cells': int(sub.sum()),
                'y_lo': None if sub.sum() == 0 else round(float(Y[my][rr].min()), 3),
                'y_hi': None if sub.sum() == 0 else round(float(Y[my][rr].max()), 3),
                'min_x_in_box': None if sub.sum() == 0 else round(float(X[mx][cc].min()), 3),
            }
    with open(a.json, 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

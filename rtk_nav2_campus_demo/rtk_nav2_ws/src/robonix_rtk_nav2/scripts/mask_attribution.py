#!/usr/bin/env python3
"""100% 逐格归属：把 mask 的**每一个**格按 §7.18 的真落差清单算距离（不再用分区框）。

reviewer 2026-10-08 要求：上一轮我用分区框，只覆盖 164/638 格（26%）⇒ "分区外无新增"根本没被覆盖 ✗。
本脚本对**全部格**逐格判定，并落一份逐格表（格坐标 / 最近线段编号 / 距离）+ 汇总。

用法：mask_attribution.py --json out.json [--tol-dilated 0.35] [--points-file 未外扩点文件(可选)]
  · 未外扩 `hazard_points` 文件（每行 "x y"）若给，则权威口径 = 到清单 ≤0.05 m；
  · 否则用已发布 mask，口径 = 到清单 ≤tol-dilated（= hazard_margin 0.30 + 0.05 余量）。
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE)

# ★ 真落差清单 = **单一真源** `world_edges.py`（2026-10-08 改场景后重建）。
#   先前本文件自带一份副本 ⇒ 三个脚本各一份、改场景要改三处、极易漂移 ✗ ⇒ 已收敛 ✓。
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import world_edges as WE  # noqa: E402

SEGMENTS = WE.SEGMENTS
NOT_EDGES = WE.NOT_EDGES   # ★ 负向清单（0 落差、**不应当被标**）


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den <= 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / den))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def nearest(p):
    best = (None, 1e9)
    for name, (a, b) in SEGMENTS:
        d = seg_dist(p, a, b)
        if d < best[1]:
            best = (name, d)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--tol-dilated', type=float, default=0.35)
    ap.add_argument('--points-file', default='')
    ap.add_argument('--seconds', type=float, default=25.0)
    a = ap.parse_args()

    pts = []
    src = ''
    if a.points_file:
        for line in open(a.points_file):
            v = line.split()
            if len(v) >= 2:
                try:
                    pts.append((float(v[0]), float(v[1])))
                except ValueError:
                    pass
        src = 'undilated:%s' % a.points_file
    if not pts:
        rclpy.init()
        n = Node('mask_attribution')
        d = {}
        n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map',
                              lambda m: d.__setitem__('m', m), Q)
        t = time.time()
        while time.time() - t < a.seconds and 'm' not in d:
            rclpy.spin_once(n, timeout_sec=0.2)
        m = d.get('m')
        if m is not None:
            i = m.info
            g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
            xs = i.origin.position.x + (np.arange(i.width) + 0.5) * i.resolution
            ys = i.origin.position.y + (np.arange(i.height) + 0.5) * i.resolution
            for iy, ix in zip(*np.nonzero(g >= 100)):
                pts.append((float(xs[ix]), float(ys[iy])))
            src = 'published_mask(res=%.3f)' % i.resolution
        n.destroy_node()
        rclpy.shutdown()

    tol = 0.05 if a.points_file else a.tol_dilated
    rows = []
    inside = outside = 0
    for p in pts:
        name, dist = nearest(p)
        ok = dist <= tol
        rows.append({'x': round(p[0], 3), 'y': round(p[1], 3), 'seg': name,
                     'dist': round(dist, 3), 'in_inventory': bool(ok)})
        inside += int(ok)
        outside += int(not ok)
    out = {'source': src, 'n_cells': len(pts), 'tol_m': tol,
           'inside_inventory': inside, 'outside_inventory': outside,
           'outside_cells': [r for r in rows if not r['in_inventory']][:200],
           'per_cell': rows}
    with open(a.json, 'w') as f:
        json.dump(out, f, indent=1)
    print('源=%s  格数=%d  容差=%.2f m' % (src, len(pts), tol))
    print('清单内 %d 格（%.0f%%）  清单外 %d 格（%.0f%%）'
          % (inside, 100.0 * inside / max(1, len(pts)),
             outside, 100.0 * outside / max(1, len(pts))))
    if outside:
        print('清单外格（前 12）：', [(r['x'], r['y'], r['dist'], r['seg']) for r in rows
                                     if not r['in_inventory']][:12])


if __name__ == '__main__':
    main()

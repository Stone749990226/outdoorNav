#!/usr/bin/env python3
"""★ reviewer 2026-10-08 要求的新读数：**规划路径 vs 南沿**。

回答"**规划路径是否越过了未标注的南沿**"，把"地图缺格"与"车真的开过去"连成因果链：
  1. 计划路径（`/rtk/plan_path`，latched）**穿 `y = 0.5` 处的 `x`**（= 从南沿哪一段切过去）；
  2. 计划路径 ↔ **已发布 mask** 的最小距离（外扩后口径）；
  3. 计划路径 ↔ **真落差清单**（§7.18）的最小距离，并指出命中的是哪条线段；
  4. 一并落盘路径点，便于与 mask 叠加看图。

用法：plan_vs_edge.py --json out.json [--cliff-x 0.0]
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE)
SEGMENTS = [
    ('1 断崖(高台东沿)', [(0.0, -3.0), (0.0, 0.5)]),
    ('2 高台西沿', [(-4.0, -3.0), (-4.0, 3.0)]),
    ('3 高台北沿', [(-4.0, 3.0), (0.0, 3.0)]),
    ('4 高台南沿', [(-4.0, -3.0), (0.0, -3.0)]),
    ('5 坡南侧沿', [(0.0, 0.5), (1.6, 0.5)]),
    ('6 坡北侧沿', [(0.0, 3.0), (1.6, 3.0)]),
]


def seg_dist(p, a, b):
    ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den <= 1e-12 else max(0.0, min(1.0, ((p[0]-ax)*dx + (p[1]-ay)*dy) / den))
    return math.hypot(p[0]-(ax+t*dx), p[1]-(ay+t*dy)), (ax+t*dx, ay+t*dy)


def min_to_segments(p):
    best = (None, 1e9, None)
    for name, (a, b) in SEGMENTS:
        d, foot = seg_dist(p, a, b)
        if d < best[1]:
            best = (name, d, foot)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--seconds', type=float, default=25.0)
    ap.add_argument('--mask-seconds', type=float, default=60.0,
                    help='等 latched mask 的上限（reviewer 要求：快照时 mask 常没到 ⇒ 加长）')
    a = ap.parse_args()
    rclpy.init()
    n = Node('plan_vs_edge')
    d = {}
    n.create_subscription(Path, '/rtk/plan_path', lambda m: d.__setitem__('p', m), Q)
    n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map',
                          lambda m: d.__setitem__('m', m), Q)
    t = time.time()
    while time.time() - t < a.seconds and 'p' not in d:
        rclpy.spin_once(n, timeout_sec=0.2)
    # 再等一会儿 mask（plan 到了、mask 还没到的情形上一轮出现过两次 ✗）
    t2 = time.time()
    while time.time() - t2 < a.mask_seconds and 'm' not in d:
        rclpy.spin_once(n, timeout_sec=0.2)
    pth = d.get('p')
    out = {'has_plan': pth is not None, 'has_mask': 'm' in d}
    if pth is not None:
        pts = [(float(q.pose.position.x), float(q.pose.position.y)) for q in pth.poses]
        out['frame'] = pth.header.frame_id
        out['n_points'] = len(pts)
        out['path'] = [[round(x, 3), round(y, 3)] for x, y in pts]
        # 1) 穿 y=0.5 的 x（取第一处跨越的插值）
        cross = []
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            if (y1 - 0.5) * (y2 - 0.5) <= 0 and abs(y2 - y1) > 1e-9:
                t_ = (0.5 - y1) / (y2 - y1)
                cross.append(round(x1 + t_ * (x2 - x1), 3))
        out['cross_x_at_y0.5'] = cross
        # ★ reviewer 2026-10-08：报穿点 x 时必须同时给**该处台阶高 H**（否则因果链两义：
        #   H<drop_threshold ⇒ 再降阈也不会醒，那是另一种结论）
        hdef = lambda xx: round(0.15 - 0.18752 * (xx - 0.8), 4)
        out['cross_x_with_H'] = [{'x': xc, 'H_m': hdef(xc),
                                  'H_ge_0.15': bool(hdef(xc) >= 0.15),
                                  'H_ge_0.10': bool(hdef(xc) >= 0.10)} for xc in cross]
        # 该处的 mask 格数（以穿点为圆心 0.35 m 内的致命格）
        m2 = d.get('m')
        if m2 is not None and cross:
            i2 = m2.info
            g2 = np.asarray(m2.data, dtype=np.int16).reshape(i2.height, i2.width)
            ys2, xs2 = np.nonzero(g2 >= 100)
            if len(xs2):
                mx2 = i2.origin.position.x + (xs2 + 0.5) * i2.resolution
                my2 = i2.origin.position.y + (ys2 + 0.5) * i2.resolution
                M2 = np.stack([mx2, my2], axis=1)
                out['mask_cells_near_crossing'] = [
                    {'x': xc, 'y': 0.5, 'r': 0.35,
                     'cells': int(np.sum(np.linalg.norm(M2 - np.array([xc, 0.5]), axis=1) <= 0.35)),
                     'nearest_cell_m': round(float(np.min(np.linalg.norm(
                         M2 - np.array([xc, 0.5]), axis=1))), 4)}
                    for xc in cross]
                out['mask_cells_near_crossing_note'] = '已测（口径：穿点 (x,0.5) 为中心、半径 0.35 m 内的致命格）'
            else:
                out['mask_cells_near_crossing'] = []
        else:
            # ★ 语义修正（reviewer 2026-10-08 指出"产物与叙述对不上"的根因）：
            #   **mask 没收到 = "未测"** ⇒ 用 None，**不要用 []**（[] 会被读成"0 格" ✗）
            out['mask_cells_near_crossing'] = None
            out['mask_cells_near_crossing_note'] = 'mask 未在等待窗口内到达 ⇒ 未测（不是 0 格）'
        # 2) 路径 ↔ 已发布 mask 的最小距离
        m = d.get('m')
        if m is not None:
            i = m.info
            g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
            ys, xs = np.nonzero(g >= 100)
            if len(xs):
                mx = i.origin.position.x + (xs + 0.5) * i.resolution
                my = i.origin.position.y + (ys + 0.5) * i.resolution
                M = np.stack([mx, my], axis=1)
                best = 1e9
                for p in pts:
                    dd = np.min(np.linalg.norm(M - np.asarray(p), axis=1))
                    best = min(best, float(dd))
                out['plan_min_dist_to_mask_m'] = round(best, 4)
            else:
                out['plan_min_dist_to_mask_m'] = None
        # 3) 路径 ↔ 真落差清单
        best = (None, 1e9, None, None)
        for p in pts:
            name, dd, foot = min_to_segments(p)
            if dd < best[1]:
                best = (name, dd, foot, p)
        out['plan_min_dist_to_inventory_m'] = round(best[1], 4)
        out['plan_closest_segment'] = best[0]
        out['plan_closest_path_point'] = None if best[3] is None else [round(best[3][0], 3), round(best[3][1], 3)]
        out['plan_closest_foot'] = None if best[2] is None else [round(best[2][0], 3), round(best[2][1], 3)]
    with open(a.json, 'w') as f:
        json.dump(out, f, indent=1)
    # 字段存在性自检（reviewer 要求：上一轮 plan_seg_B 缺字段 ✗）
    need = ['n_points', 'cross_x_at_y0.5', 'mask_cells_near_crossing',
            'plan_min_dist_to_inventory_m']
    missing = [k for k in need if k not in out]
    if pth is not None and 'm' not in d:
        missing.append('plan_min_dist_to_mask_m(因 mask 未到)')
    out['missing_fields'] = missing
    print(json.dumps({k: v for k, v in out.items() if k != 'path'}, indent=1, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

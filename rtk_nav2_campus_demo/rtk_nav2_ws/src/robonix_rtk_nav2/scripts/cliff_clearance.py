#!/usr/bin/env python3
"""§7.8 的 ②/④/① 三项：计划路径/真值轨迹到崖线的距离 + 全局图断崖带 hazard 数。

崖线 = x=0, y∈[-3,0.5]（"未外扩"的等价物：mask 的原始格就落在这条线附近 ±0.04 m，
既往多轮实测 x∈[-0.02,0.04]；发布的 mask 现已被外扩，故用崖线本身当基准，同时也报"到已发布 mask"的距离）。
用法：cliff_clearance.py --json out.json [--traj /work/artifacts/.../trajectory.csv]
"""
import argparse, csv, json, math, time
import numpy as np, rclpy
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
CLIFF = [(0.0, y) for y in np.arange(-3.0, 0.5 + 1e-9, 0.05)]


def seg_dist(p, a, b):
    ax, ay = a; bx, by = b; dx, dy = bx - ax, by - ay
    den = dx*dx + dy*dy
    t = 0.0 if den <= 1e-12 else max(0.0, min(1.0, ((p[0]-ax)*dx + (p[1]-ay)*dy)/den))
    return math.hypot(p[0]-(ax+t*dx), p[1]-(ay+t*dy))


def min_dist_to_cliff(pts):
    return min(seg_dist(p, CLIFF[i], CLIFF[i+1]) for p in pts for i in range(len(CLIFF)-1)) if len(pts) else None


def min_dist_to_cells(pts, cells):
    if not len(pts) or not len(cells):
        return None
    return float(min(math.hypot(p[0]-c[0], p[1]-c[1]) for p in pts for c in cells))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--traj', default='')
    ap.add_argument('--seconds', type=float, default=25.0)
    a = ap.parse_args()
    rclpy.init(); n = Node('cliff_clearance'); d = {}
    n.create_subscription(Path, '/rtk/plan_path', lambda m: d.__setitem__('p', m), Q)
    n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map', lambda m: d.__setitem__('k', m), Q)
    n.create_subscription(Costmap, '/global_costmap/costmap_raw', lambda m: d.__setitem__('g', m), Q)
    t = time.time()
    while time.time() - t < a.seconds and len(d) < 3:
        rclpy.spin_once(n, timeout_sec=0.2)
    out = {}
    if 'p' in d and d['p'].poses:
        pts = [(q.pose.position.x, q.pose.position.y) for q in d['p'].poses]
        out['plan_points'] = len(pts)
        out['plan_min_dist_to_cliff_m'] = round(min_dist_to_cliff(pts), 4)
    if 'k' in d:
        m = d['k']; i = m.info
        g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
        yy, xx = np.where(g >= 100)
        cells = np.c_[i.origin.position.x + (xx + 0.5) * i.resolution,
                      i.origin.position.y + (yy + 0.5) * i.resolution] if len(xx) else np.zeros((0, 2))
        out['mask_cells'] = int(len(cells))
        out['mask_min_x'] = round(float(cells[:, 0].min()), 3) if len(cells) else None
        if 'p' in d and d['p'].poses:
            pts = [(q.pose.position.x, q.pose.position.y) for q in d['p'].poses]
            out['plan_min_dist_to_mask_m'] = round(min_dist_to_cells(pts, cells), 4)
    if 'g' in d:
        m = d['g']; i = m.metadata
        g = np.asarray(m.data, dtype=np.int16).reshape(int(i.size_y), int(i.size_x))
        X = i.origin.position.x + (np.arange(int(i.size_x)) + 0.5) * i.resolution
        Y = i.origin.position.y + (np.arange(int(i.size_y)) + 0.5) * i.resolution
        band = (np.abs(X) <= 0.25) & (Y <= 0.5) & (Y >= -3.0)
        out['global_cliff_band_lethal'] = int(g[np.ix_(np.where((Y <= 0.5) & (Y >= -3.0))[0],
                                                       np.where(np.abs(X) <= 0.25)[0])].astype(np.int16).__eq__(254).sum())
    if a.traj:
        rows = [(float(r['gt_x']), float(r['gt_y'])) for r in csv.DictReader(open(a.traj)) if r.get('gt_x')]
        out['traj_points'] = len(rows)
        out['traj_min_dist_to_cliff_m'] = round(min_dist_to_cliff(rows), 4) if rows else None
        out['traj_max_x'] = round(max(p[0] for p in rows), 3) if rows else None
        if 'k' in d and len(xx):
            out['traj_min_dist_to_mask_m'] = round(min_dist_to_cells(rows, cells), 4)
    with open(a.json, 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

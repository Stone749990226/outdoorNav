#!/usr/bin/env python3
"""给验收脚本算"机器人正前方 d 米、净空最好的那个点"（放临时矮障碍用）。

为什么要挑净空：紧盒（±0.35 m）读数的口径要求**障碍离真实障碍表面 >0.8 m**，
否则盒子里会吃到墙的膨胀带，读数被污染（第三阶段实测踩过一次）。
所以这里在机器人当前朝向前方 ±25° 内试几个方位，取"离最近真实障碍表面最远"的那个。

用法：
  nav_target_point.py <trajectory.csv> <distance_m> [--pkg-scripts <dir>]
输出：一行 "x y"（map 坐标，米）。位姿取 trajectory.csv 的**最后一行**第 2/3/4 列。
"""

import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_geometry as G  # noqa: E402


def last_pose(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r['gt_x']), float(r['gt_y']), float(r['gt_yaw'])))
            except (KeyError, TypeError, ValueError):
                continue
    if not rows:
        raise SystemExit('** %s 里没有可用的真值行 **' % path)
    x, y, yaw = rows[-1]
    deg = math.degrees(yaw) if abs(yaw) <= math.pi + 1e-6 else yaw
    return x, y, deg


def los_distance(x, y, az_deg, max_range, step=0.05):
    """从 (x,y) 沿 az_deg 走，返回**第一个真实障碍**的距离；走满则返回 max_range。

    **必须查通视**：只用"落点净空"是不够的——实测踩到过：车在 (2.6,-2.6) 朝 +x 时，
    1.33 m 处就有 obstacle_00 那面独立墙，4 m 的落点虽然自己净空很好，
    却被那面墙挡在视线之外（激光回波给的是 1.3252 m，不是 3.8 m）。
    """
    th = math.radians(az_deg)
    d = step
    while d <= max_range:
        if G.nearest_surface(x + math.cos(th) * d, y + math.sin(th) * d) <= 0.0:
            return d
        d += step
    return max_range


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    ap.add_argument('distance', type=float)
    ap.add_argument('--spread-deg', type=float, default=20.0,
                    help='在前方 ±这个角度内挑方位（默认 20°：相机视场 ±30°，留一点余量）')
    ap.add_argument('--clearance-m', type=float, default=0.8,
                    help='落点离真实障碍表面的最小净空（默认 0.8 m，紧盒才不会被墙污染）')
    ap.add_argument('--los-margin-m', type=float, default=1.0,
                    help='通视上的余量：机器人到落点的连线上，障碍至少要再远这么多米')
    args = ap.parse_args(argv)

    x, y, yaw = last_pose(args.trajectory)
    need_los = args.distance + args.los_margin_m
    best = None
    cands = []
    k = -args.spread_deg
    while k <= args.spread_deg + 1e-9:
        az = yaw + k
        px, py = x + math.cos(math.radians(az)) * args.distance, y + math.sin(math.radians(az)) * args.distance
        clear = G.nearest_surface(px, py)
        los = los_distance(x, y, az, max(need_los + 0.5, 9.0))
        cands.append((k, clear, los))
        ok = (clear >= args.clearance_m) and (los >= need_los)
        if ok and (best is None or clear > best[2]):
            best = (px, py, clear, k, los)
        k += 5.0
    if best is None:
        # 没有满足条件的方位：退而求其次，取"通视最远"的那个，并把诊断打出来（由调用方决定要不要用）
        cands.sort(key=lambda t: (-t[2], -t[1]))
        k, clear, los = cands[0]
        az = yaw + k
        px = x + math.cos(math.radians(az)) * args.distance
        py = y + math.sin(math.radians(az)) * args.distance
        print('%.3f %.3f' % (px, py))
        print('# ** 没有方位同时满足 净空>=%.2f 且 通视>=%.2f **；退化为通视最远的方位'
              % (args.clearance_m, need_los), file=sys.stderr)
        print('# 机器人 (%.2f, %.2f, %.1f°)，距离 %.2f m，偏 %+.0f°，净空 %.2f m，通视 %.2f m'
              % (x, y, yaw, args.distance, k, clear, los), file=sys.stderr)
        for k2, c2, l2 in cands[:6]:
            print('#   候选 偏%+.0f°: 净空 %.2f m 通视 %.2f m' % (k2, c2, l2), file=sys.stderr)
        return 0
    print('%.3f %.3f' % (best[0], best[1]))
    print('# 机器人 (%.2f, %.2f, %.1f°)，距离 %.2f m，偏 %+.0f°，净空 %.2f m，通视 %.2f m'
          % (x, y, yaw, args.distance, best[3], best[2], best[4]), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

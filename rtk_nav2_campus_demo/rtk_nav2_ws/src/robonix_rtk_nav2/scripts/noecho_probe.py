#!/usr/bin/env python3
"""判据 ①「无回波 ⇒ 量程内判空闲」的**前后计数 + 格级证据**（在跑着的栈上量）。

要回答两件事，且**必须分开回答**：

  1) **前后计数**：同一套场景下，把"无效像素补远点"这个开关关掉再打开
     （`depth_points` 的 `emit_far_points` 参数，改参数重启**这一个节点**即可，
     不用重起栈），量全图 `unknown` 下降 / `free` 上升。
     —— 用 `--label before|after` 分别跑两次，脚本自己给出对比。
  2) **格级证据**：不是"应该会变 free"，而是**逐格读出来的**。
     做法：以机器人真值位姿为原点，在相机水平视场（±30°）内按 1° 逐条投射线，
     用**世界几何**判断这条方位上量程内有没有真实障碍：
       · 没有（"无回波方位"）→ 沿这条线按步长采样格子，数 free / unknown / lethal，
         并把**前若干个格子的世界坐标与代价**逐格打出来（这就是格级证据）；
       · 有 → 作对照组（那些方向的格子本来就该是 lethal/膨胀）。

**口径**：只读 `/global_costmap/costmap`（latch 的 QoS）+ `trajectory.csv` 的真值位姿，
不订阅任何传感数据、不发任何控制。位姿取 `trajectory.csv` 的**第 2/3/4 列**
（gt_x, gt_y, gt_yaw；**第 1 列 t 停车后停更**，不用它）。

用法（容器内）：
  noecho_probe.py --traj /work/artifacts/rtk-nav2/<run>/trajectory.csv \
      --label before --json .../noecho_before.json
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_geometry as G  # noqa: E402

TOPIC = '/global_costmap/costmap'
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
# 相机水平视场 60°（world 里写死 1.0472 rad），半视场 30°
HALF_FOV_DEG = 30.0


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Once(Node):
    def __init__(self):
        super().__init__('noecho_probe')
        self.msg = None
        self.create_subscription(OccupancyGrid, TOPIC, self._cb, LATCHED)

    def _cb(self, msg):
        if self.msg is None:
            self.msg = msg


def last_truth_pose(path):
    """`trajectory.csv` 的最后一行 → (gt_x, gt_y, gt_yaw_deg)。

    **第 1 列 t 停车后停更**，所以这里只用第 2/3/4 列；
    yaw 的单位按量程自适应（|值| <= π 当弧度），与 `outward_coverage.py` 同口径。
    """
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
    return x, y, deg, len(rows)


def ray_hits_obstacle(x, y, deg, max_range, step=0.05):
    """从 (x,y) 沿 deg 方向走，返回**第一个落在真实障碍内**的距离；没有则 None。

    用 `map_geometry.nearest_surface`（世界几何，与生成器同源）：距离 0 即在障碍里。
    这就是"这条方位上有没有回波"的**世界侧**判据——与传感器无关，可复核。
    """
    d = step
    th = math.radians(deg)
    dx, dy = math.cos(th), math.sin(th)
    while d <= max_range:
        if G.nearest_surface(x + dx * d, y + dy * d) <= 0.0:
            return d
        d += step
    return None


def cell_of(meta, x, y):
    c = int((x - meta['origin_x']) / meta['resolution'])
    r = int((y - meta['origin_y']) / meta['resolution'])
    if 0 <= r < meta['height'] and 0 <= c < meta['width']:
        return r, c
    return None


def classify(v):
    if v >= 100:
        return 'lethal'
    if v >= 50:
        return 'inflated'
    if v > 0:
        return 'lowcost'
    if v == 0:
        return 'free'
    return 'unknown'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj', required=True, help='trajectory.csv（取最后一行真值位姿）')
    ap.add_argument('--label', default='')
    ap.add_argument('--json', default='')
    ap.add_argument('--range', type=float, default=6.0,
                    help='判"无回波方位"与采样格子的最大距离（默认 6.0 = 相机的 raytrace_max_range）')
    ap.add_argument('--step', type=float, default=0.25, help='沿射线采样步长')
    ap.add_argument('--samples', type=int, default=6, help='每种方位打印几格逐格证据')
    ap.add_argument('--timeout', type=float, default=60.0)
    args = ap.parse_args(argv)

    x0, y0, yaw0, nrows = last_truth_pose(args.traj)

    rclpy.init()
    node = Once()
    t0 = time.time()
    while node.msg is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.msg is None:
        print('** %.0f 秒内没收到 %s **' % (args.timeout, TOPIC), file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = node.msg
    meta = {'resolution': msg.info.resolution, 'width': msg.info.width,
            'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y, 'frame': msg.header.frame_id}
    data = np.asarray(msg.data, dtype=np.int16).reshape(meta['height'], meta['width'])

    whole = {'cells': int(data.size), 'free': int((data == 0).sum()),
             'lethal': int((data >= 100).sum()),
             'inflated_50_99': int(((data >= 50) & (data < 100)).sum()),
             'unknown': int((data < 0).sum())}

    # ---- 视场内逐方位：世界侧有没有障碍 ----
    no_echo, blocked = [], []
    for k in range(-int(HALF_FOV_DEG), int(HALF_FOV_DEG) + 1):
        az = yaw0 + k
        d_hit = ray_hits_obstacle(x0, y0, az, args.range)
        (no_echo if d_hit is None else blocked).append({'rel_deg': k, 'hit_m': d_hit})

    def scan_azimuths(items):
        """沿每条方位按步长采样格子，数状态分布，并留逐格样例。"""
        counts = {'free': 0, 'unknown': 0, 'lethal': 0, 'inflated': 0, 'lowcost': 0}
        samples = []
        n_cells = 0
        for it in items:
            az = yaw0 + it['rel_deg']
            th = math.radians(az)
            cells = []
            d = args.step
            while d <= args.range:
                px, py = x0 + math.cos(th) * d, y0 + math.sin(th) * d
                rc = cell_of(meta, px, py)
                if rc is None:
                    d += args.step
                    continue
                v = int(data[rc[0], rc[1]])
                st = classify(v)
                counts[st] = counts.get(st, 0) + 1
                n_cells += 1
                cells.append({'d_m': round(d, 2), 'x': round(px, 3), 'y': round(py, 3),
                              'cost': v, 'state': st})
                d += args.step
            # 每 10 条方位留一条逐格证据（够格级复核，又不至于产物爆掉）
            if items and (abs(it['rel_deg']) % 10 == 0):
                samples.append({'rel_deg': it['rel_deg'], 'az_deg': round(az, 2),
                                'cells': cells[:args.samples + 4]})
        return {'azimuth_count': len(items), 'cells_sampled': n_cells,
                'state_counts': counts,
                'free_ratio': round(counts['free'] / max(1, n_cells), 4),
                'unknown_ratio': round(counts['unknown'] / max(1, n_cells), 4),
                'samples': samples}

    out = {
        'label': args.label,
        'trajectory': os.path.abspath(args.traj),
        'traj_rows': nrows,
        'pose': {'gt_x': round(x0, 3), 'gt_y': round(y0, 3), 'gt_yaw_deg': round(yaw0, 2)},
        'map': {'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'meta': meta, 'whole': whole},
        'range_m': args.range,
        'step_m': args.step,
        'fov_half_deg': HALF_FOV_DEG,
        'no_echo_azimuths': scan_azimuths(no_echo),
        'blocked_azimuths': scan_azimuths(blocked),
        'no_echo_rel_deg': [it['rel_deg'] for it in no_echo],
        'note': ('`no_echo_azimuths` = 相机水平视场 ±30° 内、世界几何上 --range 米内**没有**真实障碍'
                 '的方位（这些方位上"看过去没回波"）；`blocked_azimuths` 是有障碍的对照组。'
                 '判据 ①要的是：**前者沿线的格子不再是 unknown**（free_ratio 高 / unknown_ratio ≈ 0）。'
                 '前后对比由 `--label before|after` 两次运行的 whole.unknown / free 给出。'),
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

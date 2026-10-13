#!/usr/bin/env python3
"""**标记对账**：把"当前这帧点云按 nav2 的规则会标成致命格的格子"和"代价地图里实际
是致命格的格子"逐格对账。

为什么需要它：`camera_ground_probe.py` 说"点云里没有幽灵点"，可代价地图上就是有幽灵格。
这两个结论不能同时成立，除非
  (A) 幽灵格是**陈旧残留**（早先某个时刻标的，现在的点云不会再标它，射线也没把它清掉），或
  (B) 我对 nav2 标记规则的复现**漏了什么**（那就是我自己的 bug）。

对账能一刀切开这两种可能：
  - `lethal_not_from_cloud` 里有幽灵格 → 是 (A) 陈旧残留（再看清不掉的原因）；
  - `lethal_not_from_cloud` 基本为空、而幽灵格仍在 `would_mark` 里 → 是 (B)，复现有漏洞。

复现口径（对着 nav2 humble 的 `ObstacleLayer::updateBounds` 逐行抄的）：
  点变到 **map 帧** → `min_obstacle_height <= z <= max_obstacle_height` 才要 →
  `worldToMap(px,py)` 取格 → 与**传感器原点所在格**的格距 `hypot(dx,dy) <=
  obstacle_max_range/resolution` 才标 → `costmap_[index] = LETHAL`。

用法：
  nav2_mark_reconcile.py [--json 结果.json] [--timeout 30]
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_geometry as G  # noqa: E402
from camera_ground_probe import cloud_to_array, transform_to_np  # noqa: E402

GRID_TOPIC = '/global_costmap/costmap'
CLOUD_TOPIC = '/rtk/camera/points'
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
Z_LO, Z_HI, R_MAX = 0.12, 1.2, 6.0


class Rec(Node):
    def __init__(self):
        super().__init__('nav2_mark_reconcile')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.grid = None
        self.cloud = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, GRID_TOPIC, self._g, LATCHED)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._c, qos_profile_sensor_data)

    def _g(self, m):
        self.grid = m

    def _c(self, m):
        self.cloud = m


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', default=None)
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Rec()
    t0 = time.time()
    while (node.grid is None or node.cloud is None) and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.grid is None or node.cloud is None:
        print('** 没收到图或点云 **', file=sys.stderr)
        return 1

    g = node.grid
    data = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
    res = g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    lethal = (data >= 100)

    src = node.cloud.header.frame_id
    stamp = node.cloud.header.stamp.sec + node.cloud.header.stamp.nanosec * 1e-9
    tm = None
    tb = None
    t0 = time.time()
    while time.time() - t0 < args.timeout and (tm is None or tb is None):
        for target, key in (('map', 'm'), ('base_footprint', 'b')):
            try:
                tr = node.tf_buffer.lookup_transform(target, src, rclpy.time.Time())
                if key == 'm':
                    tm = tr
                else:
                    tb = tr
            except Exception:
                pass
        rclpy.spin_once(node, timeout_sec=0.1)
    if tm is None or tb is None:
        print('** 拿不到 TF **', file=sys.stderr)
        return 1

    pts = cloud_to_array(node.cloud)
    finite = np.isfinite(pts).all(axis=1)
    n_raw = len(pts)
    pts = pts[finite]
    Rm, Tm = transform_to_np(tm)
    pm = pts @ Rm.T + Tm
    rb = tb.transform.translation
    # **传感器原点要用 `map <- 相机` 那条变换的平移**（= 相机在 map 帧里的位置）。
    # 用 `base_footprint <- 相机` 的平移会得到 (0.055, 0, 0.19)（相机在车体里的安装位置），
    # 格距过滤会拿它当原点、几乎把所有点都判成"太远"，`would_mark` 只剩个位数 —— 这个坑踩过。
    origin_cell = (int((Tm[0] - ox) / res), int((Tm[1] - oy) / res))
    max_cells = int(R_MAX / res)

    want = np.zeros_like(lethal)
    in_band = (pm[:, 2] >= Z_LO) & (pm[:, 2] <= Z_HI)
    sel = np.nonzero(in_band)[0]
    cols = np.floor((pm[sel, 0] - ox) / res).astype(np.int64)
    rows = np.floor((pm[sel, 1] - oy) / res).astype(np.int64)
    ok = (cols >= 0) & (cols < g.info.width) & (rows >= 0) & (rows < g.info.height)
    cols, rows = cols[ok], rows[ok]
    d = np.hypot(cols - origin_cell[0], rows - origin_cell[1])
    keep = d <= max_cells
    cols, rows = cols[keep], rows[keep]
    want[rows, cols] = True

    miss = lethal & ~want            # 图里有、这帧点云解释不了
    mr, mc = np.nonzero(miss)
    mxs = ox + (mc + 0.5) * res
    mys = oy + (mr + 0.5) * res
    md = np.array([G.nearest_surface(x, y) for x, y in zip(mxs, mys)])
    far = md > 0.35

    out = {
        'cloud': {'frame': src, 'stamp': round(stamp, 2),
                  'raw_points': int(n_raw), 'finite_points': int(len(pts)),
                  'nonfinite_points': int(n_raw - len(pts))},
        'would_mark_cells': int(want.sum()),
        'costmap_lethal_cells': int(lethal.sum()),
        'lethal_explained_by_cloud': int((lethal & want).sum()),
        'lethal_not_from_cloud': int(miss.sum()),
        'lethal_not_from_cloud_far': int(far.sum()),
        'cell_distance_limit_m': R_MAX,
        'origin_cell': list(origin_cell),
        'origin_xy': [round(float(Tm[0]), 3), round(float(Tm[1]), 3)],
        'robot_xy': [round(rb.x, 3), round(rb.y, 3)],
        'stale_examples': [[round(float(a), 2), round(float(b), 2), round(float(c), 3)]
                           for a, b, c in list(zip(mxs[far], mys[far], md[far]))[:12]],
        'note': 'would_mark = 当前这帧点云按 nav2 规则会标的格；lethal_not_from_cloud = '
                '图里有、这帧解释不了的格。后者若包含"离真实障碍很远"的格，说明那些格是'
                '**陈旧残留**（不是当前点云标的），要接着查"为什么没被清掉"。',
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

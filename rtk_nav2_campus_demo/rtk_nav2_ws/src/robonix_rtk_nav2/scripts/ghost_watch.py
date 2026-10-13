#!/usr/bin/env python3
"""**抓现行**：盯着全局图，一旦出现"离真实障碍很远"的新致命格，立刻把当时的点云对上去。

`camera_ground_probe.py` 只能说明"某一段时间里点云里有没有幽灵点"；当地图上**已经有**
幽灵格时，它回答不了"这些格子是哪一帧、哪个像素写进去的"。本脚本干这件事：

  1. 用 transient_local 订阅 `/global_costmap/costmap`，相邻两帧做**逐格 diff**，
     找出"这一帧新变成致命（cost == 100）"的格子；
  2. 其中**离最近真实障碍表面 > --far_m** 的，就是新生的幽灵格（其余是正常障碍表面）；
  3. 把最近几秒缓存下来的 `/rtk/camera/points` 拿出来，对每个幽灵格找**落在它里面**的点，
     报出那些点在相机光学帧里的**像素坐标 (u,v)、深度值、离地高度、到车的距离**，
     以及当时的机器人位姿。

  有了 (u,v) 就能直接跟 `camera_depth_profile.py` 的逐行画像对上——是"没打到东西"的行，
  还是真几何被算歪了。

用法：
  ghost_watch.py --duration 120 [--jsonl 记录.jsonl] [--far_m 0.35]
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
CACHE_S = 3.0        # 缓存多少秒的点云（标记与发布在同一周期，3 s 足够覆盖）


class Watch(Node):
    def __init__(self):
        super().__init__('ghost_watch')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.grid = None
        self.grid_stamp = None
        self.clouds = []          # [(stamp, msg)]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, GRID_TOPIC, self._on_grid, LATCHED)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud,
                                 qos_profile_sensor_data)

    def _on_grid(self, m):
        self.grid = m
        self.grid_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _on_cloud(self, m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.clouds.append((s, m))
        if len(self.clouds) > 200:
            self.clouds = self.clouds[-200:]


def grid_of(msg):
    return np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--jsonl', default=None)
    ap.add_argument('--far_m', type=float, default=0.35)
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Watch()
    if not node.grid:
        t0 = time.time()
        while node.grid is None and time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
    if node.grid is None:
        print('** 没收到 %s **' % GRID_TOPIC, file=sys.stderr)
        return 1

    prev = grid_of(node.grid)
    meta = {'res': node.grid.info.resolution,
            'ox': node.grid.info.origin.position.x,
            'oy': node.grid.info.origin.position.y}
    print('起始：lethal=%d  （只报**新出现**的幽灵格）' % int((prev >= 100).sum()))

    t0 = time.time()
    events = 0
    while time.time() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.grid is None:
            continue
        cur = grid_of(node.grid)
        if cur.shape != prev.shape:
            prev = cur
            continue
        new = (cur >= 100) & ~(prev >= 100)
        prev = cur
        rows, cols = np.nonzero(new)
        if not len(rows):
            continue
        xs = meta['ox'] + (cols + 0.5) * meta['res']
        ys = meta['oy'] + (rows + 0.5) * meta['res']
        dist = np.array([G.nearest_surface(x, y) for x, y in zip(xs, ys)])
        far = dist > args.far_m
        if not far.any():
            continue
        gx, gy = xs[far], ys[far]
        gd = dist[far]
        # 找出这些幽灵格是**哪一帧点云**写进去的。
        # **不能只取"离地图时间戳最近的那一帧"**：nav2 的代价地图 2 Hz 发布，
        # 而标记发生在更早的处理周期里 —— 实测第一版这么取的时候 `n_hits` 恒为 0
        # （2026-09-21，nav8-c2 的 ghost_watch.jsonl 里 stamp 225.5 那条就是），
        # 于是拿不到"哪个像素写进去的"这个关键证据。
        # 现在把**缓存里的每一帧**都试一遍，取命中最多的一帧。
        stamp = node.grid_stamp
        best = None
        best_hits = -1
        detail = {'stamp': round(stamp, 2), 'n_new_phantom': int(far.sum()),
                  'cells': [[round(float(a), 3), round(float(b), 3), round(float(d), 3)]
                            for a, b, d in zip(gx[:12], gy[:12], gd[:12])]}
        cand_frames = sorted(node.clouds, key=lambda sm: abs(sm[0] - stamp))[:12]
        for s, msg in cand_frames:
            src = msg.header.frame_id
            tg = None
            t1 = time.time()
            while time.time() - t1 < 3.0 and tg is None:
                try:
                    tg = node.tf_buffer.lookup_transform('base_footprint', src,
                                                         rclpy.time.Time(seconds=s, nanoseconds=0))
                except Exception:
                    rclpy.spin_once(node, timeout_sec=0.05)
            if tg is None:
                continue
            pts = cloud_to_array(msg)
            ok = np.isfinite(pts).all(axis=1)
            idx = np.nonzero(ok)[0]
            pts = pts[idx]
            R, T = transform_to_np(tg)
            base = pts @ R.T + T
            # 幽灵格在 map 帧；这里只需要"哪些点落在格子里"，所以再把点变到 map
            tm = None
            t1 = time.time()
            while time.time() - t1 < 3.0 and tm is None:
                try:
                    tm = node.tf_buffer.lookup_transform('map', src,
                                                         rclpy.time.Time(seconds=s, nanoseconds=0))
                except Exception:
                    rclpy.spin_once(node, timeout_sec=0.05)
            if tm is not None:
                    Rm, Tm = transform_to_np(tm)
                    pm = pts @ Rm.T + Tm
                    hits = []
                    for k in range(min(len(gx), 40)):
                        # 只对"点云落到这片区域"的格子查，避免每格都扫全点云
                        near = np.nonzero((np.abs(pm[:, 0] - gx[k]) < 0.5)
                                          & (np.abs(pm[:, 1] - gy[k]) < 0.5))[0]
                        if not len(near):
                            continue
                        d = np.hypot(pm[near, 0] - gx[k], pm[near, 1] - gy[k])
                        inside = near[np.nonzero((d < meta['res']) & (pm[near, 2] >= 0.12)
                                                 & (pm[near, 2] <= 1.2))[0]]
                        for i in inside[:2]:
                            hits.append([round(float(gx[k]), 2), round(float(gy[k]), 2),
                                         int(idx[i] % msg.width), int(idx[i] // msg.width),
                                         round(float(pts[i, 2]), 3), round(float(pm[i, 2]), 3),
                                         round(float(math.hypot(base[i, 0], base[i, 1])), 3)])
                    detail['cloud_stamp'] = round(s, 2)
                    detail['cloud_frame'] = src
                    if len(hits) > best_hits:
                        best_hits = len(hits)
                        detail['hits_uvZ_zmap_range'] = hits[:20]
                        detail['n_hits'] = len(hits)
                    else:
                        detail.setdefault('n_hits', 0)
        events += 1
        print(json.dumps(detail, ensure_ascii=False))
        if args.jsonl:
            with open(args.jsonl, 'a', encoding='utf-8') as handle:
                handle.write(json.dumps(detail, ensure_ascii=False) + '\n')
    print('观测 %.0f s，捕获 %d 次"新幽灵格"事件' % (args.duration, events))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

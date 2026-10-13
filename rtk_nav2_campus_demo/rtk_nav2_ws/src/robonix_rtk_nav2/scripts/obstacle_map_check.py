#!/usr/bin/env python3
"""在**跑着的栈**上取一帧全局图，核对"外墙没了、其余障碍还在"（一次性取数脚本）。

背景（2026-09-21）：按 AuYang 要求去掉了四周围墙、另加了 4 段独立墙。判据是
  ① 原来外墙那圈（中心线 ±5.0 m 的方框）**不应再有致命格**；
  ② 中央方块 / 隔墙 / 立柱 / 新增的 4 段独立墙的位置上**仍应有致命格**（它们确实进了图）。

判据 ①/② 必须落在**同一帧**上取，所以本脚本只取一帧，一次性把两边都数出来。

"致命格"用 `代价 == 100`（nav2 的 LETHAL_OBSTACLE）。**不用 `>=50`**：那会把膨胀带
（cost 50~99）算进去，外墙那圈就会被自己的膨胀误报成"还有墙"（见 nav2_probe.py 的说明）。

用法：
  obstacle_map_check.py [--json 结果.json] [--timeout 60]
"""

import argparse
import json
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
# nav2 的 Costmap2DPublisher 用 transient_local 发：订阅端必须同 durability，否则要等下一个周期
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

# **被删掉的外墙几何，故意写在这里**：这个脚本的用途之一就是证明它已经不在图上了。
# 四个整条带子（中心线 ±5.0、厚 0.15、x/y 跨到 ±5.075），外面再放 0.1 m 的余量，
# 免得"墙没了但膨胀还在"被漏判。
OLD_WALL_BAND = 0.1
OLD_WALLS = {
    'wall_north': (-5.075 - OLD_WALL_BAND, 5.075 + OLD_WALL_BAND,
                   4.925 - OLD_WALL_BAND, 5.075 + OLD_WALL_BAND),
    'wall_south': (-5.075 - OLD_WALL_BAND, 5.075 + OLD_WALL_BAND,
                   -5.075 - OLD_WALL_BAND, -4.925 + OLD_WALL_BAND),
    'wall_east': (4.925 - OLD_WALL_BAND, 5.075 + OLD_WALL_BAND,
                  -5.075 - OLD_WALL_BAND, 5.075 + OLD_WALL_BAND),
    'wall_west': (-5.075 - OLD_WALL_BAND, -4.925 + OLD_WALL_BAND,
                  -5.075 - OLD_WALL_BAND, 5.075 + OLD_WALL_BAND),
}

# "仍应存在"的障碍：从 map_geometry 取，保证与生成器/世界文件同源
PRESENT = {'block': (-G.BLOCK_HALF, G.BLOCK_HALF, -G.BLOCK_HALF, G.BLOCK_HALF)}
_p = G.PARTITION_OFFSET
_t = G.PARTITION_THICK / 2
PRESENT['partition_p1'] = (-G.PARTITION_HALF, G.PARTITION_HALF, _p - _t, _p + _t)
PRESENT['partition_p2'] = (-_p - _t, -_p + _t, -G.PARTITION_HALF, G.PARTITION_HALF)
_h = G.PILLAR_SIDE / 2
for _i, (_px, _py) in enumerate(G.PILLARS):
    PRESENT['pillar_%d' % _i] = (_px - _h, _px + _h, _py - _h, _py + _h)
for _seg in G.SEGMENTS:
    PRESENT[_seg[0]] = G.segment_rect(_seg)


class Once(Node):
    def __init__(self):
        super().__init__('obstacle_map_check')
        self.msg = None
        self.sub = self.create_subscription(OccupancyGrid, TOPIC, self._cb, LATCHED)

    def _cb(self, msg):
        if self.msg is None:
            self.msg = msg


def rect_counts(data, meta, rect):
    """矩形内的格数与分类计数（格中心落在矩形内才算）。"""
    xmin, xmax, ymin, ymax = rect
    res = meta['resolution']
    ox, oy = meta['origin_x'], meta['origin_y']
    c0 = max(0, int(np.floor((xmin - ox) / res)))
    c1 = min(data.shape[1] - 1, int(np.ceil((xmax - ox) / res)))
    r0 = max(0, int(np.floor((ymin - oy) / res)))
    r1 = min(data.shape[0] - 1, int(np.ceil((ymax - oy) / res)))
    sub = data[r0:r1 + 1, c0:c1 + 1]
    return {
        'rect': [round(v, 3) for v in rect],
        'cells': int(sub.size),
        'lethal': int((sub >= 100).sum()),
        'occupied_ge50': int((sub >= 50).sum()),
        'free': int((sub == 0).sum()),
        'unknown': int((sub < 0).sum()),
        'max_cost': int(sub.max()) if sub.size else None,
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', default=None)
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--far_m', type=float, default=0.35,
                    help='致命格离任何真实障碍表面超过这个距离就算"可疑"（默认 0.35 m）')
    args = ap.parse_args(argv)

    rclpy.init()
    node = Once()
    t0 = time.time()
    while node.msg is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.msg is None:
        print('** %s 上 %.0f 秒内没收到全局图 **' % (TOPIC, args.timeout), file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = node.msg
    data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    meta = {'resolution': msg.info.resolution,
            'width': msg.info.width, 'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'frame': msg.header.frame_id,
            'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}

    lethal = np.argwhere(data >= 100)
    cells = [(meta['origin_x'] + (c + 0.5) * meta['resolution'],
              meta['origin_y'] + (r + 0.5) * meta['resolution']) for r, c in lethal]
    dists = [G.nearest_surface(x, y) for x, y in cells]
    far = [(x, y, d) for (x, y), d in zip(cells, dists) if d > args.far_m]
    far_bbox = None
    if far:
        far_bbox = [round(min(p[0] for p in far), 3), round(max(p[0] for p in far), 3),
                    round(min(p[1] for p in far), 3), round(max(p[1] for p in far), 3)]

    result = {
        'meta': meta,
        'counts': {
            'cells': int(data.size),
            'free': int((data == 0).sum()),
            'lethal': int((data >= 100).sum()),
            'occupied_ge50': int((data >= 50).sum()),
            'unknown': int((data < 0).sum()),
        },
        'old_outer_walls_should_be_gone': {
            name: rect_counts(data, meta, rect) for name, rect in OLD_WALLS.items()
        },
        'old_outer_wall_lethal_total': sum(
            rect_counts(data, meta, rect)['lethal'] for rect in OLD_WALLS.values()),
        'present_obstacles_should_exist': {
            name: rect_counts(data, meta, rect) for name, rect in PRESENT.items()
        },
        'lethal_far_from_real_surface': {
            'threshold_m': args.far_m,
            'count': len(far),
            'max_distance_m': round(max(dists), 3) if dists else None,
            'bbox_xy': far_bbox,
            'note': '改了世界之后 nearest_surface 里已经没有外墙了，所以"离任何真实障碍都很远"'
                    '的致命格就是可疑来源（外墙残留 / 假障碍）。改前那圈外墙会全部落进这一项。',
        },
        'source': TOPIC,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        print('已写入 %s' % args.json, file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

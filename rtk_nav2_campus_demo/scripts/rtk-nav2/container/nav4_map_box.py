#!/usr/bin/env python3
"""在全局图上量一个**紧盒子**：判断"临时障碍有没有进图/有没有被清掉"（判据 9 的第二半）。

为什么单独一个小工具而不是复用 `nav2_probe.py localmap`：判据要的是"**某一个世界坐标附近**
的格状态随时间的变化"，而 nav2_probe 给的是全图计数——全图计数会被车自己新看到的区域
盖过去（边走边建时全图 lethal 一直在涨，清没清掉那一个障碍根本看不出来）。

盒子取两档（沿用 reviewer 在第三阶段的教训）：
  tight ±0.35 m：只覆盖 0.4 m 见方障碍本身 + 一点膨胀，**不受墙污染**；
  wide  ±0.9  m：同一个数的对照组，盒子放大后会开始吃到远处墙的膨胀带，读数上升属预期。
障碍位置必须离墙 >1 m，否则 tight 里也会有墙（第三阶段实测过一次，读数被墙污染）。

用法（容器内）：
  nav4_map_box.py --x 2.6 --y 0.0 --label baseline --json /work/.../box_baseline.json
"""

import argparse
import json
import os
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class BoxProbe(Node):

    def __init__(self):
        super().__init__('nav4_map_box')
        self.msg = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_map, LATCHED)

    def on_map(self, msg):
        self.msg = msg


def box_stats(data, meta, x, y, half):
    r = meta['resolution']
    ox, oy = meta['origin_x'], meta['origin_y']
    col0 = max(0, int((x - half - ox) / r))
    col1 = min(meta['width'] - 1, int((x + half - ox) / r))
    row0 = max(0, int((y - half - oy) / r))
    row1 = min(meta['height'] - 1, int((y + half - oy) / r))
    sub = data[row0:row1 + 1, col0:col1 + 1]
    return {'half_m': half,
            'cells': int(sub.size),
            'lethal': int((sub >= 100).sum()),
            'occupied_ge50': int((sub >= 50).sum()),
            'inflated_only': int(((sub > 0) & (sub < 50)).sum()),
            'free': int((sub == 0).sum()),
            'unknown': int((sub < 0).sum())}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--label', default='')
    parser.add_argument('--json', default='')
    parser.add_argument('--snapshot', default='')
    parser.add_argument('--timeout', type=float, default=40.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = BoxProbe()
    import time
    end = time.time() + args.timeout
    while node.msg is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.msg is None:
        print(json.dumps({'label': args.label, 'error': '没收到全局图'},
                         ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = node.msg
    meta = {'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'width': msg.info.width, 'height': msg.info.height,
            'frame': msg.header.frame_id}
    data = np.asarray(msg.data, dtype=np.int16).reshape(meta['height'], meta['width'])
    out = {'label': args.label, 'x': args.x, 'y': args.y,
           'map_stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
           'meta': meta,
           'whole_map': {'lethal': int((data >= 100).sum()),
                         'occupied_ge50': int((data >= 50).sum()),
                         'free': int((data == 0).sum()),
                         'unknown': int((data < 0).sum()),
                         'cells': int(data.size)},
           'tight_0p35': box_stats(data, meta, args.x, args.y, 0.35),
           'wide_0p90': box_stats(data, meta, args.x, args.y, 0.90),
           'note': 'tight 盒子只覆盖 0.4 m 见方障碍本体 + 一点膨胀；wide 是对照组，'
                   '盒子放大后会开始吃到别处的膨胀带。障碍位置离墙 >1 m 时 tight 基线应为 0。'}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
    if args.snapshot:
        np.savez_compressed(args.snapshot, data=data, resolution=meta['resolution'],
                            origin_x=meta['origin_x'], origin_y=meta['origin_y'],
                            frame=meta['frame'])
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

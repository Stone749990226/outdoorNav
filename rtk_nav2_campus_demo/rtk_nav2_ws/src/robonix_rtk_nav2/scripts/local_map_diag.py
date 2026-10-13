#!/usr/bin/env python3
"""局部代价地图诊断：把占据格的**位置分布**打出来（排障用，不是判据）。

为什么需要它：判据 2 失败时，"lethal 计数"只能说明"有东西在图上"，
说明不了"那个东西在哪、是不是真的障碍"。把 lethal 格换算成 base_link 坐标后，
一眼就能看出它是墙、是障碍、还是幽灵。

用法：local_map_diag.py [--topic /local_costmap/costmap] [--repeat 3] [--interval 4]
"""

import argparse
import sys
import time

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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--topic', default='/local_costmap/costmap')
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--interval', type=float, default=4.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('local_map_diag')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    got = {}
    node.create_subscription(OccupancyGrid, args.topic, lambda m: got.update({'m': m}), LATCHED)

    t0 = time.time()
    while time.time() - t0 < 20 and 'm' not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'm' not in got:
        print('** 没收到 %s **' % args.topic, file=sys.stderr)
        return 1

    for i in range(args.repeat):
        m = got['m']
        d = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        counts = dict(zip(*[v.tolist() for v in np.unique(d, return_counts=True)]))
        rows, cols = np.nonzero(d >= 100)
        print('--- sample %d  stamp=%.2f  frame=%s  origin=(%.2f,%.2f) res=%.3f ---'
              % (i + 1, stamp, m.header.frame_id, ox, oy, res))
        print('    values: %s' % counts)
        if len(rows):
            xs = ox + (cols + 0.5) * res
            ys = oy + (rows + 0.5) * res
            dist = np.hypot(xs, ys)
            # 按"前方扇区"和距离分桶，看 lethal 到底在哪
            print('    lethal=%d  x=[%.2f,%.2f]  y=[%.2f,%.2f]  dist=[%.2f,%.2f]'
                  % (len(rows), xs.min(), xs.max(), ys.min(), ys.max(),
                     dist.min(), dist.max()))
            close = dist < 1.0
            forward = xs > 0
            print('    lethal 中：距离 <1.0 m 的 %d 个；在正前方半平面的 %d 个'
                  % (int(close.sum()), int(forward.sum())))
        else:
            print('    lethal=0')
        time.sleep(args.interval)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

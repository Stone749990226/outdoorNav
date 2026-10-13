#!/usr/bin/env python3
"""原地旋转探针：量**局部图挂 base_link（旋转帧）**带来的拖尾/漂移（判据 8）。

背景（brief §4）：nav2 的滚动窗口只搬原点、**不旋转已有格内容**。局部图的 global_frame
是 `base_link`，机器人恒在图心、朝向恒为 +x——所以机器人一转，栅格的坐标轴跟着机体转，
但**旧格内容还留在原来的格子索引上**。于是"上一次朝向时记下的障碍"会以错误的方位留在图里，
而相机只看前方 ±30°，擦不到它们。

这个探针就是把这个效应量化：
  - 原地以固定角速度转若干圈，同时按周期记录局部图里**占据格的方位分布**：
      occupied_total          全部占据格数
      in_frustum              落在当前相机视场（±half_angle）内的占据格数
      out_of_frustum          视场之外的（**拖尾的候选**）
      nearest_m               最近占据格的距离
  - 在有代表性的转角（默认 0/90/180/270/360 度）各存一份 `npz` 快照，供离线逐格对比。
  - **判读**：转满 360° 后朝向与出发时相同，若地图无拖尾，末态应与初态接近；
    两个同朝向快照之间的占据格差异就是"转过一圈留下了多少幽灵"。

用法：
  spin_probe.py --out-dir <目录> [--rate 0.6] [--turns 1] [--half-angle 30]
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

LOCAL_TOPIC = '/local_costmap/costmap'
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Spin(Node):

    def __init__(self, args):
        super().__init__('spin_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest = None
        self.create_subscription(OccupancyGrid, LOCAL_TOPIC, self._on_map, LATCHED)

    def _on_map(self, msg):
        self.latest = msg

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def stats(self):
        msg = self.latest
        if msg is None:
            return None
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                            msg.info.width)
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        rows, cols = np.nonzero(data >= 50)
        if len(rows) == 0:
            return {'occupied_total': 0, 'in_frustum': 0, 'out_of_frustum': 0,
                    'nearest_m': None}
        x = ox + (cols + 0.5) * res
        y = oy + (rows + 0.5) * res
        dist = np.hypot(x, y)
        ang = np.abs(np.arctan2(y, x))
        half = math.radians(self.args.half_angle)
        inside = (x > 0) & (ang <= half)
        return {
            'occupied_total': int(len(rows)),
            'in_frustum': int(inside.sum()),
            'out_of_frustum': int((~inside).sum()),
            'nearest_m': round(float(dist.min()), 4),
        }

    def snapshot(self):
        msg = self.latest
        if msg is None:
            return None
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                            msg.info.width)
        return {'data': data, 'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'frame': msg.header.frame_id}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--rate', type=float, default=0.6, help='角速度 rad/s')
    parser.add_argument('--turns', type=float, default=1.0)
    parser.add_argument('--half-angle', type=float, default=30.0,
                        help='判"在视场内"用的半角（度）。相机水平半视场约 30 度')
    parser.add_argument('--settle', type=float, default=3.0,
                        help='开始旋转前先静止观察的时间（秒）')
    parser.add_argument('--tail', type=float, default=3.0,
                        help='转完后继续静止观察的时间（秒）')
    args = parser.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Spin(args)
    node.get_logger().info('等待局部代价地图…')
    t_end = time.time() + 30
    while node.latest is None and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest is None:
        print('** 等不到局部代价地图 **', file=sys.stderr)
        return 1

    total_angle = 2 * math.pi * args.turns
    rows = []
    snaps = {}
    started = None
    turned = 0.0
    last_t = time.time()
    marks = [0.0, 0.25, 0.5, 0.75, 1.0]
    mark_names = ['0deg', '90deg', '180deg', '270deg', '360deg']

    def record(phase):
        s = node.stats()
        if s is None:
            return
        rows.append({'phase': phase, 't_wall_s': round(time.time() - t0, 3),
                     'sim_s': round(node.sim_now(), 3),
                     'turned_deg': round(math.degrees(turned), 2),
                     'turned_ratio': (round(turned / total_angle, 4) if total_angle else 0),
                     **s})

    t0 = time.time()
    # 1) 静止观察：拿到"出发朝向下的图"
    node.get_logger().info('静止观察 %.1f s' % args.settle)
    while time.time() - t0 < args.settle:
        rclpy.spin_once(node, timeout_sec=0.05)
        record('settle')
    snap = node.snapshot()
    if snap:
        np.savez_compressed(os.path.join(args.out_dir, 'local_0deg.npz'), **snap)
        snaps['0deg'] = snap

    # 2) 原地旋转
    node.get_logger().info('开始原地旋转：%.2f rad/s，共 %.2f 圈' % (args.rate, args.turns))
    twist = Twist()
    twist.angular.z = args.rate
    while turned < total_angle:
        node.cmd_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time()
        turned += args.rate * (now - last_t)
        last_t = now
        record('spin')
        for mark, name in zip(marks[1:], mark_names[1:]):
            if name not in snaps and turned / total_angle >= mark - 1e-3:
                snap = node.snapshot()
                if snap:
                    np.savez_compressed(os.path.join(args.out_dir,
                                                     'local_%s.npz' % name), **snap)
                    snaps[name] = snap

    # 3) 停住，再观察一段
    stop = Twist()
    node.cmd_pub.publish(stop)
    node.get_logger().info('旋转结束，静止观察 %.1f s' % args.tail)
    t_stop = time.time()
    while time.time() - t_stop < args.tail:
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
        record('tail')
    node.cmd_pub.publish(stop)

    with open(os.path.join(args.out_dir, 'spin_samples.csv'), 'w', encoding='utf-8') as fh:
        fh.write('phase,t_wall_s,sim_s,turned_deg,turned_ratio,occupied_total,'
                 'in_frustum,out_of_frustum,nearest_m\n')
        for r in rows:
            fh.write('%s,%s,%s,%s,%s,%d,%d,%d,%s\n' % (
                r['phase'], r['t_wall_s'], r['sim_s'], r['turned_deg'], r['turned_ratio'],
                r['occupied_total'], r['in_frustum'], r['out_of_frustum'],
                '' if r['nearest_m'] is None else r['nearest_m']))

    # 初态 / 末态（同朝向）逐格对比
    compare = {'rate_rad_s': args.rate, 'turns': args.turns,
               'half_angle_deg': args.half_angle,
               'snapshots': sorted(snaps),
               'samples': len(rows)}
    if '0deg' in snaps and '360deg' in snaps:
        a = snaps['0deg']['data']
        b = snaps['360deg']['data']
        if a.shape == b.shape:
            ao, bo = a >= 50, b >= 50
            compare.update({
                'same_heading_occupied_start': int(ao.sum()),
                'same_heading_occupied_end': int(bo.sum()),
                'newly_occupied_cells': int((bo & ~ao).sum()),
                'cleared_cells': int((ao & ~bo).sum()),
                'unchanged_both_occupied': int((ao & bo).sum()),
                'note': '转满一圈后朝向与出发时相同。newly_occupied_cells 是"转一圈新长出来的"'
                        '幽灵格，就是旋转帧拖尾的量级；unchanged_both_occupied 是稳定复现的真实障碍',
            })
    with open(os.path.join(args.out_dir, 'spin_summary.json'), 'w', encoding='utf-8') as fh:
        json.dump(compare, fh, ensure_ascii=False, indent=2)
    print(json.dumps(compare, ensure_ascii=False, indent=2))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

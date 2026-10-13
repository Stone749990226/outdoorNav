#!/usr/bin/env python3
"""用 nav2 的 `ComputePathToPose` 规划一条路径，落产物并做几何核对。

第三阶段 3.2 的入口就是它——给一个目标点（路线上的下一个航点也行），拿到路径，
把"路径不穿墙"这条判据做成数据，而不是靠肉眼看 RViz。

核对三件事：
  1. **不穿墙**：把路径逐段拿去和已知场地几何求交（几何来自 `map_geometry.py`，
     与判据 1 用的是同一套常量）。同时给一个 margin 版本，看"贴着墙走"有多近。
  2. **未知区域也能规划**（`allow_unknown: true` 生效）：报告路径上各点落在
     全局图的 未知 / 空闲 / 占据 哪一类里。如果不允许未知，起点周围一片 unknown 时
     规划器会直接失败——所以"起点在未知区、终点在未知区、还能出路径"就是这条判据的证据。
  3. 路径本身：长度、点数、起终点。

用法：
  plan_path.py --goal 2.6 2.6 --json out/plan.json --csv out/plan.csv \
               [--start X Y] [--frame map] [--margin 0.05] [--timeout 30]
  （不传 --start 时用 TF 里 map→base_footprint 的当前位置）
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
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_geometry import segment_hits_obstacle  # noqa: E402

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Planner(Node):

    def __init__(self):
        super().__init__('plan_path')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path_pub = self.create_publisher(Path, '/rtk/plan_path', 1)
        self.global_map = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._on_map, LATCHED)

    def _on_map(self, msg):
        self.global_map = msg

    def robot_pose(self, frame='map'):
        tf = self.tf_buffer.lookup_transform(frame, 'base_footprint', rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def wait_for(self, cond, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        print('** 等不到 %s（%.0f s）**' % (what, timeout), file=sys.stderr)
        return False

    def plan(self, start, goal, timeout):
        if not self.client.wait_for_server(timeout_sec=timeout):
            print('** 等不到 compute_path_to_pose 动作服务 **', file=sys.stderr)
            return None, '动作服务不可用'
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = PoseStamped()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.position.x = float(start[0])
        goal_msg.start.pose.position.y = float(start[1])
        goal_msg.start.pose.orientation.w = 1.0
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = float(goal[0])
        goal_msg.goal.pose.position.y = float(goal[1])
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.use_start = True

        send = self.client.send_goal_async(goal_msg)
        end = time.time() + timeout
        while not send.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send.done():
            return None, '发送目标超时'
        handle = send.result()
        if handle is None or not handle.accepted:
            return None, '目标被拒绝'
        res = handle.get_result_async()
        end = time.time() + timeout
        while not res.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res.done():
            return None, '规划超时'
        path = res.result().result.path
        if path is None or len(path.poses) == 0:
            return None, '规划器返回了空路径'
        return path, None


def classify(msg, x, y):
    if msg is None:
        return 'no_map'
    col = int((x - msg.info.origin.position.x) / msg.info.resolution)
    row = int((y - msg.info.origin.position.y) / msg.info.resolution)
    if row < 0 or col < 0 or row >= msg.info.height or col >= msg.info.width:
        return 'outside'
    v = msg.data[row * msg.info.width + col]
    if v < 0:
        return 'unknown'
    if v >= 50:
        return 'occupied'
    return 'free'


def main(argv=None):
    parser = argparse.ArgumentParser(description='用 NavFn 规划并对路径做几何核对')
    parser.add_argument('--goal', nargs=2, type=float, required=True, metavar=('X', 'Y'))
    parser.add_argument('--start', nargs=2, type=float, default=None, metavar=('X', 'Y'))
    parser.add_argument('--json', default='')
    parser.add_argument('--csv', default='')
    parser.add_argument('--margin', type=float, default=0.05,
                        help='穿墙核对时给障碍加的余量（米）')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--label', default='', help='这条路径的语义标签，落进产物')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Planner()
    if not node.wait_for(lambda: node.global_map is not None, args.timeout, '全局代价地图'):
        return 1

    if args.start is None:
        if not node.wait_for(lambda: True, 0.1, 'TF'):
            pass
        end = time.time() + args.timeout
        pose = None
        while time.time() < end and pose is None:
            try:
                pose = node.robot_pose()
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.1)
        if pose is None:
            print('** 拿不到 map→base_footprint **', file=sys.stderr)
            return 1
        start = (pose[0], pose[1])
        start_source = 'TF map→base_footprint（机器人当前位姿）'
    else:
        start = (args.start[0], args.start[1])
        start_source = '命令行指定'

    path, err = node.plan(start, args.goal, args.timeout)
    result = {
        'label': args.label,
        'start': [round(start[0], 3), round(start[1], 3)],
        'start_source': start_source,
        'goal': [round(args.goal[0], 3), round(args.goal[1], 3)],
        'margin_m': args.margin,
        'planned': path is not None,
        'error': err,
    }

    if path is not None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        crossings = []
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            hit = segment_hits_obstacle(a, b, margin=args.margin)
            if hit:
                crossings.append({'segment': i, 'from': [round(a[0], 3), round(a[1], 3)],
                                  'to': [round(b[0], 3), round(b[1], 3)], 'obstacle': hit})
        classes = [classify(node.global_map, x, y) for x, y in pts]
        counts_cls = {c: classes.count(c) for c in set(classes)}
        # 起点/终点附近的图状态：证明"未知区也能规划"
        near_start = classify(node.global_map, *start)
        near_goal = classify(node.global_map, *args.goal)
        result.update({
            'poses': len(pts),
            'path_length_m': round(length, 4),
            'crossings': crossings,
            'crossing_count': len(crossings),
            'crosses_wall': bool(crossings),
            'cell_class_on_path': counts_cls,
            'start_cell_class': near_start,
            'goal_cell_class': near_goal,
            'note': 'crossings 用已知场地几何逐段求交（margin 见 margin_m）；'
                    'cell_class_on_path 是路径点落在全局图的哪一类格子里，'
                    'unknown 占比高说明 allow_unknown 确实在让规划器走未知区域',
        })
        node.path_pub.publish(path)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if args.csv:
            with open(args.csv, 'w', encoding='utf-8') as handle:
                handle.write('i,x,y,cell_class\n')
                for i, ((x, y), cls) in enumerate(zip(pts, classes)):
                    handle.write('%d,%.4f,%.4f,%s\n' % (i, x, y, cls))
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2))

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
    return 0 if path is not None else 1


if __name__ == '__main__':
    raise SystemExit(main())

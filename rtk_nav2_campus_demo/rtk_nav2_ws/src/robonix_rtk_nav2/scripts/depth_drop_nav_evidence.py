#!/usr/bin/env python3
"""Stage-6 target drive evidence: live maps, path geometry, goal state, and arrival.

The target is an operator input, not detector configuration. The detector never sees
this target or the SDF edge coordinate. A target initially in unknown space is recorded
as such; a lethal target is rejected before publishing.
"""
import argparse
import hashlib
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class Evidence(Node):
    def __init__(self, goal):
        super().__init__('depth_drop_nav_evidence')
        self.goal = goal
        self.maps = {}
        self.path = None
        self.states = []
        self.path_rows = []
        self.map_rows = []
        self.pose = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        for topic, key in (('/global_costmap/costmap', 'global'),
                           ('/local_costmap/costmap', 'local')):
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, k=key: self.on_map(m, k), qos)
        self.create_subscription(Path, '/rtk/plan_path', self.on_path, qos)
        self.create_subscription(String, '/rtk/goal_navigator_state', self.on_state, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_pose, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.last_path_key = None
        self.last_map_stamp = {}

    def on_map(self, msg, key):
        self.maps[key] = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_map_stamp.get(key) != stamp:
            self.last_map_stamp[key] = stamp
            self.map_rows.append({'wall': time.time(), 'key': key, 'stamp': stamp,
                                  'lethal': int(sum(v >= 100 for v in msg.data)),
                                  'occupied': int(sum(v > 0 for v in msg.data))})

    def on_path(self, msg):
        pts = [(round(p.pose.position.x, 4), round(p.pose.position.y, 4)) for p in msg.poses]
        raw = json.dumps(pts, separators=(',', ':')).encode()
        key = hashlib.sha256(raw).hexdigest()[:16]
        length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        if key != self.last_path_key:
            self.last_path_key = key
            self.path_rows.append({'wall': time.time(), 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                                   'hash': key, 'points': len(pts), 'length_m': round(length, 4),
                                   'start': pts[0] if pts else None, 'end': pts[-1] if pts else None})
        self.path = msg

    def on_state(self, msg):
        self.states.append({'wall': time.time(), 'state': msg.data})

    def on_pose(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def map_cell(self, msg, x, y, radius=1):
        ix = int(math.floor((x-msg.info.origin.position.x)/msg.info.resolution))
        iy = int(math.floor((y-msg.info.origin.position.y)/msg.info.resolution))
        if not (0 <= ix < msg.info.width and 0 <= iy < msg.info.height): return None
        a = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        return sorted(set(int(v) for v in a[max(0,iy-radius):iy+radius+1,
                                               max(0,ix-radius):ix+radius+1].ravel()))

    def publish_goal(self):
        m = self.maps.get('global')
        classification = 'no_global_map'
        if m:
            vals = self.map_cell(m, *self.goal, radius=1) or []
            classification = 'lethal' if any(v >= 100 for v in vals) else ('unknown' if -1 in vals else 'free')
        return classification

    def send_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'; msg.pose.position.x, msg.pose.position.y = self.goal
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


def main():
    p = argparse.ArgumentParser(); p.add_argument('--goal-x', type=float, required=True); p.add_argument('--goal-y', type=float, required=True)
    p.add_argument('--seconds', type=float, default=120); p.add_argument('--json', required=True)
    a = p.parse_args(); rclpy.init(); n = Evidence((a.goal_x, a.goal_y))
    t0 = time.time(); deadline = t0 + 20
    while time.time() < deadline and len(n.maps) < 2: rclpy.spin_once(n, timeout_sec=.1)
    classification = n.publish_goal()
    if classification == 'lethal':
        print('TARGET_REJECTED_LETHAL'); return 2
    print('TARGET_CLASSIFICATION=%s' % classification)
    for _ in range(8): n.send_goal(); rclpy.spin_once(n, timeout_sec=.12)
    last_send = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=.1)
        if time.time() - last_send > 1.0:
            n.send_goal(); last_send = time.time()
        if any('arrived' in x['state'] for x in n.states): break
    arrived = any('arrived' in x['state'] for x in n.states)
    result = {'produced_by':'depth_drop_nav_evidence.py','produced_at':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
              'goal_map': [a.goal_x,a.goal_y], 'goal_initial_classification':classification,
              'arrived': arrived, 'states':n.states, 'path_rows':n.path_rows, 'map_rows':n.map_rows,
              'final_pose':n.pose, 'global_map_frames':sum(x['key']=='global' for x in n.map_rows),
              'local_map_frames':sum(x['key']=='local' for x in n.map_rows),
              'path_geometry_changed':len({x['hash'] for x in n.path_rows}) > 1,
              'note':'Periodic path republishes with identical hash are not called geometry changes.'}
    os.makedirs(os.path.dirname(a.json), exist_ok=True)
    json.dump(result, open(a.json,'w'), indent=2)
    print(json.dumps({k:result[k] for k in ('goal_initial_classification','arrived','final_pose','global_map_frames','local_map_frames','path_geometry_changed')}, indent=2))
    n.destroy_node(); rclpy.shutdown(); return 0 if arrived else 1

if __name__ == '__main__': raise SystemExit(main())

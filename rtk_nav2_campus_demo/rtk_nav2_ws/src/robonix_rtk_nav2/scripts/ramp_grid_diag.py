#!/usr/bin/env python3
"""Same-time ramp diagnosis for the fixed (-2,0) -> (2,2) acceptance run.

This records the detector masks, published costmaps, raw Nav2 costs, path geometry,
and the first blocked path sample.  The ramp membership test is an independent
scene-audit convention; it is not used by the detector or planner.
"""
import argparse
import hashlib
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE)


def grid(msg):
    if msg is None:
        return None
    if hasattr(msg, 'metadata'):
        info = msg.metadata
    else:
        info = msg.info
    return np.asarray(msg.data, dtype=np.int16).reshape(info.size_y if hasattr(info, 'size_y') else info.height,
                                                          info.size_x if hasattr(info, 'size_x') else info.width)


def info(msg):
    return msg.metadata if hasattr(msg, 'metadata') else msg.info


def dimensions(i):
    return (int(i.size_x) if hasattr(i, 'size_x') else int(i.width),
            int(i.size_y) if hasattr(i, 'size_y') else int(i.height))


def raw_stats(msg):
    """Raw Costmap values: 254 is lethal, 255 is NO_INFORMATION."""
    if msg is None:
        return None
    i = info(msg)
    a = grid(msg)
    yy, xx = np.where(a == 254)
    uy, ux = np.where(a == 255)
    return {
        'frame': msg.header.frame_id,
        'origin': [i.origin.position.x, i.origin.position.y],
        'size': list(dimensions(i)),
        'lethal_n': int(len(xx)),
        'unknown_n': int(len(ux)),
        'bbox_lethal': bbox(i, xx, yy),
    }


def occupancy_stats(msg):
    if msg is None:
        return None
    i = info(msg)
    a = grid(msg)
    yy, xx = np.where(a >= 100)
    uy, ux = np.where(a < 0)
    return {
        'frame': msg.header.frame_id,
        'origin': [i.origin.position.x, i.origin.position.y],
        'size': list(dimensions(i)),
        'lethal_n': int(len(xx)),
        'unknown_n': int(len(ux)),
        'bbox_lethal': bbox(i, xx, yy),
    }


def bbox(i, xx, yy):
    if len(xx) == 0:
        return None
    return [float(i.origin.position.x + xx.min() * i.resolution),
            float(i.origin.position.x + xx.max() * i.resolution),
            float(i.origin.position.y + yy.min() * i.resolution),
            float(i.origin.position.y + yy.max() * i.resolution)]


def quaternion_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def transform_xy(points, transform):
    if transform is None or len(points) == 0:
        return points
    r = quaternion_matrix(transform.transform.rotation)
    t = transform.transform.translation
    out = np.asarray(points, dtype=float)
    xyz = np.c_[out, np.zeros(len(out))] @ r.T
    xyz[:, 0] += t.x
    xyz[:, 1] += t.y
    return xyz[:, :2]


RAMP_BOX = (0.0, 1.65, 0.5, 3.0)   # x_min, x_max, y_min, y_max（审计带，不是场景输入）


def ramp_membership(points):
    """Independent acceptance geometry: the wedge footprint x=[0,1.6], y=[0.5,3]."""
    p = np.asarray(points, dtype=float)
    x0, x1, y0, y1 = RAMP_BOX
    return ((p[:, 0] >= x0) & (p[:, 0] <= x1) &
            (p[:, 1] >= y0) & (p[:, 1] <= y1)) if len(p) else np.zeros(0, bool)


class N(Node):
    def __init__(self):
        super().__init__('ramp_grid_diag')
        self.d = {}
        self.path = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        topics = [('/rtk/drop/mask_map', 'mask_map', OccupancyGrid),
                  ('/rtk/drop/mask_odom', 'mask_odom', OccupancyGrid),
                  ('/global_costmap/costmap', 'g', OccupancyGrid),
                  ('/local_costmap/costmap', 'l', OccupancyGrid),
                  ('/global_costmap/costmap_raw', 'graw', Costmap),
                  ('/local_costmap/costmap_raw', 'lraw', Costmap)]
        for topic, key, typ in topics:
            self.create_subscription(typ, topic, lambda msg, k=key: self.d.__setitem__(k, msg), Q)
        self.create_subscription(Path, '/rtk/plan_path', self.on_path, Q)

    def on_path(self, msg):
        self.path = msg

    def path_points_in(self, frame):
        if self.path is None:
            return None
        points = np.array([(p.pose.position.x, p.pose.position.y) for p in self.path.poses], dtype=float)
        source = self.path.header.frame_id
        if source != frame:
            try:
                stamp = Time.from_msg(self.path.header.stamp)
                points = transform_xy(points, self.tf.lookup_transform(frame, source, stamp))
            except Exception:
                return None
        return points

    @staticmethod
    def cell(msg, point):
        if msg is None or point is None:
            return None
        i = info(msg)
        x = int(math.floor((point[0] - i.origin.position.x) / i.resolution))
        y = int(math.floor((point[1] - i.origin.position.y) / i.resolution))
        width, height = dimensions(i)
        if not (0 <= x < width and 0 <= y < height):
            return None
        a = grid(msg)
        return int(a[y, x])

    def path_probe(self, msg, raw=False):
        points = self.path_points_in(msg.header.frame_id) if msg is not None else None
        if points is None:
            return None
        values = [self.cell(msg, p) for p in points]
        blocked = (lambda v: v == 254) if raw else (lambda v: v is not None and v >= 100)
        first = next((idx for idx, v in enumerate(values) if blocked(v)), None)
        return {
            'path_frame': self.path.header.frame_id if self.path else None,
            'points': int(len(points)),
            'ramp_points': int(ramp_membership(points).sum()),
            'ramp_fraction': float(ramp_membership(points).mean()) if len(points) else 0.0,
            'first_blocked_index': first,
            'first_blocked_point': points[first].tolist() if first is not None else None,
            'first_blocked_value': values[first] if first is not None else None,
            'sample_values': values[::max(1, len(values)//20)],
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--json', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = N()
    rows = []
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.d) >= 6:
            path_pts = node.path_points_in('map')
            path = None
            if path_pts is not None:
                raw = json.dumps(np.round(path_pts, 4).tolist(), separators=(',', ':')).encode()
                path = {
                    'hash': hashlib.sha256(raw).hexdigest()[:16],
                    'points': int(len(path_pts)),
                    'length_m': float(sum(math.dist(a, b) for a, b in zip(path_pts, path_pts[1:]))),
                    'ramp_points': int(ramp_membership(path_pts).sum()),
                    'ramp_fraction': float(ramp_membership(path_pts).mean()) if len(path_pts) else 0.0,
                    'start': path_pts[0].tolist() if len(path_pts) else None,
                    'end': path_pts[-1].tolist() if len(path_pts) else None,
                }
            rows.append({
                'wall': time.time(),
                'maps': {k: (raw_stats(v) if k.endswith('raw') else occupancy_stats(v))
                         for k, v in node.d.items()},
                'path': path,
                'path_global_probe': node.path_probe(node.d.get('g')),
                'path_local_probe': node.path_probe(node.d.get('l')),
                'path_global_raw_probe': node.path_probe(node.d.get('graw'), raw=True),
                'path_local_raw_probe': node.path_probe(node.d.get('lraw'), raw=True),
            })
    result = {
        'rows': rows,
        'produced_by': 'ramp_grid_diag.py',
        'produced_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'ramp_geometry_audit': {'x_min': RAMP_BOX[0], 'x_max': RAMP_BOX[1],
                                'y_min': RAMP_BOX[2], 'y_max': RAMP_BOX[3]},
        'raw_costmap_semantics': {'lethal': 254, 'no_information': 255},
    }
    with open(args.json, 'w', encoding='utf-8') as out:
        json.dump(result, out, indent=2)
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

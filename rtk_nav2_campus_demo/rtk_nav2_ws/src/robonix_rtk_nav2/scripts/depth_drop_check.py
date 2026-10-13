#!/usr/bin/env python3
"""Stage-6 live evidence: read both real costmaps and the uninflated detector mask.

Run inside the ROS container after start-depth-drop.sh. The edge x=0 is used ONLY
for independent evaluation, never passed to the detector. No verdict is inferred
from screenshots or from a topic name existing in the graph.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


class Collector(Node):
    def __init__(self):
        super().__init__('depth_drop_check')
        self.latest = {}
        self.stamps = {}
        self.counts = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        for topic, key in (('/rtk/drop/mask_map', 'mask_map'),
                           ('/rtk/drop/mask_odom', 'mask_odom'),
                           ('/global_costmap/costmap', 'global'),
                           ('/local_costmap/costmap', 'local')):
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, k=key: self.on_map(m, k), qos)
        self.create_subscription(Image, '/rtk/camera/depth_image', self.on_depth,
                                 QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.depth_valid = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def on_map(self, msg, key):
        self.latest[key] = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp != self.stamps.get(key):
            self.counts[key] = self.counts.get(key, 0) + 1
        self.stamps[key] = stamp

    def on_depth(self, msg):
        if msg.encoding != '32FC1':
            return
        z = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        self.depth_valid = int(np.count_nonzero(np.isfinite(z) & (z > 0.05) & (z < 7.95)))


def values(msg, x, y, radius=0):
    x0, y0 = msg.info.origin.position.x, msg.info.origin.position.y
    r = msg.info.resolution
    ix, iy = int(math.floor((x-x0)/r)), int(math.floor((y-y0)/r))
    if not (0 <= ix < msg.info.width and 0 <= iy < msg.info.height):
        return None
    data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    patch = data[max(0, iy-radius):min(msg.info.height, iy+radius+1),
                 max(0, ix-radius):min(msg.info.width, ix+radius+1)]
    return sorted(set(int(v) for v in patch.reshape(-1)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--edge-x', type=float, default=0.0,
                   help='independent world geometry for EVALUATION only')
    p.add_argument('--y', type=float, default=0.0)
    p.add_argument('--seconds', type=float, default=25.0)
    p.add_argument('--json', default='')
    a = p.parse_args()
    rclpy.init()
    node = Collector()
    rows = []
    end = time.monotonic() + a.seconds
    last_cycle = -1
    target = (a.edge_x + 0.025, a.y)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.latest) != 4:
            continue
        cycle = node.counts.get('local', 0)
        if cycle == last_cycle:
            continue
        last_cycle = cycle
        row = {'wall': time.time(), 'cycle': cycle,
               'map': {}, 'depth_valid_pixels': node.depth_valid}
        try:
            transform = node.tf.lookup_transform('odom', 'map', Time())
            q = transform.transform.rotation
            theta = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            p = transform.transform.translation
            odom_xy = (p.x + math.cos(theta)*target[0] - math.sin(theta)*target[1],
                       p.y + math.sin(theta)*target[0] + math.cos(theta)*target[1])
        except Exception:
            odom_xy = None
        for key in ('mask_map', 'mask_odom', 'global', 'local'):
            msg = node.latest[key]
            xy = odom_xy if key in ('mask_odom', 'local') else target
            row['map'][key] = {'frame': msg.header.frame_id,
                               'stamp': node.stamps[key],
                               'queried_xy': xy,
                               'cell': values(msg, *xy) if xy else None,
                               'nearby': values(msg, *xy, 2) if xy else None}
        rows.append(row)
    if not rows:
        print('No live samples from all four maps; NOT TESTED')
        return 1
    mask = node.latest['mask_map']
    array = np.asarray(mask.data, dtype=np.int16).reshape(mask.info.height, mask.info.width)
    iy, ix = np.where(array == 100)
    edge_xs = mask.info.origin.position.x + (ix + .5) * mask.info.resolution
    err = float(np.median(np.abs(edge_xs-a.edge_x))) if len(edge_xs) else None
    sustained = {}
    for key in ('global', 'local'):
        observed = [r['map'][key]['nearby'] for r in rows]
        sustained[key] = sum(v is not None and 100 in v for v in observed)
    result = {'produced_by': 'depth_drop_check.py',
              'produced_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
              'via': 'live OccupancyGrid subscriptions, not RViz',
              'stage6_edge_x_for_evaluation_only': a.edge_x,
              'queried_map_xy': target,
              'sustained_near_edge_cycles': sustained,
              'resolution': mask.info.resolution,
              'edge_error_median_m': err,
              'edge_error_max_m': float(np.max(np.abs(edge_xs-a.edge_x))) if len(edge_xs) else None,
              'sampled_cycles': node.counts,
              'rows': rows}
    print(json.dumps({k: v for k, v in result.items() if k != 'rows'}, indent=2))
    print('last values:', json.dumps(rows[-1]['map']))
    if a.json:
        with open(a.json, 'w') as out:
            json.dump(result, out, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if (err is not None and err <= .25 and len(rows) >= 10 and
                 min(sustained.values()) >= 10) else 2


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Record a few existing front-lidar frames for offline self/ground echo checks.

Read-only ROS subscribers; no RTK, velocity, posture, new driver or map clearing.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_msgs.msg import TFMessage

from outdoornav.diagnostics import StaticFrames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='New output directory, never overwrites an existing one')
    parser.add_argument('--topic', default='/airy_201/points')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('readonly_lidar_snapshot')
    frames, records = StaticFrames(), []
    node.create_subscription(TFMessage, '/tf_static', frames.receive,
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def cloud(message):
        if len(records) >= 3:
            return
        matrix = frames.matrix(message.header.frame_id)
        if matrix is None:
            return  # Cannot label sensor coordinates as body coordinates.
        xyz = np.asarray([tuple(p) for p in read_points(message,
                         field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float).reshape(-1, 3)
        body = xyz @ matrix[:3, :3].T + matrix[:3, 3]
        distances = np.hypot(body[:, 0], body[:, 1])
        path = output / ('frame-%d.npz' % len(records))
        np.savez_compressed(path, sensor_xyz=xyz, base_xyz=body, base_from_sensor=matrix)
        records.append({'file': path.name, 'frame_id': message.header.frame_id,
            'stamp_s': message.header.stamp.sec + message.header.stamp.nanosec/1e9,
            'points': len(xyz), 'ranges_m': [0, .6, 1.2, 3, 8],
            'counts_by_horizontal_range': np.histogram(distances, bins=[0, .6, 1.2, 3, 8])[0].tolist(),
            'negative_z_points_within_3m': int(np.count_nonzero((distances < 3) & (body[:, 2] < 0)))})
    node.create_subscription(PointCloud2, args.topic, cloud, qos_profile_sensor_data)
    try:
        end = time.monotonic()+5
        while rclpy.ok() and time.monotonic() < end and len(records) < 3:
            rclpy.spin_once(node, timeout_sec=.1)
        result = {'scope': 'read-only lidar frames; no physical motion or RTK acquisition',
            'topic': args.topic, 'frames': records, 'static_faults': sorted(frames.faults),
            'complete': len(records) == 3}
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result))
        if not records:
            raise SystemExit('No framed pointcloud in 5 s; keep status UNKNOWN, not an empty scene')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

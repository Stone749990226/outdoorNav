"""Capture costmaps, sensor points and timestamped TF for offline diagnostics."""

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from tf2_ros import Buffer, TransformListener

from .maps import LATCHED
from outdoornav.coordinates import quaternion_matrix


def stamp(message):
    return message.sec + message.nanosec / 1e9


class Capture(Node):
    def __init__(self, directory, max_points):
        super().__init__("diagnostic_capture")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.directory, self.max_points = directory, max_points
        self.directory.mkdir(parents=True, exist_ok=False)
        self.seq, self.map, self.clouds = 0, None, {}
        self.complete = 0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_map, LATCHED
        )
        for name, topic in [
            ("scan", "/rtk/scan_points"),
            ("cam", "/rtk/camera/points"),
        ]:
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, name=name: self.clouds.update({name: msg}),
                qos_profile_sensor_data,
            )

    def on_map(self, message):
        self.map = message

    def transform(self, source, at=None):
        tf = self.tf.lookup_transform(
            "map", source, Time.from_msg(at) if at else Time()
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        matrix = np.column_stack(
            (quaternion_matrix(q.x, q.y, q.z, q.w), [t.x, t.y, t.z])
        )
        return matrix, {"M": matrix.tolist(), "stamp_s": stamp(tf.header.stamp)}

    def write(self, name, data):
        destination = self.directory / name
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_bytes(data if isinstance(data, bytes) else data.encode())
        temporary.replace(destination)

    def tick(self):
        if self.map is None:
            return False
        self.seq += 1
        suffix = f"{self.seq:04d}"
        message, info = self.map, self.map.info
        meta = {
            "frame_id": message.header.frame_id,
            "stamp": stamp(message.header.stamp),
            "resolution": info.resolution,
            "width": info.width,
            "height": info.height,
            "origin": {"x": info.origin.position.x, "y": info.origin.position.y},
        }
        record = {
            "schema_version": 1,
            "seq": self.seq,
            "wall_epoch": time.time(),
            "sim_t": self.get_clock().now().nanoseconds / 1e9,
            "map": meta,
        }
        values = np.asarray(message.data, dtype=np.int8).reshape(
            info.height, info.width
        )
        self.write(f"map_{suffix}.bin", values.tobytes())
        self.write(f"map_{suffix}.json", json.dumps(meta))
        # Snapshot diff uses a self-contained NumPy file with explicit OccupancyGrid costs.
        np.savez_compressed(
            self.directory / f"snapshot_{suffix}.npz",
            data=values,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            frame=message.header.frame_id,
        )
        try:
            matrix, record["robot_tf"] = self.transform("base_footprint")
            record["robot_xy_map"] = matrix[:2, 3].tolist()
        except Exception as error:
            record["robot_tf"] = {"error": str(error)}
        for name, cloud in self.clouds.items():
            item = {
                "frame_id": cloud.header.frame_id,
                "stamp": stamp(cloud.header.stamp),
            }
            try:
                raw = read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
                points = (
                    np.column_stack([raw[name] for name in ("x", "y", "z")])
                    if isinstance(raw, np.ndarray) and raw.dtype.names
                    else np.array([tuple(p) for p in raw], dtype=float).reshape(-1, 3)
                )
                item["n_total"] = len(points)
                stride = max(1, math.ceil(len(points) / self.max_points))
                points = points[::stride]
                item.update(n_kept=len(points), stride=stride)
                # Match cloud timestamps; never substitute today's TF for an old cloud.
                matrix, item["tf"] = self.transform(
                    cloud.header.frame_id, cloud.header.stamp
                )
                points = points @ matrix[:, :3].T + matrix[:, 3]
                self.write(
                    f"{name}_{suffix}.xyz",
                    "".join("%.4f %.4f %.4f\n" % tuple(p) for p in points),
                )
            except Exception as error:
                item["error"] = str(error)
            record[name] = item
        if all(
            name in record and "error" not in record[name] for name in ("scan", "cam")
        ):
            self.complete += 1
        self.write(f"tick_{suffix}.json", json.dumps(record, indent=2))
        print(f"Captured {suffix} at simulation time {record['sim_t']:.2f}", flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--interval", type=float, default=2)
    parser.add_argument("--max-points", type=int, default=60000)
    args = parser.parse_args()
    if not all(
        math.isfinite(v) and v > 0
        for v in (args.seconds, args.interval, args.max_points)
    ):
        parser.error("Durations and point limit must be positive and finite")
    rclpy.init()
    node = Capture(args.output, args.max_points)
    deadline, next_tick = (
        time.monotonic() + args.seconds,
        time.monotonic() + args.interval,
    )
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_tick:
                node.tick()
                next_tick += args.interval
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.complete else 2

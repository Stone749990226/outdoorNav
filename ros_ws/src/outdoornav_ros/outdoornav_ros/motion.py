"""Lidar-derived motion envelopes, kept separate from permanent and route constraints."""

import numpy as np
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from outdoornav.motion import MotionTracker, prediction_mask, scan_clusters
from .maps import LATCHED
from outdoornav.coordinates import quaternion_matrix


class MotionPrediction:
    def __init__(self, node):
        self.node = node
        self.tracker = MotionTracker()
        self.mask = node.create_publisher(
            OccupancyGrid, "/navigation/predicted_obstacles", LATCHED
        )
        self.info = node.create_publisher(
            CostmapFilterInfo, "/navigation/prediction_info", LATCHED
        )
        message = CostmapFilterInfo()
        message.header.frame_id = "map"
        message.type = 0
        message.filter_mask_topic = "/navigation/predicted_obstacles"
        message.base, message.multiplier = 0.0, 1.0
        self.info.publish(message)
        self.previous = None
        self.previous_origin = None
        node.create_subscription(
            LaserScan, "/rtk/scan", self.scan, qos_profile_sensor_data
        )
        node.create_timer(0.5, self.publish)
        self.publish()

    def scan(self, scan):
        try:
            transform = self.node.tf.lookup_transform(
                "map", scan.header.frame_id, Time.from_msg(scan.header.stamp)
            )
        except Exception:
            return
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (
            np.isfinite(ranges)
            # The roof-mounted lidar sees the robot body at 2..11 cm. Costmaps
            # clear that footprint; extrapolating it would invent an obstacle ahead.
            & (ranges >= max(0.25, scan.range_min))
            & (ranges < min(7.5, scan.range_max))
        )
        points = np.column_stack(
            (
                ranges[valid] * np.cos(angles[valid]),
                ranges[valid] * np.sin(angles[valid]),
                np.zeros(valid.sum()),
            )
        )
        q, t = transform.transform.rotation, transform.transform.translation
        points = points @ quaternion_matrix(q.x, q.y, q.z, q.w).T + [t.x, t.y, t.z]
        stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
        self.tracker.observe(scan_clusters(points[:, :2]), stamp)

    def publish(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        # Only the observed neighborhood needs predictions. Publishing the full
        # campus raster at 2 Hz would compete with the sensor and costmap streams.
        position = self.node.pose()
        x, y = (
            (position["x"], position["y"])
            if position
            else self.node.bundle["spawn"][:2]
        )
        grid = {
            "resolution": 0.1,
            "width": 240,
            "height": 240,
            "origin": [float(np.floor(x) - 12), float(np.floor(y) - 12)],
        }
        values = prediction_mask(grid, self.tracker.envelopes(now))
        if (
            self.previous is None
            or grid["origin"] != self.previous_origin
            or not np.array_equal(self.previous, values)
        ):
            self.mask.publish(self.node.maps.message(values, grid))
            self.previous = values
            self.previous_origin = grid["origin"]

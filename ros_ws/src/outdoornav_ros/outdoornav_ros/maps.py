"""Permanent static map and task-level keepout, published with explicit revisions."""

import numpy as np
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class Maps:
    def __init__(self, node, bundle, free):
        self.node, self.bundle = node, bundle
        self.static = node.create_publisher(
            OccupancyGrid, "/navigation/static_map", LATCHED
        )
        self.keepout = node.create_publisher(
            OccupancyGrid, "/navigation/keepout", LATCHED
        )
        self.info = node.create_publisher(
            CostmapFilterInfo, "/navigation/keepout_info", LATCHED
        )
        self.local_keepout = node.create_publisher(
            OccupancyGrid, "/navigation/local_keepout", LATCHED
        )
        self.local_info = node.create_publisher(
            CostmapFilterInfo, "/navigation/local_keepout_info", LATCHED
        )
        self.permanent = np.where(free, 0, 100).astype(np.int8)
        self.mask = self.permanent.copy()
        self.revision_ns = 0
        self.publish()

    def message(self, values, grid=None):
        grid = grid or self.bundle["grid"]
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.info.resolution = grid["resolution"]
        msg.info.width, msg.info.height = grid["width"], grid["height"]
        msg.info.origin.position.x, msg.info.origin.position.y = map(
            float, grid["origin"]
        )
        msg.info.origin.orientation.w = 1.0
        msg.data = values.ravel().tolist()
        return msg

    def activate(self, values):
        self.mask = values
        self.revision_ns = self.node.get_clock().now().nanoseconds
        self.publish()

    def publish(self):
        self.static.publish(self.message(self.permanent))
        self.keepout.publish(self.message(self.mask))
        # Route preference belongs to global selection. RPP's cost-based speed
        # regulation must continue to measure physical/boundary clearance locally.
        self.local_keepout.publish(
            self.message(np.where(self.mask == 100, 100, 0).astype(np.int8))
        )
        info = CostmapFilterInfo()
        info.header.frame_id = "map"
        info.header.stamp = self.node.get_clock().now().to_msg()
        info.type = 0
        info.filter_mask_topic = "/navigation/keepout"
        info.base, info.multiplier = 0.0, 1.0
        self.info.publish(info)
        info.filter_mask_topic = "/navigation/local_keepout"
        self.local_info.publish(info)

    def applied(self, message, tf_buffer):
        """Check actual published costs, not only that the mask was sent.

        Both maps must show task-forbidden cells lethal before a path is sent.
        Local odom cells are transformed into map before indexing the mask.
        """
        if message is None:
            return False
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= self.revision_ns:
            return False
        meta = message.info
        values = np.asarray(message.data).reshape(meta.height, meta.width)
        yy, xx = np.mgrid[: meta.height, : meta.width]
        x = meta.origin.position.x + (xx + 0.5) * meta.resolution
        y = meta.origin.position.y + (yy + 0.5) * meta.resolution
        if message.header.frame_id != "map":
            import rclpy

            try:
                transform = tf_buffer.lookup_transform(
                    "map", message.header.frame_id, rclpy.time.Time()
                )
            except Exception:
                return False
            t, q = transform.transform.translation, transform.transform.rotation
            yaw = np.arctan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            x, y = (
                np.cos(yaw) * x - np.sin(yaw) * y + t.x,
                np.sin(yaw) * x + np.cos(yaw) * y + t.y,
            )
        grid = self.bundle["grid"]
        cols = np.floor((x - grid["origin"][0]) / grid["resolution"]).astype(int)
        rows = np.floor((y - grid["origin"][1]) / grid["resolution"]).astype(int)
        valid = (
            (cols >= 0) & (rows >= 0) & (cols < grid["width"]) & (rows < grid["height"])
        )
        forbidden = np.ones_like(valid)
        forbidden[valid] = self.mask[rows[valid], cols[valid]] == 100
        return bool(np.all(values[forbidden] >= 99))

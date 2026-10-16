#!/usr/bin/env python3
"""Depth image visualization."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthDisplay(Node):
    def __init__(self):
        super().__init__("depth_display")
        self.declare_parameter("input_topic", "/rtk/camera/depth_image")
        self.declare_parameter("output_topic", "/rtk/camera/depth_display")
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("max_range", 6.0)

        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value, 5
        )
        self.create_subscription(
            Image,
            self.get_parameter("input_topic").value,
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "深度显示：%s -> %s（假彩色，只给人看）"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
            )
        )

    def on_depth(self, msg: Image):
        try:
            data = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )
        except ValueError:
            return
        valid = np.isfinite(data)
        clipped = np.clip(data, self.min_range, self.max_range)
        norm = (self.max_range - clipped) / max(1e-9, (self.max_range - self.min_range))
        out = np.where(valid, norm * 255.0, 0.0).astype(np.uint8)
        out[~valid] = 0

        img = Image()
        img.header = msg.header
        img.height = msg.height
        img.width = msg.width
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = msg.width
        img.data = out.tobytes()
        self.pub.publish(img)


def main(argv=None):
    rclpy.init(args=argv)
    node = DepthDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

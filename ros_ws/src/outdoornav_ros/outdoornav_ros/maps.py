"""Grid messages for sensor-derived predictions; no scene or road occupancy prior."""

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class Maps:
    def __init__(self, node, bundle):
        self.node, self.bundle = node, bundle

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

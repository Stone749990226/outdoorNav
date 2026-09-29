"""Small differential-drive simulator for validating the real Nav2 controller."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion(yaw):
    return (math.sin(yaw / 2), math.cos(yaw / 2))


class SimRobot(Node):
    def __init__(self):
        super().__init__("outdoor_nav_sim_robot")
        self.x = self.y = self.yaw = 0.0
        self.linear = self.angular = 0.0
        self.last_command = time.monotonic()
        self.last_step = self.last_command
        self.create_subscription(Twist, "/cmd_vel", self.on_velocity, 10)
        self.create_subscription(PoseStamped, "/outdoor_nav/sim_reset", self.on_reset, 10)
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.05, self.step)

    def on_velocity(self, command):
        self.linear = max(-0.6, min(0.6, command.linear.x))
        self.angular = max(-1.5, min(1.5, command.angular.z))
        self.last_command = time.monotonic()

    def on_reset(self, pose):
        self.x = pose.pose.position.x
        self.y = pose.pose.position.y
        q = pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.linear = self.angular = 0.0
        self.last_command = time.monotonic()
        self.publish_pose()

    def step(self):
        now = time.monotonic()
        dt = min(now - self.last_step, 0.1)
        self.last_step = now
        if now - self.last_command > 0.5:
            self.linear = self.angular = 0.0
        self.yaw += self.angular * dt
        self.x += self.linear * math.cos(self.yaw) * dt
        self.y += self.linear * math.sin(self.yaw) * dt
        self.publish_pose()

    def publish_pose(self):
        stamp = self.get_clock().now().to_msg()
        z, w = quaternion(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.twist.linear.x = self.linear
        odom.twist.twist.angular.z = self.angular
        self.odom.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)


def main():
    rclpy.init()
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

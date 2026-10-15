#!/usr/bin/env python3
"""Wheel odometry adaptation and configurable simulation fault injection."""

import math
import random

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry


class OdomBridge(Node):
    def __init__(self):
        super().__init__("wheel_odom_bridge")

        self.declare_parameter("input_topic", "/wheel_odom_raw")
        self.declare_parameter("output_topic", "/wheel_odom_used")
        # 第二路输出：同一份数据的"低权重"版本，只给全局 EKF 做速度平滑用。
        # 协方差在消息里放大（twist 方差 1.0），与正常那一路（1e-3）相差三个数量级——
        # 这就是"里程计低权重"在实现上落到哪里。
        self.declare_parameter("low_weight_topic", "/wheel_odom_low_weight")
        self.declare_parameter("low_weight_twist_variance", 1.0)
        # 正常那一路的协方差：**必须显式给**。Gazebo DiffDrive 发出来的 odometry
        # 协方差是全 0，robot_localization 拿到全 0 会当成"没有协方差信息"，
        # 行为不可预期。这里按"轮式里程计位姿不是绝对基准、但速度可信"来填。
        self.declare_parameter("pose_variance_xy", 1.0e-3)
        self.declare_parameter("pose_variance_yaw", 1.0e-2)
        self.declare_parameter("twist_variance_xy", 1.0e-3)
        self.declare_parameter("twist_variance_yaw", 1.0e-3)
        # 劣化开关与劣化参数（都可在运行中改）
        self.declare_parameter("degrade", False)
        self.declare_parameter("scale", 2.0)  # 位置/朝向缩放倍数
        self.declare_parameter("noise_stddev_m", 0.05)  # 位置噪声（米）
        self.declare_parameter("yaw_noise_stddev_deg", 5.0)

        self.degrade = bool(self.get_parameter("degrade").value)
        self.scale = float(self.get_parameter("scale").value)
        self.noise_m = float(self.get_parameter("noise_stddev_m").value)
        self.noise_yaw = math.radians(
            float(self.get_parameter("yaw_noise_stddev_deg").value)
        )

        self.pub = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10
        )
        self.low_pub = self.create_publisher(
            Odometry, self.get_parameter("low_weight_topic").value, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("input_topic").value, self.on_odom, 10
        )

        # 允许运行中改参数：没有这个回调，ros2 param set 只会改值、不会生效
        self.add_on_set_parameters_callback(self.on_param)

        self._pose_var_xy = float(self.get_parameter("pose_variance_xy").value)
        self._pose_var_yaw = float(self.get_parameter("pose_variance_yaw").value)
        self._twist_var_xy = float(self.get_parameter("twist_variance_xy").value)
        self._twist_var_yaw = float(self.get_parameter("twist_variance_yaw").value)

        self.count = 0
        self.get_logger().info(
            "轮式里程计处理节点：%s -> %s，当前 degrade=%s"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
                self.degrade,
            )
        )

    @staticmethod
    def covariance(var_xy, var_yaw):
        """6x6 对角协方差：x,y,yaw 用给定值，z/roll/pitch 给 1e6（表示不参与融合）。"""
        cov = [0.0] * 36
        cov[0 * 7] = var_xy
        cov[1 * 7] = var_xy
        cov[2 * 7] = 1.0e6
        cov[3 * 7] = 1.0e6
        cov[4 * 7] = 1.0e6
        cov[5 * 7] = var_yaw
        return cov

    def on_param(self, params):
        for p in params:
            if p.name == "degrade":
                self.degrade = bool(p.value)
            elif p.name == "scale":
                self.scale = float(p.value)
            elif p.name == "noise_stddev_m":
                self.noise_m = float(p.value)
            elif p.name == "yaw_noise_stddev_deg":
                self.noise_yaw = math.radians(float(p.value))
        self.get_logger().warn(
            "**里程计劣化状态变更**：degrade=%s scale=%.2f 位置噪声σ=%.3f m 航向噪声σ=%.1f°"
            % (self.degrade, self.scale, self.noise_m, math.degrees(self.noise_yaw))
        )
        # 这条 warn 也是"劣化确实生效"的现场证据：跑对照实验时从日志里就能看到它。
        return SetParametersResult(successful=True)

    def on_odom(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z),
        )

        if self.degrade:
            x = pos.x * self.scale + random.gauss(0.0, self.noise_m)
            y = pos.y * self.scale + random.gauss(0.0, self.noise_m)
            yaw = yaw * self.scale + random.gauss(0.0, self.noise_yaw)
            # 速度也一起败坏，避免"位置差但速度准"这种不自然的组合
            out.twist.twist.linear.x = msg.twist.twist.linear.x * self.scale
            out.twist.twist.linear.y = (
                msg.twist.twist.linear.y * self.scale
                + random.gauss(0.0, self.noise_m * 2.0)
            )
            out.twist.twist.angular.z = (
                msg.twist.twist.angular.z * self.scale
                + random.gauss(0.0, self.noise_yaw * 2.0)
            )
        else:
            x, y = pos.x, pos.y
            out.twist = msg.twist

        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = pos.z
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # z / roll / pitch 不参与二维融合，给一个很大的值（明确表达"不要用"）
        out.pose.covariance = self.covariance(self._pose_var_xy, self._pose_var_yaw)
        out.twist.covariance = self.covariance(self._twist_var_xy, self._twist_var_yaw)
        self.pub.publish(out)

        # 低权重版本：位置协方差拉满（反正全局 EKF 不取），速度协方差放大到 1.0。
        # 逐字段拷贝而不是 low.pose = out.pose —— 后者是**引用**，
        # 改 low 的协方差会把正常那一路的消息一起改掉。
        low = Odometry()
        low.header = out.header
        low.child_frame_id = out.child_frame_id
        low.pose.pose = out.pose.pose
        low.twist.twist = out.twist.twist
        var = float(self.get_parameter("low_weight_twist_variance").value)
        cov = self.covariance(1.0e6, var)
        low.pose.covariance = cov
        low.twist.covariance = cov
        self.low_pub.publish(low)

        self.count += 1


def main(argv=None):
    rclpy.init(args=argv)
    node = OdomBridge()
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

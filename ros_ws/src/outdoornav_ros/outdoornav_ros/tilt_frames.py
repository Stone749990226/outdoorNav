#!/usr/bin/env python3
"""Sensor transforms with measured IMU tilt for ground-height consistency."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from outdoornav.coordinates import quaternion_matrix
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def quaternion_of(r):
    """Rotation matrix → (x, y, z, w)，用最大分量分支，避免除零。"""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
            0.25 * s,
        )
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return (
            0.25 * s,
            (r[0, 1] + r[1, 0]) / s,
            (r[0, 2] + r[2, 0]) / s,
            (r[2, 1] - r[1, 2]) / s,
        )
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return (
            (r[0, 1] + r[1, 0]) / s,
            0.25 * s,
            (r[1, 2] + r[2, 1]) / s,
            (r[0, 2] - r[2, 0]) / s,
        )
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return (
        (r[0, 2] + r[2, 0]) / s,
        (r[1, 2] + r[2, 1]) / s,
        0.25 * s,
        (r[1, 0] - r[0, 1]) / s,
    )


class TiltFrames(Node):
    FRAMES = ("camera", "scan")

    def __init__(self):
        super().__init__("tilt_frames")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("imu_topic", "/rtk/imu")

        self.declare_parameter("altitude_topic", "/rtk/navsat")
        self.declare_parameter("altitude_scale", 1.0)
        # 发布率给得比 nav2 的 use 频率高一个量级：这两个帧原来是 latched 静态的、
        # "任何时刻都可查"，改成动态之后查询端（代价地图 / 检测器）仍按消息时间戳查，
        # 密一点才不至于在时间戳上落空。
        self.declare_parameter("publish_rate", 30.0)
        for name in self.FRAMES:
            self.declare_parameter("%s_frame" % name, "")
            for i, axis in enumerate("xyz"):
                self.declare_parameter("%s_%s" % (name, axis), 0.0)
            for axis in ("roll", "pitch", "yaw"):
                self.declare_parameter("%s_%s" % (name, axis), 0.0)
            self.declare_parameter("%s_parent" % name, "base_footprint")

        self.base = str(self.get_parameter("base_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.static = {}
        for name in self.FRAMES:
            child = str(self.get_parameter("%s_frame" % name).value)
            if not child:
                continue
            parent = str(self.get_parameter("%s_parent" % name).value)
            xyz = [float(self.get_parameter("%s_%s" % (name, a)).value) for a in "xyz"]
            rpy = [
                float(self.get_parameter("%s_%s" % (name, a)).value)
                for a in ("roll", "pitch", "yaw")
            ]
            self.static[name] = (parent, child, np.array(xyz), rpy_matrix(*rpy))

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.orientation = None
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.on_imu,
            qos_profile_sensor_data,
        )
        # 高度基准：第一次收到的高度就是"高台面"（map 的 z=0 由出生点定），
        # 之后的差值就是车相对 map 平面掉了多少。
        self.alt0 = None
        self.dz = 0.0
        self.alt_scale = float(self.get_parameter("altitude_scale").value)
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("altitude_topic").value),
            self.on_altitude,
            qos_profile_sensor_data,
        )
        self.pub = TransformBroadcaster(self)
        self.rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / self.rate, self.publish)
        self.get_logger().info(
            "tilt-aware sensor frames: %s（无 IMU/定位时退回静态）"
            % ", ".join("%s→%s" % (v[0], v[1]) for v in self.static.values())
        )

    def on_imu(self, msg):
        self.orientation = msg.orientation

    def on_altitude(self, msg):
        if msg.status.status < 0:  # STATUS_NO_FIX
            return
        if self.alt0 is None:
            self.alt0 = msg.altitude
            self.get_logger().info(
                "高度基准 %.3f m（之后按它补传感器帧的 z）" % self.alt0
            )
            return
        dz = (msg.altitude - self.alt0) * self.alt_scale
        dz = max(-2.0, min(2.0, dz))
        if abs(dz - self.dz) > 0.002:
            self.get_logger().info(
                "传感器帧高度修正 %.3f m（车比出生点低了这么多）" % dz,
                throttle_duration_sec=5.0,
            )
        self.dz = dz

    def tilt(self):
        """R_tilt = R_yaw⁻¹ · R_imu（去掉偏航，只留倾斜）；拿不到就返回 None。

        **必须非阻塞**：这些帧是动态发布的，若这里用带超时的 lookup（原来的写法），
        起栈早期 `map` 还没出现时每拍都会阻塞到超时 ⇒ 定时器实际只在 1 Hz 上下跑，
        而且节点看着像"卡住"。宁可这一拍退回静态，也不要阻塞发布。
        """
        if self.orientation is None:
            return None
        if not self.tf.can_transform(self.map_frame, self.base, Time()):
            return None
        try:
            tr = self.tf.lookup_transform(self.map_frame, self.base, Time())
        except Exception:
            return None
        q = tr.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        q = self.orientation
        return rpy_matrix(0.0, 0.0, -yaw) @ quaternion_matrix(q.x, q.y, q.z, q.w)

    def publish(self):
        r_imu = self.tilt()
        for name, (parent, child, xyz, r_static) in self.static.items():
            if r_imu is None:
                r = r_static
            else:
                r = r_imu @ r_static
            qx, qy, qz, qw = quaternion_of(r)
            out = TransformStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = parent
            out.child_frame_id = child
            out.transform.translation.x = float(xyz[0])
            out.transform.translation.y = float(xyz[1])
            # z = 安装高度 + 车相对 map 平面的高度变化（见 altitude_topic 的说明）
            out.transform.translation.z = float(xyz[2]) + self.dz
            out.transform.rotation.x = qx
            out.transform.rotation.y = qy
            out.transform.rotation.z = qz
            out.transform.rotation.w = qw
            self.pub.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    node = TiltFrames()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

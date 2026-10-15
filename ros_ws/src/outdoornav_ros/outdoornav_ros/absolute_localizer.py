#!/usr/bin/env python3
"""Compose GNSS position and IMU heading into map to odom; local EKF owns odom to base."""

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class AbsoluteLocalizer(Node):
    def __init__(self):
        super().__init__("absolute_localizer")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("fix_topic", "/odometry/gps")
        self.declare_parameter("imu_topic", "/rtk/imu")

        #       `Lookup would require extrapolation into the future`

        self.declare_parameter("tf_future", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)
        # 超时：源数据不新鲜时**停止发布** map→odom，让上层（路线跟随）按"定位失效"停车，
        # 而不是拿旧值继续跑
        self.declare_parameter("fix_timeout", 1.0)
        self.declare_parameter("imu_timeout", 1.0)

        self.declare_parameter("max_jump_m", 0.30)
        # 稳定门：连续 N 次未被判为跳变的读数之后才对外发布 map→odom，
        # 避免"刚起来的几帧还不准就把控制器带跑偏"。
        self.declare_parameter("settle_fixes", 3)

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_future = float(self.get_parameter("tf_future").value)
        self.fix_timeout = float(self.get_parameter("fix_timeout").value)
        self.imu_timeout = float(self.get_parameter("imu_timeout").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.fix = None  # (x, y) 已接受的位置
        self.fix_time = None
        self._settled = 0
        self.max_jump = float(self.get_parameter("max_jump_m").value)
        self.settle_fixes = int(self.get_parameter("settle_fixes").value)
        self.rejected = 0
        self.imu_yaw = None
        self.imu_time = None
        self.last_global = None
        self.status = None

        self.create_subscription(
            Odometry, self.get_parameter("fix_topic").value, self.on_fix, 10
        )
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self.on_imu, 20
        )
        self.global_pub = self.create_publisher(Odometry, "/odometry/global", 10)
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, "/rtk/localization_status", latched
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            "绝对定位：map→%s 直接取 %s 的位置 + %s 的航向；"
            "map→%s 由 (map→%s)∘(%s→%s)⁻¹ 合成。**不做传感器融合**。"
            % (
                self.base_frame,
                self.get_parameter("fix_topic").value,
                self.get_parameter("imu_topic").value,
                self.odom_frame,
                self.base_frame,
                self.odom_frame,
                self.base_frame,
            )
        )

    # ------------------------------------------------------------ 回调
    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_fix(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.fix is not None:
            jump = math.hypot(x - self.fix[0], y - self.fix[1])
            if jump > self.max_jump:
                # 不更新位置，也不重置稳定计数：坏点不该把"已稳定"的状态打掉
                self.rejected += 1
                self.get_logger().warn(
                    "丢弃一个跳变读数：与上一位置相差 %.2f m（上限 %.2f m），累计 %d 次"
                    % (jump, self.max_jump, self.rejected)
                )
                return
            self._settled += 1
        else:
            self._settled = 1
        self.fix = (x, y)
        self.fix_time = self.now_seconds()

    def on_imu(self, msg: Imu):
        self.imu_yaw = quaternion_to_yaw(msg.orientation)
        self.imu_time = self.now_seconds()

    def set_status(self, status):
        if status == self.status:
            return
        self.status = status
        out = String()
        out.data = status
        self.status_pub.publish(out)
        self.get_logger().warn("定位状态：%s" % status)

    # ------------------------------------------------------------ 主循环
    def tick(self):
        now = self.now_seconds()

        if self.fix is None or (now - self.fix_time) > self.fix_timeout:
            self.set_status("no_fix" if self.fix is None else "fix_stale")
            return
        if self._settled < self.settle_fixes:
            self.set_status("settling")
            return
        if self.imu_yaw is None or (now - self.imu_time) > self.imu_timeout:
            self.set_status("no_imu" if self.imu_yaw is None else "imu_stale")
            return

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            self.set_status("no_odom")
            return

        # ---- map→base：位置直接来自 RTK，航向直接来自 IMU ----
        mx, my = self.fix
        myaw = self.imu_yaw

        # ---- odom→base ----
        t = odom_to_base.transform.translation
        ox, oy = t.x, t.y
        oyaw = quaternion_to_yaw(odom_to_base.transform.rotation)

        odom_yaw = myaw - oyaw
        cos_p, sin_p = math.cos(odom_yaw), math.sin(odom_yaw)
        odom_x = mx - (cos_p * ox - sin_p * oy)
        odom_y = my - (sin_p * ox + cos_p * oy)

        now = self.get_clock().now()
        tf_stamp = (now + Duration(seconds=self.tf_future)).to_msg()
        msg_stamp = now.to_msg()
        tfm = TransformStamped()
        tfm.header.stamp = tf_stamp
        tfm.header.frame_id = self.map_frame
        tfm.child_frame_id = self.odom_frame
        tfm.transform.translation.x = odom_x
        tfm.transform.translation.y = odom_y
        tfm.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(odom_yaw)
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tfm)

        msg = Odometry()
        msg.header.stamp = msg_stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = mx
        msg.pose.pose.position.y = my
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(myaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(myaw / 2.0)
        self.global_pub.publish(msg)
        self.last_global = msg

        self.set_status("ok")


def main(argv=None):
    rclpy.init(args=argv)
    node = AbsoluteLocalizer()
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

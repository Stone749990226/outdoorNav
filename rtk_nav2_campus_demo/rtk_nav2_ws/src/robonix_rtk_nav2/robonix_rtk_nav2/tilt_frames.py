#!/usr/bin/env python3
"""给相机与激光的传感器帧补上**实测姿态**（替换两个静态变换）。

为什么必须这样（2026-09-30 阶段七实测的根因）：
  定位是二维的 —— `map→odom`（全局 EKF）与 `odom→base_footprint`（局部 EKF）都把车
  放在 z=0、且不含俯仰。可**车在坡上时是真的会俯仰的**（实测：坡 15% 时 Gazebo 里 8.5°；
  现行楔形坡 18.75% ⇒ 10.6°）。
  静态传感器帧一旦是"水平"的，那么用它们变换回来的地面点会**整体抬高约 f·sin(俯仰)**
  （f = 该点在前向的投影距离）：
    · 相机源的高度门是 0.30 m：坡道下方的平地被算到 0.30 m 以上 ⇒ 被当成障碍
      （15% 坡实测：f ≥ 4.07 m 起；现行 18.75% 坡 ⇒ f ≥ 3.2 m 起）；
    · 激光源的门是 0.02 m：它自己的地面回波被算到 `f·sin(俯仰)` ≈ 0.1~0.3 m
      ⇒ **只要车在坡上，正前方一两米就出现一条"地面墙"**
      → RPP 报 `detected collision ahead!` → `Controller patience exceeded` → 车停死。
  实测（ramp7-b）：车走到坡面入口（x≈0.12）就再也走不动，全局图里坡面与低台被标成致命格
  （5261 个致命格）；加本节点后同一场景 ramp7-c 降到 48 格且车走完坡面到达目标。

本节点做的事很小：把 `base_footprint → <传感器帧>` 从**静态**换成**按 IMU 实测姿态**发布。
  · 只补**倾斜**（roll/pitch）：把 IMU 的世界姿态里属于车体朝向的那部分去掉
    （`R_tilt = R_yaw⁻¹ · R_imu`，`R_yaw` 取 `map→base_footprint` 的偏航），
    再左乘原来的静态安装旋转。这样 `map → 传感器` 的总旋转 = `R_imu · R_static` ✓
    —— 偏航仍然完全由定位给出，本节点不碰它。
  · **平移：安装高度 + 高度修正**（相机 0.55、激光 0.11，另加 `dz`）。
    倾斜与高度是两回事：车下到低台后，二维定位仍把车放在 z=0，所有传感器点会被整体
    **算高 0.30 m**——reviewer 的 `rev7-ramp-e2` 实测到这一项的后果：**从低台回看**时
    障碍层把坡面带标成致命（global 420 格 ≥99、其中 136 = 100），规划器于是绕开坡面、
    改走平台外沿并掉出世界。本节点用**原始 GPS 高度**补这一项：以启动时第一次高度为基准，
    把 `dz = alt - alt0` 加到两个传感器帧的 z 上（**不动定位**：`navsat_transform` 仍
    `zero_altitude: true`、EKF 仍是二维）。
  · **真障碍不受损**：坡上一块 0.35 m 高的障碍被算成 0.35+h ≤ 0.65 m，仍在相机源的
    [0.30, 1.20] 里 ✓；激光源要探的矮障碍（0.02~0.30 m）也被抬高 ≤0.30 m，
    仍落在 [0.02, 1.20] 里 ✓ —— 第五阶段的"探低矮障碍"能力**没有被牺牲**。
    （这正是**不去抬 `min_obstacle_height`** 的原因：那样做会把矮障碍一起挡在门外。）

拿不到 IMU 或定位时**退回原来的静态变换**（不发布倾斜），启动阶段不会因此卡住。
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def rot_of(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr],
    ])


def quaternion_of(r):
    """Rotation matrix → (x, y, z, w)，用最大分量分支，避免除零。"""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s,
                (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s,
                (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
            0.25 * s, (r[1, 0] - r[0, 1]) / s)


class TiltFrames(Node):

    FRAMES = ('camera', 'scan')

    def __init__(self):
        super().__init__('tilt_frames')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('imu_topic', '/rtk/imu')
        # 高度项：二维定位**根本看不到车的高度**（`zero_altitude: true` 把 RTK 高度丢了），
        # 于是车下到低台（比高台低 0.30 m）之后，所有传感器点会被整体算高 0.30 m——
        # 实测（reviewer `rev7-ramp-e2`）：**从低台回看**，障碍层把坡面带标成致命
        # （global 420 格 ≥99、其中 136 = 100），规划器于是绕开坡面、改走平台外沿。
        # 倾斜补不了这一项（倾斜与高度是两回事），本节点改用**原始 GPS 高度**把它补回来：
        # 以启动时的第一次高度为基准，把 delta 加到两个传感器帧的 z 上。
        # 这**不改定位**（`navsat_transform` 仍然 `zero_altitude: true`，EKF 仍是二维），
        # 只让"传感器点往 map 系投影"这一步拿到正确的高度。
        #
        # ★ 2026-10-08 记一条**被撤回**的假设（别再走一遍）：阶段七实验 P 期间曾怀疑
        #   "第一帧高度未就位 ⇒ `dz` 带 +0.35 常量偏置 ⇒ 检测器 `floor` 浮空 ⇒ 永久失明"。
        #   复核后**不成立**：`ramp7-r`/`ramp7-s` 的节点日志是 `高度基准 4.140 m` +
        #   `传感器帧高度修正 -0.010 m`，基准是准的。那次的 `floor=0.35` 来自**只读探针
        #   自己把 `camera_height` 默认写成 0.19**（部署值是 0.55），是探针侧的量错，
        #   不是本节点的缺陷。故此处**保持"第一帧为基准"的原行为**；若将来真要改成
        #   "稳定后再锚"，必须先拿出基准确实取错的实测（例如节点日志里 `dz` 与真值下降量不符）。
        self.declare_parameter('altitude_topic', '/rtk/navsat')
        self.declare_parameter('altitude_scale', 1.0)
        # 发布率给得比 nav2 的 use 频率高一个量级：这两个帧原来是 latched 静态的、
        # "任何时刻都可查"，改成动态之后查询端（代价地图 / 检测器）仍按消息时间戳查，
        # 密一点才不至于在时间戳上落空。
        self.declare_parameter('publish_rate', 30.0)
        for name in self.FRAMES:
            self.declare_parameter('%s_frame' % name, '')
            for i, axis in enumerate('xyz'):
                self.declare_parameter('%s_%s' % (name, axis), 0.0)
            for axis in ('roll', 'pitch', 'yaw'):
                self.declare_parameter('%s_%s' % (name, axis), 0.0)
            self.declare_parameter('%s_parent' % name, 'base_footprint')

        self.base = str(self.get_parameter('base_frame').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.static = {}
        for name in self.FRAMES:
            child = str(self.get_parameter('%s_frame' % name).value)
            if not child:
                continue
            parent = str(self.get_parameter('%s_parent' % name).value)
            xyz = [float(self.get_parameter('%s_%s' % (name, a)).value) for a in 'xyz']
            rpy = [float(self.get_parameter('%s_%s' % (name, a)).value)
                   for a in ('roll', 'pitch', 'yaw')]
            self.static[name] = (parent, child, np.array(xyz), rpy_matrix(*rpy))

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.orientation = None
        self.create_subscription(Imu, str(self.get_parameter('imu_topic').value),
                                 self.on_imu, qos_profile_sensor_data)
        # 高度基准：第一次收到的高度就是"高台面"（map 的 z=0 由出生点定），
        # 之后的差值就是车相对 map 平面掉了多少。
        self.alt0 = None
        self.dz = 0.0
        self.alt_scale = float(self.get_parameter('altitude_scale').value)
        self.create_subscription(NavSatFix, str(self.get_parameter('altitude_topic').value),
                                 self.on_altitude, qos_profile_sensor_data)
        self.pub = TransformBroadcaster(self)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / self.rate, self.publish)
        self.get_logger().info(
            'tilt-aware sensor frames: %s（无 IMU/定位时退回静态）'
            % ', '.join('%s→%s' % (v[0], v[1]) for v in self.static.values()))

    def on_imu(self, msg):
        self.orientation = msg.orientation

    def on_altitude(self, msg):
        if msg.status.status < 0:      # STATUS_NO_FIX
            return
        if self.alt0 is None:
            self.alt0 = msg.altitude
            self.get_logger().info('高度基准 %.3f m（之后按它补传感器帧的 z）' % self.alt0)
            return
        dz = (msg.altitude - self.alt0) * self.alt_scale
        dz = max(-2.0, min(2.0, dz))
        if abs(dz - self.dz) > 0.002:
            self.get_logger().info('传感器帧高度修正 %.3f m（车比出生点低了这么多）' % dz,
                                   throttle_duration_sec=5.0)
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
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return rpy_matrix(0.0, 0.0, -yaw) @ rot_of(self.orientation)

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


if __name__ == '__main__':
    main()

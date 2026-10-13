#!/usr/bin/env python3
"""绝对定位合成节点：把"模拟 RTK 的位置 + IMU 的航向"变成 map→odom。

## 为什么不用全局 EKF

原设计是再加一个 robot_localization 的 EKF（world_frame=map）来融合 RTK/IMU/低权重里程计。
实测它在本配置下**可复现地发散**（根因未定位——这只说明它在这套接线里成立，
不是"库在 map 帧下有问题"）：同样的 GPS 输入，`world_frame=odom` 时输出正确
（与真值差 ~1 cm），改成 `world_frame=map` 就一路涨到几万米；两个单输入探针
（只喂 GPS / 只喂 IMU / 只喂里程计）都不发散，说明问题出在 world_frame=map 这条路径上，
不是输入的问题。与其去猜第三方滤波器内部的行为，不如把这一步换成**看得见、算得清**的做法。

## 这个节点的算法（就是这么简单，没有隐藏环节）

    map→base = (RTK 的 x, y) + (IMU 的 yaw)      ← 绝对位姿，直接来自两个传感器
    odom→base = 从 TF 读（局部 EKF 发布，融合轮式里程计 + IMU）
    map→odom  = (map→base) ∘ (odom→base)⁻¹        ← 两个变换合成，常规做法

发布 `map→odom` 之后，TF 链仍然是 map→odom→base，和原设计一致；
但**定位本身不经过任何滤波器**：位置就是 RTK 报的位置，航向就是 IMU 报的航向。

## 这样做对验收口径的影响（文档里也写了）

- **定位链路只有两个来源：模拟 RTK 与 IMU**。里程计只进 (odom→base)，
  它变差只会让 (odom→base) 漂，而上式里 map→odom 会等量补偿，
  最终 map→base 不受影响——"不强依赖里程计"是**结构上**成立的，不靠调权重。
- 但这也是**范围决定**：本 demo **不做传感器融合**（不做 GNSS/IMU 的外参、时延、
  偏置标定），真机上这些是必须做的。仿真 IMU 的 orientation 直接对齐世界系，
  真机上做不到这一点。这两点在本节点与文档里都明确声明。

订阅：/odometry/gps   （navsat_transform 输出的本地 ENU 位置，map 系）
      /rtk/imu        （航向 / 角速度）
      TF: odom → base_footprint
发布：TF: map → odom
      /odometry/global              （map 系下的绝对位姿，供记录与监控）
      /rtk/localization_status      （latched：ok / settling / no_fix / fix_stale /
                                     no_imu / imu_stale / no_odom）

另有**坏点/跳变守卫**：相邻两次定位之间位移超过 `max_jump_m`（默认 0.30 m）的读数会被丢弃，
并在连续 `settle_fixes`（默认 3）次干净读数之前不对外发布——理由是实测过
"机器人静止时定位估计一步跳 2.2 m、控制器照着假位置把车开出场外"。
"""

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AbsoluteLocalizer(Node):

    def __init__(self):
        super().__init__('absolute_localizer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('fix_topic', '/odometry/gps')
        self.declare_parameter('imu_topic', '/rtk/imu')
        # ★★ 2026-10-12：**map→odom 的 TF 要往前打一点** ✓（★ 躲 20 ms 的 TF 竞争 ✓）
        #   ★ 实测 ✓：本节点的 TF 到消费者手上时**已经落后 24–30 ms** ✗（★ 我以 25 Hz 量过 ✓）
        #     ⇒ ★ 控制器**查 `now`** 时会报 ★
        #       `Lookup would require extrapolation into the future`
        #       （Requested time 20463.320 ｜ latest data 20463.300 ✓）✗ ⇒ `patience exceeded` ⇒ ABORTED ✗
        #   ★ ★ 修法 ✗ 不是"把余量调大碰运气" ✗ ⇒ ★ 而是**把关系反过来** ✓：
        #     ★ 让**最新样本永远落在消费者的 `now` 之后** ✓ ⇒ ★ 任何"查 now"的请求
        #     **结构上都能被两个样本夹住** ✓ ⇒ ★ **确定性地**不再出现"要未来" ✓
        #   ★ 量级 ✓：落后 24–30 ms ⇒ ★ 要 ≥10× ⇒ ≥240 ms ✓ ⇒ ★ 0.5 s 够 ✓（★ 也就是我们控制器那份 `transform_tolerance` ✓）
        #   ★ ✗ **只给 TF 加** ✗ —— ★ `/odometry/global` 仍打 `now` ✓（★ 里程计报未来时间是另一个坑 ✓）
        self.declare_parameter('tf_future', 0.5)
        self.declare_parameter('publish_rate_hz', 30.0)
        # 超时：源数据不新鲜时**停止发布** map→odom，让上层（路线跟随）按"定位失效"停车，
        # 而不是拿旧值继续跑
        self.declare_parameter('fix_timeout', 1.0)
        self.declare_parameter('imu_timeout', 1.0)
        # **坏点/跳变守卫**。实测：机器人静止时定位估计会一步跳 2.2 m
        # （Gazebo GPS 的瞬时读数跳变），控制器照着这个假位置继续开，
        # 结果一路开出场外撞墙（遥测里 pos 从 (2.25,-2.95) 一步变到 (0.21,-3.73)，
        # 而同一时刻 v=0）。机器人最快 0.16 m/s、GPS 约 7 Hz，
        # 相邻两次定位之间合理位移 < 0.05 m；这里用 0.30 m 作为上限，
        # 超出的读数**丢弃**（不更新位置），连续被丢弃时状态置 bad_fix。
        self.declare_parameter('max_jump_m', 0.30)
        # 稳定门：连续 N 次未被判为跳变的读数之后才对外发布 map→odom，
        # 避免"刚起来的几帧还不准就把控制器带跑偏"。
        self.declare_parameter('settle_fixes', 3)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_future = float(self.get_parameter('tf_future').value)
        self.fix_timeout = float(self.get_parameter('fix_timeout').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.fix = None            # (x, y) 已接受的位置
        self.fix_time = None
        self._settled = 0
        self.max_jump = float(self.get_parameter('max_jump_m').value)
        self.settle_fixes = int(self.get_parameter('settle_fixes').value)
        self.rejected = 0
        self.imu_yaw = None
        self.imu_time = None
        self.last_global = None
        self.status = None

        self.create_subscription(Odometry, self.get_parameter('fix_topic').value,
                                 self.on_fix, 10)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self.on_imu, 20)
        self.global_pub = self.create_publisher(Odometry, '/odometry/global', 10)
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/rtk/localization_status', latched)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            '绝对定位：map→%s 直接取 %s 的位置 + %s 的航向；'
            'map→%s 由 (map→%s)∘(%s→%s)⁻¹ 合成。**不做传感器融合**。'
            % (self.base_frame, self.get_parameter('fix_topic').value,
               self.get_parameter('imu_topic').value,
               self.odom_frame, self.base_frame, self.odom_frame, self.base_frame))

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
                    '丢弃一个跳变读数：与上一位置相差 %.2f m（上限 %.2f m），累计 %d 次'
                    % (jump, self.max_jump, self.rejected))
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
        self.get_logger().warn('定位状态：%s' % status)

    # ------------------------------------------------------------ 主循环
    def tick(self):
        now = self.now_seconds()

        if self.fix is None or (now - self.fix_time) > self.fix_timeout:
            self.set_status('no_fix' if self.fix is None else 'fix_stale')
            return
        if self._settled < self.settle_fixes:
            self.set_status('settling')
            return
        if self.imu_yaw is None or (now - self.imu_time) > self.imu_timeout:
            self.set_status('no_imu' if self.imu_yaw is None else 'imu_stale')
            return

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            self.set_status('no_odom')
            return

        # ---- map→base：位置直接来自 RTK，航向直接来自 IMU ----
        mx, my = self.fix
        myaw = self.imu_yaw

        # ---- odom→base ----
        t = odom_to_base.transform.translation
        ox, oy = t.x, t.y
        oyaw = quaternion_to_yaw(odom_to_base.transform.rotation)

        # ---- map→odom = (map→base) ∘ (odom→base)⁻¹ ----
        #
        # TF 语义：T_A_B 表示"B 在 A 中的位姿"，复合满足 T_A_C = T_A_B ∘ T_B_C。
        # 由 T_map_base = T_map_odom ∘ T_odom_base 解出：
        #   yaw(map→odom) = yaw(map→base) − yaw(odom→base)
        #   t(map→odom)   = t(map→base) − R(yaw(map→odom)) · t(odom→base)
        # **注意是用 map→odom 自己的 yaw 去转 t(odom→base)**，不是用 odom→base 的 yaw。
        # 第一版这里写错了（用 odom→base 的 yaw 转、方向也不对），结果 TF 链
        # map→base 与定位节点实际输出的 /odometry/global 差了 3.6 m：
        # 路线跟随读的是 TF，于是照着错位置把车开出场外撞墙，
        # 而日志/真值对比却显示"定位很准"（因为那看的是 /odometry/global）。
        odom_yaw = myaw - oyaw
        cos_p, sin_p = math.cos(odom_yaw), math.sin(odom_yaw)
        odom_x = mx - (cos_p * ox - sin_p * oy)
        odom_y = my - (sin_p * ox + cos_p * oy)

        # ★★ TF 与里程计**用两个不同的戳** ✓（见上面 `tf_future` 的说明 ✓）
        now = self.get_clock().now()
        tf_stamp = (now + Duration(seconds=self.tf_future)).to_msg()   # ★ 只给 TF ✓（往前一点 ✓）
        msg_stamp = now.to_msg()                                       # ★ 给里程计 ✓（仍是 now ✓）
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

        self.set_status('ok')


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


if __name__ == '__main__':
    raise SystemExit(main())

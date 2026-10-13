#!/usr/bin/env python3
"""frontend_adapter.py —— 把**我们的车**接到**他们的前端那条链**上（★ 我方实现 ✓）

★ 背景 ✓（只读侦察见 `docs/frontend-integration-recon.md` ✓）：
  ★ 他们的链里，车要提供**两样** ✓ —— ★ 收 `/cmd_vel` ✓（★ 我们现有链已经有了 ✓）＋ ★
  ★ **发 `/odom`（`nav_msgs/Odometry`）** ✓，★ 而 `bridge.py:125–129` **原样**取它的
  `pose.pose.position` 当作 ★ **frame `map` 的米值** ✓（★ 一个字的变换都不做 ✗）。
  ★ 而他们的 `sim.launch.py:34` 起的是 ★ 静态 `map→odom`（identity）✓ ⇒ ★ 在他们的约定里
  **`map` ≡ `odom`** ✓。
  ★ ⇒ ★ 本节点发 `/odom` 时，★ **position 用我们 TF 里 `map→base_link` 的米值** ✓
    ✗ **不做任何变换** ✗（★ 恒等 ✓ —— 见侦察文档里那条唯一变换 ✓）。

★ 本节点**只加不改** ✓：
  ★ ① 发 `/odom` ✓（★ 顶掉他们 `sim_robot.py` 的角色 ✓）
  ★ ② ★ **吞掉 `/outdoor_nav/sim_reset`** ✓ —— ★ 业主口径"**起始点就是小车位置**" ✓
    ⇒ ★ 车本来就在那儿 ⇒ ★ **绝不让车被瞬移** ✗（★ 收到只记一条日志 ✓）
  ★ ✗ 不碰我们的 TF ✓（★ `map→odom` 仍**只由我们的 `navsat_transform` 发** ✓ ⇒ ★ 发布者恒为 1 ✓）
  ★ ✗ 不动我们的 `map`／`odom` 语义 ✓（★ 我们那条链照旧 ✓）

用法（容器内 ✓）：
  python3 /work/scripts/rtk-nav2/container/frontend_adapter.py
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

MAP_FRAME = "map"
BASE_FRAME = "base_link"      # ★ 他们的 costmap 用 base_link（nav2_sim.yaml: robot_base_frame ✓）
ODOM_TOPIC = "/odom"          # ★ 他们的 bridge 订阅的就是这个名字 ✓
RESET_TOPIC = "/outdoor_nav/sim_reset"
ODOM_FRAME_ID = "odom"        # ★ 他们的约定：map ≡ odom（静态 identity ✓）
CHILD_FRAME_ID = "base_link"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FrontendAdapter(Node):
    def __init__(self):
        super().__init__("frontend_adapter")
        # ★★ 2026-10-12：**必须开仿真时间** ✗ —— 实测踩到（★ 这一条只有真跑才暴露 ✓）：
        #   ★ 不开 ⇒ 本节点的时间戳是**墙钟**（1.79e9 秒 ✓），而我们的栈走 **Gazebo 仿真钟**
        #     （TF 的时间戳是 **209 s** ✓）⇒ ★ 控制器报
        #       `Transform data too old when converting from map to odom` ✗
        #       （Data time 1791809151 vs Transform time 209 ✓）
        #     ⇒ ★ 而且它随后打了 ★ **`Reached the goal!`** ✗ —— ★ **车一步没动就"到达"** ✗✗
        #     （★ 又一个"信号 ≠ 完成" ✓ ⇒ ★ 这也是"**到达必须要求停稳**"那条规矩的现场证据 ✓）
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        # ★ retained ＝ 与他们的 `sim_robot.py` 一致（晚到的订阅者也能拿到最后一条 ✓）
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Odometry, ODOM_TOPIC, retained)
        self.create_subscription(PoseStamped, RESET_TOPIC, self.on_reset, 10)
        self.create_timer(0.05, self.publish_odom)      # 20 Hz，与他们的假车同频 ✓
        self.resets_seen = 0
        self.published = 0
        self.last_error = ""
        self.get_logger().info(
            "frontend_adapter 起 ✓ 发 %s（position ＝ TF %s→%s 的米值 ✓、✗ 不变换）＋ 吞掉 %s"
            % (ODOM_TOPIC, MAP_FRAME, BASE_FRAME, RESET_TOPIC))

    def on_reset(self, message):
        # ★★ 业主口径：起始点就是小车位置 ⇒ ★ **吞掉**，✗ 不瞬移 ✗
        self.resets_seen += 1
        p = message.pose.position
        self.get_logger().info(
            "收到 %s（第 %d 次）⇒ ★ 按口径**吞掉** ✓ 车不动 ✓ 它想设的是 (%.3f, %.3f)"
            % (RESET_TOPIC, self.resets_seen, p.x, p.y))

    def publish_odom(self):
        try:
            tf = self.tf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception as error:                      # noqa: BLE001（TF 没到就等下一拍 ✓）
            self.last_error = str(error)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = ODOM_FRAME_ID
        message.child_frame_id = CHILD_FRAME_ID
        message.pose.pose.position.x = float(t.x)
        message.pose.pose.position.y = float(t.y)
        message.pose.pose.orientation = q
        message.pose.covariance[0] = message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.02
        self.publisher.publish(message)
        self.published += 1


def main():
    rclpy.init()
    node = FrontendAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("共发 %d 条 %s ｜ 共吞掉 %d 次 sim_reset"
                               % (node.published, ODOM_TOPIC, node.resets_seen))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

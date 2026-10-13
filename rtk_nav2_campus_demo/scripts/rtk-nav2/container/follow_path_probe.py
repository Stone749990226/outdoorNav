#!/usr/bin/env python3
"""follow_path_probe.py —— 把一条**我们自己的路径**发给 nav2 的 `follow_path` 动作（★ 我方实现 ✓）

★ 为什么有它 ✓：★ 「顶掉他们的 `sim_robot.py`」的可验版本 ✓ ——
  ★ 他们那条链里，路径是**前端算好的** ✓，★ 经 `bridge.py` 发 ★ **`FollowPath` 动作** ✓
  （`bridge.py:98` ✓、`:211–215` ✓）⇒ ★ 只要**我们能接住这个动作、车真跟着走** ✓
  ⇒ ★ 就证明"车这一侧已经能顶替" ✓（★ 那时前端／桥一接上就成 ✓）。

★★ 实测的**兼容性差异**（★ 这条是它要的判据 ✓）：
  ★ `controller_id` ✓：★ 两边**同名 `FollowPath`** ✓（我们 `campus_nav2.yaml:356` ✓／他们 `nav2_sim.yaml:14` ✓）
  ★ `goal_checker_id` ✗：★ 我们的是 **`general_goal_checker`** ✓（`campus_nav2.yaml:355` ✓）；
    ★ 他们的桥**硬写 `goal_checker`** ✗（`bridge.py:214` ✓）⇒ ★ **直接拿他们的桥打我们的
    controller_server 会被拒** ✗ ⇒ ★ 本探针默认用**我们的** id ✓（可用参数覆盖 ✓）
  ★ `progress_checker` 两边都对得上 ✓（他们用旧名 `progress_checker_plugin` ✓，不影响动作调用 ✓）

用法（容器内 ✓）：
  python3 /work/scripts/rtk-nav2/container/follow_path_probe.py --path "x1,y1 x2,y2 ..." \
      [--controller FollowPath] [--goal-checker general_goal_checker] [--timeout 300]

★ 判据落在**原始量**上 ✓：★ 报"**离目标 X m**"（`FollowPath` 的 `distance_to_goal` ✓），
★ 并且 ★ **到达必须要求停稳** ✓（★ 只判距离会把行进中记成到达 ✗ —— 本项目栽过三次 ✓）。
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as RosPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

MAP_FRAME = "map"
BASE_FRAME = "base_link"
SPACING = 0.25          # ★ 与 `bridge.py:67` 的稠密化间距一致 ✓


def parse_path(text):
    points = []
    pairs = [p for p in text.split() if p.strip()]
    for pair in pairs:
        x, y = pair.split(",")
        points.append((float(x), float(y)))
    if len(points) < 2:
        raise ValueError("路径至少要两个点")
    return points


def dense(points, spacing=SPACING):
    result = [points[0]]
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        length = math.hypot(bx - ax, by - ay)
        if length < 1e-9:
            continue
        steps = max(1, math.ceil(length / spacing))
        for i in range(1, steps + 1):
            t = i / steps
            result.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return result


class Probe(Node):
    POSE_TOPIC = "/ground_truth/odom"   # ★ 仿真真值（★ 只做**验收取证**用 ✓，标明来源 ✓）

    def __init__(self, points, controller, goal_checker, timeout, settle):
        super().__init__("follow_path_probe")
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
        self.settle = settle
        self.points = dense(points)
        self.controller = controller
        self.goal_checker = goal_checker
        self.deadline = time.monotonic() + timeout
        self.path_publisher = self.create_publisher(RosPath, "/follow_path_probe/path", 1)
        self.client = ActionClient(self, FollowPath, "follow_path")
        self.feedback = None
        self.done = False
        self.status = None
        self.goal_handle = None
        # ★★ 2026-10-12：**验收证据要能从产物复算** ✗ 不能只在屏幕上 ✗
        #   ★ 到点数与"停稳"必须**落进日志** ✓ ⇒ 订阅位姿源，记 起止位姿 ＋ 末速 ✓
        self.samples = []           # [(x, y, vx, wz)]
        self.create_subscription(Odometry, self.POSE_TOPIC, self.on_pose, 20)

    def on_pose(self, message):
        p = message.pose.pose.position
        t = message.twist.twist
        self.samples.append((float(p.x), float(p.y),
                             float(t.linear.x), float(t.angular.z)))

    def pose(self):
        try:
            tf = self.tf.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except Exception:                                   # noqa: BLE001
            return None
        t = tf.transform.translation
        return (float(t.x), float(t.y))

    def build_path(self):
        # ★ 用**零时间戳** ⇒ 消费方按"用最新的 TF"解释 ✓（✗ 不拿节点时钟赌 ✓）
        from builtin_interfaces.msg import Time as RosTime
        stamp = RosTime()
        path = RosPath()
        path.header.frame_id = MAP_FRAME
        path.header.stamp = stamp
        for i, (x, y) in enumerate(self.points):
            ax, ay = self.points[max(0, i - 1)]
            bx, by = self.points[min(len(self.points) - 1, i + 1)]
            item = PoseStamped()
            item.header = path.header
            item.pose.position.x = x
            item.pose.position.y = y
            yaw = math.atan2(by - ay, bx - ax)
            item.pose.orientation.z = math.sin(yaw / 2)
            item.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(item)
        return path

    def go(self):
        self.get_logger().info("等 follow_path 动作服务器（controller_id=%s）…" % self.controller)
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("★ follow_path 动作服务器**没出现** ⇒ 顶不上 ✗")
            return 1
        self.get_logger().info("动作服务器在 ✓")
        path = self.build_path()
        self.path_publisher.publish(path)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller
        goal.goal_checker_id = self.goal_checker
        start = self.pose()
        self.get_logger().info("起点 = %s ｜ 目标 = (%.3f, %.3f) ｜ 稠密点数 = %d"
                               % (start, self.points[-1][0], self.points[-1][1], len(self.points)))
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_response)
        # ★★ 2026-10-12：**必须 spin** ✗ —— 实测踩到 ✓（★ 一个缺陷解释了两个症状 ✓）：
        #   ★ 原写法 `while not done: time.sleep(0.2)` ✗ ⇒ ★ **回调永远不执行** ✗ ⇒
        #     ★ ① 动作结果收不到 ⇒ 打 `status = NO_RESULT` ✗
        #     ★ ② 位姿订阅收不到 ⇒ 打「位姿源没采到样本」✗
        #   ⇒ ★ 两者**同一个因** ✓（★ 我一度以为是"记账缺陷" ✗，那是**表相** ✗）
        while not self.done and time.monotonic() < self.deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        # ★★ 2026-10-12：**动作返回 ≠ 车停了** ✗ —— 实测踩到 ✓：
        #   ★ 动作一返回就停止采样 ⇒ ★ 末速 0.284 m/s ✗ ⇒ ★ 那个窗口量的是"**减速中**" ✗
        #   ★ 而"到达必须要求停稳"是本项目的规矩 ✓ ⇒ ★ **完成后还要再采几秒** ✓
        if self.done:
            self.get_logger().info("动作已结束 ⇒ ★ 再采 %.1f s 看**停稳** ✓（✗ 不等就等于没验 ✗）"
                                   % self.settle)
            settle_until = time.monotonic() + self.settle
            while time.monotonic() < settle_until:
                rclpy.spin_once(self, timeout_sec=0.2)
        return 0

    def on_feedback(self, message):
        self.feedback = message.feedback.distance_to_goal

    def on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("★ **被拒** ✗ —— controller_id／goal_checker_id 对不上？"
                                    "（★ 实测：我们的 goal_checker 叫 general_goal_checker ✓，"
                                    "他们的桥硬写 goal_checker ✗）")
            self.status = "REJECTED"
            self.done = True
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        self.status = future.result().status
        self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", required=True, help='例如 "0,0 5,0 5,5"（frame map，米 ✓）')
    ap.add_argument("--controller", default="FollowPath")
    ap.add_argument("--goal-checker", default="general_goal_checker")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--settle", type=float, default=6.0, help="动作结束后继续采样多久（★ 判停稳 ✓）")
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Probe(parse_path(args.path), args.controller, args.goal_checker, args.timeout, args.settle)
    try:
        rc = node.go()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
             GoalStatus.STATUS_CANCELED: "CANCELED", None: "NO_RESULT",
             "REJECTED": "REJECTED"}
    import math
    print("=" * 68)
    print("★ 验收取证（★ 全部原始量 ✓，来源：%s ✓）" % node.POSE_TOPIC)
    print("  status                = %s" % names.get(node.status, node.status))
    print("  最后一条 distance_to_goal = %s m" % node.feedback)
    print("  稠密点数                = %d" % len(node.points))
    if node.samples:
        sx, sy, _, _ = node.samples[0]
        ex, ey, evx, ewz = node.samples[-1]
        tx, ty = node.points[-1]
        print("  起点（位姿源）           = (%.4f, %.4f)" % (sx, sy))
        print("  终点（位姿源）           = (%.4f, %.4f)" % (ex, ey))
        print("  ★ 终点到目标的距离        = %.4f m" % math.dist((ex, ey), (tx, ty)))
        print("  ★ 末速（线 x / 角 z，settle 之后） = %.4f / %.4f" % (evx, ewz))
        tail = node.samples[-25:] if len(node.samples) >= 25 else node.samples   # ★ 覆盖 settle 窗口 ✓
        print("  ★ 末段（含 %.0f s 静置）|线速度| 最大 = %.4f m/s" % (node.settle, max(abs(v) for _, _, v, _ in tail)))
        print("  ★ 末段（含 %.0f s 静置）|角速度| 最大 = %.4f rad/s" % (node.settle, max(abs(w) for _, _, _, w in tail)))
    else:
        print("  ★ 位姿源没采到样本 ✗（%s 没在发？）" % node.POSE_TOPIC)
    print("=" * 68)
    return rc


if __name__ == "__main__":
    sys.exit(main())

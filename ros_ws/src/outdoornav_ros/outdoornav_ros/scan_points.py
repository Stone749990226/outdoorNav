#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Project lidar scans with far clearing endpoints outside the marking range."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

# nav2 `laserScanValidInfCallback` 里那个常量（反汇编：0x38D1B717 = 1e-4）。
# **这个 1e-4 是"取替换值"的依据，不是随手写的保险量**：它同时是
# `range_max - 1e-4 < range_max` 这条"不被 projectLaser 丢掉"的余量。
INF_EPS = 1e-4


class ScanPoints(Node):
    def __init__(self):
        super().__init__("scan_points")
        self.declare_parameter("input_topic", "/rtk/scan")
        self.declare_parameter("output_topic", "/rtk/scan_points")
        # 替换值：默认 = range_max(8.0) - 1e-4，与 nav2 的 inf_is_valid 常量逐位相同。
        self.declare_parameter("far_range", 8.0 - INF_EPS)
        self.declare_parameter("obstacle_max_range", 7.5)
        self.declare_parameter("emit_far_points", True)
        self.declare_parameter("log_every", 100)

        p = self.get_parameter
        self.input_topic = p("input_topic").value
        self.output_topic = p("output_topic").value
        self.far_param = float(p("far_range").value)
        self.obstacle_max_range = float(p("obstacle_max_range").value)
        self.emit_far = p("emit_far_points").value
        self.log_every = int(p("log_every").value)

        # laser_geometry 的 Python 绑定（与 nav2 内部用的是同一份实现）。
        # **不放在模块顶层 import**：那样 `--help` 或导入本模块做单测时也会拉起来，
        # 而它是一个编译扩展；放在 __init__ 里，缺了能在启动日志里直接看到。
        from laser_geometry import LaserProjection

        self._projector = LaserProjection()

        self.frames = 0
        self.beams_total = 0
        self.noecho_total = 0
        self.far_effective = None  # 第一次收到 scan 后按 range_max 定死
        self.dropped_total = 0
        self.warned_no_point = False

        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self.input_topic, self.on_scan, qos_profile_sensor_data
        )
        self.get_logger().info(
            "激光->点云：%s -> %s（emit_far_points=%s，far_range 参数=%.5f，"
            "本源的 obstacle_max_range=%.2f）"
            % (
                self.input_topic,
                self.output_topic,
                "开" if self.emit_far else "关",
                self.far_param,
                self.obstacle_max_range,
            )
        )

    # ------------------------------------------------------------------ 自检
    def _check_far(self, range_max):
        """按**这一帧**的 `range_max` 定死替换值并做三条前提的自检（只打一次）。"""
        far = min(self.far_param, range_max - INF_EPS)
        # 前提 1：必须 > obstacle_max_range，否则会被标成致命障碍（假墙）。
        bad = far <= self.obstacle_max_range
        self.far_effective = far
        self.get_logger().info(
            "自检：range_max=%.5f ⇒ 替换值 far=%.5f（参数给的是 %.5f，%s）"
            % (
                range_max,
                far,
                self.far_param,
                "未触发上调" if far == self.far_param else "**被 range_max 压下来了**",
            )
        )
        self.get_logger().info(
            "自检（前提 1，最危险的一条）：far=%.5f vs obstacle_max_range=%.2f ⇒ %s"
            % (
                far,
                self.obstacle_max_range,
                "far 在**标记量程之外**，无回波束不会被标成障碍 ✓"
                if not bad
                else "** far 落在 obstacle_max_range 以内 ⇒ 开阔方向会长出一圈假墙 ✗ **",
            )
        )
        # 前提 3：<= raytrace_max_range（8.0）。这里只报数，不判定——
        # raytrace_max_range 不在本节点，写死一个阈值就成了"抄来的数"。
        self.get_logger().info(
            "自检（前提 3）：清除射线只到 far=%.5f m，请与本源 raytrace_max_range 对照"
            % far
        )
        if bad:
            self.get_logger().error(
                "** far 落在标记量程内 ⇒ 会出现假墙；请把 far_range 调到 "
                "> obstacle_max_range(%s) 且 < range_max **" % self.obstacle_max_range
            )

    # ------------------------------------------------------------------ 主循环
    def on_scan(self, msg: LaserScan):
        if self.far_effective is None:
            self._check_far(float(msg.range_max))

        n = len(msg.ranges)
        # ---- 无回波判定：inf / nan / 超 range_max / 小于 range_min ----
        # 用 np.asarray 一次取完（LaserScan.ranges 在 Python 绑定里是 list，逐束 float() 会很慢）。
        r = np.asarray(msg.ranges, dtype=np.float64)
        finite = np.isfinite(r)
        noecho = ~finite | (r >= msg.range_max) | (r < msg.range_min)
        n_noecho = int(noecho.sum())

        if self.emit_far and n_noecho:
            # 复制一份再替换：**不改调用方的消息对象**（订阅回调里改 msg 会在
            # 同一进程内的其他订阅者眼里也变样，属于难查的耦合）。
            sub = LaserScan()
            sub.header = msg.header
            sub.angle_min = msg.angle_min
            sub.angle_max = msg.angle_max
            sub.angle_increment = msg.angle_increment
            sub.time_increment = msg.time_increment
            sub.scan_time = msg.scan_time
            sub.range_min = msg.range_min
            sub.range_max = msg.range_max
            rr = np.where(noecho, self.far_effective, r)
            sub.ranges = rr.tolist()
            sub.intensities = []
            src = sub
        else:
            # 关掉时**不改动**：无回波的束在 projectLaser 里直接变成 0 个点
            # （= nav2 `inf_is_valid: false` 的行为）。
            src = msg

        cloud = self._projector.projectLaser(src)

        n_out = cloud.width * cloud.height
        if n_out == 0 and not self.warned_no_point:
            self.warned_no_point = True
            self.get_logger().error(
                "** projectLaser 投影出 0 个点 ⇒ nav2 拿不到任何端点（既不清也不标）**；"
                "最常见的原因是 emit_far_points 关了并且整圈都是无回波，"
                "或 far 撞上了 range_max 被丢掉。"
            )
        self.pub.publish(cloud)

        self.frames += 1
        self.beams_total += n
        self.noecho_total += n_noecho
        self.dropped_total += (n - n_out) if not self.emit_far else 0
        if self.log_every > 0 and self.frames % self.log_every == 0:
            self.get_logger().info(
                "已发 %d 帧：本帧波束 %d、无回波 %d；累计无回波占比 %.1f%%；"
                "本帧出点 %d（替换值 %.5f，丢弃 %d 束）"
                % (
                    self.frames,
                    n,
                    n_noecho,
                    100.0 * self.noecho_total / max(1, self.beams_total),
                    n_out,
                    self.far_effective or 0.0,
                    self.dropped_total,
                )
            )


def main(argv=None):
    rclpy.init(args=argv)
    node = ScanPoints()
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

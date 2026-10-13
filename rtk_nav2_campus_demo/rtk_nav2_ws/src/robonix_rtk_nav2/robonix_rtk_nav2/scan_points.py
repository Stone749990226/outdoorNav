#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光 `LaserScan` → `PointCloud2`（**把激光的入口换成与相机同一种**），
并给**无回波（inf）束**沿它自己的射线补一个落在**标记量程之外**的端点，
让 nav2 能把那些方向清成空闲。

## 为什么要有这个节点（本轮的待检验假设）

已确认的缺陷：**只要有 `scan`（`LaserScan`）这个源，全局代价地图会在某个时刻冻死**
（此后整图字节级不变、对新放的障碍零响应），而同轮**局部图仍有反应**，
**相机的 `PointCloud2` 入口是好的**。已排除"清空射线"那条路（关掉 `clearing` 照样冻）。
⇒ 待检验的假设：**毒在 `LaserScan` 这条摄取路径本身**。
⇒ 做法：把激光也转成 `PointCloud2` 喂进两张图，**其余配置一个字不动**。
本节点就是那条转换。它**不是**给演示加的新功能，是这一次对照实验的自变量。

## 与 nav2 内部 LaserScan 路径的等价性（这是本对照能成立的前提）

nav2 的 `ObstacleLayer` 收到 `LaserScan` 时走的是：
  ① `laserScanValidInfCallback`：`inf_is_valid: true` 时把 `inf` **替换成 `range_max - 1e-4`**
     （反汇编读出的常量 `1e-4`）——注意这一步**会先把束变成有限值**；
  ② `laser_geometry::LaserProjection::projectLaser()`：投成 `PointCloud2`。
本节点把 ①②**原样搬过来**（②直接调用同一个 `laser_geometry` 的 Python 绑定，
不是自己重写三角函数），所以**几何逐点一致**，唯一的差别是
"谁来订阅 `/rtk/scan`、喂给 nav2 的是哪种消息类型"。

★ 离线实测（2026-09-21，`projectLaser` 的逐类语义，别当成"应该没问题"）：

  | 输入的束 | `projectLaser` 输出 |
  | --- | --- |
  | 有限且 `range_min <= r < range_max` | 1 个点，`(r·cosθ, r·sinθ, 0)` |
  | `inf` | **0 个点**（这一束直接消失） |
  | `nan` | **0 个点** |
  | `r >= range_max`（含 `r == range_max`） | **0 个点** —— **会丢** |
  | `r < range_min`（含 `r == 0`） | **0 个点** |

  ⇒ 两条推论，直接决定本节点的写法：
  (a) **`inf` 不能直接喂给 `projectLaser`**（投影出 0 个点 ⇒ 没有端点 ⇒ 不清空），
      必须**先替换成一个有限值**——这正是 nav2 的 `inf_is_valid` 在做的事；
  (b) 替换值**必须严格小于 `range_max`**，否则会撞上表里第四行、**同样被丢掉**，
      而且**丢得无声无息**（现象是"那些方向一直不清"，很容易被误判成"nav2 没消费我"）。

## 替换值取多少（本方案能成立的**关键前提**，改之前先读这三条）

设替换值为 `far`，本源的 `obstacle_max_range = 7.5`、`raytrace_max_range = 8.0`：

  1. **必须 `> obstacle_max_range`（7.5）**：nav2 的 marking 判据是**严格小于**
     （`if (sq_obstacle_max_range <= sq_dist) skip`）⇒ `far` 落在 7.5 以内会被**标成致命障碍**，
     **开阔方向凭空长出一圈假墙**（判据 ③"幽灵格不回来"直接不通过）。这是**最危险**的一条。
  2. **必须 `< range_max`（8.0）**：见上表第四行，`>= range_max` 会被 `projectLaser` 丢掉。
  3. **必须 `<= raytrace_max_range`（8.0）**：清除射线到 `far` 为止（再远由 8.0 截断）。
     取到 8.0 即可让"清到 8 m"与 nav2 的 LaserScan 路径完全一致。

取 `far = range_max - 1e-4 = 7.9999` 同时满足三条，且**与 nav2 自己的替换常量逐位相同**。
离线实测（360 束全替换成 7.9999）：`obstacle_max_range = 7.5` ⇒ **0/360 被标**；
若错取 8.0 ⇒ **360/360 被标**（就是那圈假墙）。本节点启动时会把这条**自检**打出来。

★ 替换只对"本来就没有回波"的束做：`inf` / `nan` / **`r >= range_max`** / **`r < range_min`**。
后两类严格说 nav2 的 LaserScan 路径是**丢掉**（不是当无回波），但两者的后果都是
"这一束没有端点 ⇒ 这个方向不清"；把它们一并算作"量程内没看到东西"，
与相机侧 `depth_points.py` 的口径一致（那边把"超远裁剪 / 无效深度"都算无回波）。
**如实记一条差异**：这一处是本节点相对 nav2 LaserScan 路径的**唯一**扩展，
它是放宽（多清）而不是收紧，不会造出假障碍（受上面第 1 条保护）。

## 代价与治不了的三块（与相机侧同族，不许省略）

这是**用"没看到"推断"没有"**：对透明 / 镜面 / 吸收（黑色橡胶、水）物体不成立。
它治不了：
  ① 超过 `raytrace_max_range`（8.0 m）的格子；
  ② **7.5~8.0 m 的真实障碍只清不标**（`obstacle_max_range = 7.5` 这一档的既有代价，
     与 LaserScan 路径同值，不是本节点引入的）；
  ③ 反射面把光束偏折到别处时，真实障碍所在的方位反而拿到 `inf` ⇒ 会被误清。

订阅：`input_topic`(LaserScan)  发布：`output_topic`(PointCloud2)
frame_id 沿用 LaserScan 的 header（与 nav2 的 LaserScan 路径一致）。

参数：
  `far_range`（默认 7.9999）：替换值，见上"三条前提"。
  `obstacle_max_range`（默认 7.5）：**只用于启动自检**（判断 far 会不会被标成障碍），
      不参与转换本身；写成参数是为了"配置改了但自检还按老值说话"这种情况能被发现。
  `emit_far_points`（默认 true）：**关掉它就是"无回波 ⇒ 0 个点"**（= nav2 的
      `inf_is_valid: false` 行为）。留这个开关是为了做同一次运行内的 A/B 对照。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

# nav2 `laserScanValidInfCallback` 里那个常量（反汇编：0x38D1B717 = 1e-4）。
# **这个 1e-4 是"取替换值"的依据，不是随手写的保险量**：它同时是
# `range_max - 1e-4 < range_max` 这条"不被 projectLaser 丢掉"的余量。
INF_EPS = 1e-4


def _as_bool(value, default=False):
    """把参数值解析成 bool，**字符串与布尔都要能收**。

    为什么需要：`emit_far_points` 要能被 launch 传（做判据 ①的 A/B 对照），
    而 **`LaunchConfiguration` 代入到节点参数里的是字符串**（'true'/'false'），
    命令行 `-p emit_far_points:=true` 传的却是**布尔**。
    如果把它声明成 bool，launch 传字符串会在**声明那一刻**抛类型错、节点直接起不来
    ——现象是"这个话题没有发布者"，很容易被当成"传感器没数据"去查。
    所以这里不声明类型（`declare_parameter` 不给默认值），两种都收，再统一解析。
    （与 `depth_points.py` 的 `_as_bool` 同源；`far_range` 不走这条路，保持 float 声明。）
    """
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class ScanPoints(Node):

    def __init__(self):
        super().__init__('scan_points')
        self.declare_parameter('input_topic', '/rtk/scan')
        self.declare_parameter('output_topic', '/rtk/scan_points')
        # 替换值：默认 = range_max(8.0) - 1e-4，与 nav2 的 inf_is_valid 常量逐位相同。
        self.declare_parameter('far_range', 8.0 - INF_EPS)
        # 只用于启动自检（见模块 docstring 的"三条前提"第 1 条）。
        self.declare_parameter('obstacle_max_range', 7.5)
        # 无回波束是否补端点（关掉 = nav2 inf_is_valid: false 的行为），供 A/B。
        self.declare_parameter('emit_far_points')
        self.declare_parameter('log_every', 100)

        p = self.get_parameter
        self.input_topic = p('input_topic').value
        self.output_topic = p('output_topic').value
        self.far_param = float(p('far_range').value)
        self.obstacle_max_range = float(p('obstacle_max_range').value)
        self.emit_far = _as_bool(p('emit_far_points').value, default=True)
        self.log_every = int(p('log_every').value)

        # laser_geometry 的 Python 绑定（与 nav2 内部用的是同一份实现）。
        # **不放在模块顶层 import**：那样 `--help` 或导入本模块做单测时也会拉起来，
        # 而它是一个编译扩展；放在 __init__ 里，缺了能在启动日志里直接看到。
        from laser_geometry import LaserProjection
        self._projector = LaserProjection()

        self.frames = 0
        self.beams_total = 0
        self.noecho_total = 0
        self.far_effective = None          # 第一次收到 scan 后按 range_max 定死
        self.dropped_total = 0
        self.warned_no_point = False

        self.pub = self.create_publisher(PointCloud2, self.output_topic,
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.input_topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            '激光->点云：%s -> %s（emit_far_points=%s，far_range 参数=%.5f，'
            '本源的 obstacle_max_range=%.2f）'
            % (self.input_topic, self.output_topic,
               '开' if self.emit_far else '关（= inf_is_valid: false 行为）',
               self.far_param, self.obstacle_max_range))

    # ------------------------------------------------------------------ 自检
    def _check_far(self, range_max):
        """按**这一帧**的 `range_max` 定死替换值并做三条前提的自检（只打一次）。"""
        # 前提 2：必须 < range_max，否则 projectLaser 会丢掉（模块 docstring 表第四行）。
        far = min(self.far_param, range_max - INF_EPS)
        # 前提 1：必须 > obstacle_max_range，否则会被标成致命障碍（假墙）。
        bad = far <= self.obstacle_max_range
        self.far_effective = far
        self.get_logger().info(
            '自检：range_max=%.5f ⇒ 替换值 far=%.5f（参数给的是 %.5f，%s）'
            % (range_max, far, self.far_param,
               '未触发上调' if far == self.far_param else '**被 range_max 压下来了**'))
        self.get_logger().info(
            '自检（前提 1，最危险的一条）：far=%.5f vs obstacle_max_range=%.2f ⇒ %s'
            % (far, self.obstacle_max_range,
               'far 在**标记量程之外**，无回波束不会被标成障碍 ✓'
               if not bad else
               '** far 落在 obstacle_max_range 以内 ⇒ 开阔方向会长出一圈假墙 ✗ **'))
        # 前提 3：<= raytrace_max_range（8.0）。这里只报数，不判定——
        # raytrace_max_range 不在本节点，写死一个阈值就成了"抄来的数"。
        self.get_logger().info(
            '自检（前提 3）：清除射线只到 far=%.5f m，请与本源 raytrace_max_range 对照'
            % far)
        if bad:
            self.get_logger().error(
                '** far 落在标记量程内 ⇒ 会出现假墙；请把 far_range 调到 '
                '> obstacle_max_range(%s) 且 < range_max **' % self.obstacle_max_range)

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
                '** projectLaser 投影出 0 个点 ⇒ nav2 拿不到任何端点（既不清也不标）**；'
                '最常见的原因是 emit_far_points 关了并且整圈都是无回波，'
                '或 far 撞上了 range_max 被丢掉。')
        self.pub.publish(cloud)

        self.frames += 1
        self.beams_total += n
        self.noecho_total += n_noecho
        self.dropped_total += (n - n_out) if not self.emit_far else 0
        if self.log_every > 0 and self.frames % self.log_every == 0:
            self.get_logger().info(
                '已发 %d 帧：本帧波束 %d、无回波 %d；累计无回波占比 %.1f%%；'
                '本帧出点 %d（替换值 %.5f，丢弃 %d 束）'
                % (self.frames, n, n_noecho,
                   100.0 * self.noecho_total / max(1, self.beams_total),
                   n_out, self.far_effective or 0.0, self.dropped_total))


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


if __name__ == '__main__':
    raise SystemExit(main())

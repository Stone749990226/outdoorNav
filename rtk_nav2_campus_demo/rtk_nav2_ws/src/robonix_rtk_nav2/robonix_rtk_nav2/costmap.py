#!/usr/bin/env python3
"""前向危险距离：从 **nav2 的局部代价地图** 里取"前方扇区内最近的障碍距离"。

第三阶段的位置（见 docs/rtk-nav2-brief.md §2）：地图与膨胀**都交给 nav2**，
本节点从第二阶段的"建图 + 膨胀 + 危险距离"收缩成**只输出危险距离**，
供 `route_follower` 的安全判据使用——控制回路的接口不变（同样的两个话题）。

    订阅：/local_costmap/costmap     nav_msgs/OccupancyGrid（nav2 局部图，**base_link 帧**；
                                    nav2 原生命名——相对名 "costmap" 解析到节点命名空间）
          /rtk/camera/points          sensor_msgs/PointCloud2（**只用来判活**，见下）
    发布：/rtk/costmap_danger         std_msgs/Float32，前方扇区内最近的障碍距离（米），无则 NaN
          /rtk/costmap_status         std_msgs/String，latched：ok / wait / stale /
                                      no_camera_info / costmap_stale

**为什么要订阅点云**（看似多余、其实必需）：nav2 的局部图在相机断流后**照常按
publish_frequency 发布**（内容冻住、话题还活着）。只看代价地图话题的新鲜度，
"断相机 → 停车"这条判据就永远测不出来。所以判活要看**感知链路的源头**：
点云还在来，才说明相机与 `depth_image_proc` 都活着。

**扇区怎么算**：局部图的 global_frame 是 `base_link`，机器人恒在图心、朝向恒为 +x，
所以扇区判据不需要任何 TF，直接在栅格坐标里做：
    x = origin_x + (col + 0.5) * res,  y = origin_y + (row + 0.5) * res
    cost >= 阈值 且 x > 0 且 |atan2(y, x)| <= 半角 且 hypot(x, y) <= 量程
取满足条件里 hypot 的最小值。

**未知格不算危险**：沿用第一阶段的口径——未知的含义是"没看到"，不是"有东西"；
把它算成危险会让机器人一步都动不了。局部图的 `track_unknown_space: false`，
所以未知格在发布出来时是 0（空闲），本来也不会 >= 阈值。
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String

from robonix_rtk_nav2.clock_guard import ClockGuard

STATE_OK = 'ok'
STATE_WAIT = 'wait'
STATE_STALE = 'stale'
STATE_NO_CLOUD = 'no_camera_info'
STATE_COSTMAP_STALE = 'costmap_stale'


class Danger(Node):

    def __init__(self):
        super().__init__('danger')

        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('cloud_topic', '/rtk/camera/points')
        # 局部图的帧。断言它就是这个值——若 nav2 那边把局部图改挂别的帧，
        # 扇区判据的前提（机器人恒在图心、朝向 +x）就不成立，必须立刻发现而不是算错。
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('danger_half_angle_deg', 35.0)
        self.declare_parameter('danger_max_range_m', 3.0)
        # 发布出来的栅格值域是 0..100（nav2 内部 0..255 除以 2.55）。
        # 阈值 50 —— 与第一阶段同一个数，但**含义不同**：第一阶段是自写膨胀的线性衰减，
        # 这一阶段是 nav2 膨胀层的指数衰减（cost = 253 * exp(-factor * (d - inscribed))）。
        # 因此"代价 >= 50"对应的离障距离变了，实测停车距离见验证文档。
        self.declare_parameter('danger_cost_threshold', 50)
        self.declare_parameter('cloud_timeout', 1.0)
        self.declare_parameter('costmap_timeout', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.half_angle = math.radians(
            float(self.get_parameter('danger_half_angle_deg').value))
        self.max_range = float(self.get_parameter('danger_max_range_m').value)
        self.cost_threshold = int(self.get_parameter('danger_cost_threshold').value)
        self.cloud_timeout = float(self.get_parameter('cloud_timeout').value)
        self.costmap_timeout = float(self.get_parameter('costmap_timeout').value)

        self.clock_guard = ClockGuard()

        self.last_cloud_time = None
        self.cloud_msgs = 0
        self.cloud_points = 0
        self.last_costmap_time = None
        self.costmap_msgs = 0
        self.grid = None          # (data int16[H,W], resolution, origin_x, origin_y)
        self.status = None

        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.danger_pub = self.create_publisher(Float32, '/rtk/costmap_danger', 10)
        self.status_pub = self.create_publisher(String, '/rtk/costmap_status', latched)

        self.create_subscription(OccupancyGrid,
                                 self.get_parameter('costmap_topic').value,
                                 self.on_costmap, 10)
        # **点云用 sensor-data QoS（BEST_EFFORT）**：`depth_image_proc` 的发布端是
        # BEST_EFFORT，用默认 RELIABLE 订阅会一条都收不到，而现象是"点云看起来没在出"
        # ——那样本节点会一直报 no_camera_info，route_follower 直接以退出码 3 中止。
        self.create_subscription(PointCloud2,
                                 self.get_parameter('cloud_topic').value,
                                 self.on_cloud, qos_profile_sensor_data)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            '危险距离节点启动：图 %s（帧 %s），前方 ±%.0f 度 / %.1f m 内取 cost >= %d 的最近距离'
            % (self.get_parameter('costmap_topic').value, self.base_frame,
               math.degrees(self.half_angle), self.max_range, self.cost_threshold))

    # ------------------------------------------------------------ 回调
    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_cloud(self, msg: PointCloud2):
        stamp = self.clock_guard.stamp(self.now_seconds())
        if stamp is None:
            return
        self.last_cloud_time = stamp
        self.cloud_msgs += 1
        self.cloud_points = msg.width * msg.height

    def on_costmap(self, msg: OccupancyGrid):
        stamp = self.clock_guard.stamp(self.now_seconds())
        if stamp is None:
            return
        if msg.header.frame_id != self.base_frame:
            # 只在第一次报错，避免刷屏
            if self.costmap_msgs == 0:
                self.get_logger().error(
                    '局部代价地图的帧是 %s，不是预期的 %s：扇区判据的前提不成立，'
                    '危险距离将按错误的几何算出来' % (msg.header.frame_id, self.base_frame))
        self.last_costmap_time = stamp
        self.costmap_msgs += 1
        self.grid = (np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height,
                                                                  msg.info.width),
                     msg.info.resolution,
                     msg.info.origin.position.x,
                     msg.info.origin.position.y)

    # ------------------------------------------------------------ 主循环
    def compute_danger(self):
        grid, res, ox, oy = self.grid
        rows, cols = np.nonzero(grid >= self.cost_threshold)
        if len(rows) == 0:
            return None
        x = ox + (cols + 0.5) * res
        y = oy + (rows + 0.5) * res
        dist = np.hypot(x, y)
        keep = (x > 0.0) & (dist <= self.max_range) & (np.abs(np.arctan2(y, x)) <= self.half_angle)
        if not np.any(keep):
            return None
        return float(dist[keep].min())

    def tick(self):
        danger = Float32()
        danger.data = float('nan')

        # 时钟跳变（含 0 -> 真实时间的首次就绪）：丢掉这一轮的判定依据，重来。
        # 这是从视觉版与 RTK 第二阶段各踩了一次的坑（见 clock_guard 模块文档）。
        if self.clock_guard.update(self.now_seconds()):
            self.last_cloud_time = None
            self.last_costmap_time = None

        now = self.now_seconds()
        if self.last_cloud_time is None:
            self.set_status(STATE_NO_CLOUD)
        elif (now - self.last_cloud_time) > self.cloud_timeout:
            self.set_status(STATE_STALE)
        elif self.last_costmap_time is None or self.grid is None:
            self.set_status(STATE_WAIT)
        elif (now - self.last_costmap_time) > self.costmap_timeout:
            self.set_status(STATE_COSTMAP_STALE)
        else:
            self.set_status(STATE_OK)
            value = self.compute_danger()
            if value is not None:
                danger.data = float(value)

        self.danger_pub.publish(danger)

    def set_status(self, status):
        # **每个周期都发**。理由同第二阶段：状态消息极小，多发没有代价；
        # 而"只在变化时发 + 定时重发"实测会被计算周期拉长成 ~1.3 s 的间隔，
        # 正好撞上路线跟随的超时判据，导致"节点明明活着却被判成 stale"。
        changed = status != self.status
        self.status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        if changed:
            self.get_logger().info('危险距离状态：%s' % status)


def main(argv=None):
    rclpy.init(args=argv)
    node = Danger()
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

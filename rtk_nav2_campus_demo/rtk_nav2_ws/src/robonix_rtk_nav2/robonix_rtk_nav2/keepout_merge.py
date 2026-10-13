#!/usr/bin/env python3
"""★ **合并 keepout 掩膜**（reviewer/AuYang 2026-10-09 批的 ④ 修法第一件 ✓）。

**为什么需要它** ✗（`ramp7-au` 第一手证据 ✓）：实体封边块**被 `obstacle_layer` 擦掉** ✗ ——
`obstacle_layer` = `combination_method: 0`（Overwrite）+ `clearing: true` ✓ ⇒
传感器射线**越过 0.15 m 高的块顶** ⇒ **raytrace 把那几格清成自由** ✗ ⇒ **覆盖掉 `static_layer` 写的致命值** ✗。
实测：`ramp7-au` 里 `x=-4.15` 纵线上 `static_map` 全 100 ✓ 而 `global_costmap` 从 `y≳1.5` 起被擦成 70/79/**0** ✗ ⇒
**规划路径从那儿穿过去** ✗（161/690 个点在高台之外 ✓）。

**⇒ 走"擦不掉"的那条通道：keepout** ✓ —— 它由 **filters** 承载、在 **layers 之后**应用 ✓（对照已证 ✓：
落差带在 `(0,±1)` 读 100 完好 ✓，而同世界的实心块被擦 ✗ ⇒ **差别在通道、不在几何** ✓）。

**做什么** ✓：把**检测器掩膜 ∪ 静态封边块**合成一张掩膜 ✓，发到**新话题** ✓，再由 keepout 读它 ✓。
· **不动检测器** ✗（`depth_drop.py` 冻着 ✓）—— 本节点只是**订阅**它的产物 ✓；
· **几何从 `world_edges.SEALED_EDGES` 派生** ✓（单一真源 ✓、**不手抄坐标** ✗）；
· **不改 `combination_method` 为 Maximum** ✗（那会把"幽灵格永不清"请回来 ✓，先前刻意否决 ✓）。

**★ 两个安全设计**（都为了让"改指新话题"这一步**不会把正在工作的通道打掉** ✗）：
1. **世界坐标相加** ✓（**不按数组下标** ✗）：两张图的分辨率/原点/尺寸可以不同 ✓，也不会"图里有、位置错" ✗；
2. **拿不到静态图 ⇒ 原样转发检测器掩膜** ✓ 并**大声记日志** ✓ ⇒ **最坏情况 = 回到现在的行为** ✓、绝不静默变成空掩膜 ✗。

用法：`ros2 run robonix_rtk_nav2 keepout_merge`
"""
import os
import sys

import numpy as np
import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

_SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')
sys.path.insert(0, _SCRIPTS)
try:
    import world_edges as WE
except ImportError:                                            # pragma: no cover
    WE = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
MARKED = 100          # 检测器掩膜与被并进来的块都用这个值（**检测器源码里就是 100** ✓）
NO_INFO = -1


class KeepoutMerge(Node):
    def __init__(self):
        super().__init__('keepout_merge')
        self.declare_parameter('filter_info_in', '/rtk/drop/filter_info_map')
        self.declare_parameter('filter_info_out', '/rtk/drop/filter_info_merged')
        self.declare_parameter('mask_out', '/rtk/drop/mask_merged')
        self.declare_parameter('static_map', '/rtk/static_map')
        # ★ 2026-10-09（AuYang 诊断 ✓）：**外扩一圈** —— 见 `republish()` 里那段长注释 ✓
        # ★ 取值 = **`robot_radius`（0.105）**，**不是** `inflation_radius`（0.25）✗ —— 来历（`ramp7-bb` 第一手 ✓）：
        #   首版按 0.25 外扩 ⇒ 车在观测段**过冲 0.6 m** 掉进外扩带 ⇒ **它自己那格变成致命（实测读 100 ✗）**
        #   ⇒ **nav2 从一个致命格出发永远规划不出来** ⇒ `follow_status_6` 无限重规划 ⇒ **死锁** ✗（不是慢 ✓）。
        #   ⇒ 外扩带必须**窄到"车不该去的地方本来就不会去"** ✓：
        #     `inflation_layer` 把 `inscribed_radius`(≈`robot_radius`) 以内标成**硬致命** ✓、
        #     再往外只是**代价梯度**（软 ✓、不会把车锁死 ✓）—— 我们只能复刻**硬的那一圈** ✓
        #     ⇒ 取 `robot_radius` 才与 nav2 原版**等宽** ✓（0.25 那版等于把硬边放大 2.4 倍 ✗）。
        self.declare_parameter('dilate_m', 0.105)
        p = self.get_parameter
        self.info_in = p('filter_info_in').get_parameter_value().string_value
        self.info_out = p('filter_info_out').get_parameter_value().string_value
        self.mask_out = p('mask_out').get_parameter_value().string_value
        self.static_in = p('static_map').get_parameter_value().string_value
        self.dilate_m = p('dilate_m').get_parameter_value().double_value

        self.template = None          # 检测器的 CostmapFilterInfo（元数据照抄 ✓，只换 filter_mask_topic ✓）
        self.src_mask = None          # 检测器掩膜（OccupancyGrid ✓）
        self.static = None            # 静态封边图（OccupancyGrid ✓）
        self.sealed_boxes = None      # 从 world_edges.SEALED_EDGES 派生 ✓
        self.sub_mask = None

        self.pub_mask = self.create_publisher(OccupancyGrid, self.mask_out, LATCHED)
        self.pub_info = self.create_publisher(CostmapFilterInfo, self.info_out, LATCHED)
        self.create_subscription(CostmapFilterInfo, self.info_in, self.on_info, LATCHED)
        self.create_subscription(OccupancyGrid, self.static_in, self.on_static, LATCHED)
        self.get_logger().info('keepout 合并：%s ∪ %s ⇒ %s（info %s）'
                               % (self.info_in, self.static_in, self.mask_out, self.info_out))
        if WE is None:
            self.get_logger().error('拿不到 `world_edges` ⇒ **块几何无法派生** ✗ —— 只转发检测器掩膜 ✓')
        elif len(WE.SEALED_EDGES) == 0:
            self.get_logger().warn('`SEALED_EDGES` 为空 ⇒ 没有块要并 ✓（纯转发 ✓）')

    # ── 输入 ──────────────────────────────────────────────────────────────
    def on_info(self, msg):
        self.template = msg
        if self.sub_mask is None:
            self.sub_mask = self.create_subscription(
                OccupancyGrid, msg.filter_mask_topic, self.on_mask, LATCHED)
            self.get_logger().info('检测器掩膜话题 = %s ✓' % msg.filter_mask_topic)

    def on_static(self, msg):
        self.static = msg
        self.get_logger().info('静态封边图 ✓ %dx%d @ %.3f 原点 (%.1f,%.1f)'
                               % (msg.info.width, msg.info.height, msg.info.resolution,
                                  msg.info.origin.position.x, msg.info.origin.position.y))
        self.republish()

    def on_mask(self, msg):
        self.src_mask = msg
        self.republish()

    # ── 世界坐标下的并集 ✓（不按下标 ✗）───────────────────────────────────
    @staticmethod
    def _marked_world(grid):
        """返回该图上"被标记"的格的**世界坐标** (xs, ys)。空图返回空数组 ✓。"""
        i = grid.info
        a = np.asarray(grid.data, dtype=np.int16).reshape(i.height, i.width)
        iy, ix = np.nonzero(a >= MARKED)
        res = i.resolution
        if len(ix) == 0:
            return np.empty(0), np.empty(0)
        return (i.origin.position.x + (ix + 0.5) * res,
                i.origin.position.y + (iy + 0.5) * res)

    @staticmethod
    def _dilate(mask, r):
        """二值膨胀 `r` 格（4 邻域重复 r 次 ✓）—— 用**位移取或** ✓，不依赖 scipy ✓。"""
        m = mask.copy()
        for _ in range(int(r)):
            nxt = m.copy()
            nxt[1:, :] |= m[:-1, :]
            nxt[:-1, :] |= m[1:, :]
            nxt[:, 1:] |= m[:, :-1]
            nxt[:, :-1] |= m[:, 1:]
            m = nxt
        return m

    def _sealed_world(self):
        """**从 `world_edges.SEALED_EDGES` 派生** ✓ —— 把线段按 0.05 m 采样成点 ✓（单一真源 ✓）。"""
        if WE is None or not len(WE.SEALED_EDGES):
            return np.empty(0), np.empty(0)
        xs, ys = [], []
        for _name, (a, b) in WE.SEALED_EDGES:
            n = max(1, int(np.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
            for t in np.linspace(0.0, 1.0, n + 1):
                xs.append(a[0] + t * (b[0] - a[0]))
                ys.append(a[1] + t * (b[1] - a[1]))
        return np.asarray(xs), np.asarray(ys)

    def republish(self):
        if self.src_mask is None or self.template is None:
            return                          # 检测器那边还没上来 ⇒ 不发（免得发一张空掩膜 ✗）
        base = self.static if self.static is not None else self.src_mask
        if self.static is None:
            self.get_logger().warn('还没收到静态图 ⇒ **原样转发检测器掩膜** ✓（最坏=现状、不会更坏 ✓）',
                                   throttle_duration_sec=10.0)
        i = base.info
        w, h, res = i.width, i.height, i.resolution
        ox, oy = i.origin.position.x, i.origin.position.y
        out = np.full((h, w), NO_INFO, dtype=np.int16)

        def stamp(xs, ys):
            if len(xs) == 0:
                return
            cx = np.floor((xs - ox) / res).astype(int)
            cy = np.floor((ys - oy) / res).astype(int)
            ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
            out[cy[ok], cx[ok]] = MARKED

        sx, sy = self._marked_world(self.src_mask)
        stamp(sx, sy)                       # ① 检测器掩膜 ✓
        if self.static is not None:
            tx, ty = self._marked_world(self.static)
            stamp(tx, ty)                   # ② 静态封边块 ✓
            ex, ey = self._sealed_world()
            stamp(ex, ey)                   # ③ SEALED_EDGES 派生的封边线 ✓（单一真源 ✓）
            self.get_logger().info(
                '并集：检测器 %d 格 + 静态图 %d 格 + SEALED_EDGES %d 点 ⇒ 合计 %d 格 ✓'
                % (len(sx), len(tx), len(ex), int((out == MARKED).sum())),
                throttle_duration_sec=30.0)
        # ★★ 2026-10-09 **外扩一圈**（AuYang 诊断 ✓、实测坐实 ✓）────────────────────────
        # 现象（`ramp7-ba` 第一手 ✓）：车**擦着封边墙角卡住** —— 末态 `(-0.038, 6.993)`、
        #   而 `ramp_seal_east` 的墙面在 `x=0.00` ⇒ **净距 0.038 m** ✗。
        # 根因（**通道顺序** ✓，不是参数问题 ✗）：nav2 是 **layers → `inflation_layer` → filters** ✓
        #   ⇒ **keepout 是 filter ⇒ 它画的致命格永远拿不到膨胀** ✗。
        #   实测同一格 `(x=-0.10, y=5.00)`：S1 时读 **64**（那时 `static_layer` 那份还活着 ⇒ 有梯度 ✓），
        #   车跑过之后读 **0** ✗ ⇒ **`obstacle_layer` 的射线擦掉静态那份时，膨胀是跟着一起消失的** ✓
        #   —— 本节点把**致命格**补回来了 ✓，但**补不回那一圈膨胀** ✗（我们跑在膨胀之后 ✓）。
        # ⇒ 在**唯一能活下来的这条通道里**把边距补上 ✓：把并集**外扩 `dilate_m`**（默认 = `inflation_radius` ✓）。
        # 只**加**格、不减格 ✗ ⇒ `seal_west` 整条纵线仍致命 ✓、① 的判读（读检测器那份 `mask_map`）**不受影响** ✓。
        _r = int(round(self.dilate_m / res)) if self.dilate_m > 0 else 0
        if _r > 0:
            _before = int((out == MARKED).sum())
            out[self._dilate(out == MARKED, _r)] = MARKED
            self.get_logger().info(
                '外扩 %d 格（%.2f m / res %.3f）：%d ⇒ %d 格 ✓'
                % (_r, self.dilate_m, res, _before, int((out == MARKED).sum())),
                throttle_duration_sec=30.0)

        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = base.header.frame_id or 'map'
        m.info = base.info
        m.data = out.reshape(-1).astype(np.int8).tolist()
        self.pub_mask.publish(m)
        info = CostmapFilterInfo()
        info.header.stamp = m.header.stamp
        info.header.frame_id = m.header.frame_id
        info.type = self.template.type
        info.base = self.template.base
        info.multiplier = self.template.multiplier
        info.filter_mask_topic = self.mask_out      # ★ 只换这一处 ✓（其余元数据照抄 ✓）
        self.pub_info.publish(info)


def main(argv=None):
    rclpy.init(args=argv)
    n = KeepoutMerge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

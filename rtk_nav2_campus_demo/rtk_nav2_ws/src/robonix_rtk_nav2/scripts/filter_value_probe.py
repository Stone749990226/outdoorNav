#!/usr/bin/env python3
"""第 0 步最小验证：KeepoutFilter 会不会把 mask 的**中间值**写成代价？

背景（reviewer/AuYang 2026-10-08 批的"致命区代价递减圈"）：若能，最省的路是检测器直接发
"带圈 mask"；若不能，就得另加一层 filter，成本完全不同。**先验再设计，不猜。**

做法（只读 + 只往 `/rtk/drop/mask_map` 发一张**测试** mask，不碰检测器/节点）：
 1. 起栈时把车放在**视野里没有落差**的位置（`x=+4, y=0`、朝东，低台面一片平地）
    ⇒ 检测器的 `hazard_points` 为空 ⇒ 它**不发布** mask（节点只在有格时发布）
    ⇒ 这条话题由本探针独占 ✓。
 2. 发一张**分档测试 mask**（latched）：x 方向 6 条竖带，值依次 0 / 30 / 50 / 70 / 90 / 100，
    另加一块 255（未知）。
 3. 等代价地图跑几拍，读 `/global_costmap/costmap_raw`（Costmap，代价 0–255）在这些带的取值，
    并读 `/global_costmap/costmap`（OccupancyGrid，0–100）作对照。
 4. 打印 值→代价 的映射表。

判定：若 50 那一带读到 50 左右 ⇒ **中间值透传为代价**（可走"带圈 mask"）；
      若读到 254/100 等固定值 ⇒ 只认"禁/不禁"（要另做一层）。
"""
import argparse
import json
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
BANDS = [(2.5, 3.0, 0), (3.0, 3.5, 30), (3.5, 4.0, 50), (4.0, 4.5, 70),
         (4.5, 5.0, 90), (5.0, 5.5, 100)]


class Probe(Node):
    def __init__(self, out):
        super().__init__('filter_value_probe')
        self.out = out
        self.raw = None
        self.occ = None
        self.pub = self.create_publisher(OccupancyGrid, '/rtk/drop/mask_map', LATCHED)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw',
                                 lambda m: setattr(self, 'raw', m), LATCHED)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'occ', m), LATCHED)

    def build_mask(self):
        res = 0.05
        x0, y0, nx, ny = 2.5, -1.5, 60, 60
        g = np.zeros((ny, nx), dtype=np.int8)
        xs = x0 + (np.arange(nx) + 0.5) * res
        ys = y0 + (np.arange(ny) + 0.5) * res
        for a, b, v in BANDS:
            g[:, (xs >= a) & (xs < b)] = v
        g[(ys >= 1.0) & (ys < 1.5), (xs >= 2.5) & (xs < 3.0)] = 127   # 未知块用 127 试
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = res
        m.info.width, m.info.height = nx, ny
        m.info.origin.position.x, m.info.origin.position.y = x0, y0
        m.info.origin.orientation.w = 1.0
        m.data = g.reshape(-1).tolist()
        return m

    def sample(self, msg, x, y):
        if msg is None:
            return None
        i = msg.info
        if abs(i.resolution - 0.05) > 1e-6:
            pass
        cx = int((x - i.origin.position.x) / i.resolution)
        cy = int((y - i.origin.position.y) / i.resolution)
        if not (0 <= cx < i.width and 0 <= cy < i.height):
            return None
        return int(np.asarray(msg.data).reshape(i.height, i.width)[cy, cx])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--settle', type=float, default=6.0)
    a = ap.parse_args()
    rclpy.init()
    n = Probe(a.json)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    m = n.build_mask()
    n.pub.publish(m)
    print('已发布测试 mask（latched）：%s' % [(b[2], 'x∈[%.1f,%.1f)' % (b[0], b[1])) for b in BANDS])
    end = time.time() + a.settle
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
        n.pub.publish(m)          # 反复发，跨过代价地图的更新周期
        time.sleep(0.4)
    rows = []
    print('%-6s %-14s %-10s %-10s' % ('mask值', '采样点(x,y)', 'raw代价', 'occ(0-100)'))
    for lo, hi, v in BANDS:
        for yy in (0.2, -0.2):
            x, y = (lo + hi) / 2.0, yy
            rr = n.sample(n.raw, x, y)
            oo = n.sample(n.occ, x, y)
            print('%-6d (%.2f,%+.2f)     %-10s %-10s' % (v, x, y, rr, oo))
            rows.append({'mask': v, 'x': x, 'y': y, 'raw': rr, 'occ': oo})
    # 未覆盖区（远离测试 mask）作对照
    for x, y in ((6.0, 0.0), (4.0, 2.5)):
        rr = n.sample(n.raw, x, y)
        oo = n.sample(n.occ, x, y)
        print('%-6s (%.2f,%+.2f)     %-10s %-10s' % ('未覆盖', x, y, rr, oo))
        rows.append({'mask': None, 'x': x, 'y': y, 'raw': rr, 'occ': oo})
    with open(a.json, 'w') as f:
        json.dump(rows, f, indent=1)
    print('has_raw', n.raw is not None, 'has_occ', n.occ is not None)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

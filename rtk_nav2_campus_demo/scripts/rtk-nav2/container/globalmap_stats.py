#!/usr/bin/env python3
"""globalmap_stats.py —— 取一次**全局代价图**的累积统计（第 4 步的判据量 ✓）

用法（**在容器里、栈运行中**）:
    source /opt/ros/humble/setup.bash; source <ws>/install/setup.bash
    python3 globalmap_stats.py --out <run>/nav2/globalmap_stats.json

★ 为什么要有它：第 4 步要证的是"**车走过沿线的实时障碍会累积在全局图上**" ✓ ——
  而**光有 RViz 截图不够** ✗（"图上有东西" 与 "东西在轨迹沿线" 是两件事 ✓）。
  ⇒ 这里把**全局图的非零格**逐格取出来 ✓，落成 json ✓ ⇒ 可与 `trajectory.csv` 的**覆盖区**对账 ✓。

★ 同时记录**判读基线**（✗ 不靠推断 ✓）：
  · `nonzero`（非零格数，含致命/膨胀）✓
  · `lethal`（>=99）✓
  · 非零格的**包围盒**（map 坐标 ✓）⇒ 与轨迹包围盒比 ✓
  · 栅格 info（分辨率/原点/尺寸 ✓）⇒ 换算要用 ✓
退出码: 0 = 取到并写出 ｜ 2 = 取不到（**不许把"取不到"当"没有"** ✗）
"""
import argparse
import json
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='落盘的 json 路径')
    ap.add_argument('--topic', default='/global_costmap/costmap')
    ap.add_argument('--timeout', type=float, default=30.0)
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import OccupancyGrid

    rclpy.init()
    n = Node('globalmap_stats')
    # nav2 的 Costmap2DPublisher 用 transient_local（latched）✓ ⇒ 订阅端必须同 durability ✓
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    got = {}
    n.create_subscription(OccupancyGrid, a.topic, lambda m: got.setdefault('m', m), qos)
    t0 = time.time()
    while 'm' not in got and time.time() - t0 < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.5)
    if 'm' not in got:
        print('✗ %s 上没收到消息（%.0f s）⇒ ★ 记成"未取到"，✗ 不记成"没有障碍" ✗'
              % (a.topic, a.timeout), file=sys.stderr)
        return 2

    m = got['m']
    inf = m.info
    res, w, h = inf.resolution, inf.width, inf.height
    ox, oy = inf.origin.position.x, inf.origin.position.y
    # ★★ 2026-10-10 修正（★ 第一版报了个**假读数** ✗）★★
    #   第一版把"非零格"当"有障碍" ⇒ 实测报 **1314356 格（占全图 91%）** ✗ ——
    #   ★ 那是把 **未知格 -1** 也算进去了 ✓（`allow_unknown: true` ⇒ 全局图**大片是 -1** ✓）。
    #   ⇒ ★ **"非零" ≠ "有障碍"** ✓（与"非空 ≠ 有数据"同一族 ✓）。
    #   ⇒ 现在按**取值分档**报 ✓，每一档都单独给数 ✓ ⇒ 谁也不能再把 -1 读成障碍 ✓。
    occupied = lethal = inflated = unknown = free = other = 0
    x0 = y0 = 1e18
    x1 = y1 = -1e18
    # ★ 自由格(0) 的包围盒 = "**车扫过并清空**的区域" ✓ —— ★ 这是与 `trajectory.csv`
    #   覆盖区对账的**主量** ✓（障碍只出现在"真的看到了东西"的地方 ⇒ 障碍包围盒会**小于**轨迹 ✓，
    #   ✗ 拿障碍包围盒对账会误判成"没覆盖" ✗ —— 这正是我第一版想歪的地方 ✓）。
    fx0 = fy0 = 1e18
    fx1 = fy1 = -1e18
    for i, c in enumerate(m.data):
        if c == 0:
            free += 1
            gx, gy = i % w, i // w
            wx, wy = ox + (gx + 0.5) * res, oy + (gy + 0.5) * res
            fx0, fx1 = min(fx0, wx), max(fx1, wx)
            fy0, fy1 = min(fy0, wy), max(fy1, wy)
            continue
        if c < 0:
            unknown += 1
            continue
        if c >= 99:
            lethal += 1
        else:
            inflated += 1
        occupied += 1
        gx, gy = i % w, i // w
        wx, wy = ox + (gx + 0.5) * res, oy + (gy + 0.5) * res
        x0, x1 = min(x0, wx), max(x1, wx)
        y0, y1 = min(y0, wy), max(y1, wy)
    out = {
        'topic': a.topic,
        'via': 'rclpy-once',
        'produced_at_epoch': time.time(),
        'grid': {'resolution': res, 'width': w, 'height': h,
                 'origin': {'x': ox, 'y': oy},
                 'rolling_window': None},
        'cells_total': w * h,
        # ★ 按取值分档 ✓ —— ✗ 不再给一个含糊的"非零" ✗
        'free_0': free,
        'unknown_neg': unknown,
        'occupied_gt0': occupied,          # ★ 这才是"累积下来的观测" ✓
        'inflated_1_98': inflated,
        'lethal_ge99': lethal,
        'occupied_bbox_map': (None if occupied == 0 else
                              {'x0': round(x0, 3), 'x1': round(x1, 3),
                               'y0': round(y0, 3), 'y1': round(y1, 3)}),
        'free_bbox_map': (None if free == 0 else
                          {'x0': round(fx0, 3), 'x1': round(fx1, 3),
                           'y0': round(fy0, 3), 'y1': round(fy1, 3)}),
        'note': ('★ `occupied_gt0` 才是"累积下来的观测" ✓；✗ `unknown_neg`（-1）**不是障碍** ✗；'
                 'bbox 用来与 trajectory.csv 的覆盖区对账 ✓'),
    }
    with open(a.out, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=2)
    print('  ✓ **有障碍格（>0）= %d**（其中致命 >=99 = %d、膨胀 1~98 = %d）' %
          (occupied, lethal, inflated))
    print('    ★ 另：未知格(-1) = %d、自由格(0) = %d（★ 未知**不是**障碍 ✗）' % (unknown, free))
    print('    障碍包围盒 = %s' % out['occupied_bbox_map'])
    print('    ★ 自由空间包围盒（= 车扫过的区域，★ 与轨迹对账用这个 ✓）= %s' % out['free_bbox_map'])
    print('  已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

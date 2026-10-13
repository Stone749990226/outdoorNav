#!/usr/bin/env python3
"""rev9_costmap_cell_probe.py —— ★ **点名几个世界坐标，dump 全局图在该格的代价** ✓（第 5 步 ✓）

用法（容器内、栈运行中）:
  python3 rev9_costmap_cell_probe.py --points "x,y[,名字]; x,y; …" \
      [--topic /global_costmap/costmap] [--win 2] [--out <json>]

★★ 为什么要有它（★ reviewer 第 5 步的要求 ✓）★★
  ★ 第 4 步里 wp41 卡住、日志说 `Resulting plan has 0 poses in it` ✗，我**提了一个假设**
    （"该处未被观测／被标成致命格" ✓）—— ★ 但**假设不是结论** ✗。
    ⇒ 本工具的用途就是**把那个假设落成实测** ✓：★ **点名 wp41 那一格**，把它当时的代价打出来 ✓。
  ★★ 并且必须配**正向对照** ✓（✗ "只标帧换算不够" ✗）：
    ① **逐点邻域**（默认 5×5 ✓）—— ★ 让"0"与"**图是空的**"分得开 ✓；
    ② **全图按值分带直方图** ✓ —— ★ 证明"**该有数据的地方真的有**" ✓
      （free / unknown(-1) / inflated(1..98) / lethal(>=99) 各自多少格 ✓）；
    ③ ★ 允许同时点**若干"已知能到"的点** ✓ ⇒ 与"到不了的那个点"**并排** ✓（对照才有意义 ✓）。
  ★ 记下读图时刻的 **header.stamp ＋ 消息数** ✓（✗ 不看"有没有收到过" ✗）。

★ 取值口径（★ nav2 的约定 ✓）：`-1` = **未知**（`track_unknown_space: true` ✓）；
  `0` = 空闲；`1..98` = 膨胀带；`99/100` = **致命**（`lethal_cost` ✓）。
"""
import argparse
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)


def parse_points(s):
    """★ 两种写法都收 ✓：`x,y[,名字]` 与 `名字,x,y` ——
    ★ 实测踩过：`"wp41,-0.443,-0.151"` 会被前者当 x ⇒ `could not convert 'wp41' to float` ✗。
    ⇒ 判据：**哪个字段能解析成 float** 就当坐标 ✓（✗ 不靠"第几个"✗）。"""
    out = []
    for part in s.split(';'):
        part = part.strip()
        if not part:
            continue
        bits = [b.strip() for b in part.split(',')]

        def _f(v):
            try:
                return float(v)
            except ValueError:
                return None
        nums = [_f(b) for b in bits]
        if len(bits) >= 2 and nums[0] is not None and nums[1] is not None:
            name, x, y = (bits[2] if len(bits) > 2 else '(%g,%g)' % (nums[0], nums[1])), nums[0], nums[1]
        elif len(bits) >= 3 and nums[1] is not None and nums[2] is not None:
            name, x, y = bits[0], nums[1], nums[2]
        else:
            print('✗ 看不懂的点：%r（要给 "x,y[,名字]" 或 "名字,x,y" ✓）' % part)
            continue
        out.append((name, x, y))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--points', required=True, help='"x,y[,名字]; x,y; …"')
    ap.add_argument('--topic', default='/global_costmap/costmap')
    ap.add_argument('--win', type=int, default=2, help='邻域半径（格）；2 ⇒ 5x5 ✓')
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()

    pts = parse_points(a.points)
    got = {}
    n_msg = [0]
    rclpy.init()
    n = Node('rev9_costmap_cell_probe')
    # ★ 与 costmap 发布端一致的 QoS ✓（RELIABLE + TRANSIENT_LOCAL ✓，否则拿不到 latched 的那份 ✓）
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)

    def cb(m):
        n_msg[0] += 1
        got['m'] = m

    n.create_subscription(OccupancyGrid, a.topic, cb, qos)
    t0 = time.time()
    while time.time() - t0 < a.timeout and 'm' not in got:
        rclpy.spin_once(n, timeout_sec=0.3)
    if 'm' not in got:
        print('✗ %s 上没有取到图（★ "取不到" ≠ "没有" ✗ ⇒ 不许当结论 ✓）' % a.topic)
        return 2
    m = got['m']
    info = m.info
    W, H, res = info.width, info.height, info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    d = m.data
    print('=== 全局代价图 %s ===' % a.topic)
    print('  栅格 %dx%d @ %.4f m ｜ origin (%.3f, %.3f) ⇒ x∈[%.1f, %.1f] y∈[%.1f, %.1f]'
          % (W, H, res, ox, oy, ox, ox + W * res, oy, oy + H * res))
    print('  ★ 读图时刻：header.stamp=%.3f ｜ 本进程收到 %d 条（★ 与活性无关，只是取图记录 ✓）'
          % (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, n_msg[0]))

    # ★ ② 全图按值分带（★ 正向对照：证明"该有数据的地方真的有" ✓）
    tot = W * H
    bands = {'unknown_-1': 0, 'free_0': 0, 'inflated_1_98': 0, 'lethal_ge99': 0, 'other': 0}
    for v in d:
        if v < 0:
            bands['unknown_-1'] += 1
        elif v == 0:
            bands['free_0'] += 1
        elif v < 99:
            bands['inflated_1_98'] += 1
        elif v >= 99:
            bands['lethal_ge99'] += 1
        else:
            bands['other'] += 1
    print('  ---- 全图按值分带（★ 正向对照 ✓）----')
    for k, v in bands.items():
        print('    %-14s %9d 格  %6.2f%%' % (k, v, 100.0 * v / tot))

    # ★ ① 点名取格 ＋ 邻域
    def at(px, py):
        return int(math.floor((px - ox) / res)), int(math.floor((py - oy) / res))

    rows = []
    print('  ---- ★ 点名取格（★ 每格给「值 ＋ 邻域」✓）----')
    for (name, px, py) in pts:
        cx, cy = at(px, py)
        inside = (0 <= cx < W) and (0 <= cy < H)
        val = d[cy * W + cx] if inside else None
        nb = []
        for j in range(-a.win, a.win + 1):
            for i in range(-a.win, a.win + 1):
                gx, gy = cx + i, cy + j
                nb.append(d[gy * W + gx] if (0 <= gx < W and 0 <= gy < H) else None)
        nz = [v for v in nb if v not in (None, 0, -1)]
        print('    %-12s world(%7.3f,%8.3f) ⇒ cell(%d,%d) %s ｜ **代价 = %s**'
              % (name, px, py, cx, cy, '在图内 ✓' if inside else '★ 在图外 ✗',
                 val if inside else '—'))
        print('       邻域 %dx%d：%s' % (2 * a.win + 1, 2 * a.win + 1,
              ' '.join('.' if v is None else ('?' if v < 0 else str(v)) for v in nb)))
        print('       邻域内 非零且非未知 的格数 = %d ｜ 致命(>=99) = %d'
              % (len(nz), sum(1 for v in nz if v >= 99)))
        rows.append({'name': name, 'world': [px, py], 'cell': [cx, cy],
                     'inside': inside, 'cost': val,
                     'neighborhood': nb,
                     'n_nonzero_nonunknown': len(nz),
                     'n_lethal': sum(1 for v in nz if v >= 99)})
    if a.out:
        json.dump({'topic': a.topic, 'info': {'w': W, 'h': H, 'res': res,
                                              'origin': [ox, oy]},
                   'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                   'bands': bands, 'cells': rows},
                  open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
        print('  ⇒ 落盘 %s ✓' % a.out)
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

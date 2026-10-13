#!/usr/bin/env python3
"""★ 阶段八判据 ⑤：**车还没动**时，静态障碍在全局图上必须已是致命格、且带 0.25 m 梯度。

**为什么需要**（reviewer 2026-10-09）：`nav2_params.yaml` 的全局图只有 `obstacle_layer`
（观测驱动、Overwrite + clearing），而 `planner_server.GridBased.allow_unknown: true`
把未知格当可通行 ⇒ **车没走到之前楼在图上根本不存在，规划器会径直朝楼规划** ✗。
10 m 场景看不出来（6 m 量程勉强够），100 m 场景里这是致命的。
修法是全局图挂 `static_layer`（图由 `static_walls.py` 从 world 现生成）。

**判据（逐条都能失败）**：
  ① `/rtk/static_map` 上，**每件障碍的形心格 = 100（致命）**；
  ② `/global_costmap/costmap` 上，同一点 = 100；
  ③ **梯度**：从障碍面向外走，`/global_costmap/costmap` 在 0.05~0.25 m 之间**必须出现过
     `0 < cost < 100` 的充气格**、而 0.30 m 之外**必须不再是致命格（100）** —— 这条证明
     `static_layer` 跑在 `inflation_layer` **之前**（对照：`KeepoutFilter` 是 filter、
     跑在 inflation 之后 ⇒ 面外直接掉到 0/未知、没有中间档）。
     ★ **远侧判"不是致命"，不是判"等于 0"**（第一版写成 `== 0` ⇒ 全线假报 ✗）：
     本图 `track_unknown_space: true` + 静态层只标障碍 ⇒ 未观测处是 **-1（未知）**，
     不是 0。写成 `== 0` 会把**正常行为**判成失败（`campus8-b` 实测：23/23 全红，
     而四项核心读数其实全对）。
  ④ `/rtk/static_map` 上**非障碍处必须是 -1（未知）**，不许被写成 0（可通行）——
     否则静态层会把"没去过"谎报成"走得通"。

★ **单一真源**：障碍几何从 SDF **现解析**（`stop_margin_check.parse_raised_boxes`，
与生成器/静态层同一把尺子），**不手抄坐标**。
★ 与 `static_walls.py --self-test` 的分工：那个是**离线**核"图生成得对不对"；
这个是在**跑着的栈**上核"图有没有真的进到代价图、且带梯度"。

用法（容器内）：
  python3 scripts/campus_static_layer_check.py --world worlds/campus_100.sdf \
      --json /work/artifacts/rtk-nav2/<run-id>/nav2/static_layer_check.json [--timeout 60]
退出码：0 = 四条全过；1 = 用法/取数失败；2 = 有判据不过。
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stop_margin_check import parse_raised_boxes          # noqa: E402

STATIC_TOPIC = '/rtk/static_map'
COST_TOPIC = '/global_costmap/costmap'

# nav2 的 Costmap2DPublisher 与 `static_walls.py` 都用 transient_local 发 ⇒
# 订阅端必须同 durability，否则要等下一个发布周期（`static_walls.py` 1 Hz，等得起；
# 但代价图的 latched 只有一帧时用默认 QoS 可能永远收不到）。
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

LETHAL = 100
# 梯度探测的横向距离（米）：靠面一侧必须出现充气格，远侧必须回到 0
GRAD_IN = [0.05, 0.15, 0.25, 0.30]
GRAD_OUT = 0.35


def grid_at(grid, x, y):
    """把世界坐标换算成格子值；出界返回 None。"""
    info = grid['info']
    cx = int((x - info['origin'][0]) / info['res'])
    cy = int((y - info['origin'][1]) / info['res'])
    if not (0 <= cx < info['w'] and 0 <= cy < info['h']):
        return None
    return int(grid['data'][cy * info['w'] + cx])


class Collector(Node):
    def __init__(self):
        super().__init__('campus_static_layer_check')
        self.maps = {}
        for topic, key in ((STATIC_TOPIC, 'static'), (COST_TOPIC, 'cost')):
            self.create_subscription(OccupancyGrid, topic,
                                     self._mk(topic, key), LATCHED)

    def _mk(self, topic, key):
        def cb(msg):
            # ★★ reviewer 2026-10-10：**必须记 `header.stamp`** ✗（原来只有 `frame_id` ✗）★★
            #   ⇒ 否则即使跑两次，**第二次也给不出活性证据** ✗ ⇒ 复算清单第 4 项照样是"**未判**" ✗。
            #   ★ 判据 = **第二次的 `stamp` > 第一次** ✓（= 两次确实是**两拍不同的观测** ✓）；
            #     ★ 而**内容可以完全相同** ✓ —— 这正是"**内容变没变不是活性证据**"的**对称面** ✓
            #     （静止的健康栈两次本来就一样 ⇒ 拿内容判活会**误杀** ✓）。
            #   ★★ 取哪个话题的 stamp 要认对：用 `/global_costmap/costmap` ✓（**2 Hz 会推进** ✓）；
            #     ✗ **别用 `/rtk/static_map`** ✗ —— 它是 **SDF 喂的** ✓，很可能是 latched/只在变化时发
            #     ⇒ `stamp` **不推进** ⇒ 拿它会**误判"未判"** ✗。
            #   ★ 计数直接从上一拍取（**免新增状态** ✓ ⇒ 跟着原有序列化一起落盘 ✓，不必改 JSON 写出处 ✓）。
            _prev = self.maps.get(key) or {}
            try:
                _pubs = int(self.count_publishers(topic))
            except Exception:
                _pubs = -1          # ★ 拿不到就记 -1（**不假装有数** ✗；且它只作**诊断**，不当判据 ✓）
            self.maps[key] = {
                'info': {'res': msg.info.resolution,
                         'w': msg.info.width, 'h': msg.info.height,
                         'origin': [msg.info.origin.position.x, msg.info.origin.position.y]},
                'data': np.asarray(msg.data, dtype=np.int16),
                'frame': msg.header.frame_id,
                'stamp': {'sec': msg.header.stamp.sec,
                          'nanosec': msg.header.stamp.nanosec},
                'recv_count': int(_prev.get('recv_count', 0)) + 1,
                'publishers': _pubs,
            }
        return cb


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--json', default='')
    ap.add_argument('--timeout', type=float, default=60.0)
    # ★ reviewer 2026-10-09 要的："把两张**原始栅格**落盘" ⇒ 他才能**逐格**重算
    #   23/23 与两张直方图，而不是只能"交叉验证"（我这边算的汇总数他没法逐格复现）。
    ap.add_argument('--dump-grids', default='',
                    help='把两张原始栅格落成 .npz（键：static/cost 及其 info）。'
                         '给了这个路径就写，不给就不写（默认行为不变）。')
    a = ap.parse_args(argv)

    if not os.path.exists(a.world):
        print('!! 找不到世界 %s' % a.world, file=sys.stderr)
        return 1
    boxes = parse_raised_boxes(a.world)
    if not boxes:
        print('!! 世界里没解析到障碍', file=sys.stderr)
        return 1

    rclpy.init()
    node = Collector()
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    t0 = time.time()
    while time.time() - t0 < a.timeout and len(node.maps) < 2:
        rclpy.spin_once(node, timeout_sec=0.2)
    got = sorted(node.maps)
    if len(node.maps) < 2:
        print('!! 没取到两张图（拿到：%s）⇒ 查发布者与 QoS' % got, file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    st, co = node.maps['static'], node.maps['cost']
    rows, bad = [], []
    for b in boxes:
        cx, cy = (b['x0'] + b['x1']) / 2.0, (b['y0'] + b['y1']) / 2.0
        s_center = grid_at(st, cx, cy)
        c_center = grid_at(co, cx, cy)
        # ---- ③ 梯度：沿 ±x / ±y 四个方向从面往外走，取第一条不落在别的箱体里的方向 ----
        grad = None
        dirs = [('+x', b['x1'], cy, 1.0, 0.0), ('-x', b['x0'], cy, -1.0, 0.0),
                ('+y', cx, b['y1'], 0.0, 1.0), ('-y', cx, b['y0'], 0.0, -1.0)]
        for (dn, px, py, ux, uy) in dirs:
            vals = []
            blocked_by_other = False
            for d in GRAD_IN + [GRAD_OUT]:
                qx, qy = px + ux * d, py + uy * d
                for o in boxes:
                    if o['name'] == b['name']:
                        continue
                    if o['x0'] - 1e-9 <= qx <= o['x1'] + 1e-9 and o['y0'] - 1e-9 <= qy <= o['y1'] + 1e-9:
                        blocked_by_other = True
                        break
                if blocked_by_other:
                    break
                vals.append((d, grid_at(co, qx, qy)))
            if blocked_by_other or not vals:
                continue
            inner = [v for (d, v) in vals if d <= 0.30 and v is not None]
            outer = [v for (d, v) in vals if d >= GRAD_OUT and v is not None]
            has_band = any(v is not None and 0 < v < LETHAL for v in inner)
            # ★ 远侧判"**不再是致命格**"，不是判"== 0" —— 未观测处是 -1（未知），
            #   那是本图的**正常**取值（`track_unknown_space: true`）。见文件头判据 ③。
            outer_not_lethal = all(v is not None and v != LETHAL for v in outer) if outer else False
            grad = {'dir': dn, 'samples': [{'d': d, 'cost': v} for (d, v) in vals],
                    'has_inflation_band': has_band, 'outer_not_lethal': outer_not_lethal}
            break
        rec = {'name': b['name'], 'center': [round(cx, 3), round(cy, 3)],
               'static_center': s_center, 'cost_center': c_center,
               'gradient': grad}
        ok = (s_center == LETHAL and c_center == LETHAL
              and grad is not None and grad['has_inflation_band']
              and grad['outer_not_lethal'])
        rec['ok'] = bool(ok)
        if not ok:
            bad.append(rec)
        rows.append(rec)

    # ---- ④ 静态图非障碍处必须是未知(-1) ----
    s_vals, s_counts = np.unique(st['data'], return_counts=True)
    dist = {int(v): int(c) for v, c in zip(s_vals, s_counts)}
    free_cells = dist.get(0, 0)
    c_vals, c_counts = np.unique(co['data'], return_counts=True)
    cdist = {int(v): int(c) for v, c in zip(c_vals, c_counts)}

    result = {
        'world': os.path.abspath(a.world),
        'world_sha256': __import__('hashlib').sha256(open(a.world, 'rb').read()).hexdigest(),
        'static_topic': STATIC_TOPIC, 'cost_topic': COST_TOPIC,
        'static_grid': st['info'], 'cost_grid': co['info'],
        'obstacles_total': len(rows), 'obstacles_ok': len(rows) - len(bad),
        'failed': bad, 'per_obstacle': rows,
        'static_value_histogram': dist,
        'cost_value_histogram': cdist,
        'criterion_4_static_free_cells': free_cells,
        'criterion_4_ok': free_cells == 0,
        'notes': {
            'criterion_3': '梯度 = 面外 0.05~0.25 m 里出现过 0 < cost < 100，且 0.30 m 外不再是致命格；'
                           '它证明 static_layer 跑在 inflation 之前（filter 跑在之后 ⇒ 面外直接掉到 0/未知）；'
                           '★ 远侧允许 -1（未知）—— 本图 track_unknown_space: true，未观测处就是 -1',
            'criterion_4': '静态图上非障碍处必须全是 -1（未知）；出现 0（可通行）就等于'
                           '把"没去过"谎报成"走得通"',
        },
    }
    ok = (not bad) and result['criterion_4_ok']
    result['verdict'] = '通过' if ok else '不通过'

    print(json.dumps({k: result[k] for k in
                      ('verdict', 'obstacles_total', 'obstacles_ok', 'static_grid',
                       'cost_grid', 'criterion_4_static_free_cells')},
                     ensure_ascii=False, indent=2))
    for r in rows:
        print('  %-16s 形心 static=%4s cost=%4s  %s' %
              (r['name'], r['static_center'], r['cost_center'],
               ('梯度 %s' % r['gradient']['dir']) if r['gradient'] else '梯度 无可用方向'))
    if a.dump_grids:
        # 逐格复算用的原始数据：两张图**原样**（含 -1），外加各自的栅格元数据。
        # 不做任何处理 —— 对方要能拿它自己重算直方图与 23 个形心格。
        np.savez_compressed(
            a.dump_grids,
            static=st['data'].astype(np.int16), cost=co['data'].astype(np.int16),
            static_res=st['info']['res'], static_w=st['info']['w'], static_h=st['info']['h'],
            static_origin=np.asarray(st['info']['origin'], dtype=np.float64),
            cost_res=co['info']['res'], cost_w=co['info']['w'], cost_h=co['info']['h'],
            cost_origin=np.asarray(co['info']['origin'], dtype=np.float64),
            world_sha256=np.asarray(result['world_sha256']),
        )
        result['grids_dump'] = a.dump_grids
        print('原始栅格已写 %s' % a.dump_grids, file=sys.stderr)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(result, fh, ensure_ascii=False, indent=2)
        print('已写入 %s' % a.json, file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())

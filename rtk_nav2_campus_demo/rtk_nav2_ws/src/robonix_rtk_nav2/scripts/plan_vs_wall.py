#!/usr/bin/env python3
"""★ 新世界验收读数 ②：**规划路径是否穿越台阶线 `x=0`（`y∈[-3,3]`）** —— 应为**否** ✓。

（取代旧世界的 `plan_vs_edge.py`（那份按 `y=0.5` 的南沿算）；清单来自 `world_edges.py` **单一真源** ✓。）

报：
· **是否穿越**（逐段判，给穿越点的 `y`）⇒ 期望**无一处在 `y∈[-3,3]` 内** ✓；
· **绕行段位置**：路径的 `y` 范围、以及它**在哪段 `y` 上跨过 `x=0`** ⇒ 应显示从 `y>3` 的低台侧跨过 ✓；
· 路径 ↔ 最近清单线段的距离；
· 穿点/跨点附近半径 0.35 m 内的致命格数（**"未发布"用 `has_mask=false` 区分** ✓，不写成 0 格 ✗）。

用法：plan_vs_wall.py --json out.json [--seconds 25] [--mask-seconds 60]
"""
import argparse
import json
import math
import os
import sys
import time

# ★ ROS/numpy 一律**惰性导入**（只在"取数"路径里 import）——
#   这样 `--rederive` 这条**纯离线**路径在**没装 ROS 的机器上也能跑** ✓
#   （reviewer 的"可复算"要求：补救产物的工具不该只能在容器里跑 ✗）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_edges as WE  # noqa: E402


def detour_from_crossings(cross_any):
    """★ 纯函数：由 `cross_x0_all_y` 推 `(detour_ok, note)` —— 三态，离线可复算 ✓。

    抽出来是为了**补救已落盘的 JSON**（旧版 note 与字段不同源 ✗）：那些文件的
    `cross_x0_all_y` 是在的 ⇒ 用同一判据即可**离线重算**、不必再起栈 ✓。
    """
    if not cross_any:
        return (None, '— 该规划路径**完全不跨 `x=0`** ⇒ **无从判绕行** ⇒ 记 `None`，不是 `False` ✗')
    if all(y > WE.DETOUR_MUST_CROSS_ABOVE_Y for y in cross_any):
        return (True, '✅ 跨 `x=0` 全部发生在 `y>%.1f`（低台侧）⇒ 绕行成立' % WE.DETOUR_MUST_CROSS_ABOVE_Y)
    bad = [y for y in cross_any if y <= WE.DETOUR_MUST_CROSS_ABOVE_Y]
    return (False, '✗ 跨 `x=0` 有 **%d 处**落在 `y≤%.1f`（= %s）⇒ 绕行不成立'
            % (len(bad), WE.DETOUR_MUST_CROSS_ABOVE_Y, bad))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--seconds', type=float, default=25.0)
    ap.add_argument('--mask-seconds', type=float, default=60.0)
    ap.add_argument('--rederive', action='store_true',
                    help='★ 离线重算模式：读已存在的 --json，按 cross_x0_all_y 重算 detour 字段并覆写'
                         '（补救旧版"note 与字段不同源"的产物；**不起栈** ✓）')
    ap.add_argument('--traj-csv', default='',
                    help='★ **轨迹口径**（reviewer 2026-10-09 拍）：给 `pose_attitude.csv` ⇒ 追加'
                         '`trajectory_points_in_ramp` / `route_used_ramp_traj`（问"**车是否真的走过坡**" ✓；'
                         '与"规划器是否在走坡"是两个问题 ✓）；**离线** ✓')
    a = ap.parse_args()

    # ★ 轨迹口径可以**单独跑**（不起栈、不订阅）✓
    if a.traj_csv and not a.rederive:
        import csv as _csv
        rows = []
        try:
            with open(a.traj_csv) as f:
                for r in _csv.DictReader(f):
                    try:
                        rows.append((float(r['gt_x']), float(r['gt_y'])))
                    except Exception:
                        pass
        except FileNotFoundError:
            print('!! 找不到 %s ⇒ 退出码 1' % a.traj_csv)
            return 1
        inr = [(x, y) for x, y in rows
               if WE.RAMP_X0 <= x <= WE.RAMP_X1 and WE.RAMP_Y0 <= y <= WE.RAMP_Y1]
        d0 = {}
        if os.path.exists(a.json):
            d0 = json.load(open(a.json))
        d0['traj_csv'] = os.path.basename(a.traj_csv)
        d0['traj_samples'] = len(rows)
        d0['ramp_region'] = {'x': [WE.RAMP_X0, WE.RAMP_X1], 'y': [WE.RAMP_Y0, WE.RAMP_Y1]}
        d0['trajectory_points_in_ramp'] = len(inr)
        d0['route_used_ramp_traj'] = bool(inr)
        d0['route_used_ramp_traj_basis'] = '**整轮真值轨迹**（`pose_attitude.csv` 的 `gt_x/gt_y`）'
        d0['route_used_ramp_traj_note'] = (
            '✅ 轨迹有 %d 个样本落在坡区 ⇒ **车确实走过坡** ✓（基准：整轮真值轨迹）' % len(inr) if inr else
            '✗ 轨迹**没有**样本落在坡区（`x∈[%.2f,%.2f], y∈[%.2f,%.2f]`）⇒ **车没走过坡** ✓'
            % (WE.RAMP_X0, WE.RAMP_X1, WE.RAMP_Y0, WE.RAMP_Y1))
        with open(a.json, 'w') as f:
            json.dump(d0, f, indent=1, ensure_ascii=False)
        print(json.dumps({k: v for k, v in d0.items()
                          if k in ('traj_csv', 'traj_samples', 'trajectory_points_in_ramp',
                                   'route_used_ramp_traj', 'route_used_ramp_traj_basis',
                                   'route_used_ramp_traj_note', 'route_uses_ramp_basis')},
                         indent=1, ensure_ascii=False))
        return 0

    if a.rederive:
        with open(a.json) as f:
            d = json.load(f)
        ca = d.get('cross_x0_all_y')
        if ca is None:
            print('!! 该文件没有 `cross_x0_all_y` ⇒ 无法重算（退出码 1）')
            return 1
        ok, note = detour_from_crossings(ca)
        before = (d.get('detour_ok'), d.get('detour_note'))
        d['detour_ok'] = ok
        d['detour_note'] = note
        d['detour_basis'] = ('**本快照的规划路径**（`/rtk/plan_path` 当次 latched 值）——'
                            '**不是**整轮轨迹、也不是真值轨迹 ✗')
        d['detour_rederived'] = True
        with open(a.json, 'w') as f:
            json.dump(d, f, indent=1, ensure_ascii=False)
        print('%s' % a.json)
        print('  旧: detour_ok=%r  note=%s' % before)
        print('  新: detour_ok=%r  note=%s' % (ok, note))
        print('  cross_x0_all_y=%s ⇒ 三态判据：空=None、全>%.1f=True、有≤=False ✓'
              % (ca, WE.DETOUR_MUST_CROSS_ABOVE_Y))
        return 0

    # ★ 以下才需要 ROS/numpy（惰性导入 ✓）
    import numpy as np
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)

    rclpy.init()
    n = Node('plan_vs_wall')
    d = {}
    n.create_subscription(Path, '/rtk/plan_path', lambda m: d.__setitem__('p', m), Q)
    n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map', lambda m: d.__setitem__('m', m), Q)
    t = time.time()
    while time.time() - t < a.seconds and 'p' not in d:
        rclpy.spin_once(n, timeout_sec=0.2)
    t2 = time.time()
    while time.time() - t2 < a.mask_seconds and 'm' not in d:
        rclpy.spin_once(n, timeout_sec=0.2)

    pth = d.get('p')
    out = {'world_sha256': WE.WORLD_SHA256, 'main_edge': WE.MAIN_EDGE,
           'has_plan': pth is not None, 'has_mask': 'm' in d}
    if pth is not None:
        pts = [(float(q.pose.position.x), float(q.pose.position.y)) for q in pth.poses]
        out['frame'] = pth.header.frame_id
        # ★ 时刻字段（reviewer 2026-10-08 要求：**"看见了还是没看见"只能靠对时** ✗）
        #   没有它，规划路径就无法与 `merge_events` 的入库时刻对齐 ⇒ (i) 时序 / (ii) 图没被消费 分不清 ✓
        out['plan_wall'] = round(time.time(), 3)
        out['plan_stamp_s'] = round(pth.header.stamp.sec + pth.header.stamp.nanosec * 1e-9, 6)
        out['plan_frame_id'] = pth.header.frame_id
        out['n_points'] = len(pts)
        out['path'] = [[round(x, 3), round(y, 3)] for x, y in pts]
        out['path_y_min'] = round(min(y for _, y in pts), 3)
        out['path_y_max'] = round(max(y for _, y in pts), 3)

        # ── ① 是否穿越主边（x=0 且 y∈[-3,3]）──
        cross_main = []
        for p1, p2 in zip(pts, pts[1:]):
            hit = WE.enters_main_cliff(p1, p2)
            if hit is not None:
                cross_main.append([round(hit[0], 3), round(hit[1], 3)])
        out['crosses_main_cliff'] = bool(cross_main)
        out['cross_main_points'] = cross_main
        out['cross_main_note'] = ('✅ 未穿越主边' if not cross_main
                                  else '✗ 穿越了主边 ⇒ 不满足 demo 要求')

        # ── ② 绕行段：路径在哪些 y 上跨过 x=0（**任意 y**，用于证明确实从低台侧过）──
        cross_any = []
        for p1, p2 in zip(pts, pts[1:]):
            x1, y1 = p1
            x2, y2 = p2
            if abs(x2 - x1) < 1e-12:
                continue
            tt = (0.0 - x1) / (x2 - x1)
            if 0.0 <= tt <= 1.0:
                cross_any.append(round(y1 + tt * (y2 - y1), 3))
        # ★ reviewer 2026-10-08 新加操作项：**路线必须真的走坡** ✓ ——
        #   来历：`ramp7-ak` 里主边覆盖补齐后，规划器**改走南边**、一步没进坡区 ⇒ 只看"不穿越主边"会漏掉 ✗
        #   取法：最终规划路径落在坡区（`x∈[RAMP_X0,RAMP_X1]`, `y∈[RAMP_Y0,RAMP_Y1]`）的点数必须 >0 ✓
        in_ramp = [(x, y) for x, y in pts
                   if WE.RAMP_X0 <= x <= WE.RAMP_X1 and WE.RAMP_Y0 <= y <= WE.RAMP_Y1]
        out['ramp_region'] = {'x': [WE.RAMP_X0, WE.RAMP_X1], 'y': [WE.RAMP_Y0, WE.RAMP_Y1]}
        # ★★ 两个口径**都报**（reviewer 2026-10-09 拍）：它们回答**不同问题** ✓ ——
        #   · `route_uses_ramp`（**基准 = 终态那份规划路径**）⇒ 问"**规划器现在是否在走坡**" ✓
        #   · `route_used_ramp_traj`（**基准 = 整轮真值轨迹**，需 `--traj-csv`）⇒ 问"**车是否真的走过坡**" ✓
        #   ✗ 一个字段兼两义 ⇒ 下一个人会把"终态规划不涉坡"读成"车没走坡"（`ramp7-ao` 正是如此 ✗）
        out['route_uses_ramp_basis'] = '**终态那份规划路径**（`/rtk/plan_path` 当次 latched 值）'
        out['path_points_in_ramp'] = len(in_ramp)
        out['route_uses_ramp'] = bool(in_ramp)     # ★ 判据：规划器是否在走坡
        out['route_uses_ramp_note'] = (
            '✅ 规划路径有 %d 个点落在坡区 ⇒ **规划器在走坡** ✓（基准：终态规划路径）' % len(in_ramp) if in_ramp else
            '✗ 规划路径**一个点都没进坡区**（`x∈[%.2f,%.2f], y∈[%.2f,%.2f]`）⇒ **规划器没在走坡**'
            '（基准：**终态**规划路径 —— 若车当时已离开坡，这**不代表"车没走过坡"** ✗，见 `route_used_ramp_traj`）'
            % (WE.RAMP_X0, WE.RAMP_X1, WE.RAMP_Y0, WE.RAMP_Y1))
        out['cross_x0_all_y'] = cross_any
        # ★ 判据基准必须写死，否则 note 与字段会不同源 ✗（reviewer 2026-10-08 抓出：
        #   旧版 `detour_ok = bool(cross_any) and all(...)` ⇒ **空的 cross_any 会落到 else 分支**、
        #   note 却说"存在 y≤3.0 处的跨线" ✗ —— **断言了一个字段里并不存在的情况** ✓）
        # ★ 与 `detour_from_crossings()` **同一个实现**（不写第二份 ⇒ 不会漂 ✓）
        out['detour_basis'] = ('**本快照的规划路径**（`/rtk/plan_path` 当次 latched 值）——'
                               '**不是**整轮轨迹、也不是真值轨迹 ✗')
        out['detour_ok'], out['detour_note'] = detour_from_crossings(cross_any)
    else:
        out['crosses_main_cliff'] = None
        out['crosses_main_cliff_note'] = '⚠ 未收到规划路径 ⇒ **未测**（不是"未穿越" ✗）'

    # ── ③ 路径 ↔ 最近清单线段 ──
    if pth is not None:
        best = (None, 1e9, None)
        for p in pts:
            nm, dd = WE.classify(p)
            if dd < best[1]:
                best = (nm, dd, p)
        out['plan_min_dist_to_inventory_m'] = round(best[1], 4)
        out['plan_closest_segment'] = best[0]
        out['plan_closest_point'] = None if best[2] is None else [round(best[2][0], 3), round(best[2][1], 3)]

    # ── ④ 穿点附近 mask 盖没盖住（"未发布"与"0 格"必须分开）──
    m = d.get('m')
    if m is not None and pth is not None and out.get('cross_main_points'):
        i = m.info
        g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
        ys, xs = np.nonzero(g >= 100)
        if len(xs):
            mx = i.origin.position.x + (xs + 0.5) * i.resolution
            my = i.origin.position.y + (ys + 0.5) * i.resolution
            M = np.stack([mx, my], axis=1)
            out['mask_near_cross'] = [
                {**{'x': c[0], 'y': c[1], 'r': 0.35},
                 'cells': int(np.sum(np.linalg.norm(M - np.array(c), axis=1) <= 0.35)),
                 'nearest_cell_m': round(float(np.min(np.linalg.norm(M - np.array(c), axis=1))), 4)}
                for c in out['cross_main_points']]
        else:
            out['mask_near_cross'] = []
            out['mask_near_cross_note'] = 'mask 已发布但一个致命格都没有'
    elif m is None:
        out['mask_near_cross'] = None
        out['mask_near_cross_note'] = '⚠ mask 未在等待窗口内到达 ⇒ **未测**（不是 0 格 ✗）'

    need = ['crosses_main_cliff', 'cross_x0_all_y', 'has_mask']
    out['missing_fields'] = [k for k in need if k not in out]
    with open(a.json, 'w') as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    print(json.dumps({k: v for k, v in out.items() if k != 'path'}, indent=1, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)

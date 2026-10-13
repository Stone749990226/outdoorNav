#!/usr/bin/env python3
"""★ 自动判据 `main_edge_gap_check`（reviewer 2026-10-08 定）：**主边覆盖有没有空隙**。

来历：`ramp7-ag` 里主边掩膜在 `y` 向有**唯一一处空隙 `1.44→2.54`（宽 1.10 m）** ✗，
而车**恰在 `y=1.45` 从那里穿过去** ⇒ 这条空隙是那轮 demo 失败的**直接成因** ✓。
⇒ 把它做成**自动判据**，下一轮**直接看它有没有清掉** ✓，不必再靠人翻图 ✓。

**口径（取数前声明）**：
· 主边 = `world_edges.py` 的 `SEGMENTS[0]` = **`x=0, y∈[-3,3]`**（清单**单一真源** ✓）；
· 沿 `y` 按 **0.05 m 分箱** ✓；某箱"**已覆盖**" = **该边缘点落在某个（已外扩的）致命格内** ⇒ `--tol` 默认
  **0.035 = 半格** ✓（★ **不要再叠外扩量 0.30/0.35** ✗ —— 外扩**已烘进格坐标**，再用一次会**高估覆盖、低估空隙**）；
· **空隙** = 连续未覆盖箱构成的一段，且**宽度 > `--min-gap`**（默认 0.10 m）✓；
· ★★ **两层门槛（reviewer 2026-10-09 更正他自己）** ✓ ——
  **检测层** = `--min-gap`（默认 **0.10 m**）= **报出来**的门槛 ✓（= 2 个 0.05 格 = **分辨率** ✓）；
  **阻塞层** = **从机器人几何导出** ✓：`block_gap_min = 2×inflation_radius + 2×robot_radius`
  （两侧各被 inflation 吃掉 → 中间还得放得下机器人本体 ✓）；本项目实配 `0.25 / 0.105` ⇒ **`0.71 m`** ✓
  ⇒ **只有宽 ≥ `block_gap_min` 的空隙才算阻塞** ✓；**更窄的照报、并单列 `residual_never_measured`** ✓。
  ✗ 先前的错：**把 0.10 当"阻塞阈值"用** ✗ —— 那其实是分辨率、**不是"规划器能不能用"** ✓
  （实测后果：`ramp7-ao` 主边南端 `0.25 m` 残差被误判成阻塞 ✗ ⇒ 更正后 = **不阻塞、且清楚标为"从未测量"** ✓）。
· ★★ **判据（2026-10-09 封边后的当前口径）= 阻塞清单里的边全部无空隙** ✓
  **阻塞清单 = `world_edges.BLOCKING_EDGES` = 只剩「东主边 `x=0, y∈[-3,3]` 全段」** ✓ ——
  **南/西已用实体块封住**（块顶高出平台面 ⇒ 读作"墙"、不是落差 ⇒ 不再是可下的通道 ✓；
  北沿加宽后整条都是坡口 ⇒ 无落差、两块北封已删 ✓）
  ⇒ 它们**改列"已封边（记录在案、不计）"**，仍逐条报出（便于对照 ✓），但**不阻塞** ✓。
  · **口径沿革（别读错历史结论）**：先"只看主边" ⇒ 规划器**改从南沿的洞**下去 ✗；扩到南沿 ⇒ 西沿还留着 2.55/0.70 m ✗
    ⇒ **只盯一条边不够：洞会换边** ✓；且"逐个补观察位姿"**不收敛、也不可复现** ✗（`ramp7-an` 实测：同一名义位姿，
    `ak` 清掉主边而 `an` 没清掉，差别在**实测停稳朝向** 10.1° vs 25.6° ✗）⇒ AuYang 选 **(ii) 场景封边** ✓ ⇒ 才有现在这条口径 ✓。
  · `--y-lo/--y-hi/--ref-y-lo` 这几个参数**已废弃**（保留只为向后兼容 ✓，不再影响判据 ✓）；
· **判据：阻塞清单里每条边都不得有 ≥ `block_gap_min` 的空隙** ✓（非空 ⇒ 打印属哪条边 + 起止**坐标** + 宽度 + 判定依据，并给退出码 2）。

两种取数方式（都可复算 ✓）：
· **在线**：`--mask-seconds N` 订阅 `/rtk/drop/mask_map`；
· **离线**：`--from-attribution <mask_attribution.json>`（读其 `per_cell`）—— **不需要 ROS** ✓（惰性导入 ✓）。
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_edges as WE  # noqa: E402

EXIT_PASS, EXIT_USAGE, EXIT_FAIL = 0, 1, 2


def gaps_along_edge(cells, a_pt, b_pt, tol, bin_m, min_gap):
    """沿线段 a→b 按 bin_m 分箱，返回 (已覆盖箱数, 总箱数, 空隙列表[dict])。

    ★★ 参数化必须用**沿线距离 s**，**不能用 `py`** ✗ ——
    我第一版用 `py`（因为主边 `x=0` 是竖边、`py` 恰好单调 ✓）⇒ 但**横向边**（南沿/北沿 `y=-3`/`y=3`）
    上 `py` **恒定** ⇒ `s_end - s_start = 0` ⇒ **空隙宽度恒为 0 ⇒ 横向边上的洞完全看不见** ✗
    ⇒ 实测代价：`ramp7-ak` 南沿 `x∈[-0.75,-0.30]` 的 0.45 m 洞**被判成"无空隙"** ✗（**假通过**）。
    ⇒ 现在按 s 参数化 ✓，并把每处空隙的**起止坐标**也报出来 ✓（便于与轨迹对位）。
    """
    ax, ay = a_pt
    bx, by = b_pt
    L = math.hypot(bx - ax, by - ay)
    n = max(1, int(round(L / bin_m)))
    cov = []
    for i in range(n + 1):
        s_ = i * L / n
        u = (s_ / L) if L > 0 else 0.0
        px = ax + u * (bx - ax)
        py = ay + u * (by - ay)
        ok = any(math.hypot(px - cx, py - cy) <= tol for cx, cy in cells)
        cov.append((s_, px, py, ok))
    gaps = []
    run = None
    for s_, px, py, ok in cov:
        if not ok:
            run = [s_, s_, px, py] if run is None else [run[0], s_, run[2], run[3]]
        else:
            if run is not None:
                if run[1] - run[0] > min_gap:
                    gaps.append({'s_start': round(run[0], 3), 's_end': round(run[1], 3),
                                 'width_m': round(run[1] - run[0], 3),
                                 'from_xy': [round(run[2], 2), round(run[3], 2)],
                                 'to_xy': [round(px, 2), round(py, 2)]})
                run = None
    if run is not None and run[1] - run[0] > min_gap:
        gaps.append({'s_start': round(run[0], 3), 's_end': round(run[1], 3),
                     'width_m': round(run[1] - run[0], 3),
                     'from_xy': [round(run[2], 2), round(run[3], 2)],
                     'to_xy': [round(bx, 2), round(by, 2)]})
    return sum(1 for *_r, ok in cov if ok), len(cov), gaps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', default='', help='把结果写到这个文件')
    ap.add_argument('--from-attribution', default='',
                    help='离线：读 mask_attribution.json 的 per_cell（**不需要 ROS** ✓）')
    ap.add_argument('--mask-seconds', type=float, default=60.0, help='在线：等 mask 的上限')
    # ★ 容差必须**紧贴半格**（默认 0.05 m 网格 ⇒ 0.035）✗：
    #   我第一版取 0.35（照抄"外扩量"）是**错的** ✗ —— 外扩**已经烘进格坐标**了，
    #   再用 0.35 去找"边缘点附近的格"，会**把只在一侧 0.30 m 外的格也算成覆盖** ⇒
    #   **覆盖被高估、空隙被低估**（实测：真空隙 `1.44→2.56` 被算成 `1.80→2.15` ✗）。
    #   "该 y 被格盖住" 的正确含义 = **该边缘点落在某个（已外扩的）格内** ✓ ≈ 距离 ≤ 半格 ✓。
    ap.add_argument('--tol', type=float, default=0.035,
                    help='"已覆盖"的距离容差 (m)；默认 0.035 = 半格（外扩已烘进格坐标，**不要再叠外扩量** ✗）')
    ap.add_argument('--bin', type=float, default=0.05, help='沿边分箱 (m)')
    ap.add_argument('--min-gap', type=float, default=0.10,
                    help='★ **检测层**门槛：报出任何宽于此值的空隙 ✓（= 2 个 0.05 格 = 分辨率 ✓，**不是"能用"** ✗）')
    # ★★ **阻塞层**门槛（reviewer 2026-10-09 更正他自己）：0.10 是**分辨率**、不是"规划器能不能用" ✗
    #   ⇒ 阻塞阈值**从机器人几何导出** ✓：`间隙 ≥ 2×inflation_radius + 2×robot_radius` ✓
    #   （两侧各被 inflation 吃掉 → 中间还得放得下机器人本体 ✓）
    #   本项目实配：inflation 0.25、robot_radius 0.105 ⇒ **0.50 + 0.21 = 0.71 m** ✓
    ap.add_argument('--inflation-radius', type=float, default=0.25, help='部署值 inflation_radius（本项 0.25 ✓）')
    ap.add_argument('--robot-radius', type=float, default=0.105, help='部署值 robot_radius（本项 0.105 ✓）')
    # ★ reviewer 2026-10-08 收窄：**主判据只看路线可能穿过的那一段** ——
    #   本轮路线是 `(-2,0)` 往北去坡口 ⇒ 规划器只可能在 `y∈[0,3.0]` 穿主边 ✓
    #   ⇒ **拿整条边的空隙当阻塞项是过严的** ✗（会拿一段不在路线上的空隙去否一轮 ✓）
    ap.add_argument('--y-lo', type=float, default=0.0, help='★ 主判据（阻塞）的 y 下界')
    ap.add_argument('--y-hi', type=float, default=3.0, help='★ 主判据（阻塞）的 y 上界')
    ap.add_argument('--ref-y-lo', type=float, default=-3.0, help='参考段（**不阻塞**）y 下界')
    a = ap.parse_args()

    # ★★ 说明文字里的边名**一律现算、不手写** ✗ —— 2026-10-09 第 5 次"说明过期"：
    #   北封删掉后，`reference_note`/`note` 里手写的"南/西/**北**"**漏改** ✗；
    #   更糟的是它从**上一版自检底下溜过去了** ✗ —— 那版只比"阻塞边**条数**"（3 → 1 变了才报），
    #   **名字**变了它不管 ✓ ⇒ 这次的教训：**自检要比"名字"，不能只比"条数"** ✓
    _blk_lbl = '、'.join(nm.split()[0] for nm, _ in WE.BLOCKING_EDGES) or '无'
    _sea_lbl = '/'.join(nm.split()[0] for nm, _ in WE.SEALED_EDGES) or '无'

    name, (p1, p2) = WE.SEGMENTS[0]
    out = {'world_sha256': WE.WORLD_SHA256, 'edge': name, 'edge_endpoints': [p1, p2],
           'tol_m': a.tol, 'bin_m': a.bin, 'min_gap_m': a.min_gap,
           # ★★ `scope_note` **由清单现算**（不手写 ✗）—— reviewer 2026-10-09 抓出它**第 4 次过期**：
           #   手写说明文字每次改判据都会漏改 ✗ ⇒ 改成从 `WE.BLOCKING_EDGES` / `WE.SEALED_EDGES` **现算** ✓
           #   ⇒ **结构上不可能与判据脱节** ✓（不靠记性 ✓）；并配 `self_check` 互证 ✓
           'scope_note': ('★ 判据 = **阻塞清单里的边全部无空隙**：%s ✓；**已封边（记录在案、不计）**：%s ✓'
                          '（`--y-lo/--y-hi/--ref-y-lo` 已废弃、不影响判据 ✓）'
                          % ('；'.join(nm for nm, _ in WE.BLOCKING_EDGES),
                             '；'.join(nm for nm, _ in WE.SEALED_EDGES))),
           'edges_checked_note': '见 `edges`：每条的 endpoints / blocking / coverage_frac / gaps（含起止坐标）✓'}

    if a.from_attribution:
        if not os.path.exists(a.from_attribution):
            print('!! 找不到 %s ⇒ 退出码 1' % a.from_attribution, file=sys.stderr)
            return EXIT_USAGE
        d = json.load(open(a.from_attribution))
        pc = d.get('per_cell')
        if pc is None:
            print('!! %s 里没有 `per_cell` ⇒ 退出码 1' % a.from_attribution, file=sys.stderr)
            return EXIT_USAGE
        cells = [(float(x['x']), float(x['y'])) for x in pc]
        out['source'] = 'offline:%s (per_cell)' % os.path.basename(a.from_attribution)
        out['n_cells'] = len(cells)
    else:
        # ★ 惰性导入 ROS（离线路径在没装 ROS 的机器上也能跑 ✓）
        import numpy as np
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        rclpy.init()
        n = Node('main_edge_gap_check')
        dd = {}
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map', lambda m: dd.__setitem__('m', m), q)
        import time as _t
        t0 = _t.time()
        while _t.time() - t0 < a.mask_seconds and 'm' not in dd:
            rclpy.spin_once(n, timeout_sec=0.2)
        if 'm' not in dd:
            out['verdict'] = 'UNMEASURED'
            out['note'] = 'mask 未在等待窗口内到达 ⇒ **未测**（不是"无空隙" ✗）'
            if a.json:
                json.dump(out, open(a.json, 'w'), indent=1, ensure_ascii=False)
            print(json.dumps(out, indent=1, ensure_ascii=False))
            n.destroy_node(); rclpy.shutdown()
            return EXIT_FAIL
        m = dd['m']
        i = m.info
        g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
        ys, xs = np.nonzero(g >= 100)
        cells = [(i.origin.position.x + (x + 0.5) * i.resolution,
                  i.origin.position.y + (y + 0.5) * i.resolution) for y, x in zip(ys, xs)]
        out['source'] = 'online:/rtk/drop/mask_map'
        out['n_cells'] = len(cells)
        n.destroy_node(); rclpy.shutdown()

    # ★★ 2026-10-08 reviewer 扩判据：「高台周界」三条边（东/南/北，**排除坡口**）**都**必须无空隙 ✓ ——
    #   来历：`ramp7-ak` 把主边补好后，规划器**改从南沿 `y=-3` 上 `x∈[-0.75,-0.30]` 的洞**下去 ✗
    #   ⇒ **只盯一条边不够**：洞会换边 ✓。**西边 `x=-4` 作参考项**（离路线最远 ✓）。
    # ★★ 2026-10-09 封边后（AuYang 选 (ii)）：**阻塞清单只剩东主边** ✓ ——
    #   南/西已用实体块封住（块顶高出平台面 ⇒ 读作"墙"、不是落差 ⇒ 不再是可下的通道 ✓）；
    #   北沿加宽后整条都是坡口 ⇒ 无落差、北封已删 ✓（**别再写"北"** ✗）
    #   ⇒ 它们从"阻塞项"改列**"已封边（记录在案、不计）"** ✓
    #   ★ 清单来源仍是 `world_edges.py` **单一真源** ✓（不在这里再写一份 ✗）
    EDGES = ([(nm, seg, True) for nm, seg in WE.BLOCKING_EDGES] +
             [(nm, seg, False) for nm, seg in WE.SEALED_EDGES])
    per = []
    block_gaps_all = []
    for nm, (q1, q2), blocking in EDGES:
        c_, t_, g_ = gaps_along_edge(cells, q1, q2, a.tol, a.bin, a.min_gap)
        per.append({'edge': nm, 'endpoints': [list(q1), list(q2)], 'blocking': blocking,
                    'bins_covered': c_, 'bins_total': t_,
                    'coverage_frac': round(c_ / float(t_), 4),
                    'gaps': g_})
        if blocking and g_:
            block_gaps_all.append((nm, g_))
    out['edges'] = per
    out['blocking_edges_with_gaps'] = [nm for nm, _ in block_gaps_all]
    # 兼容旧字段：**按"是不是 blocking"取，不按下标硬编码** ✗ ——
    #   我先前写死 `per[4]`（那是"5 条边时的西沿"）⇒ 封边后清单缩到 3 条 ⇒ **IndexError 当场崩** ✗
    #   （`ramp7-ap` 的逐停点快照因此**全部为空** ✗；而我的脚本把 stderr 重定向掉了 ⇒ 崩了也看不见 ✗ —— 两处都改 ✓）
    east = next((e for e in per if e['blocking']), per[0])
    refs = [e for e in per if not e['blocking']]
    west = refs[0] if refs else {'bins_covered': 0, 'bins_total': 0, 'coverage_frac': 0.0, 'gaps': []}
    cB, tB, gapsB = east['bins_covered'], east['bins_total'], east['gaps']
    cR, tR, gapsR = west['bins_covered'], west['bins_total'], west['gaps']
    out['blocking'], out['reference'] = east, west
    out['bins_covered'], out['bins_total'] = cB, tB
    out['coverage_frac'] = round(cB / float(tB), 4)
    out['gaps'] = east['gaps']
    # ★ 判据 = **三条阻塞边全部无空隙** ✓（不是只看主边 ✗）
    # ★ 自检（reviewer 2026-10-09 建议）：**说明文字提到的阻塞边数 == `edges` 里 `blocking=true` 的条数** ✓
    #   ⇒ 不靠记性就能发现"改判据漏改说明" ✗（与"退出码必须是失败信号"同族：**让脱节可见** ✓）
    # ★ 两层门槛（reviewer 2026-10-09 更正）：检测层 = min_gap（报出来）；阻塞层 = 几何导出 ✓
    block_min = 2.0 * a.inflation_radius + 2.0 * a.robot_radius
    out['block_gap_min_m'] = round(block_min, 4)
    out['block_gap_formula'] = ('2*inflation_radius(%.3f) + 2*robot_radius(%.3f) = %.4f m'
                                % (a.inflation_radius, a.robot_radius, block_min))
    out['two_layer_note'] = ('★ **检测层** (--min-gap %.2f) = 报出任何宽于此值的空隙（= 分辨率 ✓）；'
                             '**阻塞层** (%.2f m) = 只有宽到规划器真能用才算阻塞 ✓ —— '
                             '0.10 是**分辨率、不是"能用"** ✗（reviewer 2026-10-09 更正他自己 ✓）'
                             % (a.min_gap, block_min))
    # ★ 逐条空隙标 blocking（宽 ≥ block_min 才阻塞 ✓）
    blocking_gaps = []
    for e in per:
        if not e['blocking']:
            continue
        for g in e['gaps']:
            g['blocking'] = bool(g['width_m'] >= block_min)
            g['blocking_basis'] = 'width %.3f m %s block_gap_min %.3f m' % (
                g['width_m'], '>=' if g['blocking'] else '<', block_min)
            if g['blocking']:
                blocking_gaps.append((e['edge'], g))
    out['blocking_gaps'] = [{'edge': nm, **g} for nm, g in blocking_gaps]
    out['residual_never_measured'] = [
        {'edge': nm, **g} for nm, g in
        [(e['edge'], g) for e in per if e['blocking'] for g in e['gaps'] if not g['blocking']]]

    _nb = sum(1 for e in per if e['blocking'])
    _nl = len([nm for nm, _ in WE.BLOCKING_EDGES])
    # ★ 判据 = **阻塞层**（几何导出的阈值 ✓）；检测层的空隙**照报**但不必阻塞 ✓
    out['verdict'] = 'PASS' if not blocking_gaps else 'FAIL'
    out['note'] = (
        ('✅ 阻塞边（%s）**无 ≥%.2f m 的阻塞空隙** ✓（检测层 >%.2f m 的空隙：%s）'
         % (_blk_lbl, block_min, a.min_gap,
            [round(g['width_m'], 2) for e in per if e['blocking'] for g in e['gaps']] or '无'))
        if not blocking_gaps else
        ('✗ **%d 条阻塞边有空隙**（>%.2f m）：%s ⇒ 规划器可从任一处穿过'
         % (len(block_gaps_all), a.min_gap,
            '；'.join('%s: %s' % (nm, ['%.2f→%.2f(%.2f m)@%s' % (g['s_start'], g['s_end'], g['width_m'], g['from_xy']) for g in gs])
                      for nm, gs in block_gaps_all))))
    def _fmt(gl):
        return ['%.2f→%.2f(%.2f m)@%s' % (g['s_start'], g['s_end'], g['width_m'], g['from_xy']) for g in gl]
    out['reference_note'] = ('（封边后：%s 已列在 `edges` 里作**"已封边、不计"** ✓；'
                             '阻塞清单只剩 %s ✓ —— 来源 `world_edges.BLOCKING_EDGES` / `SEALED_EDGES` **现算** ✓）'
                             % (_sea_lbl, _blk_lbl))
    # ★★ 自检必须放在**三处说明文字都赋完值之后** ✗ ——
    #   我第一版把它放在 `out['note']` 之前 ⇒ 线上首跑就 `KeyError: 'note'` ✗
    #   （**这正是"改完必须跑一次退化方向"的理由**：`py_compile` 查不出这种顺序错 ✓）
    #   第二道判据：**比"名字"、不只比"条数"** ✗ —— 上一版只比条数（3→1 才报警），
    #   于是北封删掉后手写的那个"北"**照样溜过去** ✗ ⇒ 扫三处文字里的**方位字**，
    #   凡出现却不在当前清单里的 ⇒ 当场报出来 ✓
    _notes_txt = out['note'] + out['reference_note'] + out['scope_note']
    _cur_txt = ' '.join(nm for nm, _ in WE.BLOCKING_EDGES) + ' ' + ' '.join(nm for nm, _ in WE.SEALED_EDGES)
    _stray = [d for d in '东南西北' if d in _notes_txt and d not in _cur_txt]
    _ok = (_nb == _nl) and not _stray
    out['self_check'] = {'blocking_edges_count': _nb, 'scope_note_lists_blocking': _nl,
                         'stray_direction_words': _stray, 'ok': _ok,
                         'note': ('✅ 说明文字与实际清单一致 ✓（阻塞边 %d == %d；方位字无游离 %s）' % (_nl, _nb, _stray)
                                  if _ok else
                                  '✗ **说明文字与判据脱节**（阻塞边 %d vs %d；**游离方位字** %s）' % (_nl, _nb, _stray))}
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1, ensure_ascii=False)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    return EXIT_PASS if not blocking_gaps else EXIT_FAIL


if __name__ == '__main__':
    sys.exit(main())

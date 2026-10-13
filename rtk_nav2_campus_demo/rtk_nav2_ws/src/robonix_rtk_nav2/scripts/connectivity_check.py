#!/usr/bin/env python3
"""**纯几何**核对：场地（去掉外墙、加上几段独立墙之后）仍然是**一块连通的自由区域**。

为什么要有这个脚本：2026-09-21 AuYang 要求在去掉外墙后加几段"非封闭"的墙当障碍，
硬要求是"不能把场地切成不连通的两块、车要能从任何位置绕过每一段墙"。光靠肉眼看坐标
不算核对，所以这里拿 `map_geometry.py`（与生成器同源的几何）做一次逐格泛洪：

  1. 把 |x|,|y| <= 场地半边长 的格子铺一遍（0.05 m 一格，与代价地图同分辨率）；
  2. 到任何障碍表面的距离 < `--clearance` 的格子算**不可通行**（默认 0.25 m = nav2 的
     `inflation_radius`，也就是规划实际会避开的那一圈）；
  3. 从出生点 (2.6, -2.6) 泛洪，看**自由区域是不是只有一个连通块**、
     以及一组探针点（出生点、四个走廊角、各段独立墙的两侧）是不是都在同一块里。

判据：`single_component == true` 且 `all_probes_reachable == true`。
不 import rclpy，跑在哪儿都一样。

用法：
  connectivity_check.py [--clearance 0.25] [--json 结果.json]

--------------------------------------------------------------------------------
★ **2026-10-09（阶段八）新增 `--world` 模式**（reviewer 批准，附两条承诺）：

上面那条默认路径是**第二/三阶段的庭院场地**，几何写死在 `map_geometry.py` 里。
100 m 校园场地（`worlds/campus_100.sdf`）的那份几何在**生成器**里，本脚本不该再抄一遍
（纪律：**单一真源**）。所以加一条 `--world <sdf>`：

  · 几何**从 SDF 现解析** —— 复用 `stop_margin_check.parse_raised_boxes`
    （判据 `顶面 z > 0`，与阶段七同一把尺子 ✓；校园世界里它正好收下楼/绿地/车/周界墙四类、
    排除顶面 z=0 的地面板 ✓）；
  · 给出**逐格泛洪连通性**（起点可达块 / 被切出去的格子数 / 连通分量数）；
  · 给出**最小通道宽度表**：① 逐对障碍的**缝**（0 = 贴合，>0 的按升序列表），
    ② **侵蚀阶梯**（把自由区按"离障碍 >= w"逐级收窄，看第几级开始裂开 ⇒
    2 x 该级 w 就是"全域最窄的**必经**通道"）。
  · 探针默认**自动铺满全域**（`--probe-grid N`，N x N 块各取块内净空最大的自由格）
    —— 照"抽样 ⇒ 全域"的纪律，不靠手挑几个点 ✓。

★★ **两条承诺的落实（reviewer 要求）**：
  ① **不给 `--world` 时行为一字不动** —— 原代码路径整段保留、未改一个字符；
     证据：改动前用阶段七原参数跑的 JSON 与改动后逐字节相同（见交付说明）。
  ② 改完拿**阶段七原参数**复跑一次对照 ✓。

用法（阶段八校园场景）：
  connectivity_check.py --world ../worlds/campus_100.sdf --start 2.0,-30.0 --min-gap 1.5
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_geometry as G  # noqa: E402

# 场地范围（地面方板半边长 = make_world.py 的 (GROUND_HALF + GROUND_MARGIN) + 0.2）。
# 外墙已删除，所以场地的边界只剩"地面铺到哪儿"。
# **2026-09-21 跟随世界一起改**：方板从 10.70 m 铺大到 22.70 m（理由见 make_world.py
# 里 GROUND_MARGIN 的注释：**去外墙后留可行驶的地面余量**；原来写的"消幽灵格"已被 ablation 反证），
# 所以这里的半边长从 5.35 改成 **11.35**。**这个数必须跟 make_world.py 同步**——
# 用地板边当"场地边界"的脚本只有这一处，写小了会把连通性判断做在一块假想的方块里
# （做"板铺多大"的对照实验时用 `--field-half` 显式传，别去改这个常量）。
FIELD_HALF = 11.35

# 探针点：出生点、四个走廊角、以及"要走到某段墙另一边去"的那几个点。
# 每个探针都必须与出生点在同一连通块里，否则"绕不过去"。
PROBES = {
    'start': (2.6, -2.6),
    'ring_ne': (2.6, 2.6),
    'ring_nw': (-2.6, 2.6),
    'ring_sw': (-2.6, -2.6),
    'north_of_p1': (0.0, 4.4),
    'west_of_p2': (-4.4, 0.0),
    'south_of_obstacle_02': (0.0, -4.7),
    'east_of_obstacle_00': (4.7, -2.3),
    'west_of_obstacle_00': (3.2, -2.3),
    'north_of_obstacle_01': (3.6, 4.0),
}


def blocked(x, y, clearance):
    return G.nearest_surface(x, y) < clearance


# =============================================================================
# ★ 阶段八：`--world` 模式（几何从 SDF 现解析 —— 单一真源，见文件头）
# =============================================================================
def load_world_rects(world_path):
    """SDF ⇒ 障碍矩形清单。**复用 `stop_margin_check.parse_raised_boxes`**（同一把尺子）。"""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from stop_margin_check import parse_raised_boxes
    return parse_raised_boxes(world_path)


def rect_dist(px, py, r):
    """点到轴对齐矩形的最短距离（点在矩形内 ⇒ 0）。"""
    dx = max(r['x0'] - px, 0.0, px - r['x1'])
    dy = max(r['y0'] - py, 0.0, py - r['y1'])
    return math.hypot(dx, dy)


def rect_gap(a, b):
    """两个轴对齐矩形的最近距离；相交/贴合 ⇒ 0。"""
    dx = max(a['x0'] - b['x1'], b['x0'] - a['x1'], 0.0)
    dy = max(a['y0'] - b['y1'], b['y0'] - a['y1'], 0.0)
    return math.hypot(dx, dy)


def build_free(n, field_half, cell, rects, clearance):
    """**逐行切片**铺"离任何障碍都 >= clearance"的自由格（bytearray，1 = 自由）。

    为什么不用"逐格调 `rect_dist`"：100 m / 0.05 m = 2012^2 ≈ 405 万格、23 个矩形
    ⇒ 近一亿次调用，纯 Python 要分钟级。**逐行切片**是 O(行数 x 矩形数)：
    第 y 行上，某个矩形膨胀 `clearance` 之后的 x 区间是 `[x0-dx, x1+dx]`，
    其中 `dx = sqrt(c^2 - dy^2)`（`dy` = 该行到矩形 y 范围的距离）⇒ 一次切片赋值铺满整行 ✓。
    膨胀形状因此是**圆角矩形**（真正的欧氏膨胀）而不是方角 ✓。
    """
    free = bytearray(b'\x01') * (n * n)
    for r in rects:
        y_lo = r['y0'] - clearance
        y_hi = r['y1'] + clearance
        r0 = int(math.floor((y_lo + field_half) / cell))
        r1 = int(math.ceil((y_hi + field_half) / cell))
        for row in range(max(0, r0), min(n, r1 + 1)):
            y = -field_half + (row + 0.5) * cell
            dy = max(r['y0'] - y, 0.0, y - r['y1'])
            if dy > clearance:
                continue
            dx = clearance if dy == 0.0 else math.sqrt(max(clearance * clearance - dy * dy, 0.0))
            c0 = int(math.floor((r['x0'] - dx + field_half) / cell))
            c1 = int(math.ceil((r['x1'] + dx + field_half) / cell))
            c0 = max(0, c0)
            c1 = min(n, c1)
            if c1 <= c0:
                continue
            base = row * n
            free[base + c0:base + c1] = b'\x00' * (c1 - c0)
    return free


def free_components(free, n):
    """**行程 + 并查集**求自由格连通分量（比逐格泛洪快得多，纯 Python）。

    返回 `(分量数, [每块格数], 每行的行程表)`；行程表 `[(起, 止, 根), ...] / 行`
    供后面查"某个格子属于哪一块"（只要查几十个点，不必留整张标签图）。
    """
    parent = []

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    sizes = []
    rows_runs = []
    prev = []                       # [(起, 止, id)]
    for row in range(n):
        base = row * n
        line = free[base:base + n]
        runs = []
        i = 0
        while True:
            s = line.find(1, i)
            if s < 0:
                break
            e = s + 1
            while e < n and line[e]:
                e += 1
            runs.append((s, e))
            i = e
        cur = []
        for (s, e) in runs:
            cid = len(parent)
            parent.append(cid)
            sizes.append(e - s)
            cur.append((s, e, cid))
        # 与上一行的行程做**两指针**合并（两侧都按列升序 ⇒ 不需要 n^2 的两两比较）
        j = 0
        for (s, e, cid) in cur:
            while j < len(prev) and prev[j][1] <= s:
                j += 1
            k = j
            while k < len(prev) and prev[k][0] < e:
                union(prev[k][2], cid)
                k += 1
        rows_runs.append(cur)
        prev = cur

    roots = {}
    for cid in range(len(parent)):
        r = find(cid)
        roots[r] = roots.get(r, 0) + sizes[cid]
    return len(roots), sorted(roots.values(), reverse=True), rows_runs, parent, find


def border_roots(free, n, rows_runs, parent, find):
    """**与格子外框连通**的那些分量 —— 它们就是"场地**外面**"（不是可通行区）。

    为什么要这一步：`--world` 的场地半边长按"最外侧障碍 + clearance"自动定，
    而封边墙有 `WALL_EXT` 外伸、膨胀又是**圆角**的 ⇒ 墙外沿总有一两圈"离墙 >= clearance"
    的自由格（实测 8 块、共 17733 格，全是 1~3 格宽的细条）⇒ 泛洪会把"场地封闭"判成
    假阴性 ✗。**通用解法**：从格子外框泛洪得到的那些块 = 场地外面，从域里剔除 ✓
    （比"把 field_half 调小"稳：那条依赖"墙的厚度/外伸量"这些细节，换个世界就失效）。
    """
    roots = set()
    edges = []
    edges.append([(0, c) for c in range(n)])
    edges.append([(n - 1, c) for c in range(n)])
    edges.append([(r, 0) for r in range(n)])
    edges.append([(r, n - 1) for r in range(n)])
    for edge in edges:
        for (r, c) in edge:
            if free[r * n + c]:
                rt = comp_at(rows_runs, parent, find, r, c)
                if rt is not None:
                    roots.add(rt)
    return roots


def component_sizes(rows_runs, parent, find, skip_roots=()):
    """按**根**汇总自由格数（`skip_roots` 里的整块不算）。返回 {根: 格数}。"""
    agg = {}
    for row_runs in rows_runs:
        for (_s, e, cid) in row_runs:
            rt = find(cid)
            if rt in skip_roots:
                continue
            agg[rt] = agg.get(rt, 0) + (e - _s)
    return agg


def cell_of(pt, field_half, cell, n):
    c = int((pt[0] + field_half) / cell)
    r = int((pt[1] + field_half) / cell)
    if not (0 <= r < n and 0 <= c < n):
        return None
    return (r, c)


def comp_at(rows_runs, parent, find, r, c):
    """某格所属连通分量的**根 id**（不在任何行程里 ⇒ None，即该格被挡）。"""
    for (s, e, cid) in rows_runs[r]:
        if s <= c < e:
            return find(cid)
    return None


def run_world_mode(a, argv):
    """`--world` 路径：连通性 + 最小通道宽度表。"""
    world = os.path.abspath(a.world)
    if not os.path.exists(world):
        print('!! 找不到世界文件 %s ⇒ 退出码 1' % world, file=sys.stderr)
        return 1
    rects = load_world_rects(world)
    if not rects:
        print('!! 世界里没解析到"高出地面的箱体" ⇒ 解析或世界不对 ⇒ 退出码 1', file=sys.stderr)
        return 1

    # 场地半边长：默认按**最外侧障碍**自动定，也可显式传。
    # ★ 余量必须 **<= clearance**（不是随手 +0.5）：留 0.5 会在封边墙**外面**留出一圈
    #   "离墙 >= clearance"的自由格 ⇒ 泛洪立刻报出第二个连通分量（实测 57805 格 ≈ 144 m²，
    #   正好是墙外那圈），把"场地封闭"这条判据变成假阴性 ✗。取 `+ clearance` 则最外一圈
    #   格的格心仍在 `外表面 + clearance` 之内 ⇒ 墙外一格自由格都不剩 ✓。
    auto_half = max(max(abs(r['x0']), abs(r['x1']), abs(r['y0']), abs(r['y1']))
                    for r in rects) + a.clearance
    field_half = a.field_half if a.field_half is not None else auto_half
    cell = a.cell
    n = int(2 * field_half / cell)

    free = build_free(n, field_half, cell, rects, a.clearance)
    free_total = free.count(1)
    ncomp, sizes, rows_runs, parent, find = free_components(free, n)   # 含"场地外面"那块

    # ---- 起点：默认取**全域净空最大**的自由格（最开阔处），也可 --start 指定 ----
    if a.start:
        start_pt = a.start
        rc = cell_of(start_pt, field_half, cell, n)
        if rc is None:
            print('!! --start 落在场地之外 ⇒ 退出码 1', file=sys.stderr)
            return 1
        start_row, start_col = rc
        if not free[start_row * n + start_col]:
            print('!! --start %s 落在**被挡**格里（离障碍 < %.2f m）⇒ 退出码 1'
                  % (start_pt, a.clearance), file=sys.stderr)
            return 1
    else:
        best, best_d = None, -1.0
        for row in range(0, n, 2):
            base = row * n
            for (s, e, _cid) in rows_runs[row]:
                col = (s + e) // 2
                x = -field_half + (col + 0.5) * cell
                y = -field_half + (row + 0.5) * cell
                d = min(rect_dist(x, y, r) for r in rects)
                if d > best_d:
                    best_d, best = d, (row, col)
        start_row, start_col = best
        start_pt = [-field_half + (start_col + 0.5) * cell,
                    -field_half + (start_row + 0.5) * cell]
    start_root = comp_at(rows_runs, parent, find, start_row, start_col)

    # ★ 剔除"场地外面"（与外框连通的自由块，见 border_roots 的说明）
    outside = border_roots(free, n, rows_runs, parent, find)
    inside_sizes = component_sizes(rows_runs, parent, find, skip_roots=outside)
    inside_cells = sum(inside_sizes.values())
    outside_cells = free_total - inside_cells
    if start_root in outside:
        print('!! 起点落在**场地外面**（与外框连通的那一块）⇒ 这个世界没有把可通行区封起来 ✗',
              file=sys.stderr)
        return 1
    reached = inside_sizes.get(start_root, 0)
    inside_components = len(inside_sizes)

    # ---- 探针：默认**自动铺满全域**（N x N 块，每块取块内净空最大的自由格）----
    probes = {}
    for (px, py) in (a.probe or []):
        probes['manual_%.1f_%.1f' % (px, py)] = (px, py)
    if a.probe_grid > 0:
        blocks = a.probe_grid
        for br in range(blocks):
            for bc in range(blocks):
                r0 = int(br * n / blocks)
                r1 = int((br + 1) * n / blocks)
                c0 = int(bc * n / blocks)
                c1 = int((bc + 1) * n / blocks)
                best, best_d = None, -1.0
                for row in range(r0, r1):
                    for (s, e, _cid) in rows_runs[row]:
                        # ★ 取"这一行行程"与"这一块列范围"的**交集**再采样。
                        #   第一版写成 `max(s, c0)` / `min(e-1, c1-1)` ⇒ 当行程整段在块的
                        #   左边时 `max(s,c0)` 会**越出行程右端**、采到一个**被挡的格子**
                        #   （实测：块 r4c2 采到 (1000,500)，而该行行程是 (19,383) ⇒ 采到墙里）
                        #   ⇒ 探针点落在楼里、判"不可达" ⇒ 假阴性 ✗
                        lo = max(s, c0)
                        hi = min(e, c1)
                        if lo >= hi:
                            continue
                        for col in (lo, hi - 1):
                            x = -field_half + (col + 0.5) * cell
                            y = -field_half + (row + 0.5) * cell
                            d = min(rect_dist(x, y, r) for r in rects)
                            if d > best_d:
                                best_d, best = d, (row, col)
                # 整块都在障碍里（连一个自由格都没有）⇒ 这一块不产生探针
                if best is None or best_d <= 0.0:
                    continue
                row, col = best
                # 该块内自由格的**代表点**：块中心附近最近的自由格（更贴"这一块有人住吗"）
                cr = min(max((r0 + r1) // 2, 0), n - 1)
                cc = min(max((c0 + c1) // 2, 0), n - 1)
                if cr * n + cc < len(free) and free[cr * n + cc]:
                    row, col = cr, cc
                x = -field_half + (col + 0.5) * cell
                y = -field_half + (row + 0.5) * cell
                key = 'grid_r%d_c%d' % (br, bc)
                probes[key] = (x, y)

    probe_rows = {}
    for name, (x, y) in sorted(probes.items()):
        rc = cell_of((x, y), field_half, cell, n)
        if rc is None:
            probe_rows[name] = {'point': [round(x, 3), round(y, 3)], 'free': None,
                                'reachable': False, 'note': '场地之外'}
            continue
        row, col = rc
        idx = row * n + col
        root = comp_at(rows_runs, parent, find, row, col)
        probe_rows[name] = {
            'point': [round(x, 3), round(y, 3)],
            'cell_center': [round(-field_half + (col + 0.5) * cell, 3),
                            round(-field_half + (row + 0.5) * cell, 3)],
            'clearance_m': round(min(rect_dist(x, y, r) for r in rects), 3),
            'free': bool(free[idx]),
            'reachable': bool(root is not None and root == start_root),
        }
    all_probes_ok = all(v['reachable'] for v in probe_rows.values())
    detached = inside_cells - reached          # 域内被切出去的格子（"外面"那块不算）

    # ---- 最小通道宽度表 ①：逐对障碍的缝 ----
    pairs = []
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            g = rect_gap(rects[i], rects[j])
            if g <= 1e-6:
                g = 0.0
            pairs.append({'a': rects[i]['name'], 'b': rects[j]['name'], 'gap_m': round(g, 4)})
    touching = [p for p in pairs if p['gap_m'] == 0.0]
    positive = sorted([p for p in pairs if p['gap_m'] > 0.0], key=lambda p: p['gap_m'])
    illegal = [p for p in positive if p['gap_m'] < a.min_gap - 1e-6]
    min_positive = positive[0]['gap_m'] if positive else None

    # ---- 最小通道宽度表 ②：侵蚀阶梯（自由区按"离障碍 >= w"逐级收窄）----
    ladder = []
    for w in a.width_ladder:
        fw = build_free(n, field_half, cell, rects, w)
        cnt = fw.count(1)
        if cnt == 0:
            ladder.append({'w_m': w, 'free_cells': 0, 'inside_free_cells': 0,
                           'components': 0, 'largest_component_cells': 0,
                           'largest_component_ratio': 0.0, 'passable': False,
                           'start_free': False})
            continue
        _k, _sz, rr, pp, ff = free_components(fw, n)
        out_w = border_roots(fw, n, rr, pp, ff)
        ins = component_sizes(rr, pp, ff, skip_roots=out_w)
        sfree = bool(fw[start_row * n + start_col])
        largest = max(ins.values()) if ins else 0
        # ★ 把"**在哪裂开**"也报出来（首个裂开的那一级）：否则"约 6 m"无从复核。
        #   做法：找**不含起点**的那些块里最大的一个，报它的包围盒。
        bbox = None
        if largest and largest < 0.99 * sum(ins.values()):
            start_root_w = comp_at(rr, pp, ff, start_row, start_col)
            b = [n, -1, n, -1, 0]
            for row in range(n):
                for (_s, _e, cid) in rr[row]:
                    rt = ff(cid)
                    if rt in out_w or rt == start_root_w:
                        continue
                    b[0] = min(b[0], _s); b[1] = max(b[1], _e - 1)
                    b[2] = min(b[2], row); b[3] = max(b[3], row)
                    b[4] += _e - _s
            if b[4]:
                bbox = {'x': [round(-field_half + b[0] * cell, 2),
                              round(-field_half + (b[1] + 1) * cell, 2)],
                        'y': [round(-field_half + b[2] * cell, 2),
                              round(-field_half + (b[3] + 1) * cell, 2)],
                        'cells': b[4]}
        ladder.append({'w_m': w, 'free_cells': cnt,
                       'inside_free_cells': sum(ins.values()),
                       'components': len(ins),
                       'largest_component_cells': largest,
                       'start_free': sfree,
                       'detached_bbox': bbox})
    # ★ 判"这一级还连成一片"用**最大分量占比**，不用"分量数 == 1"：
    #   障碍周围天然会有小口袋（车与楼之间的凹角等），它们本来就是独立小区域，
    #   用 "分量数 == 1" 会把它们误判成"裂开了"（实测 w=0.5 时有 9 块，最大那块占 99.3%）。
    #   真正要问的是"**全域是不是还在一块里**" ⇒ `最大分量 / 自由格 >= 0.99` ✓
    def _passable(row):
        return (row['start_free'] and row['free_cells'] > 0
                and row['largest_component_cells'] >= 0.99 * row['free_cells'])
    for row in ladder:
        row['largest_component_ratio'] = (round(row['largest_component_cells']
                                                / row['free_cells'], 4)
                                          if row['free_cells'] else 0.0)
        row['passable'] = bool(_passable(row))
    still_ok = [row['w_m'] for row in ladder if row['passable']]
    min_channel_m = (2.0 * max(still_ok)) if still_ok else None
    ladder_max_w = max(a.width_ladder) if a.width_ladder else None

    result = {
        'mode': 'world',
        'world': world,
        'world_sha256': __import__('hashlib').sha256(open(world, 'rb').read()).hexdigest(),
        'cell_m': cell,
        'clearance_m': a.clearance,
        'field_half_m': field_half,
        'field_half_source': ('显式 --field-half' if a.field_half is not None
                              else '自动：最外侧障碍 %.3f + clearance %.2f'
                                   % (auto_half - a.clearance, a.clearance)),
        'min_gap_m': a.min_gap,
        'obstacles': [{'name': r['name'], 'kind': r.get('kind', ''),
                       'x': [round(r['x0'], 3), round(r['x1'], 3)],
                       'y': [round(r['y0'], 3), round(r['y1'], 3)],
                       'top': r.get('top')} for r in rects],
        'free_cells': free_total,
        'outside_free_cells': outside_cells,
        'outside_components': len(outside),
        'inside_free_cells': inside_cells,
        'components': inside_components,
        'component_sizes': sorted(inside_sizes.values(), reverse=True)[:20],
        'start_point': [round(start_pt[0], 3), round(start_pt[1], 3)],
        'reached_from_start': reached,
        'detached_free_cells': detached,
        'single_component': inside_components == 1,
        'probes': probe_rows,
        'all_probes_reachable': all_probes_ok,
        'passage_table': {
            'pairs_total': len(pairs),
            'touching_pairs': len(touching),
            'positive_pairs': len(positive),
            'smallest_positive_gap_m': min_positive,
            'illegal_band': [p for p in illegal],
            'illegal_count': len(illegal),
            'ascending_positive_gaps': positive[:15],
            'rule': '缝要么 0（贴合）、要么 >= %.2f m；落在 (0, %.2f) 一律算违规'
                    % (a.min_gap, a.min_gap),
        },
        'erosion_ladder': ladder,
        'min_channel_width_m': min_channel_m,
        'ladder_max_w_m': ladder_max_w,
        # ★ 两个数**不是一回事**，别混着读：
        #   · `passage_table.smallest_positive_gap_m` = 障碍之间**最窄的那条缝**
        #     （车/楼/绿地两两之间，含"没人要求你走"的地方）；
        #   · `min_channel_width_m` = **必经通道**的宽度（不走就过不去的那种），
        #     由侵蚀阶梯给出，受阶梯步长限制 ⇒ 是**下界**。
        'channel_note': ('自由区侵蚀到阶梯最大一级 w=%.2f m 仍连成一片 ⇒ **全域没有窄于 %.2f m 的必经通道**'
                         '（下界；阶梯只到这一级）'
                         % (ladder_max_w, 2.0 * ladder_max_w)) if (
                             still_ok and max(still_ok) == ladder_max_w) else
                        ('侵蚀到 w=%.2f m 起自由区不再连成一片 ⇒ **必经通道最窄处约 %.2f m**'
                         '（下界：阶梯步长 %.2f m 决定的粒度；`erosion_ladder` 里首个裂开那一级的 '
                         '`detached_bbox` 指出**在哪裂开**，可据此复核）'
                         % (min_channel_m / 2.0, min_channel_m,
                            (min(a.width_ladder) if a.width_ladder else 0.0))
                         if min_channel_m else '侵蚀到阶梯最小一级就已裂开 / 起点被吞'),
    }
    ok = (result['single_component'] and all_probes_ok
          and result['passage_table']['illegal_count'] == 0
          and (min_channel_m is None or min_channel_m >= a.min_gap))
    result['verdict'] = '通过' if ok else '不通过'
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        print('已写入 %s' % a.json, file=sys.stderr)
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--clearance', type=float, default=0.25,
                    help='离障碍表面多近算"过不去"（默认 0.25 = nav2 inflation_radius）')
    ap.add_argument('--cell', type=float, default=0.05, help='格子边长（默认 0.05 m）')
    ap.add_argument('--json', default=None, help='把结果写到这个 json')
    ap.add_argument('--field-half', type=float, default=FIELD_HALF,
                    help='**场地半边长**（默认取上面那个跟世界同步的常量 %s）。'
                         '板尺寸做对照实验时必须显式传，否则连通性会被做在一块假想的方块里'
                         '——这个坑 2026-09-21 已经踩过一次。' % FIELD_HALF)
    # ---- ★ 阶段八新增（`--world` 模式；不给 --world 时下面这些一概不参与）----
    ap.add_argument('--world', default=None,
                    help='★ SDF 路径 ⇒ 走**阶段八的 `--world` 模式**：几何从 SDF 现解析、'
                         '给连通性 + 最小通道宽度表。**不给就是原来的庭院场地路径，行为一字不动。**')
    ap.add_argument('--start', default=None,
                    help='--world 模式下的起点（X,Y）。不给则自动取全域净空最大的自由格。')
    ap.add_argument('--probe', action='append', default=[],
                    help='--world 模式下额外的手工探针（X,Y，可重复）')
    ap.add_argument('--probe-grid', type=int, default=8,
                    help='--world 模式下自动铺 N x N 块探针（每块取块内净空最大的自由格；'
                         '默认 8 ⇒ 64 个点铺满全域；0 = 不铺）。切块中心若是自由格则直接用中心。')
    ap.add_argument('--min-gap', type=float, default=1.5,
                    help='--world 模式的通道缝下限（默认 1.5 m，brief §1.4）')
    ap.add_argument('--width-ladder', default='0.25,0.5,0.75,1.0,1.25,1.5,2.0,2.5,3.0,3.5,4.0,5.0',
                    help='--world 模式的侵蚀阶梯（米，逗号分隔）')
    args = ap.parse_args(argv)

    if args.world:
        args.start = ([float(v) for v in args.start.split(',')[:2]]
                      if args.start else None)
        args.probe = [[float(v) for v in s.split(',')[:2]] for s in args.probe]
        args.width_ladder = [float(v) for v in args.width_ladder.split(',') if v.strip()]
        # ★ `--field-half` 在默认路径上有个**默认值**，在 world 模式下"没显式给"与
        #   "给了 FIELD_HALF"分不开 ⇒ 这里按"是否等于那个默认常量"判定，并在结果里注明来源。
        if abs(args.field_half - FIELD_HALF) < 1e-12:
            args.field_half = None
        return run_world_mode(args, argv)

    field_half = args.field_half
    n = int(2 * field_half / args.cell)
    free = [[False] * n for _ in range(n)]
    for r in range(n):
        y = -field_half + (r + 0.5) * args.cell
        for c in range(n):
            x = -field_half + (c + 0.5) * args.cell
            free[r][c] = not blocked(x, y, args.clearance)

    def cell_of(pt):
        c = int((pt[0] + field_half) / args.cell)
        r = int((pt[1] + field_half) / args.cell)
        if not (0 <= r < n and 0 <= c < n):
            return None
        return (r, c)

    # 泛洪（4 邻域；对角穿越两格夹缝不算通路，这里刻意保守）
    start = cell_of(PROBES['start'])
    seen = [[False] * n for _ in range(n)]
    stack = []
    if start is not None and free[start[0]][start[1]]:
        stack.append(start)
        seen[start[0]][start[1]] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < n and 0 <= nc < n and free[nr][nc] and not seen[nr][nc]:
                seen[nr][nc] = True
                stack.append((nr, nc))

    free_total = sum(1 for r in range(n) for c in range(n) if free[r][c])
    reached = sum(1 for r in range(n) for c in range(n) if seen[r][c])
    # 除出生点所在的这一块之外，还剩下多少自由格 -> 就是"被切出去"的部分
    detached = free_total - reached

    probes = {}
    for name, pt in PROBES.items():
        rc = cell_of(pt)
        if rc is None:
            probes[name] = {'point': pt, 'free': None, 'reachable': False,
                            'note': '点在场地之外'}
            continue
        r, c = rc
        x = -field_half + (c + 0.5) * args.cell
        y = -field_half + (r + 0.5) * args.cell
        probes[name] = {
            'point': pt,
            'cell_center': [round(x, 3), round(y, 3)],
            'clearance_m': round(G.nearest_surface(x, y), 3),
            'free': bool(free[r][c]),
            'reachable': bool(seen[r][c]),
        }

    # 逐段独立墙：它两侧各取一个探针（上面 PROBES 里已列），另外单独报"每段墙周围
    # 有没有被围死"——做法是把这段墙自身的矩形区域挖掉后，看周边自由格是否都在同一块里，
    # 这里用更直接的判据：每段墙的四个面外侧各 0.4 m 取点，全部要可达且自由。
    around = {}
    for seg in G.SEGMENTS:
        name = seg[0]
        xmin, xmax, ymin, ymax = G.segment_rect(seg)
        d = 0.4
        pts = {
            'north': ((xmin + xmax) / 2, ymax + d),
            'south': ((xmin + xmax) / 2, ymin - d),
            'east': (xmax + d, (ymin + ymax) / 2),
            'west': (xmin - d, (ymin + ymax) / 2),
        }
        rows = {}
        for side, pt in pts.items():
            rc = cell_of(pt)
            if rc is None:
                rows[side] = {'point': pt, 'reachable': False, 'note': '场地之外'}
                continue
            r, c = rc
            rows[side] = {'point': [round(pt[0], 2), round(pt[1], 2)],
                          'free': bool(free[r][c]),
                          'reachable': bool(seen[r][c])}
        around[name] = rows

    all_probes_ok = all(v['reachable'] for v in probes.values())
    all_around_ok = all(v.get('reachable') for rows in around.values()
                        for v in rows.values())
    result = {
        'cell_m': args.cell,
        'clearance_m': args.clearance,
        'field_half_m': field_half,
        'free_cells': free_total,
        'reached_from_start': reached,
        'detached_free_cells': detached,
        'single_component': detached == 0,
        'probes': probes,
        'all_probes_reachable': all_probes_ok,
        'segment_surroundings': around,
        'all_segments_approachable': all_around_ok,
        'geometry': {
            'segments': [{'name': s[0], 'center': [s[1], s[2]], 'orient': s[3],
                          'length': s[4]} for s in G.SEGMENTS],
            'pillars': G.PILLARS,
            'note': '外墙（wall_north/south/east/west）已删除，不在这份几何里',
        },
        'verdict': ('通过' if (detached == 0 and all_probes_ok and all_around_ok)
                    else '不通过'),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        print('已写入 %s' % args.json, file=sys.stderr)
    return 0 if result['verdict'] == '通过' else 1


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""**已知场地几何**：核对代价地图与路径用的公共模块（不是可执行脚本）。

几何常量与 `worlds/rtk_courtyard.sdf` 的头注释、`make_world.py` 的常量一致
（第二阶段就核对过，第三阶段原样沿用）：

  **2026-09-21 按 AuYang 要求去掉四周围墙**：原来那圈外墙（中心线 ±5.0、厚 0.15、
  内表面 ±4.925）已从生成器与本模块一起删除——代价地图上那圈位置不应再出现致命格。
  本模块保留的**真实障碍**：

    中央方块 2.5 x 2.5（|x|,|y| <= 1.25），高 1.0
    隔墙 p1: y=+3.9, |x|<=2.0, 厚 0.15 起算面 ±0.075
    隔墙 p2: x=-3.9, |y|<=2.0
    立柱 0.4 见方，四角 (4.5,4.5) (4.5,-4.5) (-4.5,4.5)
    **新增的 4 段"非封闭"独立墙**：见下面 SEGMENTS（与生成器 OBSTACLES 一一对应）

提供三件事：
  - `nearest_surface(x, y)`：点到最近真实障碍表面的距离（用于"占据格位置对不对"）；
  - `surface_samples(step)`：在**真实障碍表面**上按步长采样（用于"墙面上那些格子是不是占据"）；
  - `segment_hits_obstacle(p0, p1)`：线段是否穿过任何障碍（用于"路径不穿墙"）。
"""

import math

BLOCK_HALF = 1.25            # 中央方块半边长
PARTITION_OFFSET = 3.9       # p1: y=+, p2: x=-
PARTITION_HALF = 2.0
PARTITION_THICK = 0.15
# **只有三根立柱**（(4.5,-4.5) 那个角没有）。这个数字核对过两处：
# `worlds/rtk_courtyard.sdf` 里的 pillar_* 模型，以及 `make_world.py` 的 PILLARS 字典。
# 多加一根会让"路径穿墙核对"把不存在的障碍报成穿墙、几何核对也把不存在的表面算进去。
PILLARS = [(4.5, 4.5), (4.5, -4.5), (-4.5, 4.5)]
PILLAR_SIDE = 0.4

# **新增的 4 段独立墙**（与 make_world.py 的 OBSTACLES 一一对应，改一处必须改另一处）。
# 格式：(名字, 中心 x, 中心 y, 走向, 长度)；'h' = 沿 x 轴，'v' = 沿 y 轴。
# 它们**不是闭环的一部分**：每段两端都在空地上，不与中央方块/隔墙/立柱相接，
# 所以场地仍是连通的（connectivity_check.py 逐格泛洪核对过）。
SEGMENTS = [
    ('obstacle_00', 4.0, -2.3, 'v', 1.4),
    ('obstacle_01', 3.6, 2.8, 'v', 1.2),
    ('obstacle_02', -0.3, -4.0, 'h', 2.2),
    ('obstacle_03', -4.4, -3.9, 'v', 1.6),
]
SEGMENT_THICK = 0.15

# 路线方环（与 config/route.yaml 的 route 参数一致）。用于"近处墙不许被擦掉"的回归判据：
# 距路线一定范围内的墙，必须是"被观测到且标成占据"，不能是大片空闲。
ROUTE_SQUARE = 2.6


def segment_rect(seg, margin=0.0):
    """把一段独立墙变成轴对齐矩形 (xmin, xmax, ymin, ymax)。"""
    _, cx, cy, orient, length = seg
    half = length / 2 + margin
    thick = SEGMENT_THICK / 2 + margin
    if orient == 'h':
        return (cx - half, cx + half, cy - thick, cy + thick)
    return (cx - thick, cx + thick, cy - half, cy + half)


def distance_to_segment(x, y, seg):
    """点到一段独立墙表面的距离（矩形外距离）。"""
    xmin, xmax, ymin, ymax = segment_rect(seg)
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


def distance_to_block(x, y):
    dx = max(abs(x) - BLOCK_HALF, 0.0)
    dy = max(abs(y) - BLOCK_HALF, 0.0)
    return math.hypot(dx, dy)


def distance_to_partitions(x, y):
    # p1: 沿 x 从 -2 到 2，落在 y=+3.9（厚 0.15 -> 表面 ±0.075）
    dx = max(abs(x) - PARTITION_HALF, 0.0)
    dy = max(abs(y - PARTITION_OFFSET) - PARTITION_THICK / 2, 0.0)
    best = math.hypot(dx, dy)
    # p2: 沿 y 从 -2 到 2，落在 x=-3.9
    dx2 = max(abs(x + PARTITION_OFFSET) - PARTITION_THICK / 2, 0.0)
    dy2 = max(abs(y) - PARTITION_HALF, 0.0)
    return min(best, math.hypot(dx2, dy2))


def distance_to_pillars(x, y):
    return min(math.hypot(max(abs(x - px) - PILLAR_SIDE / 2, 0.0),
                          max(abs(y - py) - PILLAR_SIDE / 2, 0.0))
               for px, py in PILLARS)


def nearest_surface(x, y):
    """到最近真实障碍表面的距离。**外墙已删除，不再计入**（2026-09-21）。"""
    return min(distance_to_block(x, y),
               distance_to_partitions(x, y),
               distance_to_pillars(x, y),
               min(distance_to_segment(x, y, seg) for seg in SEGMENTS))


# ------------------------------------------------------------------ 表面采样
def _lin(a, b, step):
    n = max(1, int(math.ceil(abs(b - a) / step)))
    return [a + (b - a) * i / n for i in range(n + 1)]


def surface_samples(step=0.05):
    """在每个障碍的**表面上**按 step 采样，返回 [(x, y, 名称)]。

    "表面"是真实几何的边界（方块的四个面、隔墙的两个面、立柱的四边、
    独立墙段的四面），不是障碍体内部的点——因为占据格是由"看到表面"产生的，
    拿内部点去核对会把"看到表面"错判成"没看到"。

    **外墙（wall_north/south/east/west）已按 AuYang 要求于 2026-09-21 删除**，
    这里不再采样，代价地图上那圈位置也就不该出现致命格。
    """
    out = []
    b = BLOCK_HALF                       # 中央方块
    for t in _lin(-b, b, step):
        out.append((t, b, 'block'))
        out.append((t, -b, 'block'))
        out.append((b, t, 'block'))
        out.append((-b, t, 'block'))

    p = PARTITION_OFFSET                 # 隔墙 p1（y=+3.9）/ p2（x=-3.9）
    half = PARTITION_THICK / 2
    for x in _lin(-PARTITION_HALF, PARTITION_HALF, step):
        out.append((x, p - half, 'partition_p1'))
        out.append((x, p + half, 'partition_p1'))
    for y in _lin(-PARTITION_HALF, PARTITION_HALF, step):
        out.append((-p - half, y, 'partition_p2'))
        out.append((-p + half, y, 'partition_p2'))

    h = PILLAR_SIDE / 2                  # 立柱
    for px, py in PILLARS:
        for t in _lin(-h, h, step):
            out.append((px + t, py + h, 'pillar'))
            out.append((px + t, py - h, 'pillar'))
            out.append((px + h, py + t, 'pillar'))
            out.append((px - h, py + t, 'pillar'))

    for seg in SEGMENTS:                 # 新增的独立墙段（四面）
        name = seg[0]
        xmin, xmax, ymin, ymax = segment_rect(seg)
        for x in _lin(xmin, xmax, step):
            out.append((x, ymin, name))
            out.append((x, ymax, name))
        for y in _lin(ymin, ymax, step):
            out.append((xmin, y, name))
            out.append((xmax, y, name))
    return out


def distance_to_route(x, y):
    """点到路线（方环 ±ROUTE_SQUARE）的最短距离。"""
    s = ROUTE_SQUARE
    d = None
    # 四条边
    for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1)):
        other = 1 - axis
        p = (x, y)
        if abs(p[other]) <= s:
            v = abs(p[axis] - sign * s)
        else:
            v = math.hypot(abs(p[axis] - sign * s), abs(p[other]) - s)
        d = v if d is None else min(d, v)
    return d


# ------------------------------------------------------------------ 线段 vs 障碍
def _segment_hits_rect(p0, p1, xmin, xmax, ymin, ymax):
    """线段是否与轴对齐矩形相交（Liang-Barsky）。"""
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, x0 - xmin), (dx, xmax - x0), (-dy, y0 - ymin), (dy, ymax - y0)):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            if r > t1:
                return False
            t0 = max(t0, r)
        else:
            if r < t0:
                return False
            t1 = min(t1, r)
    return t0 <= t1


def segment_hits_obstacle(p0, p1, margin=0.0):
    """线段是否穿过任何障碍（可选给 margin 把障碍"加厚"）。

    返回命中的障碍名，没命中返回 None。

    **外墙已删除（2026-09-21）**：原来这里前四条是 wall_north/south/east/west 的整条带子。
    现在 `rects` 里只剩真实存在的障碍——中央方块、两道隔墙、三根立柱、四段独立墙。
    """
    rects = []
    b = BLOCK_HALF + margin
    rects.append(('block', -b, b, -b, b))
    p = PARTITION_THICK / 2 + margin
    rects.append(('partition_p1', -PARTITION_HALF - margin, PARTITION_HALF + margin,
                  PARTITION_OFFSET - p, PARTITION_OFFSET + p))
    rects.append(('partition_p2', -PARTITION_OFFSET - p, -PARTITION_OFFSET + p,
                  -PARTITION_HALF - margin, PARTITION_HALF + margin))
    h = PILLAR_SIDE / 2 + margin
    for px, py in PILLARS:
        rects.append(('pillar', px - h, px + h, py - h, py + h))
    for seg in SEGMENTS:
        rects.append((seg[0],) + segment_rect(seg, margin))
    for name, xmin, xmax, ymin, ymax in rects:
        if _segment_hits_rect(p0, p1, xmin, xmax, ymin, ymax):
            return name
    return None

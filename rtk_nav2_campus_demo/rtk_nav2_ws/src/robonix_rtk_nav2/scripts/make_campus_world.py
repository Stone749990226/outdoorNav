#!/usr/bin/env python3
"""阶段八 · 校园级 100 m 场景生成器（**单一真源**：改场景改这里，不手改 SDF）。

依据：`docs/campus-geo-brief.md` §1（场景规格）。产出 `worlds/campus_100.sdf`。

**场地**：可通行区 100 x 100 m（x, y ∈ [-49.5, +49.5]），四周封边墙（厚 0.3、高 2.0，
中心线 ±49.5 ⇒ 内表面 ±49.35），地面 120 x 120 平板**顶面 z = 0**
（用"顶面 ≤ 0 的 box"⇒ 不会被 `stop_margin_check.parse_raised_boxes`（判据 `top > 0`）当成障碍 ✓）。

**障碍三类**（全为静态世界物件，运行时 spawn 的一个都没有 ✓）：
  · **楼**：实心 box，高 ≥ 8 m（本生成器用 12 / 10 m）；
  · **绿地**：高 **0.20 m** 低方块 —— 数值是刻意的：**> 激光扫描面 0.11 m**（激光看得见）、
    **< 相机源 `min_obstacle_height` 0.30 m**（相机看不见）⇒ 与阶段七的 `lowwall_*` 同类属性 ✓；
  · **停车块**（模拟停着的车）：2.0 x 0.9 x 1.2，8~12 个。

**★ 通道宽度硬约束（本生成器自己在生成前断言，不靠肉眼看坐标）**：
任意两个障碍之间（含障碍与封边墙之间）的**最近距离**必须**要么 = 0（完全贴合/相交）、
要么 ≥ `MIN_GAP`（默认 1.5 m）**；落在 `(0, MIN_GAP)` 的**一律拒绝生成**并打印违规清单。
理由：可通行下限 = 2 x (robot_radius 0.105 + inflation_radius 0.25) = **0.71 m** ⇒
留 0.7~1.5 m 的缝会出现"看起来能过、规划过不去"（brief §1.4 点名的唯一来源）。

**★ 楼间那 0.6 m 缝的裁决（brief §1.3 表内自相矛盾 ⇒ reviewer 2026-10-09 裁定"贴合"）**：
按 brief 原坐标 `bld_main` x∈[-30.6, 18.2] 与 `bld_east` x∈[18.8, 29.4] 之间差 **0.6 m** ——
既非贴合、又低于可通行下限 0.71 m，而代价图上缝中心仍留 ~0.1 m 非致命带
（inflation 0.25 从两侧各扣 ⇒ 中心格离两侧墙各 0.30 m > 机器人内切 0.105 m）⇒ **规划器真会去钻**。
裁定的做法是**把缝消掉**（不是把楼移开）：`bld_main` **东边界 18.2 → 18.8**
（宽 48.8 → **49.4 m**，+1.2%，远在 ±20% 内；中心 x 随之 -6.2 → **-5.9**，西边界仍是 -30.6），
`bld_east` **原位一字不动** ⇒ 两楼**完全贴合（缝 = 0）** ✓。
**附带效果（reviewer 明说要的）**：参考图那条环路成为**唯一**的绕行通道，不会凭空多出一条楼间捷径 ✓
⇒ 本生成器**不再**对"主环路 ≥6 m"做断言（那条间距已不存在），改断言 **两楼贴合**。

用法：
  make_campus_world.py [--out worlds/campus_100.sdf] [--min-gap 1.5] [--json 缝表.json] [--check-only]
退出码：0 = 生成（或核验）通过；2 = 通道缝约束被违反；1 = 用法/IO 失败。
"""

import argparse
import hashlib
import json
import math
import os
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

# ---- 场地几何（单位 m）------------------------------------------------------
FIELD_HALF = 49.5          # 可通行区半边长（brief §1.2：x, y ∈ [-49.5, +49.5]）
WALL_T = 0.3               # 封边墙厚
WALL_H = 2.0               # 封边墙高
WALL_EXT = 0.3             # 墙端外伸，让四个角互相咬合（不留 L 形缝）
GROUND_HALF = 60.0         # 地面板半边长（120 x 120，比可通行区各边多铺 10.5 m）
GROUND_T = 0.02

# 通道缝约束：0（贴合）或 ≥ MIN_GAP
MIN_GAP = 1.5

# ★ **出生点**（launch 的 `x_pose` / `y_pose` 必须取这两个数 —— 单一真源）。
#   reviewer 2026-10-09 裁定 (a)：出生点要**由生成器断言**（在自由区、净空 >= SPAWN_MIN_CLEAR），
#   否则换场景时会**静默失效**（出生点落进新楼里，起栈看不出来、规划一路失败）。
#   取在主楼**南侧空地中部**：离 `bld_south` 北面约 6 m，四周开阔，
#   铺"覆盖 ±40 m"的标定路线很自然。
SPAWN = (2.6, -30.0)
SPAWN_MIN_CLEAR = 0.5

# ---- 经纬度基准（**必须**与 config/navsat.yaml 的 datum 逐字一致）-----------
# ★★ 2026-10-09（AuYang 测试需求）：**基准从上海挪到真实站点**
#   「杭州市萧山区永晖路233号 杭州湾智慧谷」lat 30.2276 / lon 120.27（alt 保持 4.0；
#   `navsat_transform` 的 `zero_altitude: true` ⇒ 它不进几何 ✓）。
#   原值 31.2304 / 121.4737（上海）—— 与站点相距约 110 km ⇒ **真实经纬度直接喂进来会落到场景外** ✗。
#   ★ **几何一个字不动** ⇒ 几何指纹应**一字不变** ✓、世界文件哈希**会变** ✓（(b) 机制要的就是这对）。
#   ★ UTM 带号不变（120.27 与 121.47 同属 51 带）⇒ 手推 γ 由 −0.79° 变为 ≈ **−1.38°**；**要重测**。
DATUM_LAT = 30.2276
DATUM_LON = 120.27
DATUM_ALT = 4.0
DATUM_EPS = 1e-9

# ---- 楼（brief §1.3 表；高度取 ≥8 m）---------------------------------------
BLD_H_MAIN = 12.0
BLD_H_EAST = 12.0
BLD_H_SOUTH = 10.0
# ★ 楼间 0.6 m 缝的裁定（reviewer 2026-10-09）：**主楼东边界 18.2 → 18.8**（宽 49.4）、
#   **东侧裙楼原位不动**（中心 x 24.1、宽 10.6 ⇒ 西边界 18.8）⇒ 两楼**完全贴合** ✓。
#   主楼中心 x = (-30.6 + 18.8)/2 = -5.9（西边界仍是 -30.6）。见文件头与 BLD_TOUCH_NOTE。
BLD_MAIN_CX = -5.9
BLD_MAIN_SX = 49.4

# ---- 绿地（高 0.20 m）-------------------------------------------------------
GREEN_H = 0.20

# ---- 停车块 ----------------------------------------------------------------
CAR_SZ = (2.0, 0.9, 1.2)   # 长 x 宽 x 高（长轴默认沿 x；'v' 走向则交换前两位）
CAR_H = CAR_SZ[2]

# 颜色（ambient == diffuse，本场景不投射阴影）
COL_BUILDING = (0.62, 0.60, 0.58)
COL_GREEN = (0.28, 0.52, 0.30)
COL_CAR = (0.72, 0.30, 0.26)
COL_WALL = (0.70, 0.68, 0.62)
COL_GROUND = (0.35, 0.35, 0.38)

# ---- 布局（**唯一的一份坐标**）---------------------------------------------
# 楼： (name, cx, cy, sx, sy, sz)
BUILDINGS = [
    ('bld_main', BLD_MAIN_CX, 7.3, BLD_MAIN_SX, 25.3, BLD_H_MAIN),
    ('bld_east', 24.1, -11.4, 10.6, 45.3, BLD_H_EAST),
    ('bld_south', -6.0, -40.0, 40.0, 8.0, BLD_H_SOUTH),
]

# 绿地： (name, cx, cy, sx, sy) —— 高一律 GREEN_H
#   西南"园区绿地"大块两片（都**贴着封边墙** ⇒ 缝 = 0，天然满足硬约束 ✓）
#   + 主楼北侧广场小片三片（彼此与主楼保持 ≥2.0 m）
GREENS = [
    ('green_sw_1', -43.175, -26.0, 12.35, 12.0),   # 贴西墙
    ('green_sw_2', -44.675, -43.675, 9.35, 11.35),  # 贴西墙 + 南墙（角块）
    ('green_e_1', 43.675, -4.0, 11.35, 12.0),      # 贴东墙
    ('green_plaza_a', -16.0, 25.0, 8.0, 4.0),      # 主楼北广场（西）
    ('green_plaza_b', 0.0, 25.0, 8.0, 4.0),        # 主楼北广场（中）
    ('green_plaza_c', 11.0, 24.0, 6.0, 3.0),       # 主楼北广场（东）
]

# 停车块： (name, cx, cy, orient) —— orient 'h' = 长轴沿 x，'v' = 长轴沿 y
#   ★ 两楼贴合后，原来的"楼间通道"不存在了 ⇒ 车位改停到**楼外立面**上（都是"贴楼"、缝 = 0 ✓）：
#     car_00..02 沿**主楼南立面**（y=-5.35）首尾相接停一排；
#     car_03/04  沿**东侧裙楼东立面**（x=29.4）停放；
#     其余 5 台零散在空地（离任何障碍都远）。
#   ★ 楼间 0.6 m 那条缝已由"主楼东边界 → 18.8"消掉（见 BLD_MAIN_SX）✓
CARS = [
    ('car_00', -26.0, -5.8, 'h'),    # 贴主楼南立面（y 上沿 = -5.35 = 楼面）
    ('car_01', -24.0, -5.8, 'h'),    # 与 car_00 首尾相接（缝 = 0 ✓）
    ('car_02', -22.0, -5.8, 'h'),    # 与 car_01 首尾相接（缝 = 0 ✓）
    ('car_03', 29.85, -8.0, 'v'),    # 贴东侧裙楼东立面（x 左沿 = 29.4 = 楼面）
    ('car_04', 29.85, -14.0, 'v'),
    ('car_05', -10.0, -20.0, 'h'),
    ('car_06', 5.0, -22.0, 'h'),
    ('car_07', 35.0, -20.0, 'h'),
    ('car_08', -15.0, -30.0, 'v'),
    ('car_09', 40.0, -25.0, 'v'),
]


def rect_of(name, cx, cy, sx, sy, kind, top=0.0):
    """矩形 + **顶面高度 `top`**。

    ★ `top` **必须带上**（reviewer 2026-10-09 抓到的缺陷）：第一版没有这个字段，而
    `geometry_fingerprint` 里写的是 `r.get('top', 0.0)` ⇒ **恒为 0.0** ⇒ 高度**根本没进指纹** ✗，
    可 docstring 与交付说明都写着"名字 + 矩形 + **顶面高度**"—— 正是"**注释不是凭据**"的落点。
    为什么高度必须进：它在本套里**有后果** —— 绿地 0.20 m 正卡在相机源 `min_obstacle_height`
    0.30 的边界、车 1.2 → 2.0、楼 12 → 8 **都会改代价图内容**（判据 ⑤ 的 `grids.npz` 会对不上），
    而拿"不含高度的指纹"去校验会**放行** ✗。
    """
    return {'name': name, 'kind': kind, 'cx': cx, 'cy': cy, 'sx': sx, 'sy': sy,
            'top': top,
            'x0': cx - sx / 2.0, 'x1': cx + sx / 2.0,
            'y0': cy - sy / 2.0, 'y1': cy + sy / 2.0}


def all_rects():
    """**障碍**矩形清单（不含地面）：楼 + 绿地 + 停车块 + 四道封边墙。

    封边墙**必须**在里面：brief §1.4 的缝约束是"相邻障碍之间"，而墙是障碍，
    墙与楼之间的缝同样会让车"看起来能过、规划过不去" ✓。
    """
    # ★ 每件都带上**顶面高度**（`_h` 原来被丢掉了 —— 见 rect_of 的说明）
    out = [rect_of(n, cx, cy, sx, sy, 'building', top=h) for (n, cx, cy, sx, sy, h) in BUILDINGS]
    out += [rect_of(n, cx, cy, sx, sy, 'green', top=GREEN_H) for (n, cx, cy, sx, sy) in GREENS]
    for (n, cx, cy, o) in CARS:
        sx, sy = (CAR_SZ[0], CAR_SZ[1]) if o == 'h' else (CAR_SZ[1], CAR_SZ[0])
        out.append(rect_of(n, cx, cy, sx, sy, 'car', top=CAR_H))
    span = 2 * (FIELD_HALF + WALL_EXT)
    out += [
        rect_of('wall_north', 0.0, FIELD_HALF, span, WALL_T, 'wall', top=WALL_H),
        rect_of('wall_south', 0.0, -FIELD_HALF, span, WALL_T, 'wall', top=WALL_H),
        rect_of('wall_east', FIELD_HALF, 0.0, WALL_T, span, 'wall', top=WALL_H),
        rect_of('wall_west', -FIELD_HALF, 0.0, WALL_T, span, 'wall', top=WALL_H),
    ]
    return out


def geometry_fingerprint(rects=None):
    """★ **几何指纹**：障碍清单（名字 + 矩形 + 顶面高度）**排序后**的 sha256。

    为什么要和"世界文件 sha256"并存（reviewer 2026-10-09 裁定 (b)）：
    世界文件里有些东西**不影响几何** —— `<gui><camera>`（默认机位）、`<light>`、`<scene>` 的
    颜色、注释。标定（`geo_calib.json`）与 `geo_route` 的启动校验**要判的是"几何变没变"**，
    拿文件哈希判会把"只改了机位"也判成"标定失效"⇒ 白烧一轮标定、还容易被误读成"环境坏了" ✗。
    ⇒ 两个指纹都记：文件哈希（审计用）+ 几何指纹（**校验用**）。
    """
    if rects is None:
        rects = all_rects()
    key = ';'.join('%s:%.6f,%.6f,%.6f,%.6f,%.6f'
                   % (r['name'], r['x0'], r['x1'], r['y0'], r['y1'],
                      r.get('top', 0.0))
                   for r in sorted(rects, key=lambda q: q['name']))
    return hashlib.sha256(key.encode('utf-8')).hexdigest()


def rect_gap(a, b):
    """两个轴对齐矩形的最近距离；相交/贴合 ⇒ 0.0。"""
    dx = max(a['x0'] - b['x1'], b['x0'] - a['x1'], 0.0)
    dy = max(a['y0'] - b['y1'], b['y0'] - a['y1'], 0.0)
    return math.hypot(dx, dy)


def rect_dist_pt(px, py, r):
    """点到轴对齐矩形的最短距离（点在矩形内 ⇒ 0）。"""
    dx = max(r['x0'] - px, 0.0, px - r['x1'])
    dy = max(r['y0'] - py, 0.0, py - r['y1'])
    return math.hypot(dx, dy)


# ★ 浮点容差：布局表里 `-43.175 - 12.35/2` 这类算式会得到 `-49.35000000000001`，
#   与墙内表面 `-49.5 + 0.3/2` 相差 ~1e-14 ⇒ 不吸附的话"贴合"会被读成"1e-14 的缝"、
#   落进 `(0, min_gap)` 而被判违规（本生成器第一版就是这么自锁的）。
GAP_EPS = 1e-6


OV_EPS = 1e-9          # 重叠面积容差（m^2）—— 见 overlap_area 里的说明


def overlap_area(a, b):
    """两矩形**相交面积**（贴合 ⇒ 0；真重叠 ⇒ > 0）。"""
    dx = min(a['x1'], b['x1']) - max(a['x0'], b['x0'])
    dy = min(a['y1'], b['y1']) - max(a['y0'], b['y0'])
    # ★ 与 `gap_table` 同一把浮点尺子：贴合面上的重叠是 `~1e-15 m^2` 量级的浮点噪声
    #   （`7.3 - 25.3/2` 得到 -5.350000000000001）⇒ 面积 <= OV_EPS 一律当 0 ✓
    area = dx * dy if dx > 0 and dy > 0 else 0.0
    return 0.0 if area <= OV_EPS else area


def overlap_report(rects):
    """★ 除"两道封边墙在四角互相咬合"外，**任何两件障碍都不许真重叠**。

    为什么单独查：`rect_gap` 对"贴合"与"重叠"都返回 0 ⇒ 光看缝表**分不出**"贴楼停放"
    与"车埋进楼里"（前者要的、后者是灾难）。重叠面积 > 0 一律报出来。
    """
    bad = []
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            a, b = rects[i], rects[j]
            area = overlap_area(a, b)
            if area <= 0:
                continue
            if a['kind'] == 'wall' and b['kind'] == 'wall':
                continue            # 封边墙四角**故意**外伸咬合（WALL_EXT），不算重叠
            bad.append({'a': a['name'], 'b': b['name'], 'overlap_m2': round(area, 4)})
    return bad


def gap_table(rects, min_gap):
    """逐对求缝 ⇒ (全部对, 违规对)。违规 = 缝落在 (0, min_gap) 开区间。

    缝 <= `GAP_EPS` 吸附成 **0.0**（= 贴合）✓；判定边界用 `GAP_EPS` 也让"数值上刚好 1.5"
    不会被浮点噪声判成违规。
    """
    pairs, bad = [], []
    for i in range(len(rects)):
        for j in range(i + 1, len(rects)):
            d = rect_gap(rects[i], rects[j])
            if d <= GAP_EPS:
                d = 0.0
            rec = {'a': rects[i]['name'], 'b': rects[j]['name'], 'gap_m': round(d, 4),
                   'verdict': '贴合' if d == 0.0 else ('达标' if d >= min_gap - GAP_EPS else '违规')}
            pairs.append(rec)
            if 0.0 < d < min_gap - GAP_EPS:
                bad.append(rec)
    return pairs, bad


# ---- SDF 生成 ---------------------------------------------------------------
def _model(name, cx, cy, sx, sy, sz, color, pz=None):
    """`pz` = 模型位姿的 z。默认 `sz/2`（箱体**坐在** z=0 地面上）；
    **地面自己例外**（`pz=-GROUND_T/2` ⇒ 顶面恰在 z=0 ⇒ 不被"顶面 z>0"的判据当成障碍）。"""
    if pz is None:
        pz = sz / 2.0
    amb = ' '.join('%.3f' % c for c in color)
    return (
        '    <model name="%s">\n'
        '      <static>true</static>\n'
        '      <pose>%s %s %s 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        '          <geometry><box><size>%s %s %s</size></box></geometry>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry><box><size>%s %s %s</size></box></geometry>\n'
        '          <material>\n'
        '            <ambient>%s 1</ambient>\n'
        '            <diffuse>%s 1</diffuse>\n'
        '          </material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
        % (name, _f(cx), _f(cy), _f(pz), _f(sx), _f(sy), _f(sz),
           _f(sx), _f(sy), _f(sz), amb, amb))


def _f(v):
    """坐标打印：去掉多余的 .0，保证 SDF 里读起来的数与布局表逐字一致。"""
    s = '%.6f' % v
    s = s.rstrip('0').rstrip('.')
    return s if s else '0'


HEADER = '''<?xml version="1.0" ?>
<!--
  阶段八 · 校园级 100 m 仿真场景（Gazebo Sim / Fortress，SDF 1.9）

  **由 scripts/make_campus_world.py 生成，不要手改**（单一真源：改场景改生成器）。
  依据 docs/campus-geo-brief.md §1。

  可通行区 100 x 100 m（x, y ∈ [-49.5, +49.5]）；四周封边墙厚 0.3、高 2.0，
  中心线 ±49.5（内表面 ±49.35）；地面 120 x 120 平板**顶面 z = 0**
  （顶面 <= 0 的 box ⇒ 不会被"顶面 z > 0"这类判据当成障碍）。

  障碍三类（全静态）：
    楼     : 实心 box，高 12 / 10 m（brief 要求 >= 8 m）
    绿地   : 高 0.20 m 低方块 —— > 激光扫描面 0.11 m（激光看得见）、
             < 相机源 min_obstacle_height 0.30 m（相机看不见），与阶段七 lowwall_* 同类
    停车块 : 2.0 x 0.9 x 1.2（模拟停着的车），共 @@NCARS@@ 个

  ★ 通道宽约束：任意两障碍之间的最近距离**要么 0（贴合）、要么 >= @@MINGAP@@ m**。
    生成时逐对断言过（见生成器 gap_table，缝表可落 json）。

  ★ 两楼**完全贴合**（缝 = 0）：brief §1.3 原坐标下 bld_main 东边界 18.2 与 bld_east
    西边界 18.8 之间差 0.6 m —— 既非贴合、又低于可通行下限 0.71 m，而代价图上缝中心
    仍留 ~0.1 m 非致命带 ⇒ 规划器会去钻。做法（reviewer 2026-10-09 裁定）是**把缝消掉**：
    bld_main **东边界 18.2 -> 18.8**（宽 48.8 -> 49.4 m，+1.2%；中心 x -6.2 -> -5.9，
    西边界仍 -30.6），**bld_east 原位一字不动** ⇒ 参考图的环路成为唯一绕行通道。

  模拟 RTK 的基准：世界原点 (0,0) 对应纬度 @@LAT@@、经度 @@LON@@、高程 @@ALT@@ m。
  navsat_transform 的 datum 必须与这三个数字一致（起栈断言会核）。
-->
<sdf version="1.9">
  <world name="campus_100">

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>@@LAT@@</latitude_deg>
      <longitude_deg>@@LON@@</longitude_deg>
      <elevation>@@ALT@@</elevation>
    </spherical_coordinates>

    <!-- type=ignored：用引擎默认物理（Gazebo Sim 里没有 ODE，
         写 type="ode" 会让整份 SDF 解析失败 "Unable to read file"） -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <!-- 没有这个插件，相机一个都不会出数据 -->
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- IMU 与 GPS（navsat）各自需要自己的系统插件：它们不属于 Sensors 系统 -->
    <plugin filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-navsat-system"
            name="ignition::gazebo::systems::NavSat"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>

    <!-- ★★ 2026-10-09（阶段八）**默认相机 = 俯瞰全场**（reviewer 要求）★★
         为什么必须写：世界原点 (0,0) **落在 `bld_main` 里面** ⇒ Gazebo 默认相机在原点附近、
         等于**在主楼内部**拍 ⇒ 起栈后画面只有一块楼内地板和地面网格，
         既看不全校园、也看不出车在哪（实测两张"起栈"与"到达"的截图**画面几乎一样**，
         只有 fps 不同 ⇒ 那样的图**不构成证据**）✗。
         取 `z=140 m`、`pitch=90°`（正俯视）⇒ 100x100 m 一次看全、车是个可见的点 ✓。
         ★ 这是 **GUI 块**、不含任何几何 ⇒ 世界的物理与判据一字不变 ✓
           （但**会改世界文件哈希** —— 已单列在交付说明里，请 reviewer 按"只加 GUI 块"核。） -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 0 140 0 1.5708 0</pose>
      </camera>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.3 -1.0</direction>
    </light>

'''


def render_sdf():
    span = 2 * (FIELD_HALF + WALL_EXT)
    # ★ 用 replace 而不是 %-format：HEADER 里有字面量 `+1.2%`（讲楼宽变化），
    #   走 %-format 会被当成转换符 ⇒ `TypeError: not enough arguments`（本生成器踩过）✓
    head = (HEADER.replace('@@NCARS@@', str(len(CARS)))
                  .replace('@@MINGAP@@', '%.1f' % MIN_GAP)
                  .replace('@@LAT@@', str(DATUM_LAT))
                  .replace('@@LON@@', str(DATUM_LON))
                  .replace('@@ALT@@', str(DATUM_ALT)))
    body = []
    # ★ 地面**必须**是"顶面 <= 0"的 box：`parse_raised_boxes`（以及 `static_walls.py`）
    #   的判据是"顶面 z > 0" ⇒ 若是 `pz=+sz/2`（顶面 0.02），**地面会被当成一整块障碍** ✗
    #   （本生成器第一版就是 `_model` 的默认 pz 干的 ⇒ 全域自由格只剩 48312/5856400 ✗）。
    #   这里显式给 `pz=-GROUND_T/2` ⇒ 顶面恰 z=0 ✓。生成后还会再断言一次（见 main）。
    body.append(_model('ground', 0.0, 0.0, 2 * GROUND_HALF, 2 * GROUND_HALF, GROUND_T,
                       COL_GROUND, pz=-GROUND_T / 2.0))
    for (n, cx, cy, sx, sy, sz) in BUILDINGS:
        body.append(_model(n, cx, cy, sx, sy, sz, COL_BUILDING))
    for (n, cx, cy, sx, sy) in GREENS:
        body.append(_model(n, cx, cy, sx, sy, GREEN_H, COL_GREEN))
    for (n, cx, cy, o) in CARS:
        sx, sy = (CAR_SZ[0], CAR_SZ[1]) if o == 'h' else (CAR_SZ[1], CAR_SZ[0])
        body.append(_model(n, cx, cy, sx, sy, CAR_H, COL_CAR))
    for (n, cx, cy, sx, sy) in [('wall_north', 0.0, FIELD_HALF, span, WALL_T),
                                ('wall_south', 0.0, -FIELD_HALF, span, WALL_T),
                                ('wall_east', FIELD_HALF, 0.0, WALL_T, span),
                                ('wall_west', -FIELD_HALF, 0.0, WALL_T, span)]:
        body.append(_model(n, cx, cy, sx, sy, WALL_H, COL_WALL))
    return head + ''.join(body) + '  </world>\n</sdf>\n'


BLD_TOUCH_NOTE = """\
brief §1.3 的楼尺寸表里，bld_main(x∈[-30.6,18.2]) 与 bld_east(x∈[18.8,29.4]) 只差 0.6 m，
既不是"完全贴合"、又低于可通行下限 0.71 m ⇒ 落在 brief §1.4 明令禁止的带里。
reviewer 2026-10-09 裁定：**把缝消掉**（不是把楼移开）—— bld_main 东边界 18.2 -> 18.8
（宽 48.8 -> 49.4 m，+1.2%；中心 x -6.2 -> -5.9，西边界仍 -30.6），bld_east 原位不动。
生成器在 --check-only 与正常生成两条路径上都断言"两楼贴合"（见 touch_gap）。"""

# 两楼之间的缝**必须**是 0（贴合）——见 BLD_TOUCH_NOTE
TOUCH_GAP_EPS = 1e-6


def main(argv=None):
    ap = argparse.ArgumentParser(description='生成 campus_100.sdf（阶段八场景）')
    here = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.join(here, '..', 'worlds', 'campus_100.sdf')
    ap.add_argument('--out', default=default_out)
    ap.add_argument('--min-gap', type=float, default=MIN_GAP)
    ap.add_argument('--json', default='', help='把缝表写到这个 json')
    ap.add_argument('--check-only', action='store_true', help='只核约束，不写文件')
    a = ap.parse_args(argv)

    rects = all_rects()
    pairs, bad = gap_table(rects, a.min_gap)
    overlaps = overlap_report(rects)

    # 两楼**必须贴合**（缝 = 0）—— 见 BLD_TOUCH_NOTE
    main = next(r for r in rects if r['name'] == 'bld_main')
    east = next(r for r in rects if r['name'] == 'bld_east')
    touch = rect_gap(main, east)
    touch_ok = touch <= TOUCH_GAP_EPS

    report = {
        'world': os.path.abspath(a.out),
        'field_half_m': FIELD_HALF,
        'passable_area_m': [2 * (FIELD_HALF - WALL_T / 2.0)] * 2,
        'min_gap_m': a.min_gap,
        'obstacles': [{'name': r['name'], 'kind': r['kind'],
                       'x': [round(r['x0'], 3), round(r['x1'], 3)],
                       'y': [round(r['y0'], 3), round(r['y1'], 3)]} for r in rects],
        'pairs_total': len(pairs),
        'pairs_touching': sum(1 for p in pairs if p['gap_m'] == 0.0),
        'violations': bad,
        'overlaps': overlaps,
        'bld_main_east_edge_m': round(main['x1'], 4),
        'bld_east_west_edge_m': round(east['x0'], 4),
        'bld_touch_gap_m': round(touch, 6),
        'bld_touch_ok': touch_ok,
        'bld_touch_note': BLD_TOUCH_NOTE,
        'smallest_positive_gap_m': min((p['gap_m'] for p in pairs if p['gap_m'] > 0.0),
                                       default=None),
    }

    if bad:
        print('!! 通道缝约束被违反（%d 对，缝落在 (0, %.2f)）：' % (len(bad), a.min_gap),
              file=sys.stderr)
        for p in bad:
            print('   %-14s <-> %-14s  %.4f m' % (p['a'], p['b'], p['gap_m']), file=sys.stderr)
    if not touch_ok:
        print('!! 两楼**没有贴合**：bld_main 东边界 %s / bld_east 西边界 %s ⇒ 缝 %.6f m'
              % (_f(main['x1']), _f(east['x0']), touch), file=sys.stderr)

    if overlaps:
        print('!! 有障碍**真重叠**（不是贴合）：%d 对' % len(overlaps), file=sys.stderr)
        for p in overlaps:
            print('   %-14s <-> %-14s  %.4f m^2' % (p['a'], p['b'], p['overlap_m2']),
                  file=sys.stderr)

    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(report, fh, ensure_ascii=False, indent=2)
        print('缝表已写入 %s' % a.json, file=sys.stderr)

    if bad or overlaps or not touch_ok:
        return 2

    # 后面几道自检都核**将要写出去的那份文本**（不是磁盘上可能已过期的旧文件）
    text = render_sdf()

    # ---- ★ 自检 A：**模板占位符不许有残留** --------------------------------
    # 来历（reviewer 2026-10-09 抓到的 blocker）：本模板里**两种占位符风格混用** ——
    # 注释已改成 `@@LAT@@`（后来改的）、SDF 本体却还留着**老 `make_world.py` 的 `%(lat)s`**
    # ⇒ `<latitude_deg>%(lat)s</latitude_deg>` **从来没被替换过** ✗，
    # 而当时四道自检**全放过它**：`ET.fromstring` 只查 well-formedness（`%(lat)s` 是合法文本 ✓）、
    # `parse_raised_boxes` 不看坐标基准、连通性只看几何 ✓ ⇒ 当场什么都看不出来 ✗。
    # ⇒ 现在**两种风格都扫**，扫到即拒绝生成。
    residual = sorted(set(re.findall(r'@@[A-Za-z_]+@@', text))
                      | set(re.findall(r'%\([A-Za-z_]+\)[sdf]', text)))
    if residual:
        print('!! 模板占位符**没被替换干净**：%s ⇒ 拒绝生成 ✗' % ', '.join(residual),
              file=sys.stderr)
        return 1

    # ---- ★ 自检 B：合法 XML（注释里写 `--` 会让整份文件 not well-formed）----
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        print('!! 生成的 SDF 不是合法 XML：%s' % exc, file=sys.stderr)
        print('   （最常见原因：注释里写了 `--`，XML 注释不许含双连字符）', file=sys.stderr)
        return 1

    # ---- ★ 自检 C：从**渲染出来的文本**回读三个坐标基准，必须等于常量 ------
    # 这条是"注释不是凭据"的直接落实：自检 A 那个 blocker 里**注释是对的、XML 本体是错的**，
    # 只读注释的人会以为没事 ⇒ 必须 `ET` 回读**元素文本**再比，而不是信注释。
    sc = root.find('.//spherical_coordinates')
    got = {}
    for tag in ('latitude_deg', 'longitude_deg', 'elevation'):
        node = sc.find(tag) if sc is not None else None
        got[tag] = node.text if node is not None else None
    expect = {'latitude_deg': str(DATUM_LAT), 'longitude_deg': str(DATUM_LON),
              'elevation': str(DATUM_ALT)}
    if got != expect:
        print('!! `<spherical_coordinates>` 回读与常量**不一致**：%s（期望 %s）⇒ 拒绝生成 ✗'
              % (got, expect), file=sys.stderr)
        return 1

    # ---- ★ 自检 D：出生点必须在自由区、且净空 >= SPAWN_MIN_CLEAR -----------
    spawn_clear = min(rect_dist_pt(SPAWN[0], SPAWN[1], r) for r in rects)
    if spawn_clear < SPAWN_MIN_CLEAR:
        print('!! 出生点 %s 的净空只有 %.3f m < %.2f m ⇒ 拒绝生成 ✗（改 SPAWN 或挪障碍）'
              % (list(SPAWN), spawn_clear, SPAWN_MIN_CLEAR), file=sys.stderr)
        return 1

    # ---- ★ 自检 E：地面没被当成障碍（判据是"顶面 z > 0"，地面顶面恰 0）-----
    # 这条踩过一次（`_model` 的默认 pz 让地面顶面变成 0.02）⇒ 生成时就拦住。
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from stop_margin_check import parse_raised_boxes
        with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False,
                                         encoding='utf-8') as tf:
            tf.write(text)
            tmp_sdf = tf.name
        try:
            raised = {b['name'] for b in parse_raised_boxes(tmp_sdf)}
        finally:
            os.unlink(tmp_sdf)
    except Exception as exc:                                   # noqa: BLE001
        print('!! 无法用 parse_raised_boxes 复核（%s）' % exc, file=sys.stderr)
        return 1
    if 'ground' in raised:
        print('!! **地面被当成障碍**了（顶面 z > 0）⇒ 全域自由格会被它吃光 ⇒ 拒绝生成 ✗',
              file=sys.stderr)
        return 1

    summary = {'obstacles': len(rects), 'pairs': len(pairs), 'violations': 0,
               'geometry_fingerprint': geometry_fingerprint(rects),
               'bld_touch_gap_m': report['bld_touch_gap_m'],
               'smallest_positive_gap_m': report['smallest_positive_gap_m'],
               'datum_from_xml': got, 'spawn': list(SPAWN),
               'spawn_clearance_m': round(spawn_clear, 3)}

    # ---- ★ 自检 F：**几何指纹必须对高度敏感**（负对照，防"docstring 说三样、代码算两样"重演）----
    # 做法：把某一件障碍抬高 1 m、重算指纹，**必须变**；不变就说明高度没进 key ✗。
    probe = [dict(r) for r in rects]
    probe[0]['top'] = probe[0]['top'] + 1.0
    if geometry_fingerprint(probe) == summary['geometry_fingerprint']:
        print('!! **几何指纹对"顶面高度"不敏感** ⇒ 高度没进 key ⇒ 拒绝生成 ✗'
              '（改高度会改代价图内容而指纹放行 —— reviewer 2026-10-09 抓到过一次）',
              file=sys.stderr)
        return 1

    # ---- ★ 自检 G：`--check-only` 顺手比对**盘上那份世界**（reviewer 建议）----
    # 为什么需要：几何指纹是从**生成器模型**算的 ⇒ 它**发现不了**"盘上世界是旧模型产物"
    # 或"被人手改过" ✗ ⇒ 渲染出来与盘上那份比一次，两个凭据（生成器、盘上文件）当场对表 ✓。
    on_disk = ''
    if os.path.exists(default_out):
        on_disk = open(default_out, encoding='utf-8').read()
    summary['disk_world_matches_model'] = (on_disk == text)
    if on_disk and on_disk != text:
        print('⚠ **盘上那份世界与生成器现在渲染出来的不一样**（盘上可能是旧模型产物、或被手改过）'
              '⇒ 若正要起栈，先重跑一次生成器 ✓', file=sys.stderr)

    if a.check_only:
        print(json.dumps(summary, ensure_ascii=False))
        return 0

    out = os.path.abspath(a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w', encoding='utf-8') as fh:
        fh.write(text)
    sha = hashlib.sha256(text.encode('utf-8')).hexdigest()
    summary.update({'written': out, 'sha256': sha,
                    'bytes': len(text.encode('utf-8'))})
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())

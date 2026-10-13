#!/usr/bin/env python3
"""★ 世界真落差清单（**单一真源**）—— 2026-10-08 改场景后的**新世界**版本。

背景：先前 `SEGMENTS` 在 `mask_attribution.py` / `plan_vs_edge.py` / `an_af.py` 里**各有一份** ✗
⇒ 改场景要改三处、**极易漂移**（与"同一份数据两个中位数"同族）⇒ 收敛成本文件 ✓。

**当前世界几何**（`worlds/depth_drop_ramp.sdf` sha256 **`24815fa7…`**，2026-10-09 **(b) 封坡后**；
沿革：`44b445e5…`（东侧坡原版）→ `97d2d457…`（坡搬北侧）→ `cc611405…`（坡拉长 4.0 m ⇒ 4.29°）→
`e7cca8ca…`（封边，4 块）→ `7de22e33…`（坡加宽 2.5→4.0 m，删两块北封）→
**`24815fa7…`（沿坡两侧加实体块 ⇒ 坡侧沿从"落差"变"墙"）**）：
`upper_platform` x∈[-4,0] y∈[-3,3] 顶 z=0；`lower_platform` x∈[-8,16] y∈[-10,10] 顶 z=-0.30；
`ramp_wedge` 在北侧：**x∈[-4.0, 0.0]**（加宽后 ⇒ **坡口 = 整条北沿** ✓）、y∈[3.0,7.0]，顶面 `(y=3,z=0)` → `(y=7.0,z=-0.30)`（长 4.0 m ⇒ 4.29° ✓）；
**可用带 = `4.0 − 2×0.30 = 3.4 m`** ✓（原 1.9 m ⇒ 容得下实测那个 ~25° 的横向漂移 ✓）。
**★★ 封边（2026-10-09，AuYang 选 (ii)）**：周界**除东主边与北坡口**外，用实体块封住（顶 `z=+0.15` **高出平台面** ⇒ 读作**向上的墙**、**不是落差** ✓）；
**坡加宽后只剩 `seal_south` / `seal_west`** ✓ —— **两块北封（`seal_north_w/e`）已删** ✗：
它们原是按**旧坡宽 2.5 m**（坡口 `x∈[-3.25,-0.75]`）留口的，**加宽后会正好卡在坡口上** ✗；
删掉之后**北沿整条都是坡口 ⇒ 那里没有落差 ⇒ 也没有"从北沿下来的捷径"** ✓（它们原本的作用就是封住北沿的步段——步段没了，封块自然也不需要 ✓）。

**★★★ (b) 封坡（2026-10-09，reviewer 裁定、AuYang 预登记）**：**沿坡两侧**（`x=0` 与 `x=-4`、`y∈[3,7]`）各加一块实体
（`ramp_seal_east` / `ramp_seal_west`，贴坡侧**外侧**、顶 `z=+0.15` **全程在坡面之上** ⇒ 是"墙"不是"落差" ✓、底 `z=-0.50` 落到低台面 ✓）：
· 依据：`ramp7-as` 的 `plan_vs_wall` 实测 **`plan_min_dist_to_inventory_m = 0.0`**、最近段 = **`7 坡东侧沿 x=0, y∈[3,7]`**、
  最近点 `[0.0, 4.05]` ⇒ **规划路径贴着坡东沿走** ✗ ⇒ 车照着走就**掉下坡**（`ramp7-as`/`ao`/`ap` 三次都是这个）✗；
· ⇒ **坡侧沿不再进落差掩膜、而墙会进障碍层** ⇒ **顺手消掉"贴着坡边规划"** ✓；
· ⇒ **第 6/7 两条从 `SEGMENTS` 移入 `SEALED_EDGES`** ✓（**落差清单里现在只剩主边一条** ✓）。

**三个清单（分工要清楚 ✓）**：
· **`SEGMENTS`** = **真落差**（应当被标）= **(b) 封坡后只剩东主边一条** ✓
  —— ★ 沿革：先前是"东主边 **+ 坡两侧沿**"，且**坡东沿 `x=0, y∈[3,7]`（0.30→0 坡沿）与主边 `x=0, y∈[-3,3]`（0.30 纯台阶）共线但不同边、必须分开列** ✗
  （合并会把"主边"判宽 ✓）；**(b) 封坡后这两条同时消失** ⇒ 这个"共线陷阱"也一并没了 ✓（但**记在这里**：若将来重新开口，还得按这个规矩分开列 ✓）；
· **`SEALED_EDGES`** = **已封的周界边**（**不再是落差** ✓，记录在案、供对照 ✓）；
· **`BLOCKING_EDGES`** = **空隙判据的阻塞清单** = **只剩东主边** ✓。
**★ `NOT_EDGES`** = **0 落差、不应当被标**（坡顶沿/坡底沿 ✓ —— "不该标却没标"最容易被漏核 ✗）。
"""
import math

# 世界指纹（改场景后）
WORLD = 'worlds/depth_drop_ramp.sdf'
WORLD_SHA256 = '24815fa74a8b4c1de65210bc2d6031cd7c8775bde95ae6b9abae9ebeac64e14b'   # 2026-10-09 (b) 封坡：沿坡两侧各加一块 ⇒ 坡侧沿移入"已封边"
WORLD_SHA256_OLD_EAST = '44b445e5bc018733ea2474033cf7665a2343f6170749c68011b9915263f012c5'

RAMP_X0, RAMP_X1 = -4.0, 0.0       # 坡的横向范围（2026-10-09 加宽 2.5→4.0 m ⇒ 坡口 = 整条北沿 ✓）
RAMP_Y0, RAMP_Y1 = 3.0, 7.0        # 坡顶沿 / 坡底沿（2026-10-08 坡拉长 1.6→4.0 m ⇒ 4.29°）
CLIFF_X = 0.0                      # 主边：高台东沿
PLAT_Y0, PLAT_Y1 = -3.0, 3.0       # 高台南北范围
PLAT_X0, PLAT_X1 = -4.0, 0.0       # 高台东西范围

# ── 正向清单：真落差（应当被标）─────────────────────────────────────────────
#   ★ 2026-10-09 **(b) 封坡后**：落差**只剩东主边一条** ✓
#   （南/西已封、(b) 后坡两侧沿也封住 ⇒ 那些地方都是"向上的墙"、**不是向下的落差** ✓）
SEGMENTS = [
    ('1 断崖(高台东沿,纯台阶)', [(CLIFF_X, PLAT_Y0), (CLIFF_X, PLAT_Y1)]),      # 0.30 m 全程, 6 m 长直 ★唯一未封的周界边
]

# ── ★ 已封边：**不再是落差**（实体块顶高出平台面 0.15 m ⇒ 读作"向上的墙" ✓）─────
#   2026-10-09 AuYang 选 (ii)「场景封边」：只加 box，不动平台、不动坡 ✓
SEALED_EDGES = [
    ('S 南沿 y=-3（已封 seal_south）', [(PLAT_X0, PLAT_Y0), (PLAT_X1, PLAT_Y0)]),
    ('W 西沿 x=-4（已封 seal_west）', [(PLAT_X0, PLAT_Y0), (PLAT_X0, PLAT_Y1)]),
    # ★ 2026-10-09 坡加宽后：**北沿整条都是坡口 ⇒ 那里没有落差 ⇒ 北沿步段不存在、两块北封已删** ✓
    #   （`seal_north_w` / `seal_north_e` 原是按旧坡宽 2.5 m 留口的，加宽后会卡在坡口上 ✗）
    # ★★ 2026-10-09 (b) 封坡：**坡两侧沿移到这里** ✓ —— 它们是"坡的侧沿"（0.30→0），
    #   与主边 `x=0, y∈[-3,3]` **共线但不同边** ⇒ 先前列在 `SEGMENTS` 里时就**必须分开** ✗；
    #   现在两侧都加了实体块（顶 z=+0.15 **全程在坡面之上** ✓）⇒ **不再是落差** ✓
    ('6 坡西侧沿 x=-4, y∈[3,7]（已封 ramp_seal_west）', [(RAMP_X0, RAMP_Y0), (RAMP_X0, RAMP_Y1)]),
    ('7 坡东侧沿 x=0, y∈[3,7]（已封 ramp_seal_east）', [(RAMP_X1, RAMP_Y0), (RAMP_X1, RAMP_Y1)]),
]

# ★ 空隙判据的**阻塞清单** = 只剩东主边 ✓（封边后南/西/北不再阻塞 ✓）
BLOCKING_EDGES = [
    ('东 主边 x=0（y∈[-3,3] 全段，唯一未封的周界边）', [(CLIFF_X, PLAT_Y0), (CLIFF_X, PLAT_Y1)]),
]

# ── ★ 负向清单：0 落差（**不应当被标**）───────────────────────────────────────
#   坡顶沿与高台北沿在坡口处连续（z=0 接 z=0）、坡底沿与低台面连续（z=-0.30 接 -0.30）
#   ⇒ 这两条**不该**出现落差标记；若出现 ⇒ 坡口被自己的标记堵死（§7.7 那类故障）✗
NOT_EDGES = [
    ('N1 坡顶沿(y=3, 0 落差)', [(RAMP_X0, RAMP_Y0), (RAMP_X1, RAMP_Y0)]),
    ('N2 坡底沿(y=7.0, 0 落差)', [(RAMP_X0, RAMP_Y1), (RAMP_X1, RAMP_Y1)]),
]

# 主边（demo 里"直线会穿过的那条"）
MAIN_EDGE = SEGMENTS[0][0]

# 期望路径的跨线位置：绕行应从 y > PLAT_Y1 的低台侧跨过 x=0
DETOUR_MUST_CROSS_ABOVE_Y = PLAT_Y1


def seg_dist(p, a, b):
    """点 p 到线段 ab 的距离。"""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / den))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def classify(p, segments=None):
    """返回 (最近线段名, 距离)。"""
    segs = SEGMENTS if segments is None else segments
    best = (None, 1e9)
    for name, (a, b) in segs:
        d = seg_dist(p, a, b)
        if d < best[1]:
            best = (name, d)
    return best


def enters_main_cliff(prev, cur):
    """线段 (prev→cur) 是否**穿过主边**（x=CLIFF_X 且在 y∈[PLAT_Y0, PLAT_Y1] 内）。

    返回 None 表示不穿；否则返回穿越点的 (x, y)。
    """
    (x1, y1), (x2, y2) = prev, cur
    if abs(x2 - x1) < 1e-12:
        return None
    t = (CLIFF_X - x1) / (x2 - x1)
    if not (0.0 <= t <= 1.0):
        return None
    y = y1 + t * (y2 - y1)
    if PLAT_Y0 <= y <= PLAT_Y1:
        return (CLIFF_X, round(y, 4))
    return None


def ramp_surface_z(y):
    """坡面高度（解析；y 在 [RAMP_Y0, RAMP_Y1] 内）。"""
    return 0.0 - 0.075 * (y - RAMP_Y0)   # 0.30/4.0 = 0.075（4.29°）


def ramp_mouth_free_band_x(margin):
    """坡口（y=3 处）在给定外扩量下**可通行**的 x 带（扣掉坡两侧沿的外扩）。"""
    return (RAMP_X0 + margin, RAMP_X1 - margin)


if __name__ == '__main__':
    print('世界 %s' % WORLD)
    print('  sha256 = %s  (旧世界 %s)' % (WORLD_SHA256, WORLD_SHA256_OLD_EAST))
    print('\n正向清单（真落差，应当被标）%d 条：' % len(SEGMENTS))
    for n, (a, b) in SEGMENTS:
        print('  %-22s %s → %s' % (n, a, b))
    print('\n★ 已封边（**不再是落差** ✓，记录在案）%d 条：' % len(SEALED_EDGES))
    for n, _ in SEALED_EDGES:
        print('  %s' % n)
    print('\n★ 空隙判据的阻塞清单 %d 条（**只剩东主边** ✓）：' % len(BLOCKING_EDGES))
    for n, _ in BLOCKING_EDGES:
        print('  %s' % n)
    print('\n★ 负向清单（0 落差，**不应当被标**）%d 条：' % len(NOT_EDGES))
    for n, (a, b) in NOT_EDGES:
        print('  %-22s %s → %s' % (n, a, b))
    print('\n主边 = %s' % MAIN_EDGE)
    print('绕行判据：路径跨 x=0 必须发生在 y > %.1f' % DETOUR_MUST_CROSS_ABOVE_Y)
    print('坡口可通行带（外扩 0.30）= x∈[%.2f, %.2f] 宽 %.2f m'
          % (*ramp_mouth_free_band_x(0.30), ramp_mouth_free_band_x(0.30)[1] - ramp_mouth_free_band_x(0.30)[0]))

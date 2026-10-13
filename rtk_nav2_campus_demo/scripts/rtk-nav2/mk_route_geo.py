#!/usr/bin/env python3
"""把一条**世界系折线**生成为 `route_geo.json`（经纬度），**逐点做净空自检、不合格就拒绝出件**。

★★ 为什么要这个脚本（2026-10-10 两起同类事故）★★
`(lat,lon) → map → /goal_pose` 这条链上，**唯一会被"手填的数字"害死的环节就是航点**：
  · 我把 `(-40,-30)` 填成目标 —— 它落在 `green_sw_1` 里（净空 0.000 m）⇒ nav2 拒绝规划、
    车原地不动、取数器白采 30 min ✗；
  · 替代航线里的 `wp4=(-29,-3)` 同样**落在 `bld_main` 里**（净空 0.000 m）✗。
两次都不是软件 bug，是**"我以为这个点在外面"** ⇒ 只能靠**用世界的权威障碍表量一遍**来堵 ✓。
⇒ 固化成机制：**本脚本落盘前逐点量净空**（用 `make_campus_world.all_rects()`，与静态层同一份真源），
   任何一点 < `--min-clear` 就**不产出文件**并打印是谁挡的 ✓。

★ 注意：航线点是**目标**、不是路径 —— nav2 自己会在两点之间规划绕障 ✓
  ⇒ **只需要每个点本身在自由空间里有足够净空**，两点之间的直线穿不穿障碍不影响 ✓。

用法（容器内）：
  mk_route_geo.py --out /work/artifacts/rtk-nav2/<run>/input/route_geo.json [--min-clear 1.0]
"""

import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(_HERE, '..', '..', 'rtk_nav2_ws', 'src', 'robonix_rtk_nav2'))
sys.path.insert(0, os.path.join(PKG, 'scripts'))
import make_campus_world as M  # noqa: E402

M_PER_DEG_LAT = 111320.0

# ★★ 独立来源（reviewer 2026-10-10 裁定 ②）★★
#   问题：替代航线的经纬度是**用同一套约定**从世界点反算的 ⇒ `f⁻¹` 与 `f` **互相抵消** ⇒
#         车"走到毫米级"只证明**往返自洽**，**不证明 half-2 正确** ✗。
#   ⇒ 本函数走**严格 ECEF 路径**（ENU → ECEF → 大地坐标，Bowring 闭式反解），
#     与 `geo_calib` 用的**切平面近似**是**两套独立方法** ✓ ⇒ 不再抵消 ✓。
#   ★ 诚实边界：ECEF 与切平面在 40 m 尺度上**本就该一致到亚毫米** ⇒ 这条能查"切平面近似是否成立"
#     与"实现是否正确"，**但查不出"约定本身选得对不对"** —— 那一层只有**外部来源的真经纬度**
#     （AuYang 的真实点位）才测得了 ✓。
A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563


def geodetic_to_ecef(lat_deg, lon_deg, h):
    """大地坐标 → ECEF（WGS84）。"""
    e2 = F_WGS84 * (2.0 - F_WGS84)
    ph, lm = math.radians(lat_deg), math.radians(lon_deg)
    n = A_WGS84 / math.sqrt(1.0 - e2 * math.sin(ph) ** 2)
    return ((n + h) * math.cos(ph) * math.cos(lm),
            (n + h) * math.cos(ph) * math.sin(lm),
            (n * (1.0 - e2) + h) * math.sin(ph))


def ecef_to_geodetic(x, y, z):
    """ECEF → 大地坐标（**Bowring 闭式**，本项目尺度上足够精确）。"""
    e2 = F_WGS84 * (2.0 - F_WGS84)
    b = A_WGS84 * (1.0 - F_WGS84)
    ep2 = (A_WGS84 ** 2 - b ** 2) / b ** 2
    p = math.hypot(x, y)
    th = math.atan2(z * A_WGS84, p * b)
    ph = math.atan2(z + ep2 * b * math.sin(th) ** 3,
                    p - e2 * A_WGS84 * math.cos(th) ** 3)
    lm = math.atan2(y, x)
    n = A_WGS84 / math.sqrt(1.0 - e2 * math.sin(ph) ** 2)
    h = p / math.cos(ph) - n
    return math.degrees(ph), math.degrees(lm), h


def enu_to_latlon_ecef(x, y, lat0, lon0, h0=0.0):
    """局部 ENU 偏移 `(东, 北)` → 经纬度，**走严格 ECEF 路径**（独立于切平面近似）✓。"""
    ph, lm = math.radians(lat0), math.radians(lon0)
    ox, oy, oz = geodetic_to_ecef(lat0, lon0, h0)
    # 基准点的 ENU 基向量
    e = (-math.sin(lm), math.cos(lm), 0.0)
    n = (-math.sin(ph) * math.cos(lm), -math.sin(ph) * math.sin(lm), math.cos(ph))
    u = (math.cos(ph) * math.cos(lm), math.cos(ph) * math.sin(lm), math.sin(ph))
    tx = ox + x * e[0] + y * n[0]
    ty = oy + x * e[1] + y * n[1]
    tz = oz + x * e[2] + y * n[2]
    la, lo, _ = ecef_to_geodetic(tx, ty, tz)
    return la, lo

# 世界系折线（与替代件同一意图：**绕主楼西侧**从东南到西北）
# ★ wp1 由 (-11,-21) 改到 (-11,-22)：原值离一辆车只有 0.55 m（偏紧）✓
# ★ wp4 由 (-29,-3) 改到 (-33.5,-3)：原值**在主楼里**（净空 0.000 m）✗ —— 本次修正的核心
POLYLINE = [(2.6, -30.0),      # start
            (-4.0, -26.0),
            (-11.0, -22.0),    # ← 改（原 -11,-21 净空仅 0.55 m）
            (-18.0, -15.5),
            (-24.0, -9.5),
            (-33.5, -3.0),     # ← 改（原 -29,-3 **落在 bld_main 内**，净空 0.000 m）
            (-32.5, 3.5),
            (-35.0, 10.0),
            (-37.5, 16.5),
            (-40.0, 25.0)]     # goal


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--min-clear', type=float, default=1.0)
    ap.add_argument('--check-only', action='store_true')
    ap.add_argument('--source', choices=['tangent', 'ecef'], default='ecef',
                    help='经纬度的来源换算：`ecef` = **严格 ECEF**（独立于切平面近似 ✓，默认）；'
                         '`tangent` = 切平面近似（**与 geo_calib 同源 ⇒ 会抵消** ✗，只作对照）')
    a = ap.parse_args(argv)

    lat0, lon0, alt0, src = None, None, None, None
    # ★ 基准**现读**（不许手填 —— 2026-10-09 就是手填的旧站基准让产物与报数不一致 ✗）
    import geo_calib_map_world as H
    lon0, lat0, alt0, src = H.read_datum(os.path.join(PKG, 'worlds', 'campus_100.sdf'))
    m_lon = M_PER_DEG_LAT * math.cos(math.radians(lat0))
    print('基准 %.4f / %.4f（来自 %s）｜ M_LAT=%.0f  M_LON=%.1f' % (lat0, lon0, src, M_PER_DEG_LAT, m_lon))

    rects = M.all_rects()
    worst, bad = 1e9, []
    for i, (x, y) in enumerate(POLYLINE):
        c = min(M.rect_dist_pt(x, y, r) for r in rects)
        who = min(rects, key=lambda r: M.rect_dist_pt(x, y, r))['name']
        worst = min(worst, c)
        name = 'start' if i == 0 else ('goal' if i == len(POLYLINE) - 1 else 'wp%d' % (i - 1))
        flag = 'OK ' if c >= a.min_clear else 'BAD'
        print('  %s %-6s (%7.1f, %7.1f) 净空 %6.3f m  最近 %s' % (flag, name, x, y, c, who))
        if c < a.min_clear:
            bad.append((name, x, y, c, who))
    if bad:
        print('!! 有 %d 个航点净空 < %.2f m ⇒ **拒绝出件** ✗' % (len(bad), a.min_clear), file=sys.stderr)
        for b in bad:
            print('!!   %s (%.1f,%.1f) 净空 %.3f m —— 被 %s 挡' % b, file=sys.stderr)
        return 1
    print('最小净空 %.3f m ≥ %.2f m ✓' % (worst, a.min_clear))
    if a.check_only:
        return 0

    def ll(x, y):
        if a.source == 'ecef':
            la, lo = enu_to_latlon_ecef(x, y, lat0, lon0, alt0 or 0.0)   # ★ 独立路径 ✓
        else:
            la, lo = lat0 + y / M_PER_DEG_LAT, lon0 + x / m_lon          # 同源（会抵消）✗
        return {'lat': round(la, 9), 'lon': round(lo, 9)}

    pts = [ll(x, y) for (x, y) in POLYLINE]
    total = sum(math.dist(POLYLINE[i], POLYLINE[i + 1]) for i in range(len(POLYLINE) - 1))
    out = {
        'crs': 'WGS84',
        'spacing_m': 10.0,
        '_NOTE': '★★ **developer 自造的替代件，不是那位二维地图选点开发者的输出** ★★ '
                 '在对方真件到手前先把「经纬度接入」整条机制跑通；真件到手后**原样替换**并由 reviewer 复验。',
        '_CHANGELOG': 'v2（2026-10-10）：逐点净空自检后修正两点 —— '
                      'wp4 (-29,-3) 原**落在 bld_main 内**（净空 0.000 m ⇒ nav2 必然拒绝规划）✗ 改到 (-33.5,-3)；'
                      'wp1 (-11,-21) 离一辆车仅 0.55 m 改到 (-11,-22)。'
                      '**v1 的两个缺陷是"以为点在外面"造成的，只能靠量净空发现** ✓',
        'start': pts[0], 'goal': pts[-1],
        'waypoints': pts[1:-1],
        '_waypoints_semantics': '含起点不含终点（brief §2 默认）',
        '_world_xy_for_reference': [{'x': x, 'y': y} for (x, y) in POLYLINE],
        '_clearance_m': ['%.3f' % min(M.rect_dist_pt(x, y, r) for r in rects) for (x, y) in POLYLINE],
        '_polyline_length_m': round(total, 2),
        '_datum': {'lat': lat0, 'lon': lon0, 'alt': alt0, 'source': src},
        '_convention': ('ENU→经纬度 走**严格 ECEF**（WGS84 + Bowring 反解），**独立于切平面近似** ✓'
                        if a.source == 'ecef' else
                        '切平面近似 M_PER_DEG_LAT=111320 / 111320*cos(lat0) —— '
                        '★ **与 geo_calib 同源 ⇒ f⁻¹ 与 f 会互相抵消，只证明往返自洽** ✗'),
        '_why_this_source': '★ reviewer 2026-10-10 裁定 ②：替代航线原用同源约定 ⇒ 误差抵消、'
                            '证明不了 half-2 ✗。改用独立换算 ⇒ 不再抵消 ✓。'
                            '（诚实边界：ECEF 与切平面在 40 m 上本就该一致到亚毫米 ⇒ 这条查的是'
                            '"切平面近似成不成立 + 实现对不对"，**查不出"约定选得对不对"** —— '
                            '那一层只有**外部真经纬度**（AuYang 的真实点位）才测得了 ✓）',
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    tmp = a.out + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, a.out)
    print('★ 已写出 %s（折线长 %.1f m，%d 个航点）' % (a.out, total, len(pts)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

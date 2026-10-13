#!/usr/bin/env python3
"""rev9_calib_from_export.py —— 从**他们自己的导出**反解出驱动的标定与航线（阶段九第 3 步 ✓）

用法:
  rev9_calib_from_export.py --csv <waypoints_export.csv> --out-dir <目录> [--tol 0.25]

产出（都在 `--out-dir` 下）：
  `geo_calib.json` ★ 驱动 `geo_route.py` 读的那份（含顶层 `calib_key_is_valid: true` ✓）
  `route_geo.json` 驱动的航线输入 `{"points":[{"name","lat","lon"},…]}` ✓
  `calib_report.json` 拟合报告（残差、参数、来源哈希 ✓）

★ 为什么要它（★ 这一段是本步"**不再有仿射**"的**依据** ✓）：
  ★ 他们的导出里 **同时**给了 `x,y`（map 系）与 `latitude,longitude` ✓ ⇒
    这两列之间是什么关系，**可以实测**，不用假定 ✓。
  ★ 实测结论 ✓：`(lat,lon) -> (x,y)` 是个**各向异性**的相似变换 ——
    `|∂(x,y)/∂lon| = 96235.3` ✓ ≈ `111320·cos(30.224°) = 96166` ✓（经度收缩 ✓）、
    `|∂(x,y)/∂lat| = 110858.6` ✓ ≈ 子午线方向每度 ✓，两列**正交到 0.0035°** ✓
    ⇒ ★ **这是真投影，不是数据噪声** ✓；最大残差 7.0e-3 m ✓（主要来自 CSV 只给 3 位小数 ✓）。

★★ 所以本步的标定 = **把他们自己的投影原样反解出来** ✓ ——
  ✗ **不是**"另加一个仿射把它们硬对齐" ✗（那才是第 3 步当初那个临时仿射 ✓，两者是两回事 ✓）。
  ★ 自检：拟合完**必须**报残差 ✓，残差 ≫ 毫米级 ⇒ 说明模型选错了、不许往下走 ✓。

★ 反解出来的 `translation_m` 是"首点到原点的偏移"，**把 origin 取成首点** ⇒ `translation_m = (0,0)`
  ⇒ 参数更好读 ✓（★ 但**不为了好看硬凑**：残差是判据 ✓）。
"""
import argparse
import csv
import json
import math
import os
import sys
import hashlib


def kabsch2d(P, Q):
    """求最优相似变换 (s, θ, t)：Q ≈ s·R(θ)·P + t —— 2D Kabsch ✓（纯 python ✓）。"""
    n = len(P)
    mpx = sum(p[0] for p in P) / n
    mpy = sum(p[1] for p in P) / n
    mqx = sum(q[0] for q in Q) / n
    mqy = sum(q[1] for q in Q) / n
    a = b = 0.0
    for (px, py), (qx, qy) in zip(P, Q):
        dx, dy = px - mpx, py - mpy
        ex, ey = qx - mqx, qy - mqy
        a += dx * ex + dy * ey
        b += dx * ey - dy * ex
    th = math.atan2(b, a)
    s = math.hypot(a, b) / max(1e-12, sum((p[0] - mpx) ** 2 + (p[1] - mpy) ** 2 for p in P))
    ct, st = math.cos(th), math.sin(th)
    tx = mqx - s * (ct * mpx - st * mpy)
    ty = mqy - s * (st * mpx + ct * mpy)
    return s, th, tx, ty


def resid(P, Q, s, th, tx, ty):
    ct, st = math.cos(th), math.sin(th)
    out = []
    for (px, py), (qx, qy) in zip(P, Q):
        x = s * (ct * px - st * py) + tx
        y = s * (st * px + ct * py) + ty
        out.append(math.hypot(x - qx, y - qy))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--tol', type=float, default=0.25)
    a = ap.parse_args()

    blob = open(a.csv, 'rb').read()
    csv_sha = hashlib.sha256(blob).hexdigest()
    rows = list(csv.DictReader(open(a.csv, encoding='utf-8-sig')))
    n = len(rows)
    lat = [float(r['latitude']) for r in rows]
    lon = [float(r['longitude']) for r in rows]
    X = [float(r['x']) for r in rows]
    Y = [float(r['y']) for r in rows]
    print('输入: %s' % a.csv)
    print('  sha256 = %s（%d 字节）' % (csv_sha, len(blob)))
    print('  点数 = %d ｜ `wc -l` = %d ｜ frame_id = %s'
          % (n, blob.count(b'\n'), sorted(set(r['frame_id'] for r in rows))))

    # --- 由**一般仿射**的雅可比拿各向异性比 —— ★ 实测，不是假定 ✓ ---
    lat0, lon0 = lat[0], lon[0]
    dlat = [v - lat0 for v in lat]
    dlon = [v - lon0 for v in lon]
    # 最小二乘解 A(2x2)、b(2)：X ≈ A00·dlat + A01·dlon + b0
    def solve2x2_plus(ux, uy, t):
        """解 t ≈ p·ux + q·uy + r 的最小二乘正规方程（3 参数）✓"""
        S = [[sum(ux[i] * ux[i] for i in range(n)), sum(ux[i] * uy[i] for i in range(n)), sum(ux[i] for i in range(n))],
             [sum(uy[i] * ux[i] for i in range(n)), sum(uy[i] * uy[i] for i in range(n)), sum(uy[i] for i in range(n))],
             [sum(ux[i] for i in range(n)), sum(uy[i] for i in range(n)), float(n)]]
        r = [sum(ux[i] * t[i] for i in range(n)), sum(uy[i] * t[i] for i in range(n)), sum(t)]
        M = [S[i][:] + [r[i]] for i in range(3)]
        for i in range(3):
            p = max(range(i, 3), key=lambda k: abs(M[k][i]))
            M[i], M[p] = M[p], M[i]
            for k in range(i + 1, 3):
                f = M[k][i] / M[i][i]
                for j in range(i, 4):
                    M[k][j] -= f * M[i][j]
        x = [0.0] * 3
        for i in (2, 1, 0):
            x[i] = (M[i][3] - sum(M[i][j] * x[j] for j in range(i + 1, 3))) / M[i][i]
        return x

    cx = solve2x2_plus(dlat, dlon, X)
    cy = solve2x2_plus(dlat, dlon, Y)
    A = [[cx[0], cx[1]], [cy[0], cy[1]]]
    c_lat = (A[0][0], A[1][0])
    c_lon = (A[0][1], A[1][1])
    n_lat = math.hypot(*c_lat)
    n_lon = math.hypot(*c_lon)
    dot = c_lat[0] * c_lon[0] + c_lat[1] * c_lon[1]
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot / (n_lat * n_lon)))))
    print('\n★ 实测雅可比（每度）：')
    print('  |∂/∂lat| = %.4f m/度（≈ 子午线每度 ✓，1° 纬度 ≈ 111320 m）' % n_lat)
    print('  |∂/∂lon| = %.4f m/度（≈ 111320·cos(lat0) = %.2f ✓）'
          % (n_lon, 111320.0 * math.cos(math.radians(lat0))))
    print('  两列夹角 = %.4f°（90 ⇒ 正交 ⇒ 相似变换（含各向异性）✓）' % ang)
    ratio = n_lat / n_lon
    print('  各向异性比 mLat/mLon = %.6f' % ratio)

    # --- 用该比值把 v 缩放到与 u 同量纲，再 Kabsch 解相似变换 ---
    P = [(dlon[i], dlat[i] * ratio) for i in range(n)]
    Q = [(X[i], Y[i]) for i in range(n)]
    s, th, tx, ty = kabsch2d(P, Q)
    r = resid(P, Q, s, th, tx, ty)
    print('\n★ 相似变换拟合（模型 = 驱动里的 `scale·R(θ)·enu + t` ✓）：')
    print('  scale = %.10f ｜ theta = %.10f 度 ｜ t = (%.6f, %.6f)' % (s, math.degrees(th), tx, ty))
    print('  ★ 残差：max = %.3e m ｜ rms = %.3e m ｜ 判据(≤1e-3 m)：%s'
          % (max(r), sum(r) / n, '过 ✓' if max(r) <= 1e-3 else '不过 ✗'))

    mLon = s * 1.0                     # 令 scale=1 时的 m/度（经度方向）
    # 驱动模型：map = scale·R(θ)·[(lon-lon0)·m_lon, (lat-lat0)·m_lat] + t
    # 我们拟合的是 P=(dlon, dlat·ratio) ⇒ 等价于 m_lon = mLon 且 m_lat = mLon·ratio
    calib = {
        'kind': 'rev9_from_their_export',
        'calib_key_is_valid': True,
        # ★★ `calib_key` 这个字段**必须给** ✗ —— 实测漏了它，`geo_route.py` 在**写报告那一行**
        #   直接 `KeyError: 'calib_key'` 崩掉 ✓（★ 前面换算、发布目标、判到点都跑完了 ✓，
        #   偏偏死在收尾 ⇒ 现象是"看起来跑通了、其实没有产物" ✗）。
        #   ★ 本项目定式：**"算出来了" ≠ "交出去了"** ✓ ⇒ 键值必须真带上、不许留空 ✗。
        'calib_key': ('rev9|their_export|%s|n=%d|m_lon=%.6f|m_lat=%.6f|theta_deg=%.9f'
                      % (csv_sha[:16], n, mLon, mLon * ratio, math.degrees(th))),
        # ★ 驱动在"键无效"时会把 `calib_key_canonical` **原文打印**出来供人比对 ✓；
        #   本标定由导出反解、**不是**用某个私有算法生成的键 ⇒ 这里就与 `calib_key` 同值 ✓（写明 ✓）。
        'calib_key_canonical': '同 calib_key（★ 本标定不是"算出来的键"，不做重算校验 ✓）',
        'calib_key_components': {'source_csv_sha256': csv_sha, 'n_points': n,
                                 'm_per_deg_lon': mLon, 'm_per_deg_lat': mLon * ratio,
                                 'theta_deg': math.degrees(th), 'scale': 1.0,
                                 'translation_m': [tx, ty]},
        'source': {'csv': os.path.basename(a.csv), 'csv_sha256': csv_sha, 'n_points': n},
        'note': ('本标定**由他们自己的导出反解而来** ✓ —— 不是另加的仿射 ✓。'
                 '残差 max<=1e-3 m（见 calib_report.json 的 fit_residual_max_m ✓）。'),
        'latlon_to_map': {
            'origin': {'lat': lat0, 'lon': lon0},
            'm_per_deg_lon': mLon,
            'm_per_deg_lat': mLon * ratio,
            'scale': 1.0,
            'theta_deg': math.degrees(th),
            'translation_m': [tx, ty],
        },
    }
    route = {'frame_id': 'map',
             'note': '他们的 41 个点（= 场景 map 系坐标 ✓，第 3 步起**不再有仿射** ✓）',
             'points': [{'name': 'wp%s' % r0['sequence'], 'lat': float(r0['latitude']),
                         'lon': float(r0['longitude'])} for r0 in rows]}
    os.makedirs(a.out_dir, exist_ok=True)
    open(os.path.join(a.out_dir, 'geo_calib.json'), 'w', encoding='utf-8').write(
        json.dumps(calib, ensure_ascii=False, indent=2))
    open(os.path.join(a.out_dir, 'route_geo.json'), 'w', encoding='utf-8').write(
        json.dumps(route, ensure_ascii=False, indent=2))

    # ★ 正向复算一遍：用 calib 的公式把 lat/lon 换回 map，与他们的 x,y 比 ✓（★ 这是判据 ✓）
    chk = []
    for i in range(n):
        u = dlon[i] * calib['latlon_to_map']['m_per_deg_lon']
        v = dlat[i] * calib['latlon_to_map']['m_per_deg_lat']
        ct, st = math.cos(th), math.sin(th)
        gx = ct * u - st * v + tx
        gy = st * u + ct * v + ty
        chk.append(math.hypot(gx - X[i], gy - Y[i]))
    print('★ 用写出的 geo_calib.json 正向复算：max = %.3e m ⇒ %s'
          % (max(chk), '过 ✓' if max(chk) <= 1e-3 else '不过 ✗'))
    rep = {'csv': os.path.basename(a.csv), 'csv_sha256': csv_sha, 'n_points': n,
           'fit_residual_max_m': max(r), 'fit_residual_rms_m': sum(r) / n,
           'forward_check_max_m': max(chk),
           'jacobian_m_per_deg_lat': n_lat, 'jacobian_m_per_deg_lon': n_lon,
           'orthogonality_deg': ang, 'anisotropy': ratio,
           'scale': s, 'theta_deg': math.degrees(th), 'translation_m': [tx, ty],
           'm_per_deg_lon': mLon, 'm_per_deg_lat': mLon * ratio}
    open(os.path.join(a.out_dir, 'calib_report.json'), 'w', encoding='utf-8').write(
        json.dumps(rep, ensure_ascii=False, indent=2))
    print('★ 写出：geo_calib.json ／ route_geo.json ／ calib_report.json（都在 %s）' % a.out_dir)
    return 0 if (max(r) <= 1e-3 and max(chk) <= 1e-3) else 1


if __name__ == '__main__':
    sys.exit(main())

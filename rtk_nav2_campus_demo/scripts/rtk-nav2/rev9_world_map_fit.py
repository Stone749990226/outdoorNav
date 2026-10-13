#!/usr/bin/env python3
"""rev9_world_map_fit.py —— 从 `trajectory.csv` 实测 **world → map** 的相似变换（阶段九第 3 步 ✓）

用法: rev9_world_map_fit.py <run目录> [--min-span 20]

★ 为什么必须实测（★ 这一步是本步"能不能走准"的关键 ✓）：
  · `navsat_transform` 走的是 **UTM** ✓，而 Gazebo 的 GPS 走的是 SDF `<spherical_coordinates>`
    的**局部投影** ✓ —— 两者之间差一个 **子午线收敛 γ** 与**尺度因子 k** ✗。
  · ★ navsat_transform 自己会把这个值**打在日志里** ✓：本轮的 datum 是 30.2241/120.2598，
    它报 `meridian convergence of -0.0240886 rad` = **-1.380°** ✓。
  · 1.38° 在 88.8 m 上就是 **2.14 m** ✗ ⇒ 远超 0.25 m 的容差 ✗ ⇒
    ★ **不修正的话，车会系统性地停偏** ✗（而且越远越偏 ✓）。
  · ⇒ 本工具用**权威产物** `trajectory.csv` 的 `gt_*`（Gazebo 真值 = **world 系** ✓）
    与 `est_*`（定位输出 = **map 系** ✓）**实测**这个变换 ✓ —— ✗ 不靠解析推导 ✗。

★ 判据（三条一起看 ✓）：
  ① 拟合残差 ≤ 5 cm ✓（残差大 ⇒ 不是纯相似变换 ⇒ 模型选错，别用 ✓）
  ② ★ **跨度**要够大 ✓：点都挤在一处时，旋转是**测不出来**的 ✗（杠杆臂太短 ✓）
     ⇒ 报出 gt 的包围盒跨度，跨度 < `--min-span` 就**明说"这次测不出旋转"** ✓
  ③ 报出 θ 与 navsat 日志里的 γ **对不对得上** ✓（同号、同量级 ⇒ 互证 ✓）

退出码: 0 = 拟合成功且残差达标 ｜ 1 = 残差过大 ｜ 2 = 读文件/用法错
"""
import csv
import math
import os
import sys


def fit_similarity(P, Q):
    n = len(P)
    mpx = sum(p[0] for p in P) / n
    mpy = sum(p[1] for p in P) / n
    mqx = sum(q[0] for q in Q) / n
    mqy = sum(q[1] for q in Q) / n
    a = b = 0.0
    den = 0.0
    for (px, py), (qx, qy) in zip(P, Q):
        dx, dy = px - mpx, py - mpy
        ex, ey = qx - mqx, qy - mqy
        a += dx * ex + dy * ey
        b += dx * ey - dy * ex
        den += dx * dx + dy * dy
    th = math.atan2(b, a)
    s = math.hypot(a, b) / den if den > 0 else float('nan')
    ct, st = math.cos(th), math.sin(th)
    tx = mqx - s * (ct * mpx - st * mpy)
    ty = mqy - s * (st * mpx + ct * mpy)
    return s, th, tx, ty


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return 2
    run = a[0]
    min_span = float(a[a.index('--min-span') + 1]) if '--min-span' in a else 20.0
    p = os.path.join(run, 'trajectory.csv')
    if not os.path.isfile(p):
        print('✗ 找不到 %s' % p)
        return 2
    import hashlib
    blob = open(p, 'rb').read()
    sha = hashlib.sha256(blob).hexdigest()
    rows = []
    with open(p, encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r['t']), float(r['gt_x']), float(r['gt_y']),
                             float(r['est_x']), float(r['est_y'])))
            except (KeyError, TypeError, ValueError):
                continue
    if len(rows) < 10:
        print('✗ 样本太少（%d）' % len(rows))
        return 2
    print('=== world → map 实测拟合 ===')
    print('  目录: %s' % run)
    print('  ★ 读的那份文件: trajectory.csv sha256=%s ｜ `wc -l`=%d ｜ 数据行=%d'
          % (sha[:16], blob.count(b'\n'), len(rows)))
    P = [(r[1], r[2]) for r in rows]
    Q = [(r[3], r[4]) for r in rows]
    s, th, tx, ty = fit_similarity(P, Q)
    res = []
    ct, st = math.cos(th), math.sin(th)
    for (px, py), (qx, qy) in zip(P, Q):
        x = s * (ct * px - st * py) + tx
        y = s * (st * px + ct * py) + ty
        res.append(math.hypot(x - qx, y - qy))
    res.sort()
    spanx = max(p[0] for p in P) - min(p[0] for p in P)
    spany = max(p[1] for p in P) - min(p[1] for p in P)
    print('  ---- 变换（map ≈ s·R(θ)·world + t ✓）----')
    print('    s (尺度) = %.8f  ⇒ 尺度偏差 %+.2f ppm' % (s, (s - 1) * 1e6))
    print('    θ (旋转) = %.6f 度 = %.6f rad  ★ navsat 日志里的 γ = -1.380 度（对符号与量级 ✓）'
          % (math.degrees(th), th))
    print('    t (平移) = (%.4f, %.4f) m' % (tx, ty))
    print('  ---- 残差 ----')
    print('    max = %.4f m ｜ p95 = %.4f m ｜ 中位 = %.4f m'
          % (res[-1], res[int(0.95 * len(res))], res[len(res) // 2]))
    print('  ---- 跨度（★ 决定"旋转测不测得出" ✓）----')
    print('    world 跨度: x=%.2f m  y=%.2f m  （判据 --min-span=%.1f m）' % (spanx, spany, min_span))
    if max(spanx, spany) < min_span:
        print('    ★★ 跨度不足 ⇒ **这次测不出旋转** ✗（杠杆臂太短 ✓）—— 下面那个 θ 别当结论 ✗')
    ok = res[-1] <= 0.05
    print('  ⇒ 残差判据(≤0.05 m): %s' % ('过 ✓' if ok else '不过 ✗（不是纯相似变换 ⇒ 别用）'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

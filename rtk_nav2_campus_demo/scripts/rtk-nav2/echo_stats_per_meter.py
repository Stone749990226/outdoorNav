#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""echo_stats_per_meter.py —— ★ 逐米"回波统计"对比（★ 纯离线 ✓，用已经采好的逐拍点云 ✓）

用法: echo_stats_per_meter.py <probe目录> [--max-seq N] [--out json]

★★ 为什么（reviewer 2026-10-11 (b)① ✓）★★
  ★ 要问"**在他们场景里，激光多看到了什么**" ✗ ⇒ ★ 把四轮的点云按**每米驱动**归一化后并排看 ✓。
★ 口径（都写死 ✓）：
  · **每米**：分母 = 相邻拍 `robot_xy_map` 位移之和 ✓（★ 与指标同一把尺子 ✓）
  · **回波数/米** = 点数 ÷ 米 ✓
  · **填充占比** = 到传感器距离 ≈ `7.9999`（无回波被填 ✓）的点占多少 ✓
  · **z 分档**（世界 z ✓）：<0.05 ／ 0.05–0.15 ／ 0.15–0.5 ／ 0.5–1.2 ／ 1.2–2.6 ／ >2.6 ✓
  · **半径分档**（到**传感器**✓）：0–1 ／ 1–2 ／ … ／ 7–8 ／ ≈8(填充) ✓
  ★ ✗ 只统计 `seq <= max_seq`（= 开车结束那一拍 ✓），否则车停后的空拍会稀释每米量 ✓
"""
import argparse
import glob
import json
import math
import os
import re
import sys

FAR = 7.9999
ZBINS = [(0.0, 0.05), (0.05, 0.15), (0.15, 0.5), (0.5, 1.2), (1.2, 2.6), (2.6, 1e9)]
RBINS = [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 5.0), (5.0, 6.0), (6.0, 7.0), (7.0, 7.9)]


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, default=None)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    P = a.probe_dir

    ticks = sorted(glob.glob(os.path.join(P, 'tick_*.json')), key=seq_of)
    if a.max_seq is not None:
        ticks = [t for t in ticks if seq_of(t) <= a.max_seq]
    traj = 0.0
    prev = None
    for f in ticks:
        r = json.load(open(f, encoding='utf-8')).get('robot_xy_map')
        if r and prev:
            traj += math.hypot(r[0] - prev[0], r[1] - prev[1])
        if r:
            prev = r

    def collect(prefix):
        n_pts = n_far = 0
        zb = [0] * len(ZBINS)
        rb = [0] * len(RBINS)
        for f in ticks:
            s = seq_of(f)
            xyz = os.path.join(P, '%s_%04d.xyz' % (prefix, s))
            if not os.path.isfile(xyz):
                continue
            t = json.load(open(f, encoding='utf-8'))
            M = ((t.get('scan') or {}).get('tf') or {}).get('M')
            org = (M[0][3], M[1][3], M[2][3]) if M else None
            for line in open(xyz, encoding='utf-8'):
                v = line.split()
                if len(v) != 3:
                    continue
                x, y, z = float(v[0]), float(v[1]), float(v[2])
                n_pts += 1
                r = math.dist((x, y, z), org) if org else None
                if r is not None:
                    if abs(r - FAR) < 0.05:
                        n_far += 1
                    else:
                        for i, (lo, hi) in enumerate(RBINS):
                            if lo <= r < hi:
                                rb[i] += 1
                                break
                for i, (lo, hi) in enumerate(ZBINS):
                    if lo <= z < hi:
                        zb[i] += 1
                        break
        return {'n_points': n_pts, 'n_far_fill': n_far,
                'frac_far_fill': (n_far / n_pts) if n_pts else None,
                'points_per_m': (n_pts / traj) if traj > 1e-9 else None,
                'z_bins': {('<%.2f' % hi if lo == 0 else '%.2f-%.2f' % (lo, hi)): c for (lo, hi), c in zip(ZBINS, zb)},
                'radius_bins': {'%.0f-%.0f' % (lo, hi): c for (lo, hi), c in zip(RBINS, rb)}}

    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'n_ticks': len(ticks),
           'traj_len_m': round(traj, 3),
           'scan': collect('scan'), 'cam': collect('cam'),
           'far_fill_value': FAR,
           'convention': ('★ 每米 = 点数 ÷ 相邻拍位移之和 ✓；★ z 是**世界 z** ✓；'
                          '★ 半径 = 到**传感器**（`scan.tf.M` 的平移 ✓）；'
                          '★ 填充 = |r − 7.9999| < 0.05 ✓；★ 只统计 seq ≤ max_seq ✓')}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    s = out['scan']
    print('  ★ %-42s 拍 %4d ｜ 米 %7.2f ｜ 激光 %8d 点（%.1f/米 ✓）｜ 填充 %.1f%%'
          % (os.path.basename(P.rstrip('/')), len(ticks), traj, s['n_points'],
             s['points_per_m'] or 0, 100.0 * (s['frac_far_fill'] or 0)))
    print('     z 分档：' + ' ｜ '.join('%s:%d' % (k, v) for k, v in s['z_bins'].items()))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""near_range_source.py —— ★ 定"近距回波（≤0.5 m）"的来源：**地面** 还是 **车自己**？✓（纯离线）

用法: near_range_source.py <probe目录> --max-seq N --out json [--near 0.5]

★★ 为什么（reviewer 2026-10-11 A ✓）★★
  ★ 早先"逐米回波统计"里 ★ **近距档（0.15–0.50 m）他们 res2 = 8.7／拍 vs 我们 = 0.0／拍** ✓，
    ★ 而它**至今没被解释** ✓ ⇒ ★ ①（清除不是差别）之后，★ 差别只能落在"**什么被标上**"这一侧 ✓
    ⇒ ★ 而"车身边/脚下那点东西被看到"正好是**标记侧**的事 ✓。
★ **两头各证一次** ✓（★ 判据两头都算结论 ✓）：
  · ★ **地面** ⇒ ★ 点应落在 ★ **z ≈ 地面高度**（★ 与场景地面 z 比 ✓）★ 且 ★ **方位铺满** ✓
  · ★ **车自己** ⇒ ★ 点应落在 ★ **车体轮廓**（★ 半径档 ~0.07–0.14 m ✓）★ 且 ★ **只在固定方位扇区** ✓
  · ★ 都不是 ⇒ ★ 如实写"**另有来源**" ✓ ＋ ★ 落盘原始点清单 ✓
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--near', type=float, default=0.5)
    ap.add_argument('--pos-source', default='map', choices=['map', 'gt'], help='gt = use ground-truth pose from trajectory.csv (joined by sim_t)')
    a = ap.parse_args()
    P = a.probe_dir

    gt = {}
    if a.pos_source == 'gt':
        csvp = os.path.join(os.path.dirname(os.path.abspath(P)), 'trajectory.csv')
        if os.path.isfile(csvp):
            for row in csv.DictReader(open(csvp, encoding='utf-8')):
                try:
                    gt[round(float(row['t']), 1)] = (float(row['gt_x']), float(row['gt_y']))
                except Exception:
                    pass
    ticks = sorted([t for t in glob.glob(os.path.join(P, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(P, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    # ★ 抽样拍（为控时间 ✓）；口径写清 ✓
    stride = max(1, len(ticks) // 120)
    samp = ticks[::stride]

    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'near_m': a.near,
           'n_ticks_total': len(ticks), 'tick_stride': stride, 'pos_source': a.pos_source, 'n_ticks_sampled': len(samp),
           'convention': ('★ 近距 = **车心（`robot_xy_map`）水平距离 ≤ %.2f m** ✓；'
                          '★ 方位 = 相对**该拍车头朝向**（由该云 TF 的偏航角取 ✓）的方位角 ✓，36 桶（10°）✓；'
                          '★ "地面高度" 用该拍车的 z 基线：★ 世界地面顶面 = **z 0.0** ✓（两套世界都是 ✓，见另一件 ✓）' % a.near)}
    for cloud in ('scan', 'cam'):
        cnt = 0
        perbin = collections.defaultdict(lambda: {'n': 0, 'z': [], 'az': set()})
        zs = collections.Counter()
        az = collections.Counter()
        rad = collections.Counter()
        raw = []
        zlist = []
        for t in samp:
            s = seq_of(t)
            f = os.path.join(P, '%s_%04d.xyz' % (cloud, s))
            if not os.path.isfile(f):
                continue
            d = json.load(open(t, encoding='utf-8'))
            r = d.get('robot_xy_map')
            M = ((d.get(cloud) or {}).get('tf') or {}).get('M')
            if not r or not M:
                continue
            cx, cy = r
            if a.pos_source == 'gt':
                g = gt.get(round(d.get('sim_t', -1), 1))
                if g is None:
                    continue
                cx, cy = g
            yaw = math.atan2(M[1][0], M[0][0])          # ★ 该云 TF 的偏航 ✓（与车刚性连接 ✓）
            for line in open(f, encoding='utf-8'):
                v = line.split()
                if len(v) != 3:
                    continue
                x, y, z = map(float, v)
                d2 = math.hypot(x - cx, y - cy)
                if d2 > a.near:
                    continue
                cnt += 1
                zlist.append(z)
                zs[round(z, 2)] += 1
                rb = ('0-0.10' if d2 < 0.10 else '0.10-0.15' if d2 < 0.15 else
                      '0.15-0.30' if d2 < 0.30 else '0.30-0.50')
                ang = (math.degrees(math.atan2(y - cy, x - cx)) - math.degrees(yaw)) % 360.0
                perbin[rb]['z'].append(z)
                perbin[rb]['az'].add(int(ang // 10) * 10)
                perbin[rb]['n'] += 1
                az[int(ang // 10) * 10] += 1
                rad[('0-0.10' if d2 < 0.10 else '0.10-0.15' if d2 < 0.15 else
                     '0.15-0.30' if d2 < 0.30 else '0.30-0.50')] += 1
                if len(raw) < 300:
                    raw.append([round(x, 3), round(y, 3), round(z, 3), round(d2, 3), round(ang, 1)])
        zlist.sort()
        q = lambda p: round(zlist[min(len(zlist) - 1, int(len(zlist) * p))], 3) if zlist else None
        nz = len(zlist)
        out[cloud] = {
            'n_points_total_sampled': cnt,
            'per_tick': round(cnt / max(1, len(samp)), 3),
            'z': {'p10': q(.1), 'median': q(.5), 'p90': q(.9),
                  'frac_within_0.05_of_ground': (sum(1 for z in zlist if abs(z) <= 0.05) / nz) if nz else None,
                  'frac_below_0.02': (sum(1 for z in zlist if z <= 0.02) / nz) if nz else None},
            'azimuth_10deg_bins': dict(sorted(az.items())),
            'azimuth_bins_occupied': len(az),
            'azimuth_frac_top3_bins': (sum(c for _k, c in az.most_common(3)) / max(1, cnt)),
            'radius_bins': dict(rad),
            'by_radius_bin': {b: {'n': v['n'],
                                  'z_median': (sorted(v['z'])[len(v['z']) // 2] if v['z'] else None),
                                  'z_p10': (sorted(v['z'])[len(v['z']) // 10] if v['z'] else None),
                                  'z_p90': (sorted(v['z'])[len(v['z']) * 9 // 10] if v['z'] else None),
                                  'azimuth_bins_occupied': len(v['az'])}
                              for b, v in sorted(perbin.items())},
            'raw_sample': raw,
        }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    for cloud in ('scan', 'cam'):
        v = out[cloud]
        print('  === %s：近距（≤%.2f m）每拍 %.2f 个（抽样 %d 拍 ✓）'
              % (cloud, a.near, v['per_tick'], len(samp)))
        print('     ★ z：中位 %s ｜ p10 %s ｜ p90 %s ｜ 贴地(|z|≤0.05) %.1f%% ｜ z≤0.02 占 %.1f%%'
              % (v['z']['median'], v['z']['p10'], v['z']['p90'],
                 100 * (v['z']['frac_within_0.05_of_ground'] or 0), 100 * (v['z']['frac_below_0.02'] or 0)))
        for b, bv in sorted(v['by_radius_bin'].items()):
            print('        半径 %-9s n=%6d  ★ z 中位 %-7s (p10 %-7s p90 %-7s)  方位桶 %d/36'
                  % (b, bv['n'], bv['z_median'], bv['z_p10'], bv['z_p90'], bv['azimuth_bins_occupied']))
        print('     ★ 方位：占 %d/36 个 10°桶 ｜ 前三桶占 %.1f%% ｜ 半径档 %s'
              % (v['azimuth_bins_occupied'], 100 * v['azimuth_frac_top3_bins'], v['radius_bins']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

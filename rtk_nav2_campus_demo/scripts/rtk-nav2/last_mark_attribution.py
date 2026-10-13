#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""last_mark_attribution.py —— ★ "活到最后那次标记"发生在**离终点还剩多少米** ✓（排"删失"）

用法: last_mark_attribution.py <probe目录> --max-seq N --out json [--radii 0.105,0.50]

★★ 为什么要排（reviewer 2026-10-11 第②件 ✓）★★
  ★ "粘住"里**一大块是"清掉过又被重标"** ✓（73–88% ✓）⇒ ★ 若那些**重标就发生在最后几米** ✗
    ⇒ ★ 它们只是**来不及清** ✓（★ 删失 ✓）⇒ ★ **主指标会被高估** ✗ ⇒ 必须先把这部分量出来 ✓。
★ 口径 ✓：★ "最后一次标记" = ★ 最后一次"**由非致命变致命**"的那一拍 ✓（= 活到末拍的那次 ✓）；
　★ "还剩多少米" = ★ 总弧长 − 该拍处弧长 ✓；★ "那一拍有没有点" 与 `sticky_source_check.py` **同一把尺** ✓（≤0.15 m ✓）。
"""
import argparse
import glob
import json
import math
import os
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def densify(pts, step=0.05):
    out = []
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / step))
        for j in range(n):
            t = j / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    if pts:
        out.append(pts[-1])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--radii', default='0.105,0.50')
    ap.add_argument('--near', type=float, default=0.15)
    a = ap.parse_args()
    R = [float(x) for x in a.radii.split(',')]
    P = a.probe_dir

    ticks = sorted([t for t in glob.glob(os.path.join(P, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(P, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    meta = json.load(open(ticks[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    maps = {seq_of(t): os.path.join(P, 'map_%04d.bin' % seq_of(t)) for t in ticks}
    seqs = [seq_of(t) for t in ticks]
    b0 = open(maps[seqs[0]], 'rb').read()
    b1 = open(maps[seqs[-1]], 'rb').read()
    H = len(b1) // W
    L = lambda v: 99 <= v <= 100
    added = {k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])}
    pts, cum = [], [0.0]
    for t in ticks:
        r = json.load(open(t, encoding='utf-8')).get('robot_xy_map')
        if r:
            pts.append(tuple(r))
            if len(pts) > 1:
                cum.append(cum[-1] + math.dist(pts[-1], pts[-2]))
    dense = densify(pts, 0.05)
    xy = lambda k: (ox + ((k % W) + .5) * res, oy + ((k // W) + .5) * res)
    dtraj = lambda k: min(math.hypot(xy(k)[0] - px, xy(k)[1] - py) for px, py in dense)
    total = cum[-1]

    LETH = re.compile(rb'[\x63\x64]')
    sets = {s: {m.start() for m in LETH.finditer(open(maps[s], 'rb').read())} for s in seqs}

    # ★ 最后一次"由非致命变致命"的那一拍 ✓
    last_mark = {}
    prev = None
    for s in seqs:
        cur = sets[s]
        if prev is not None:
            for k in cur - prev:
                last_mark[k] = s          # ★ 每遇到一次就覆盖 ⇒ 最后留下的就是**最后一次** ✓
        prev = cur

    # 逐拍点云索引（只为"那一拍有没有点" ✓ —— 与 sticky_source_check 同尺 ✓）
    idx = {}
    for cloud in ('scan', 'cam'):
        idx[cloud] = {}
        for t in ticks:
            s = seq_of(t)
            f = os.path.join(P, '%s_%04d.xyz' % (cloud, s))
            if not os.path.isfile(f):
                continue
            d = json.load(open(t, encoding='utf-8'))
            M = ((d.get(cloud) or {}).get('tf') or {}).get('M')
            if not M:
                continue
            org = (M[0][3], M[1][3], M[2][3])
            m = {}
            for line in open(f, encoding='utf-8'):
                v = line.split()
                if len(v) != 3:
                    continue
                x, y, z = map(float, v)
                gi, gj = int((x - ox) / res), int((y - oy) / res)
                for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
                    k = (gj + dj) * W + (gi + di)
                    if 0 <= k < len(b1):
                        dd = math.hypot(xy(k)[0] - x, xy(k)[1] - y)
                        if k not in m or dd < m[k][0]:
                            m[k] = (dd, math.dist((x, y, z), org))
            idx[cloud][s] = m

    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'radii_m': R,
           'total_arc_m': round(total, 3), 'near_m': a.near, 'n_ticks': len(seqs),
           'convention': ('★ "最后一次标记" = 最后一次由非致命变致命的那一拍 ✓（= 活到末拍那次 ✓）；'
                          '★ "还剩多少米" = 总弧长 − 该拍处弧长 ✓；'
                          '★ "那一拍有点" = 到格心 ≤ %.2f m（与 sticky_source_check 同尺 ✓）' % a.near)}
    for r in R:
        swept = set()
        for px, py in dense:
            i0, i1 = int((px - r - ox) / res), int((px + r - ox) / res)
            j0, j1 = int((py - r - oy) / res), int((py + r - oy) / res)
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    if 0 <= i < W and 0 <= j < H:
                        k = j * W + i
                        if math.hypot(xy(k)[0] - px, xy(k)[1] - py) <= r:
                            swept.add(k)
        sticky = sorted(swept & added)
        remain, in_last2m, in_last5m, lasttick = [], 0, 0, 0
        hit_scan = hit_cam = 0
        for k in sticky:
            s = last_mark.get(k)
            if s is None:
                continue
            rem = total - cum[seqs.index(s)]
            remain.append(rem)
            if rem <= 2.0:
                in_last2m += 1
            if rem <= 5.0:
                in_last5m += 1
            if s == seqs[-1]:
                lasttick += 1
            if (idx['scan'].get(s) or {}).get(k, (9, None))[0] <= a.near:
                hit_scan += 1
            if (idx['cam'].get(s) or {}).get(k, (9, None))[0] <= a.near:
                hit_cam += 1
        n = len(remain)

        def q(p):
            v = sorted(remain)
            return round(v[min(len(v) - 1, int(len(v) * p))], 3) if v else None
        out['R=%.3f' % r] = {
            'n_sticky': len(sticky), 'n_with_last_mark': n,
            'remaining_m': {'min': round(min(remain), 3) if remain else None, 'p10': q(.1),
                            'median': q(.5), 'p90': q(.9), 'max': round(max(remain), 3) if remain else None},
            'n_in_last_2m': in_last2m, 'frac_in_last_2m': in_last2m / max(1, n),
            'n_in_last_5m': in_last5m, 'frac_in_last_5m': in_last5m / max(1, n),
            'n_last_mark_at_final_tick': lasttick,
            'n_last_mark_tick_has_scan_point': hit_scan, 'n_last_mark_tick_has_cam_point': hit_cam,
        }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 总弧长 %.1f m ｜ 拍 %d' % (total, len(seqs)))
    for r in R:
        v = out['R=%.3f' % r]
        print('     R=%.3f ⇒ 粘住 %3d ｜ ★ 最后一次标记时**还剩** 中位 %s m（p10 %s ｜ min %s）｜ '
              '★ 最后 2 m 内 %d（%.1f%%）／最后 5 m 内 %d（%.1f%%）｜ 就在末拍 %d ｜ 那拍有点 scan %d／cam %d'
              % (r, v['n_sticky'], v['remaining_m']['median'], v['remaining_m']['p10'],
                 v['remaining_m']['min'], v['n_in_last_2m'], 100 * v['frac_in_last_2m'],
                 v['n_in_last_5m'], 100 * v['frac_in_last_5m'], v['n_last_mark_at_final_tick'],
                 v['n_last_mark_tick_has_scan_point'], v['n_last_mark_tick_has_cam_point']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

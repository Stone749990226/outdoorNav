#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""survival_analysis.py —— ★ "标记 → 清除"的**存活**(★ 拍 与 米 两个单位 ✓，★ 删失单列 ✓)

用法: survival_analysis.py <probe目录> --max-seq N --out json [--radii 0.105,0.50]

★★ 要量什么（reviewer 2026-10-11 最后一件 ✓）★★
  ★ 对**每一格"曾被标成致命"的格** ⇒ ★ **第一次被标上的那一拍** ＋ ★ **它变回非致命的那一拍** ✓
  ★ **必须限定在可比空间** ✗：★ 只取 **离轨迹 ≤ R** 的格 ✓（★ 否则又是"地形不同"那个坑 ✓）
  ★ **两个单位都报** ✓：★ 拍（★ 探针同 2.0 s 间隔 ✓）＋ 米（★ 车走过的距离 ✓）
    —— ★ 两轮车速不同 ⇒ 两个单位**不可互推** ✓ ⇒ 口径都要写 ✓
★★★ **删失单列** ✗（★ 本轮的关键纪律 ✓）★★★
  ★ "**在观测窗内从未被清除**"的格 ⇒ ★ 单列 `n_never_cleared` ✓ ＋ 观测窗长度 ✓
  ★ **不许并进中位数** ✗ —— ★ 并进去会把"**永远不清**"洗成"**清得慢**" ✓
"""
import argparse
import collections
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

    # 逐拍致命集合 ✓（稀疏正则 ✓）
    LETH = re.compile(rb'[\x63\x64]')
    sets = {s: {m.start() for m in LETH.finditer(open(maps[s], 'rb').read())} for s in seqs}

    # 轨迹 ＋ 累计里程 ✓（★ 米 这个单位的基础 ✓）
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

    # ★ 第一次被标上 / 第一次被清掉 ✓（★ 只看**第一段** ✓）
    first_mark, first_clear = {}, {}
    prev = None
    for i, s in enumerate(seqs):
        cur = sets[s]
        if prev is not None:
            for k in cur - prev:
                first_mark.setdefault(k, s)
            for k in prev - cur:
                if k in first_mark and k not in first_clear:
                    first_clear[k] = s
        prev = cur

    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'radii_m': R,
           'probe_interval_s': 2.0, 'n_ticks': len(seqs),
           'window': {'first_seq': seqs[0], 'last_seq': seqs[-1]},
           'convention': ('★ 只取 **离轨迹 ≤ R** 的格 ✓（★ 可比空间 ✓）；'
                          '★ 两个单位：**拍**（探针 2.0 s/拍 ✓）与**米**（车走过的弧长 ✓）—— '
                          '★ 两轮车速不同 ⇒ **不可互推** ✗；'
                          '★ **从未被清除**的格 **单列** ✓，✗ 不并进中位数 ✓')}
    for r in R:
        cells = []
        for k in first_mark:
            if dtraj(k) <= r:
                cells.append(k)
        surv_t, surv_m = [], []
        never = []
        for k in cells:
            s0 = first_mark[k]
            i0 = seqs.index(s0)
            if k in first_clear:
                s1 = first_clear[k]
                i1 = seqs.index(s1)
                surv_t.append(i1 - i0)                       # ★ 单位：拍 ✓
                surv_m.append(round(cum[i1] - cum[i0], 3))   # ★ 单位：米 ✓
            else:
                never.append(round(cum[-1] - cum[i0], 3))    # ★ 删失：只记"到窗口末还有多长" ✓

        def q(v, p):
            v = sorted(v)
            return v[min(len(v) - 1, int(len(v) * p))] if v else None
        out['R=%.3f' % r] = {
            'n_cells_ever_marked_in_R': len(cells),
            'n_cleared': len(surv_t), 'n_never_cleared': len(never),
            'frac_never_cleared': len(never) / max(1, len(cells)),
            'survival_ticks': {'median': q(surv_t, .5), 'p10': q(surv_t, .1), 'p90': q(surv_t, .9),
                               'max': max(surv_t) if surv_t else None},
            'survival_seconds_at_2s_per_tick': {'median': (q(surv_t, .5) or 0) * 2.0},
            'survival_m': {'median': q(surv_m, .5), 'p10': q(surv_m, .1), 'p90': q(surv_m, .9),
                           'max': max(surv_m) if surv_m else None},
            'never_cleared_remaining_m': {'median': q(never, .5), 'p10': q(never, .1),
                                          'max': max(never) if never else None},
        }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 拍 %d ｜ 轨迹 %.1f m ｜ 间隔 2.0 s ✓' % (len(seqs), cum[-1]))
    for r in R:
        v = out['R=%.3f' % r]
        print('     R=%.3f ⇒ 曾被标 %5d ｜ ★ 清掉 %5d ｜ ★ **从未清** %5d（%.1f%%）｜ '
              '存活中位 **%s 拍（=%.0f s）／%s m** ｜ 删失者末了还剩 %s m'
              % (r, v['n_cells_ever_marked_in_R'], v['n_cleared'], v['n_never_cleared'],
                 100 * v['frac_never_cleared'], v['survival_ticks']['median'],
                 v['survival_seconds_at_2s_per_tick']['median'], v['survival_m']['median'],
                 v['never_cleared_remaining_m']['median']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

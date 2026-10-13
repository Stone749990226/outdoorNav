#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""clearing_opportunities.py —— ★ "清除机会"普查：每格被多少条光束**穿过** ✓（纯离线 ✓）

用法: clearing_opportunities.py <probe目录> --max-seq N --out json
        [--radii 0.105,0.50] [--target sticky|swept] [--max-targets 260] [--tick-stride 4]

★★ 为什么（reviewer 2026-10-11 第①件 ✓）★★
  ★ 清除**不是到点自动清** ✗ ⇒ ★ 得**有射线穿过那一格**才清得掉 ✓
    ⇒ ★ 先量"**这一格到底被穿过几次**" = 清除机会 ✓。
★ **"光束穿过该格"怎么判（★ 口径写死、并说明为什么 ✓）**：
  · ★ 格是 **0.1×0.1 m** ✓ ⇒ ★ 判据取 ★ **格心到线段（原点→端点）的距离 ≤ 0.05 m**（= 半格 ✓）
    且 ★ **垂足落在线段内**（= 端点越过该格 ✓）
  · ★ 为什么取 0.05 而✗不是更宽 ✓：★ 这就是"**这条射线真的从这一格里过**"的自然判据 ✓
    （★ 0.05 m 之外的射线只是**擦到格的边** ✓，✗ 不该算穿过 ✓）；★ 另给 **0.10 m** 一档做**灵敏度** ✓（★ 不拿它当口径 ✓）
  · ★ **两个点云都算** ✓（★ 激光 360 束 ＋ 深度 ~38400 点 ✓ —— ★ 清除射线来自**两者的每个端点** ✓）
  · ★ 只在 ★ **车距该格 ≤ `raytrace_max_range`（8.0 m）** 的拍里数 ✓（★ 超出就根本射不到 ✓）
★ **归一** ✓：★ **每拍** ✓ 与 **每米（采样弧长 ✓）** ✓ **都报** ✓ —— ★ 两轮车速不同 ⇒ ✗ 不互推 ✓
★ **采样** ✓：★ 每 `--tick-stride` 拍取 1 拍 ✓（★ 为控时间 ✓）⇒ ★ 归一用**采样到的弧长/拍数** ✓（口径写清 ✓）
"""
import argparse
import collections
import glob
import json
import math
import os
import random
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


def seg_dist_inside(px, py, qx, qy, cx, cy):
    dx, dy = qx - px, qy - py
    L = dx * dx + dy * dy
    if L <= 1e-12:
        return math.hypot(cx - px, cy - py), True
    t = ((cx - px) * dx + (cy - py) * dy) / L
    if not (0.0 < t < 1.0):
        return 1e9, False
    return math.hypot(cx - (px + t * dx), cy - (py + t * dy)), True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--radii', default='0.105,0.50')
    ap.add_argument('--target', default='sticky', choices=['sticky', 'swept'])
    ap.add_argument('--max-targets', type=int, default=260)
    ap.add_argument('--tick-stride', type=int, default=4)
    ap.add_argument('--ray-max', type=float, default=8.0)
    ap.add_argument('--step-m', type=float, default=0.25)
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
    pos = {}
    pts, cum = [], [0.0]
    for t in ticks:
        r = json.load(open(t, encoding='utf-8')).get('robot_xy_map')
        if r:
            pos[seq_of(t)] = tuple(r)
            pts.append(tuple(r))
            if len(pts) > 1:
                cum.append(cum[-1] + math.dist(pts[-1], pts[-2]))
    arc_of = {seqs[i]: cum[i] for i in range(len(seqs))}
    dense = densify(pts, 0.05)
    xy = lambda k: (ox + ((k % W) + .5) * res, oy + ((k // W) + .5) * res)
    dtraj = lambda k: min(math.hypot(xy(k)[0] - px, xy(k)[1] - py) for px, py in dense)

    rnd = random.Random(20261011)
    free = [k for k in range(len(b1)) if b1[k] == 0]
    fs = rnd.sample(free, min(len(free), 8000))
    fs_d = {k: dtraj(k) for k in fs}

    samp = seqs[::a.tick_stride]
    samp_arc = (arc_of[samp[-1]] - arc_of[samp[0]]) if len(samp) > 1 else 0.0

    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'radii_m': R,
           'target_kind': a.target, 'tick_stride': a.tick_stride, 'n_ticks_sampled': len(samp),
           'sampled_arc_m': round(samp_arc, 3), 'ray_max_m': a.ray_max, 'step_m': a.step_m,
           'convention': ('★ "光束穿过该格" = 格心到线段（该拍原点→该端点）距离 ≤ **0.05 m**（= 半格 ✓）'
                          '且垂足在线段内（端点越过 ✓）；★ 0.10 m 只作**灵敏度** ✗ 不当口径 ✓；'
                          '★ 两个点云都算 ✓（清除射线来自两者的每个端点 ✓）；'
                          '★ 只在车距该格 ≤ raytrace_max_range（8.0 m）的拍里数 ✓；'
                          '★ 采样：每 %d 拍取 1 拍 ✓ ⇒ 归一的弧长/拍数都用**采样到的那部分** ✓；'
                          '★ 速度不同 ⇒ "每拍"与"每米"✗ 不互推 ✓' % a.tick_stride)}
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
        if a.target == 'sticky':
            targets = sorted(swept & added)
        else:
            targets = sorted(swept)
        if len(targets) > a.max_targets:
            targets = sorted(rnd.sample(targets, a.max_targets))
        tset = set(targets)
        # ★ 对照也得一起数 ✗（★ 否则它们根本不在索引里 ⇒ 拿不到计数 ✗）
        #   ⇒ ★ 用「目标 ∪ 对照」建索引 ✓，再分开聚合 ✓
        ctrl = []
        bins = collections.Counter(int(dtraj(k) / 0.25) for k in targets)
        pool = collections.defaultdict(list)
        for k in fs:
            pool[int(fs_d[k] / 0.25)].append(k)
        for b, need in bins.items():
            p2 = pool.get(b) or []
            if p2:
                ctrl += [p2[rnd.randrange(len(p2))] for _ in range(need)]
        ctrl = [k for k in ctrl if k not in tset]
        allcells = sorted(tset | set(ctrl))
        tcoarse = collections.defaultdict(list)
        for k in allcells:
            x, y = xy(k)
            tcoarse[(int(x / 0.5), int(y / 0.5))].append(k)

        cnt = {k: {'scan': 0, 'cam': 0, 'loose': 0} for k in allcells}
        for s in samp:
            if s not in pos:
                continue
            cxp, cyp = pos[s]
            for cloud in ('scan', 'cam'):
                f = os.path.join(P, '%s_%04d.xyz' % (cloud, s))
                if not os.path.isfile(f):
                    continue
                d = json.load(open(os.path.join(P, 'tick_%04d.json' % s), encoding='utf-8'))
                M = ((d.get(cloud) or {}).get('tf') or {}).get('M')
                if not M:
                    continue
                ox_, oy_ = M[0][3], M[1][3]
                for line in open(f, encoding='utf-8'):
                    v = line.split()
                    if len(v) != 3:
                        continue
                    ex, ey = float(v[0]), float(v[1])
                    hits = set()
                    seg = math.hypot(ex - ox_, ey - oy_)
                    n = max(1, int(seg / a.step_m))
                    for i2 in range(n + 1):
                        t = i2 / n
                        mx, my = ox_ + (ex - ox_) * t, oy_ + (ey - oy_) * t
                        for k in tcoarse.get((int(mx / 0.5), int(my / 0.5)), ()):
                            if k in hits:
                                continue
                            cx, cy = xy(k)
                            if math.hypot(cx - cxp, cy - cyp) > a.ray_max:
                                continue
                            dd, ins = seg_dist_inside(ox_, oy_, ex, ey, cx, cy)
                            if ins and dd <= 0.05:
                                cnt[k][cloud] += 1
                                hits.add(k)
                            elif ins and dd <= 0.10:
                                cnt[k]['loose'] += 1
                                hits.add(k)

        def agg(cells):
            cells = list(cells)
            if not cells:
                return {'n': 0}
            tot = [cnt[k]['scan'] + cnt[k]['cam'] for k in cells]
            loose = [cnt[k]['scan'] + cnt[k]['cam'] + cnt[k]['loose'] for k in cells]
            tot.sort(); loose.sort()
            q = lambda v, p: v[min(len(v) - 1, int(len(v) * p))]
            return {'n': len(cells),
                    'total_median': q(tot, .5), 'total_p10': q(tot, .1), 'total_p90': q(tot, .9),
                    'zero_frac': sum(1 for v in tot if v == 0) / len(tot),
                    'per_tick_median': round(q(tot, .5) / max(1, len(samp)), 4),
                    'per_m_median': round(q(tot, .5) / max(1e-9, samp_arc), 4),
                    'loose_0.10m_median': q(loose, .5)}
        out['R=%.3f' % r] = {'n_targets': len(targets), 'n_control': len(ctrl),
                             'cells_indexed': len(allcells),
                             'targets': agg(targets),
                             'control_free_same_distance': agg(ctrl)}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 采样 %d 拍（每 %d 拍 1 拍）｜ 采样弧长 %.1f m' % (len(samp), a.tick_stride, samp_arc))
    for r in R:
        v = out['R=%.3f' % r]
        t, c = v['targets'], v['control_free_same_distance']
        print('     R=%.3f 对象 %d 格（%s）' % (r, v['n_targets'], a.target))
        for nm, p in (('对象', t), ('同距离自由对照', c)):
            if not p.get('n'):
                print('        %-16s （空）' % nm); continue
            print('        %-16s n=%4d ｜ ★ 机会中位 %6d（每拍 %.3f ／ 每米 %.2f）｜ 零机会占比 %.1f%% ｜ 0.10m 档 %d'
                  % (nm, p['n'], p['total_median'], p['per_tick_median'], p['per_m_median'],
                     100 * p['zero_frac'], p['loose_0.10m_median']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

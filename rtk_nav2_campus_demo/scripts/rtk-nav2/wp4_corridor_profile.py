#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wp4_corridor_profile.py —— ★ 走廊沿纵向分段看：★ **车前方那条道，有没有被"堵死"** ✓

用法: wp4_corridor_profile.py <run_dir> <wp4_x> <wp4_y> [--out <json>]

★★ 为什么再切一刀 ★★
  `wp4_stuck_probe.py` 把"车→wp4 的走廊"整体计数 ✓ —— ★ 但走廊**含车自己那一段** ✗
  （★ 已知：车会把自己周围标脏 ✓）⇒ ★ "走廊里有 10 个致命格"**分不清**是：
    · ★ 车脚下那一圈（★ 自己标的 ✓，对"能否前进"**不构成路障** ✓ —— ★ 车已经在那儿了 ✓）
    · ★ **车前方**那条道被堵 ✗（★ 那才是"规划器无路可走" ✓）
  ⇒ ★ 本脚本按**纵向**（沿车→wp4）分段 ✓、只看**车前方** `--skip` m 之外 ✓，
    ★ 并判"这一段横截面是不是**整条**被堵" ✓（★ 全堵 ⇒ 过不去 ✓；★ 只堵一侧 ⇒ 绕得开 ✓）。

★ 判词 ✓：✗ 不替规划器说话 ✗ —— ★ 只报"图里那条道通不通" ✓。
"""
import json
import math
import os
import re
import sys

LETHAL = lambda v: 99 <= v <= 100
FREE = lambda v: v == 0
UNKNOWN = lambda v: v == 255
INFLATED = lambda v: 1 <= v <= 98


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def cell_of(x, y, m):
    ix = int(math.floor((x - m['ox']) / m['res']))
    iy = int(math.floor((y - m['oy']) / m['res']))
    if 0 <= ix < m['W'] and 0 <= iy < m['H']:
        return iy * m['W'] + ix
    return None


def traj(run_dir):
    rows = []
    with open(os.path.join(run_dir, 'trajectory.csv'), encoding='utf-8') as f:
        next(f)
        for ln in f:
            a = ln.strip().split(',')
            if len(a) < 3 or not a[1] or not a[2]:
                continue
            try:
                rows.append((float(a[0]), float(a[1]), float(a[2])))
            except ValueError:
                pass
    return rows


def car_at(rows, t):
    lo, hi = 0, len(rows) - 1
    if t < rows[0][0]:
        return rows[0][1], rows[0][2]
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if rows[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return rows[lo][1], rows[lo][2]


def main():
    run_dir, wx, wy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    tol, skip, band = 0.45, 0.7, 0.5
    out_path = sys.argv[sys.argv.index('--out') + 1] if '--out' in sys.argv else None
    rows = traj(run_dir)
    probe = os.path.join(run_dir, 'probe')
    js = sorted([f for f in os.listdir(probe) if re.match(r'map_\d+\.json$', f)], key=seq_of)

    recs = []
    for j in js:
        meta = json.load(open(os.path.join(probe, j), encoding='utf-8'))
        m = {'stamp': float(meta['stamp']), 'res': float(meta['resolution']),
             'W': int(meta['width']), 'H': int(meta['height']),
             'ox': float(meta['origin']['x']), 'oy': float(meta['origin']['y'])}
        binp = os.path.join(probe, j.replace('.json', '.bin'))
        if not os.path.exists(binp):
            continue
        blob = open(binp, 'rb').read()
        if len(blob) != m['W'] * m['H']:
            continue
        cx, cy = car_at(rows, m['stamp'])
        dx, dy = wx - cx, wy - cy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            continue
        ux, uy = dx / d, dy / d
        # ★ 屏幕侧向单位向量 ✓
        px, py = -uy, ux
        nlat = int(tol / m['res'])
        nlong = int(d / m['res'])
        bands = []
        for il in range(nlong + 1):
            s = il * m['res']
            if s < skip:
                continue
            row = []
            for it in range(-nlat, nlat + 1):
                x = cx + ux * s + px * (it * m['res'])
                y = cy + uy * s + py * (it * m['res'])
                k = cell_of(x, y, m)
                row.append(blob[k] if k is not None else None)
            bands.append((s, row))
        # ★ 纵向分段汇总 ✓
        segs = {}
        for s, row in bands:
            b = int((s - skip) // band)
            e = segs.setdefault(b, {'n': 0, 'free': 0, 'inflated': 0, 'lethal': 0,
                                    'unknown': 0, 'sealed_ticks': 0, 'rows': 0})
            e['rows'] += 1
            for v in row:
                e['n'] += 1
                if v is None:
                    continue
                if LETHAL(v):
                    e['lethal'] += 1
                elif UNKNOWN(v):
                    e['unknown'] += 1
                elif FREE(v):
                    e['free'] += 1
                elif INFLATED(v):
                    e['inflated'] += 1
            # ★ 这一横排是不是"整条过不去" ✓（★ 每个格都 ≥ 99 或 ≥254 ✓）
            if all((v is not None and (LETHAL(v) or UNKNOWN(v))) for v in row):
                e['sealed_ticks'] += 1
        for b, e in segs.items():
            e['band'] = [round(skip + b * band, 2), round(skip + (b + 1) * band, 2)]
            e['sealed_frac'] = e['sealed_ticks'] / e['rows'] if e['rows'] else None
        recs.append({'seq': seq_of(j), 'stamp': m['stamp'], 'd_to_wp4': round(d, 3),
                     'segs': [segs[b] for b in sorted(segs)]})

    out = {'run_dir': os.path.abspath(run_dir), 'wp4': [wx, wy],
           'tol_m': tol, 'skip_m': skip, 'band_m': band,
           'note': ('★ 只看**车前方** skip=%.2f m 之外 ✓（车脚下那圈是车自己标的 ✓，'
                    '对"能不能往前走"不构成路障 ✓）；★ sealed = 该纵向带**整条横截面**'
                    '都致命或未知 ✓' % skip),
           'records': recs}
    if out_path:
        with open(out_path, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('wrote %s' % out_path)
    for r in recs:
        s = ' | '.join('[%s] %d/%d/%d/%d sealed=%.2f' %
                       ('-'.join(str(x) for x in e['band']), e['free'], e['inflated'],
                        e['lethal'], e['unknown'], e['sealed_frac']) for e in r['segs'])
        print('%3d %7.2f d=%6.3f | %s' % (r['seq'], r['stamp'], r['d_to_wp4'], s))


if __name__ == '__main__':
    main()

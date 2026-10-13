#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sticky_nearest_group.py —— ★ 粘住的格"最近的障碍"是**哪一组**？（★ 带同距离对照 ✓，含恒真自检 ✓）

用法: sticky_nearest_group.py <probe目录> --max-seq N --geo scene_geometry.json --out json [--r 0.105,0.50]

★★ 为什么（reviewer 2026-10-11 ③ ✓ / 业主追加 ✓）★★
  ★ 业主问"能否把他们的场景**加**进我们的" ✗ —— ★ reviewer 已更正：★ **不能加** ✓
    （★ 两套坐标不重叠：★ 树干 x∈[0,105] ✓ 而我们的航线只到 x≤~7 ✓ ⇒ 加了也落不到航线上 ✓）
    ⇒ ★ 正确方向 = ★ **从他们的场景往外"减"** ✓ ⇒ ★ 那就需要知道"**该先减哪一组**" ✓ —— ★ 就是这一件 ✓。
★ 分组（★ 出处 = `scene_geometry.json` 的 `obstacles[].name` ✓）：
　★ 地面 `ground` ✓／楼 `main_tower`＋`east_podium` ✓／花坛 `plaza_planter*` ✓／长椅 `bench_*` ✓／树干 `tree_*` ✓
★★ **恒真自检**（★ 本项目栽过的坑 ✓）★★：★ "最近障碍"型判据最怕集合里有个**到处都是**的成员 ✓
　—— ★ 地面盒子铺满全场 ⇒ ★ 它会把一切都吃掉 ✓（★ 上次就是它把结论报成 **100.0%** ✗）
　⇒ ★ 所以：★ **先打印分组分布看有没有恒真成员** ✓；★ 然后 ★ **把地面摘掉再算一次** ✓，★ **两份都留** ✓。
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


def group_of(name):
    n = name or ''
    if n == 'ground':
        return '① 地面'
    if 'tower' in n or 'podium' in n:
        return '② 楼'
    if 'planter' in n:
        return '③ 花坛'
    if 'bench' in n:
        return '④ 长椅'
    if 'tree' in n:
        return '⑤ 树干'
    return '⑥ 其他'


def _poly(o):
    fp = o.get('footprint') or {}
    c = fp.get('coordinates')
    if not c:
        return None
    while isinstance(c, list) and c and isinstance(c[0], list) and c[0] and isinstance(c[0][0], list):
        c = c[0]
    if not isinstance(c, list) or len(c) < 3:
        return None
    return [(float(p[0]), float(p[1])) for p in c]


def _pt_in(x, y, poly):
    ins = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            if x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
                ins = not ins
    return ins


def _segd(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L = dx * dx + dy * dy
    if L <= 1e-12:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def nearest(x, y, items):
    best, bname = 1e9, None
    for (nm, poly) in items:
        cx = sum(p[0] for p in poly) / len(poly)
        cy = sum(p[1] for p in poly) / len(poly)
        rmax = max(math.hypot(p[0] - cx, p[1] - cy) for p in poly)
        if math.hypot(x - cx, y - cy) - rmax > best:
            continue
        d = 0.0 if _pt_in(x, y, poly) else min(
            _segd(x, y, poly[i][0], poly[i][1], poly[(i + 1) % len(poly)][0], poly[(i + 1) % len(poly)][1])
            for i in range(len(poly)))
        if d < best:
            best, bname = d, nm
    return best, bname


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--geo', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--radii', default='0.105,0.50')
    a = ap.parse_args()
    R = [float(x) for x in a.radii.split(',')]

    G = json.load(open(a.geo, encoding='utf-8'))
    allobs = []
    for o in G['obstacles']:
        p = _poly(o)
        if p:
            allobs.append((o['name'], p))
    no_ground = [o for o in allobs if o[0] != 'ground']

    ticks = sorted([t for t in glob.glob(os.path.join(a.probe_dir, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    meta = json.load(open(ticks[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    b0 = open(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(ticks[0])), 'rb').read()
    b1 = open(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(ticks[-1])), 'rb').read()
    H = len(b1) // W
    L = lambda v: 99 <= v <= 100
    added = {k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])}
    pts = [tuple(json.load(open(t, encoding='utf-8'))['robot_xy_map']) for t in ticks
           if json.load(open(t, encoding='utf-8')).get('robot_xy_map')]
    dense = densify(pts, 0.05)
    xy = lambda k: (ox + ((k % W) + .5) * res, oy + ((k // W) + .5) * res)
    dtraj = lambda k: min(math.hypot(xy(k)[0] - px, xy(k)[1] - py) for px, py in dense)

    rnd = random.Random(20261011)
    free = [k for k in range(len(b1)) if b1[k] == 0]
    fs = rnd.sample(free, min(len(free), 8000))
    fs_d = {k: dtraj(k) for k in fs}

    out = {'probe_dir': os.path.abspath(a.probe_dir), 'geo': os.path.abspath(a.geo),
           'radii_m': R, 'n_obstacles_total': len(allobs),
           'grouping_source': '★ `scene_geometry.json` 的 `obstacles[].name` ✓（分组规则见 group_of ✓）',
           'convention': ('★ 每格取"**最近的那一个障碍**" ✓ 记它的组 ✓；★ 对照 = 同距离配对的随机自由格 ✓；'
                          '★ **含地面**与**摘掉地面**两份都算 ✓（★ 恒真自检 ✓）')}
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
        bins = collections.Counter(int(dtraj(k) / 0.25) for k in sticky)
        pool = collections.defaultdict(list)
        for k in fs:
            pool[int(fs_d[k] / 0.25)].append(k)
        ctrl = []
        for b, need in bins.items():
            p = pool.get(b) or []
            if p:
                ctrl += [p[rnd.randrange(len(p))] for _ in range(need)]

        def per_group_dist(cells, items):
            # ★ 每一格 ⇒ 到**每一组**的最小距离 ✓ ⇒ 分组聚合 ✓
            gmap = collections.defaultdict(list)
            for k in cells:
                x, y = xy(k)
                per = {}
                for (nm, poly) in items:
                    d = 0.0 if _pt_in(x, y, poly) else min(
                        _segd(x, y, poly[i][0], poly[i][1],
                              poly[(i + 1) % len(poly)][0], poly[(i + 1) % len(poly)][1])
                        for i in range(len(poly)))
                    g = group_of(nm)
                    if g not in per or d < per[g]:
                        per[g] = d
                for g, d in per.items():
                    gmap[g].append(d)
            out2 = {}
            for g, ds in gmap.items():
                ds2 = sorted(ds)
                out2[g] = {'n_cells_that_have_this_group': len(ds2),
                           'min': round(ds2[0], 3),
                           'median': round(ds2[len(ds2) // 2], 3),
                           'p90': round(ds2[int(len(ds2) * .9)], 3)}
            return out2

        def prof(cells, items, with_ground):
            cnt = collections.Counter()
            for k in cells:
                _d, nm = nearest(*xy(k), items)
                cnt[group_of(nm)] += 1
            return {'n': len(cells), 'by_group': dict(cnt),
                    'frac_by_group': {g: round(v / max(1, len(cells)), 4) for g, v in cnt.items()},
                    'with_ground': with_ground}
        key = 'R=%.3f' % r
        out[key] = {'n_sticky': len(sticky),
                    'sticky_with_ground': prof(sticky, allobs, True),
                    'sticky_without_ground': prof(sticky, no_ground, False),
                    'control_free_same_distance_with_ground': prof(ctrl, allobs, True),
                    'control_free_same_distance_without_ground': prof(ctrl, no_ground, False),
                    'sticky_distance_to_each_group_m': per_group_dist(sticky, no_ground),
                    'control_distance_to_each_group_m': per_group_dist(ctrl, no_ground)}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)

    for r in R:
        v = out['R=%.3f' % r]
        print('  === R=%.3f m ⇒ 粘住 %d 格' % (r, v['n_sticky']))
        for tag in ('sticky_with_ground', 'sticky_without_ground',
                    'control_free_same_distance_with_ground', 'control_free_same_distance_without_ground'):
            p = v[tag]
            print('     %-46s n=%4d ｜ %s' % (tag, p['n'], p['by_group']))
        print('     ★ 逐组距离（粘住那批，中位）：')
        for g, v2 in sorted(v['sticky_distance_to_each_group_m'].items()):
            print('        %-8s 中位 %8.3f m ｜ 最小 %7.3f ｜ p90 %8.3f' % (g, v2['median'], v2['min'], v2['p90']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

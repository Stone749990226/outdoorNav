#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""swept_clearance.py —— ★ "车扫过的格"的**净空分布**（★ 同一把尺跑四轮 ✓，收 confound ✓）

用法: swept_clearance.py <probe目录> --max-seq N --geo <障碍清单> --out json [--r 0.105] [--sample 1500]

★★ 要收的那个 confound（reviewer 2026-10-11 ① ✓）★★
  ★ campus_100 两轮"粘住率 = 0.00%"✗ —— ★ 有可能只是**路况不同**（★ 航线沿墙但保持 ≥0.355 m ✓）
    ⇒ ★ 所以要给"**我们扫过的格**"的**净空分布** ✓ ＋ **开阔地格占比** ✓ ⇒ 与他们的那两轮**同一把尺比** ✓。
  ★ 判据（★ 两头都算结论 ✓）：
    · 我们那批里**相当一部分也是开阔地**（净空 5 m 量级 ✓）⇒ ★ **confound 闭合** ✓ ⇒ 差别**就是场景** ✓
    · 若我们那批**几乎全贴着表面** ⇒ ★ **confound 成立** ✓ ⇒ 结论弱化成"我们这条路上没遇到" ✓
★ 口径 ✓：★ 净空 = 格心到**最近障碍 footprint** 的距离 ✓（★ 地面不算 ✓ —— 它铺满全场会让判据恒真 ✗）；
　★ "开阔地" = 净空 ≥ **1.0 m** ✓（★ 阈值写死 ✓，另给 0.5／2.0 两档看灵敏度 ✓）。
"""
import argparse
import glob
import json
import math
import os
import random
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_obstacles import load_obstacles, dist_to_obstacle   # noqa: E402


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def densify(pts, step=0.05):
    out = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
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
    ap.add_argument('--geo', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--r', type=float, default=0.105)
    ap.add_argument('--sample', type=int, default=1500)
    a = ap.parse_args()

    obs = load_obstacles(a.geo)          # ★ 已剔除 ground ✓
    ticks = sorted([t for t in glob.glob(os.path.join(a.probe_dir, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    meta = json.load(open(ticks[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    b1 = open(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(ticks[-1])), 'rb').read()
    H = len(b1) // W
    pts = [tuple(json.load(open(t, encoding='utf-8'))['robot_xy_map']) for t in ticks
           if json.load(open(t, encoding='utf-8')).get('robot_xy_map')]
    dense = densify(pts, 0.05)

    swept = set()
    for px, py in dense:
        i0, i1 = int((px - a.r - ox) / res), int((px + a.r - ox) / res)
        j0, j1 = int((py - a.r - oy) / res), int((py + a.r - oy) / res)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                if 0 <= i < W and 0 <= j < H:
                    k = j * W + i
                    x, y = ox + (i + .5) * res, oy + (j + .5) * res
                    if math.hypot(x - px, y - py) <= a.r:
                        swept.add(k)
    rnd = random.Random(20261011)
    cells = list(swept)
    if len(cells) > a.sample:
        cells = rnd.sample(cells, a.sample)
    clear = []
    for k in cells:
        x, y = ox + ((k % W) + .5) * res, oy + ((k // W) + .5) * res
        clear.append(dist_to_obstacle(x, y, obs))
    clear.sort()
    q = lambda p: clear[min(len(clear) - 1, int(len(clear) * p))] if clear else None
    out = {'probe_dir': os.path.abspath(a.probe_dir), 'geo': os.path.abspath(a.geo),
           'r_m': a.r, 'n_obstacles': len(obs), 'n_swept_total': len(swept), 'n_sampled': len(clear),
           'clearance_m': {'min': clear[0] if clear else None, 'p10': q(.1), 'median': q(.5),
                           'p90': q(.9), 'max': clear[-1] if clear else None},
           'open_share': {k: (sum(1 for d in clear if d >= t) / len(clear) if clear else None)
                          for k, t in (('>=0.5m', 0.5), ('>=1.0m', 1.0), ('>=2.0m', 2.0), ('>=5.0m', 5.0))},
           'convention': ('★ 净空 = 格心到最近障碍 footprint 的距离 ✓（地面已剔除 ✓）；'
                          '★ "开阔地" 阈值三档 0.5／1.0／2.0 m ＋ 5.0 m ✓（写死 ✓）；'
                          '★ 扫过的格 = 格心到加密轨迹(0.05 m)任一点 ≤ r ✓')}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 扫过 %d 格（抽 %d）｜ 障碍 %d 个 ｜ ★ 净空中位 %.3f m ｜ p10 %.3f ｜ p90 %.3f'
          % (len(swept), len(clear), len(obs), q(.5) or -1, q(.1) or -1, q(.9) or -1))
    print('  ★ 开阔地占比：' + ' ｜ '.join('%s %.1f%%' % (k, 100 * v) for k, v in out['open_share'].items()))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

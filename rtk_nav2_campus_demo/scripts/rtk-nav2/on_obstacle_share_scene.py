#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""on_obstacle_share_scene.py —— ★ 用**他们场景的几何**判"新增致命格在不在真障碍上" ✓

用法: on_obstacle_share_scene.py <probe目录> <max_seq> <colliders.json 或 scene_geometry.json>

★ 与 `on_obstacle_share.py` 的分工 ✓：
  · 那一支用**静态先验图**（只在 campus_100 那两轮可用 ✓ —— 两轮栅格逐字相同 ✓）
  · 这一支用**场景几何本身**（north_campus 那两轮用它 ✓）—— 判据 = "格心到最近障碍 footprint 的距离"
  ★ 两支**判据不同** ✗（一支是"格被静态层标了" ✓，一支是"几何上贴不贴" ✓）⇒
    报数时必须**带上用的是哪一支** ✓（本项目"报数带口径" ✓）。
"""
import glob
import json
import math
import os
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    probe, max_seq, geo_path = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    X = float(sys.argv[4]) if len(sys.argv) > 4 else 0.10
    G = json.load(open(geo_path, encoding='utf-8'))
    obs = G['obstacles'] if isinstance(G, dict) and 'obstacles' in G else G
    boxes = []
    for c in obs:
        n = c.get('name') or c.get('n')
        if n == 'ground':
            continue                       # ★ 剔掉铺满全场的地面（否则判据恒真 ✗，本项目记过 ✓）
        # ★ 实测结构（`scene_geometry.json` ✓）：`{name, bounds:[xmin,ymin,xmax,ymax], footprint:{...}}` ✓
        if 'bounds' in c and isinstance(c['bounds'], list) and len(c['bounds']) == 4:
            x0, y0, x1, y1 = c['bounds']
            boxes.append(('box', (x0 + x1) / 2.0, (y0 + y1) / 2.0, (x1 - x0) / 2.0, (y1 - y0) / 2.0))
            continue
        g = c.get('geom') or c
        k = g.get('k') or g.get('kind') or g.get('type') or 'box'
        x, y = g.get('x'), g.get('y')
        if x is None:
            continue
        if 'hx' in g:
            boxes.append(('box', x, y, g['hx'], g['hy']))
        elif 'r' in g:
            boxes.append(('cyl', x, y, g['r'], g['r']))
        elif 'sx' in g:
            boxes.append(('box', x, y, g['sx'] / 2.0, g['sy'] / 2.0))
    if not boxes:
        print('✗ 解析不出障碍 footprint（%s）⇒ 拒绝出数 ✗' % geo_path)
        return 2

    maps = sorted(glob.glob(os.path.join(probe, 'map_*.bin')), key=seq_of)
    m0, m1 = maps[0], next(m for m in reversed(maps) if seq_of(m) <= max_seq)
    b0, b1 = open(m0, 'rb').read(), open(m1, 'rb').read()
    meta = json.load(open(os.path.join(probe, 'tick_%04d.json' % seq_of(m1)), encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']

    def dmin(wx, wy):
        best = 1e9
        for k, cx, cy, a, b in boxes:
            if k == 'box':
                d = math.hypot(max(0.0, abs(wx - cx) - a), max(0.0, abs(wy - cy) - b))
            else:
                d = max(0.0, math.hypot(wx - cx, wy - cy) - a)
            best = min(best, d)
        return best

    added = [k for k in range(len(b1)) if 99 <= b1[k] <= 100 and not (99 <= b0[k] <= 100)]
    on = 0
    ds = []
    for k in added:
        wx = ox + ((k % W) + 0.5) * res
        wy = oy + ((k // W) + 0.5) * res
        d = dmin(wx, wy)
        ds.append(d)
        if d <= X:
            on += 1
    ds.sort()
    out = {
        'probe_dir': os.path.abspath(probe), 'geometry': os.path.abspath(geo_path),
        'start_map': os.path.basename(m0), 'end_map': os.path.basename(m1),
        'threshold_m': X, 'n_obstacle_footprints': len(boxes),
        'n_added': len(added), 'n_added_on_obstacle': on,
        'frac_on_obstacle': (on / len(added)) if added else None,
        'n_added_in_open': len(added) - on,
        'frac_in_open': ((len(added) - on) / len(added)) if added else None,
        'dist_to_obstacle_m': ({'min': ds[0], 'median': ds[len(ds) // 2], 'p90': ds[int(len(ds) * .9)],
                                'max': ds[-1]} if ds else None),
        'criterion': '★ 格心到最近障碍 footprint 的距离 ≤ %s m ⇒ 算"在障碍上" ✓（⑨ 地面已剔除 ✓）' % X,
    }
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(probe)),
                                     'on_obstacle_share_scene.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('  ★ 新增致命 %d ⇒ **在障碍上** %d（%.1f%%）｜ **开阔地** %d（%.1f%%）｜ 距离中位 %.3f m'
          % (len(added), on, 100.0 * on / max(1, len(added)), len(added) - on,
             100.0 * (len(added) - on) / max(1, len(added)), ds[len(ds) // 2] if ds else -1))
    print('  ★ 已落盘 %s/on_obstacle_share_scene.json' % os.path.dirname(os.path.abspath(probe)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

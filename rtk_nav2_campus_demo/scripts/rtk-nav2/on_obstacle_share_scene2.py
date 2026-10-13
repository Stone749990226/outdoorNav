#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""on_obstacle_share_scene2.py —— ★ 更正版：用**全部 32 个障碍**（含 26 根树干的多边形 footprint ✓）
                                                                    重算"新增致命格在不在障碍上" ✗→✓

用法: on_obstacle_share_scene2.py <probe目录> --max-seq N --geo scene_geometry.json --out json

★★ 为什么要有 v2（★ 2026-10-11 我自己的错 ✗）★★
  ★ v1（`on_obstacle_share_scene.py`）**只吃 `o['bounds']`** ✗ —— ★ 而实测 32 个障碍里
    **只有 6 个有 `bounds`** ✗（★ 26 根树干**只有多边形** ✓）⇒ ★ v1 实际上只拿 **6 个**障碍在判 ✗✗
    ⇒ ★ 它报的"在障碍上 51.0%／31.6%"**口径不全** ✗ ⇒ ★ v2 **重算并更正** ✓
    （★ 旧产物**不删** ✓ —— ★ 两个都留着，并写明哪个是更正版 ✓，本项目"更正要留痕" ✓）。
★ v2 的口径 ✓：★ 距离到**最近障碍**（box 用 bounds ✓／树用多边形 ✓）✓；★ 内部 ⇒ 0 ✓；
  ★ 阈值 0.10 m（与 v1 同 ✓，便于对照 ✓）。
"""
import argparse
import glob
import json
import os
import re
import sys
import sys as _s

_s.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_obstacles import load_obstacles, dist_to_obstacle   # noqa: E402


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--geo', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--x', type=float, default=0.10)
    a = ap.parse_args()

    obs = load_obstacles(a.geo)
    trees = [o for o in obs if 'tree' in o[0]]
    ticks = sorted([t for t in glob.glob(os.path.join(a.probe_dir, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    meta = json.load(open(ticks[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    b0 = open(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(ticks[0])), 'rb').read()
    b1 = open(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(ticks[-1])), 'rb').read()
    L = lambda v: 99 <= v <= 100
    added = [k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])]
    ds = []
    on = 0
    for k in added:
        x = ox + ((k % W) + 0.5) * res
        y = oy + ((k // W) + 0.5) * res
        d = dist_to_obstacle(x, y, obs)
        ds.append(d)
        if d <= a.x:
            on += 1
    ds.sort()
    out = {'probe_dir': os.path.abspath(a.probe_dir), 'geo': os.path.abspath(a.geo),
           'version': '★ v2（更正版 ✓：障碍用**全部 32 个** incl. 26 根树干的多边形 ✓）',
           'n_obstacles_used': len(obs), 'n_trees': len(trees),
           'n_obstacles_with_bounds_only_v1': 6,
           'threshold_m': a.x, 'n_added': len(added), 'n_added_on_obstacle': on,
           'frac_on_obstacle': (on / len(added)) if added else None,
           'n_added_in_open': len(added) - on,
           'frac_in_open': ((len(added) - on) / len(added)) if added else None,
           'dist_to_obstacle_m': ({'min': ds[0], 'median': ds[len(ds) // 2],
                                   'p90': ds[int(len(ds) * .9)], 'max': ds[-1]} if ds else None),
           'why_v2': ('★ v1 只吃 bounds ⇒ 只用了 6/32 个障碍（26 根树干被漏掉 ✗）⇒ 那些数**口径不全** ✗；'
                      '★ v2 用全部 ✓（旧产物留着不删 ✓）')}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 障碍 %d 个（其中树 %d ✓）｜ 新增致命 %d ⇒ 在障碍上 %d（%.1f%%）｜ 开阔地 %d（%.1f%%）｜ 距离中位 %.3f m'
          % (len(obs), len(trees), len(added), on, 100.0 * on / max(1, len(added)),
             len(added) - on, 100.0 * (len(added) - on) / max(1, len(added)),
             ds[len(ds) // 2] if ds else -1))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

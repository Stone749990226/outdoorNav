#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""describe_unmatched.py —— ★ "任何拍都匹配不上"的那些格，**落在图的哪里**？（★ 纯离线 ✓）

用法: describe_unmatched.py <unmatched_*.json> --probe <probe目录> --out json

★ 要回答的就一句：★ **它们在哪** ✓（reviewer 2026-10-11 ✓）。
★ 量五件事 ✓：
  ① ★ 它们**自己**是什么值（未知/自由/膨胀/致命 ✓）＋ **八邻域**是什么 ✓
  ② ★ 到**最近未知格**的距离 ✓（★ 多源 BFS ✓）
  ③ ★ 到**最近自由格**的距离 ✓
  ④ ★ 到**车轨迹**的距离 ✓
  ⑤ ★ 到**图边界**的距离 ✓（★ 是不是"贴在边界上" ✓）
★ 对照 ✓：★ 同一批"新增致命"里**匹配上了**的那些格 ✓（★ 同样的五个量 ✓）⇒ ★ 两条一比就知道差在哪 ✓。
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


def bfs_dist(mask_ok, W, H):
    """★ 多源 BFS：到最近"满足 mask_ok 的格"的步数 ✓（∞ = 取不到 ✓）"""
    INF = 1 << 30
    d = [INF] * (W * H)
    q = collections.deque()
    for k in range(W * H):
        if mask_ok(k):
            d[k] = 0
            q.append(k)
    while q:
        k = q.popleft()
        x, y = k % W, k // W
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H:
                k2 = ny * W + nx
                if d[k2] > d[k] + 1:
                    d[k2] = d[k] + 1
                    q.append(k2)
    return d


def stats(vals, res):
    vals = sorted(v for v in vals if v is not None)
    if not vals:
        return None
    q = lambda p: vals[min(len(vals) - 1, int(len(vals) * p))]
    return {'n': len(vals), 'min_m': round(vals[0] * res, 3), 'median_m': round(q(.5) * res, 3),
            'p90_m': round(q(.9) * res, 3), 'max_m': round(vals[-1] * res, 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('--probe', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    D = json.load(open(a.dump, encoding='utf-8'))
    W, res = D['width'], D['resolution']
    ox, oy = D['origin']['x'], D['origin']['y']
    H = None
    b1 = open(os.path.join(a.probe, D['end_map']), 'rb').read()
    H = len(b1) // W
    b0 = open(os.path.join(a.probe, D['start_map']), 'rb').read()
    traj = D['traj_xy']

    unm = set(D['cells'])
    L = lambda v: 99 <= v <= 100
    added = {k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])}
    matched_cells = added - unm                      # ★ 对照：同一批里**匹配上了**的 ✓

    unknown_mask = lambda k: b1[k] >= 128
    free_mask = lambda k: b1[k] == 0
    d_unk = bfs_dist(unknown_mask, W, H)
    d_free = bfs_dist(free_mask, W, H)

    def dtraj(k):
        x = ox + ((k % W) + 0.5) * res
        y = oy + ((k // W) + 0.5) * res
        return min(math.hypot(x - px, y - py) for px, py in traj)

    def dedge(k):
        x, y = k % W, k // W
        return min(x, y, W - 1 - x, H - 1 - y)

    CLASS = lambda v: ('unknown' if v >= 128 else 'free' if v == 0 else
                       'lethal' if v >= 99 else 'inflated')

    def profile(cells):
        cells = list(cells)
        nb = collections.Counter()
        for k in cells:
            x, y = k % W, k // W
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H:
                        nb[CLASS(b1[ny * W + nx])] += 1
        return {'n': len(cells),
                'self_class': dict(collections.Counter(CLASS(b1[k]) for k in cells)),
                'neighbour_class': dict(nb),
                'to_nearest_unknown': stats([d_unk[k] for k in cells], res),
                'to_nearest_free': stats([d_free[k] for k in cells], res),
                'to_traj': stats([dtraj(k) for k in cells], res),
                'to_map_edge': stats([dedge(k) for k in cells], res),
                'frac_on_map_edge_1cell': sum(1 for k in cells if dedge(k) <= 1) / max(1, len(cells)),
                'frac_on_map_edge_5cell': sum(1 for k in cells if dedge(k) <= 5) / max(1, len(cells))}

    out = {'dump': os.path.abspath(a.dump), 'probe_dir': os.path.abspath(a.probe),
           'window': D['window'], 'n_unmatched': len(unm), 'n_matched_control': len(matched_cells),
           'grid': {'W': W, 'H': H, 'res': res, 'origin': [ox, oy]},
           'unmatched': profile(unm),
           'matched_control': profile(matched_cells)}

    # ★ ASCII 图（1 格 4 字符 ⇒ 每 2 m 一个字符 ✓）：`#`=未匹配残留 ✓・`*`=匹配上的 ✓・`.`=其他
    step = 20   # 20 格 = 2 m
    lines = []
    for gy in range(H - 1, -1, -step):
        row = []
        for gx in range(0, W, step):
            blk = [k for k in range(gy * W + gx, min(len(b1), gy * W + gx + step))] if False else None
            hit_u = hit_m = False
            for dy in range(0, step):
                for dx in range(0, step):
                    xx, yy = gx + dx, gy - dy
                    if 0 <= xx < W and 0 <= yy < H:
                        k = yy * W + xx
                        if k in unm:
                            hit_u = True
                        elif k in matched_cells:
                            hit_m = True
            row.append('#' if hit_u else ('*' if hit_m else ('.' if b1[min(len(b1) - 1, gy * W + gx)] != 255 else ' ')))
        lines.append(''.join(row))
    out['ascii'] = {'legend': '#=未匹配残留 / *=匹配上的 / .=其他 / 空格=未知', 'step_cells': step, 'rows': lines}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)

    u, m = out['unmatched'], out['matched_control']
    print('  ★ 未匹配 %d 格 ｜ 对照（匹配上的）%d 格' % (u['n'], m['n']))
    for key in ('self_class', 'neighbour_class'):
        print('     %-16s 未匹配 %s' % (key, u[key]))
        print('     %-16s 对照   %s' % ('', m[key]))
    for key in ('to_nearest_unknown', 'to_nearest_free', 'to_traj', 'to_map_edge'):
        print('     %-18s 未匹配中位 %s m ｜ 对照中位 %s m'
              % (key, (u[key] or {}).get('median_m'), (m[key] or {}).get('median_m')))
    print('     %-18s 未匹配贴边(≤5格) %.1f%% ｜ 对照 %.1f%%'
          % ('to_map_edge', 100 * u['frac_on_map_edge_5cell'], 100 * m['frac_on_map_edge_5cell']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

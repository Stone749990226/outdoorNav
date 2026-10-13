#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""describe_unexplained.py —— ★ 把"两边都不沾"的那批残留格**描述出来** ✓（阶段十收尾 ✓）

用法: describe_unexplained.py <残留CSV> <cols.json> [--X 0.10] --out <新json>

★ 背景（★ 2026-10-11 ✓）：★ `residue_vs_geometry.py` 报出 ★ **80.9%（4250 格）**"既不在网格边界带、
★ 也不在任何真障碍 footprint 上" ✗ ⇒ ★ **"清不掉"解释得了"为什么留着"✓，✗ 解释不了"点从哪来"** ✗。
⇒ ★ 本工具把这批格子**描述清楚** ✓（★ 目标 = ★ **一句话说出它们在哪** ✓）：
  ① ★ 与**车轨迹**的关系（★ 沿用 CSV 里现成的 `track_distance_m` ✓，★ 并按**沿轨迹位置**分段 ✓）
  ② ★ 是否在**路面多边形内部**（★ 注意 ★ **是"内部"** ✓，✗ 不是上一轮的"边界带" ✗）
  ③ ★ **成片还是零星**（★ 2 格连通聚类 ✓：★ 成团 ⇒ 像"某个东西"✓；★ 零星 ⇒ 像"逐拍噪声"✓）
  ④ ★ 与 `scan`／`cam` 归因的**交叉** ✓
  ⑤ ★ 一张**粗粒度 ASCII 图** ✓（★ 让人一眼看出"在哪" ✓）
★ 纯读 ✓；★ 产物写到 `--out` 指定处 ✓（✗ 不覆盖别的 ✓）。
"""
import argparse
import collections
import csv
import json
import math
import os
import sys


def is_pair(v):
    return isinstance(v, list) and len(v) == 2 and all(isinstance(t, (int, float)) for t in v)


def rings_of(geom):
    out = []

    def walk(c):
        if is_pair(c):
            return
        if isinstance(c, list) and c and is_pair(c[0]):
            out.append(c)
            return
        for x in c:
            walk(x)
    walk(geom)
    return out


def pip(px, py, ring):
    """★ 射线法：点在多边形（环）内 ✓"""
    inside = False
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        if (y0 > py) != (y1 > py):
            xin = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
            if px < xin:
                inside = not inside
    return inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('cols_json')
    ap.add_argument('--X', type=float, default=0.10)
    ap.add_argument('--geo', default='/home/xjy/projects/outdoorNav/gazebo_campus/generated/scene_geometry.json')
    ap.add_argument('--ticks', default=None, help='★ 探针目录（★ 用来现算到轨迹的距离 ✓）')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    G = json.load(open(a.geo, encoding='utf-8'))
    allc = json.load(open(a.cols_json, encoding='utf-8'))
    cols = [c for c in allc if c['n'] != 'ground']
    rows = list(csv.DictReader(open(a.csv_path, encoding='utf-8')))

    edges = []
    for geom in (G['free']['coordinates'], G['plaza']['coordinates']):
        for ring in rings_of(geom):
            for i in range(len(ring) - 1):
                edges.append((ring[i][0], ring[i][1], ring[i + 1][0], ring[i + 1][1]))
    B = collections.defaultdict(list)
    for e in edges:
        x0, y0, x1, y1 = e
        n = max(2, int(max(abs(x1 - x0), abs(y1 - y0)) / 0.5) + 1)
        for i in range(n + 1):
            B[(int((x0 + (x1 - x0) * i / n) / 0.5), int((y0 + (y1 - y0) * i / n) / 0.5))].append(e)

    def d_seg(px, py, e):
        x0, y0, x1, y1 = e
        dx, dy = x1 - x0, y1 - y0
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
        return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))

    def d_mesh(px, py):
        return min((d_seg(px, py, e) for e in B.get((int(px / 0.5), int(py / 0.5)), [])), default=1e9)

    def d_col(px, py):
        best = 1e9
        for c in cols:
            d = (math.hypot(max(0, abs(px - c['x']) - c['hx']), max(0, abs(py - c['y']) - c['hy']))
                 if c['k'] == 'box' else max(0.0, math.hypot(px - c['x'], py - c['y']) - c['hx']))
            if d < best:
                best = d
        return best

    free_rings = [r for geom in (G['free']['coordinates'],) for r in rings_of(geom)]
    plaza_rings = rings_of(G['plaza']['coordinates'])

    unexp = []
    for r in rows:
        px, py = float(r['wx']), float(r['wy'])
        if d_mesh(px, py) > a.X and d_col(px, py) > a.X:
            unexp.append((px, py, r))
    print('  ★ 两边都不沾 = **%d** 格（X=%.2f ✓ ｜ 总 %d ✓）' % (len(unexp), a.X, len(rows)))

    # ★ ① 与轨迹的距离（★ CSV 里**没有**逐行轨迹距离 ✗ ⇒ ★ 从探针 tick 现算 ✓）
    traj = []
    if a.ticks:
        import glob
        for f in sorted(glob.glob(os.path.join(a.ticks, 'tick_*.json'))):
            d = json.load(open(f, encoding='utf-8'))
            r0 = d.get('robot_xy_map')
            if r0:
                traj.append((r0[0], r0[1]))
    if traj:
        tB = collections.defaultdict(list)
        for (x, y) in traj:
            tB[(int(x / 0.5), int(y / 0.5))].append((x, y))
        def d_traj(px, py):
            # ★★ 2026-10-11 修 ✗：★ 第一版只查 ±1 桶（= ±0.75 m ✓）⇒ ★ 更远的格**取不到**
            #   ⇒ ★ 返回哨兵 `1e9` ✗ ⇒ ★ 中位数被它带成 `1e9` ✗（★ 实测就是这么坏的 ✓）。
            #   ⇒ ★ **逐圈扩大**（1,2,3,… 桶 ✓），★ 找到就停 ✓；★ 全空才返哨兵 ✓。
            bx, by = int(px / 0.5), int(py / 0.5)
            for r in range(1, 13):
                best = 1e9
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        if max(abs(dx), abs(dy)) != r:
                            continue
                        for (x, y) in tB.get((bx + dx, by + dy), ()):
                            best = min(best, math.hypot(px - x, py - y))
                if best < 1e9:
                    return best
            return 1e9
        td = sorted(d_traj(px, py) for px, py, _ in unexp)
    else:
        td = []
        print('  ★ ✗ 没给 --ticks ⇒ ★ 轨迹距离**算不了** ✗（★ 记成"未取到" ✓，✗ 不许落成 0 ✗）')
    def q(p):
        return td[min(len(td) - 1, int(len(td) * p))] if td else None
    print('  ★ ① 到**车轨迹**的距离（★ 从探针 tick 的轨迹现算 ✓）：'
          'min %.2f ｜ p25 %.2f ｜ ★ **中位 %.2f** ｜ p75 %.2f ｜ p90 %.2f ｜ max %.2f m'
          % (td[0], q(.25), td[len(td) // 2], q(.75), q(.90), td[-1]))
    for X in (0.5, 1.0, 2.0, 3.0):
        k = sum(1 for v in td if v <= X)
        print('       ★ 其中 ≤ %.1f m 的 = %5d（%5.1f%%）' % (X, k, 100.0 * k / max(1, len(td))))

    # ★ ② 在**路面多边形内部**（★ 注意是"内部"✓，✗ 不是边界带 ✗）
    inf = sum(1 for px, py, _ in unexp if any(pip(px, py, r) for r in free_rings))
    inp = sum(1 for px, py, _ in unexp if any(pip(px, py, r) for r in plaza_rings))
    print('  ★ ② 落在★ **路面/自由区多边形内部**的 = %d（%.1f%%）｜ ★ 落在广场内部的 = %d（%.1f%%）'
          % (inf, 100.0 * inf / max(1, len(unexp)), inp, 100.0 * inp / max(1, len(unexp))))

    # ★ ③ 成片还是零星（★ 2 格（8 邻域）连通聚类 ✓）
    cellset = {(round(px / 0.1), round(py / 0.1)) for px, py, _ in unexp}
    seen = set()
    groups = []
    for c in cellset:
        if c in seen:
            continue
        stack, comp = [c], []
        seen.add(c)
        while stack:
            cur = stack.pop()
            comp.append(cur)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (cur[0] + dx, cur[1] + dy)
                    if nb in cellset and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        groups.append(comp)
    groups.sort(key=len, reverse=True)
    print('  ★ ③ 连通簇数 = **%d**（★ 格子 %d ✓）｜ ★ 最大的 5 簇 = %s'
          % (len(groups), len(cellset), [len(g) for g in groups[:5]]))
    print('       ★ 最大簇占 %.1f%%（★ 成片 ⇒ 像"某个东西"✓；★ 全是一格一簇 ⇒ 像"逐拍噪声"✓）'
          % (100.0 * len(groups[0]) / max(1, len(cellset))))

    # ★ ④ 与 scan／cam 归因交叉
    mix = collections.Counter(r.get('src', '?') for _, _, r in unexp)
    print('  ★ ④ 这 %d 格的归因：%s' % (len(unexp), dict(mix.most_common())))

    # ★ ⑤ 粗粒度 ASCII 图（★ 1 m 一格 ✓，✗ 只画这 4250 格 ✓）
    if unexp:
        xs = [px for px, _, _ in unexp]
        ys = [py for _, py, _ in unexp]
        x0, x1 = math.floor(min(xs)), math.ceil(max(xs))
        y0, y1 = math.floor(min(ys)), math.ceil(max(ys))
        W, H = x1 - x0 + 1, y1 - y0 + 1
        grid = [[0] * W for _ in range(H)]
        for px, py, _ in unexp:
            grid[int(py) - y0][int(px) - x0] += 1
        print('  ★ ⑤ 分布图（★ 1 m 一格 ✓，★ y 从上到下 = %.0f → %.0f ✓，★ x 从左到右 = %.0f → %.0f ✓）：'
              % (y1, y0, x0, x1))
        chars = ' .:-=+*#%@'
        for j in range(H - 1, -1, -1):
            row = ''.join(chars[min(9, grid[j][i])] if grid[j][i] else ' ' for i in range(W))
            if row.strip():
                print('     %5.0f |%s' % (y0 + j, row))
        print('           +' + '-' * W)
        print('            %s' % ''.join(('%d' % ((x0 + i) // 10 % 10) if (x0 + i) % 10 == 0 else ' ')
                                         for i in range(W)))

    out = {'X': a.X, 'n_unexplained': len(unexp), 'n_total': len(rows),
           # ★★ 2026-10-11（reviewer 要求 ✓）：★ 哨兵值 `1e9` 必须在产物里**自带解释** ✗ ★★
           #   ★ 起因：★ 我第一版的桶搜索只覆盖 ±0.75 m ⇒ ★ 更远的格取不到 ⇒ ★ 返回哨兵 `1e9` ✗
           #   ⇒ ★ `track_distance_m.max = 1e9` 出现在产物里 ✓（★ 一眼看得出 ✓）
           #     ★ 但**没写它是什么意思** ✗ ⇒ ★ 加 `n_unresolved` ＋ 一句说明 ✓。
           'track_distance_m': ({'min': td[0], 'p25': q(.25), 'median': td[len(td) // 2],
                                 'p75': q(.75), 'p90': q(.90),
                                 'max': (None if td[-1] >= 1e9 else td[-1]),
                                 'n_unresolved': sum(1 for v in td if v >= 1e9),
                                 'note': ('★ `max` 为 null 且 `n_unresolved>0` ⇒ ★ 那几格在'
                                          '**搜索半径（6 m）内没找到轨迹点** ✗ ⇒ ★ 记成"未取到" ✓'
                                          '（✗ 不是 0 ✓、✗ 也不是"很远" ✓）')} if td else None),
           'frac_within_m_of_track': {str(X): (sum(1 for v in td if v <= X) / float(len(td)) if td else None)
                                      for X in (0.5, 1.0, 2.0, 3.0)},
           'in_free_polygon': inf, 'in_plaza_polygon': inp,
           'clusters': len(groups), 'cluster_sizes_top5': [len(g) for g in groups[:5]],
           'attribution_mix': dict(mix)}
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=2)
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

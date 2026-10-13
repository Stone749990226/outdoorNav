#!/usr/bin/env python3
"""**幽灵格形状分析**：把一帧全局图里"离真实障碍很远"的致命格聚成簇，打印每簇的
范围、代价取值分布、以及"到最近真实表面的距离"分布。

为什么需要它：`obstacle_map_check.py` 只给"旧外墙带里有几个致命格 / 全图有几个远格"，
说明不了**这些格子的形状**。而形状正是判根因的关键——

  - 若是一圈贴着真实障碍的薄壳 → 是膨胀/边缘像素，不是幽灵；
  - 若是成片的矩形块、而且**边界落在某个固定距离/固定坐标上**
    → 指向"越过某条轮廓线之后相机看到的东西"。
    **⚠️ 修正（2026-09-21）**：这里原来写的是"边界落在方板边缘（±5.35）→ 指向越过板边"，
    而那只是**当时的假设**；ablation 已证明**方板板边不是源头**（板边 ≥7.9 m、出 6 m 量程之外时
    同样出幽灵格）。两者仍**不同族**：**同不同族判不了**——
    本工具当初就没抓到点级证据，别把"同族"写成结论。
    另外现在场地方板半边长是 **11.35 m（不是 5.35）**；下面 `nearest_plate_edge_m` 仍按 5.35 算，
    **只作历史量纲参考，不要当现状读**；
  - 若是沿某条直线 → 指向某个像素行（见 `camera_depth_profile.py`）。

用法：
  phantom_cells.py <快照.npz> [--far_m 0.35] [--json 结果.json]
  （快照由 `nav2_probe.py geometry --dump-snapshot x.npz` 或
    `obstacle_map_check.py` 同源的全局图导出）
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_geometry as G  # noqa: E402


def clusters(cells, res, link=3):
    """把格子按"棋盘距离 <= link"做并查集聚类，返回每簇的 (x,y) 列表。"""
    n = len(cells)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    bucket = {}
    for i, (x, y) in enumerate(cells):
        bucket.setdefault((round(x / res), round(y / res)), []).append(i)
    for (cx, cy), idxs in bucket.items():
        for dx in range(-link, link + 1):
            for dy in range(-link, link + 1):
                other = bucket.get((cx + dx, cy + dy))
                if other:
                    for j in other:
                        union(idxs[0], j)
    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('snapshot')
    ap.add_argument('--far_m', type=float, default=0.35)
    ap.add_argument('--json', default=None)
    ap.add_argument('--cells-csv', default=None,
                    help='把"远格"的 x,y,到最近真实表面的距离 逐行写成 CSV（做离线拟合用）')
    args = ap.parse_args(argv)

    z = np.load(args.snapshot)
    data = z['data']
    res = float(z['resolution'])
    ox, oy = float(z['origin_x']), float(z['origin_y'])
    lethal = np.argwhere(data >= 100)
    xs = ox + (lethal[:, 1] + 0.5) * res
    ys = oy + (lethal[:, 0] + 0.5) * res
    dist = np.array([G.nearest_surface(x, y) for x, y in zip(xs, ys)])
    far = dist > args.far_m
    fx, fy, fd = xs[far], ys[far], dist[far]
    fcost = data[lethal[far, 0], lethal[far, 1]]

    out = {'snapshot': os.path.abspath(args.snapshot),
           'grid': {'res': res, 'origin': [ox, oy], 'shape': list(data.shape)},
           'lethal_total': int(len(lethal)),
           'far_m': args.far_m,
           'far_total': int(far.sum()),
           'far_bbox': ([round(float(v), 3) for v in (fx.min(), fx.max(), fy.min(), fy.max())]
                        if far.sum() else None),
           'far_cost_values': (dict(zip(*[v.tolist() for v in
                                          np.unique(fcost, return_counts=True)]))
                               if far.sum() else {}),
           'far_dist_m': ({'min': round(float(fd.min()), 3),
                           'median': round(float(np.median(fd)), 3),
                           'max': round(float(fd.max()), 3)} if far.sum() else None),
           'clusters': []}
    cells = list(zip(fx, fy))
    for grp in clusters(cells, res):
        gx = np.array([cells[i][0] for i in grp])
        gy = np.array([cells[i][1] for i in grp])
        out['clusters'].append({
            'n': len(grp),
            'x': [round(float(gx.min()), 3), round(float(gx.max()), 3)],
            'y': [round(float(gy.min()), 3), round(float(gy.max()), 3)],
            'size_m': [round(float(gx.max() - gx.min() + res), 3),
                       round(float(gy.max() - gy.min() + res), 3)],
            'nearest_plate_edge_m': round(min(5.35 - abs(float(gx.min())),
                                              5.35 - abs(float(gy.min())),
                                              5.35 - abs(float(gx.max())),
                                              5.35 - abs(float(gy.max()))), 3),
        })
    out['clusters'].sort(key=lambda c: -c['n'])
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.cells_csv:
        with open(args.cells_csv, 'w', encoding='utf-8') as handle:
            handle.write('x,y,dist_to_surface\n')
            for x, y, d in zip(fx, fy, fd):
                handle.write('%.3f,%.3f,%.3f\n' % (x, y, d))
        print('已写入 %s（%d 行）' % (args.cells_csv, len(fx)), file=sys.stderr)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())

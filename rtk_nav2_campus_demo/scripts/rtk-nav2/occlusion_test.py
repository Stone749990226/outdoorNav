#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""occlusion_test.py —— ★ 假设：那 26 根贴地树干**挡住了清除射线** ⇒ 残留留下来了 ✗（★ 带对照 ✓）

用法: occlusion_test.py <probe目录> --max-seq N --geo scene_geometry.json --out json

★★ 怎么判（★ 两头都要 ✓）★★
  ★ 对每一格残留：★ 从**车轨迹**上任一点到它画一条线 ✓ ⇒ ★ 只要有一条**擦过某根树干**（离轴 < R ✓、
    且交点**在两点之间** ✓）⇒ ★ 记"被挡" ✓。
  ★ **对照** ✓：★ 取**同样多**的**自由格**（图上值为 0 ✓）✓ —— ★ 而且★**按"到轨迹的距离"分层配对** ✓
    （★ 遮挡率天然随距离上升 ✗ ⇒ ★ 不配距离就会把"残留离路远"误读成"残留被挡" ✗ —— 本项目"混淆变量"那条 ✓）。
  ★ **判据** ✓：★ 残留的遮挡率**显著高于**同距离档的对照 ⇒ ★ 假设成立 ✓；★ 持平 ⇒ **否** ✓（★ 如实写 ✓）。
"""
import argparse
import glob
import json
import math
import os
import random
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def seg_dist(px, py, qx, qy, cx, cy):
    """★ 点 (cx,cy) 到线段 pq 的距离 ＋ 最近点是否落在**线段内** ✓"""
    dx, dy = qx - px, qy - py
    L = dx * dx + dy * dy
    if L <= 1e-12:
        return math.hypot(cx - px, cy - py), True
    t = ((cx - px) * dx + (cy - py) * dy) / L
    inside = (0.0 < t < 1.0)
    tc = min(1.0, max(0.0, t))
    return math.hypot(cx - (px + tc * dx), cy - (py + tc * dy)), inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--geo', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--radius', type=float, default=0.30)
    ap.add_argument('--n-traj', type=int, default=150, help='★ 轨迹抽多少个点（够密即可 ✓）')
    ap.add_argument('--n-control', type=int, default=None, help='★ 对照格数（默认 = 残留格数 ✓）')
    a = ap.parse_args()

    # ---- ① 残留格（★ 与指标同口径：末拍致命 ∧ 起始拍不致命 ✓）----
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
    residue = [k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])]
    free = [k for k in range(len(b1)) if b1[k] == 0]
    if not residue or not free:
        print('✗ 残留或自由格为空 ⇒ 拒绝出数 ✗')
        return 2

    # ---- ② 轨迹点（★ 等距抽 ✓）----
    pts = []
    for t in ticks:
        r = json.load(open(t, encoding='utf-8')).get('robot_xy_map')
        if r:
            pts.append((r[0], r[1]))
    step = max(1, len(pts) // a.n_traj)
    traj = pts[::step]

    # ---- ③ 树干（★ 只取名字带 tree 的 ✓，半径用**实测尺寸** ✓）----
    # ★ 2026-10-11 更正 ✗：树干**只有多边形 footprint**（没有 bounds ✓）
    #   ⇒ 统一走 `scene_obstacles`（中心 ＋ 半径 ✓）
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from scene_obstacles import load_obstacles
    trees = [(cx, cy, rmax) for (n, cx, cy, rmax, _p) in load_obstacles(a.geo) if 'tree' in n]
    if not trees:
        print('✗ 几何里找不到 tree* ⇒ 拒绝出数 ✗')
        return 2

    def xy(k):
        return ox + ((k % W) + 0.5) * res, oy + ((k // W) + 0.5) * res

    def d_path(cx, cy):
        return min(math.hypot(cx - px, cy - py) for px, py in traj)

    def occluded(cx, cy, R):
        for px, py in traj:
            for tx, ty, tr in trees:
                d, inside = seg_dist(px, py, cx, cy, tx, ty)
                if inside and d < R + tr:      # ★ 半径用**实测**（树干 ~0.3 m ✓）
                    return True
        return False

    # ---- ④ 对照：★ 按"到轨迹的距离"分层配对 ✓ ----
    rnd = random.Random(20261011)
    res_sample = [(xy(k), k) for k in rnd.sample(residue, min(len(residue), 800))]
    res_info = [{'xy': p, 'd': d_path(*p)} for p, _ in res_sample]
    bins = {}
    for it in res_info:
        bins.setdefault(int(it['d'] // 2.0), 0)
        bins[int(it['d'] // 2.0)] += 1
    ctrl = []
    free_sample = rnd.sample(free, min(len(free), 40000))
    by_bin = {}
    for k in free_sample:
        p = xy(k)
        d = d_path(*p)
        by_bin.setdefault(int(d // 2.0), []).append(p)
    for b, need in bins.items():
        pool = by_bin.get(b) or []
        if not pool:
            continue
        ctrl += [pool[rnd.randrange(len(pool))] for _ in range(min(need, len(pool) * 3))]
    n_ctrl = a.n_control or len(res_info)
    ctrl = [{'xy': p, 'd': d_path(*p)} for p in ctrl[:n_ctrl]]

    out = {'probe_dir': os.path.abspath(a.probe_dir), 'max_seq': a.max_seq, 'geo': os.path.abspath(a.geo),
           'n_residue_total': len(residue), 'n_free_total': len(free),
           'n_trees': len(trees), 'radius_m': a.radius,
           'n_traj_points': len(traj), 'n_residue_sampled': len(res_info), 'n_control': len(ctrl),
           'trees_xy_r': [[round(x, 3), round(y, 3), round(r, 3)] for x, y, r in trees],
           'convention': ('★ 遮挡 = 从**轨迹任一点**到该格的线段离某树干**轴**< R 且交点在线段内 ✓；'
                          '★ 对照按"到轨迹距离"**每 2 m 分层配对** ✓（★ 不配距离会把"离路远"误读成"被挡" ✗）'),
           'd_path_hist_residue': {str(k): v for k, v in sorted(bins.items())}}
    for R in (a.radius, a.radius * 1.5):
        ro = sum(1 for it in res_info if occluded(it['xy'][0], it['xy'][1], R))
        co = sum(1 for it in ctrl if occluded(it['xy'][0], it['xy'][1], R))
        out['R=%.2f' % R] = {'residue_occluded': ro, 'residue_n': len(res_info),
                             'residue_rate': ro / max(1, len(res_info)),
                             'control_occluded': co, 'control_n': len(ctrl),
                             'control_rate': co / max(1, len(ctrl)),
                             'ratio': (ro / max(1, len(res_info))) / max(1e-9, co / max(1, len(ctrl)))}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    for R in (a.radius, a.radius * 1.5):
        v = out['R=%.2f' % R]
        print('  ★ R=%.2f m ⇒ 残留被挡 %d/%d（%.1f%%）｜ 对照 %d/%d（%.1f%%）｜ ★ 比值 %.2f'
              % (R, v['residue_occluded'], v['residue_n'], 100 * v['residue_rate'],
                 v['control_occluded'], v['control_n'], 100 * v['control_rate'], v['ratio']))
    print('  ★ 轨迹点 %d ｜ 树干 %d ｜ 残留 %d（抽样 %d）｜ 对照 %d'
          % (len(traj), len(trees), len(residue), len(res_info), len(ctrl)))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

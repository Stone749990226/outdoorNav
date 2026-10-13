#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""footprint_test.py —— ★ "车自己的脚印"逐格对比（★ 纯离线 ✓）

用法: footprint_test.py <probe目录> --max-seq N [--unmatched <json>] --out json

★★ 要判什么（reviewer 2026-10-11 ✓）★★
  ★ 假设：★ **车把自己的脚印写进图、清除射线又擦不掉** ✓
  ★ 判据：★ "未匹配那批"**踩中脚印**的比例 **显著高于**对照 ✓
  ★ 两头标定 ✓：★ 半径给**两档以上**（0.105 = `robot_radius` ✓ ＋ 更大 ✓）看**灵不灵敏** ✓。

★★ 但这里有一个**必须写明的坑** ✗（★ 我自己的判据要两头看 ✓）★★
  ★ 单比"未匹配 vs 匹配上的"**部分是循环论证** ✗ —— ★ "未匹配"那批**本来就是按"贴着轨迹"选出来的**
    （★ 到轨迹中位 0.154 m ✓）⇒ ★ 它们当然更容易"落在脚印里" ✗。
  ⇒ ★ 所以我加两条**非循环**的量 ✓：
    ★ ① **同距离配对的自由格对照** ✓（★ 按"到轨迹的距离"分层 ✓ ⇒ 比的是"谁被标成致命" ✓，不是"谁离车近" ✓）
    ★ ② ★★ **"扫过的格 ⇒ 最终致命的占比"（= 粘住率 ✓）在两版/两场景之间比** ✓✓
      —— ★ 这一条**与"谁贴得近"无关** ✓，★ 直接回答"**为什么他们那边粘得多**" ✓。

★ 口径 ✓：★ 轨迹按 **0.05 m 加密**（★ 抽样间隔 ~0.2–0.4 m ⇒ 不加密会漏掉车压过的格 ✗）；
　★ "格被扫到" = ★ **格心**到某时刻车心 ≤ R ✓（★ 格是 0.1×0.1 ⇒ 半对角 0.0707 m ✓ ⇒
　　★ 半径档里跨过这个数 ⇒ 正好覆盖"格心 vs 格重叠"两种口径 ✓）。
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
    ap.add_argument('--unmatched', default=None)
    ap.add_argument('--out', required=True)
    ap.add_argument('--radii', default='0.105,0.20,0.30,0.50')
    a = ap.parse_args()
    R = [float(x) for x in a.radii.split(',')]

    ticks = sorted([t for t in glob.glob(os.path.join(a.probe_dir, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    meta = json.load(open(ticks[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    H = None
    b0 = open(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(ticks[0])), 'rb').read()
    b1 = open(os.path.join(a.probe_dir, 'map_%04d.bin' % seq_of(ticks[-1])), 'rb').read()
    H = len(b1) // W
    L = lambda v: 99 <= v <= 100
    added = {k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])}

    pts = [tuple(json.load(open(t, encoding='utf-8'))['robot_xy_map']) for t in ticks
           if json.load(open(t, encoding='utf-8')).get('robot_xy_map')]
    dense = densify(pts, 0.05)

    unm = set()
    if a.unmatched and os.path.isfile(a.unmatched):
        unm = set(json.load(open(a.unmatched, encoding='utf-8'))['cells'])
    matchedc = added - unm if unm else set()

    def dtraj(cx, cy):
        return min(math.hypot(cx - px, cy - py) for px, py in dense)

    def cell_xy(k):
        return ox + ((k % W) + 0.5) * res, oy + ((k // W) + 0.5) * res

    # ---- ① 扫过的格（每个 R 一份 ✓）----
    swept = {}
    for r in R:
        s = set()
        for px, py in dense:
            i0 = int((px - r - ox) / res)
            i1 = int((px + r - ox) / res)
            j0 = int((py - r - oy) / res)
            j1 = int((py + r - oy) / res)
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    if 0 <= i < W and 0 <= j < H:
                        k = j * W + i
                        cx, cy = cell_xy(k)
                        if math.hypot(cx - px, cy - py) <= r:
                            s.add(k)
        swept[r] = s

    out = {'probe_dir': os.path.abspath(a.probe_dir), 'max_seq': a.max_seq,
           'n_traj_samples_raw': len(pts), 'n_traj_samples_dense': len(dense),
           'n_added': len(added), 'n_unmatched': len(unm), 'n_matched_control': len(matchedc),
           'radii_m': R,
           'note': ('★ "格被扫到" = 格心到某时刻车心 ≤ R ✓；★ 轨迹按 0.05 m 加密 ✓；'
                    '★ 半径档跨过格半对角 0.0707 m ⇒ 同时覆盖"格心/格重叠"两种口径 ✓'),
           'sticky_rate': {}, 'footprint_hit': {}}

    # ---- ② 粘住率：扫过的格 ⇒ 最终成为致命 ✓（★ 与"谁贴得近"无关 ✓）----
    for r in R:
        s = swept[r]
        hit = len(s & added)
        out['sticky_rate']['R=%.3f' % r] = {
            'n_swept': len(s), 'n_swept_now_lethal': hit,
            'sticky_rate': hit / max(1, len(s)),
            'sticky_per_m': hit / max(1e-9, sum(math.dist(dense[i], dense[i + 1])
                                                for i in range(len(dense) - 1)))}

    # ---- ③ 三组的"踩中脚印"比例（★ 含同距离配对的对照 ✓，把循环性摊开 ✓）----
    rnd = random.Random(20261011)
    free = [k for k in range(len(b1)) if b1[k] == 0]
    fs = rnd.sample(free, min(len(free), 8000))
    fs_d = {k: dtraj(*cell_xy(k)) for k in fs}
    for r in R:
        s = swept[r]
        def rate(cells):
            cells = list(cells)
            if not cells:
                return None
            return sum(1 for k in cells if k in s) / len(cells)
        # ★ 同距离配对对照：按"到轨迹距离"每 0.25 m 分层，配到与未匹配同分布 ✓
        bins = collections.Counter(int(fs_d.get(k, 0) / 0.25) for k in unm) if unm else {}
        ctrl = []
        pool = collections.defaultdict(list)
        for k in fs:
            pool[int(fs_d[k] / 0.25)].append(k)
        for b, need in bins.items():
            p = pool.get(b) or []
            if p:
                ctrl += [p[rnd.randrange(len(p))] for _ in range(need)]
        out['footprint_hit']['R=%.3f' % r] = {
            'unmatched': rate(unm), 'matched_control': rate(matchedc),
            'random_free': rate(rnd.sample(free, min(len(free), 2000))),
            'free_same_distance_control': rate(ctrl),
            'n_free_same_distance_control': len(ctrl),
        }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)

    print('  ★ 粘住率（扫过的格 ⇒ 最终致命）：')
    for r in R:
        v = out['sticky_rate']['R=%.3f' % r]
        print('     R=%.3f m ⇒ 扫过 %6d 格 ｜ 变致命 %4d（%.2f%% ✓）' %
              (r, v['n_swept'], v['n_swept_now_lethal'], 100 * v['sticky_rate']))
    print('  ★ "踩中脚印"比例（未匹配 / 匹配对照 / 随机自由 / 同距离配对自由）：')
    for r in R:
        v = out['footprint_hit']['R=%.3f' % r]
        print('     R=%.3f m ⇒ %.1f%% ｜ %.1f%% ｜ %.1f%% ｜ ★ %.1f%%（n=%d）' %
              (r, 100 * (v['unmatched'] or 0), 100 * (v['matched_control'] or 0),
               100 * (v['random_free'] or 0), 100 * (v['free_same_distance_control'] or 0),
               v['n_free_same_distance_control']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""residue_source_range.py —— ★ 残留格是**被多远的回波**标上的？（★ 纯离线 ✓）

用法: residue_source_range.py <probe目录> --max-seq N --out json

★★ 这个问题为什么值钱（reviewer 2026-10-11 (b)① ✓）★★
  ★ "在他们场景里，激光多看到了什么" ✗ ⇒ ★ 最直接的问法是：
    ★ **把每一格残留追回到"标上它的那条回波"** ✓，看那些回波的 **距离分布** ✓。
  · 若集中在 **近距（≲1 m）** ⇒ ★ 机制在**近场**（地面/自身/家具的掠射 ✓）
  · 若集中在 **远距（4–8 m）** ⇒ ★ 机制在**远处**（稀疏回波被标成点 ✓）
★ 口径（写死 ✓）：
  · ★ 前向：先把**每一拍的回波落在哪些格**建成索引（`tick → {cell: range}` ✓），
    再对每一格残留取**它最早被标上那一拍**的匹配 ✓（★ 归因必须取"第一次被标"那一刻 ✓，
    ✗ 取末尾会把所有格都归到"最后一拍没有回波" ✗ —— 本项目已记过 ✓）。
  · ★ 匹配半径 0.15 m（= 探针那套口径 ✓）
"""
import argparse
import glob
import json
import math
import os
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--match-m', type=float, default=0.15)
    ap.add_argument('--cloud', default='scan', choices=['scan', 'cam'])
    ap.add_argument('--window', type=int, default=0, help='allow +-N ticks when matching')
    ap.add_argument('--control', default='real', choices=['real', 'random'], help='random = null-hypothesis control (draw unrelated ticks)')
    ap.add_argument('--dump-unmatched', default=None, help='write unmatched cell indices + map meta')
    a = ap.parse_args()
    P = a.probe_dir
    ticks = sorted([t for t in glob.glob(os.path.join(P, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(P, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    if not ticks:
        print('✗ 没有"既有 tick 又有 map"的拍 ⇒ 拒绝出数 ✗')
        return 2
    meta = json.load(open(ticks[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']

    maps = {seq_of(t): os.path.join(P, 'map_%04d.bin' % seq_of(t)) for t in ticks}
    b0 = open(maps[seq_of(ticks[0])], 'rb').read()

    # ① 逐拍建索引：回波落在哪些格（含到传感器的距离 ✓）
    idx = {}       # tick -> {cell: min_range}
    for t in ticks:
        s = seq_of(t)
        f = os.path.join(P, '%s_%04d.xyz' % (a.cloud, s))
        if not os.path.isfile(f):
            continue
        d = json.load(open(t, encoding='utf-8'))
        M = ((d.get(a.cloud) or {}).get('tf') or {}).get('M')
        if not M:
            continue
        org = (M[0][3], M[1][3], M[2][3])
        m = {}
        for line in open(f, encoding='utf-8'):
            v = line.split()
            if len(v) != 3:
                continue
            x, y, z = float(v[0]), float(v[1]), float(v[2])
            gx = int((x + 0.0) // 1)  # 占位（下面按世界→格换算 ✓）
            gi = int((x - ox) / res)
            gj = int((y - oy) / res)
            r = math.dist((x, y, z), org)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if abs(di) + abs(dj) > 1:
                        continue
                    k = (gj + dj) * W + (gi + di)
                    if 0 <= k < len(b0):
                        if k not in m or r < m[k]:
                            m[k] = r
        idx[s] = m

    # ② 找"第一次被标上"那一拍
    # ★ 性能（2026-10-11 实测 ✓）：逐格扫 1.775 M × 595 拍 = 10 亿次比较 ✗ 太慢 ✓
    #   ⇒ 改用**稀疏正则**一次抓出致命格的位置 ✓（本项目在 `residue_analyze.py` 上用过同一招 ✓）
    import re as _re
    LETH = _re.compile(rb'[\x63\x64]')          # 值 99 / 100 ✓
    lethal_sets = {}
    for t in ticks:
        s = seq_of(t)
        lethal_sets[s] = {m.start() for m in LETH.finditer(open(maps[s], 'rb').read())}
    first_mark = {}
    prev_set = None
    for t in ticks:
        s = seq_of(t)
        cur = lethal_sets[s]
        if prev_set is not None:
            for k in cur - prev_set:
                if k not in first_mark:
                    first_mark[k] = s
        prev_set = cur
    # ★ 第一拍之前就已致命的（例如探针起晚）不算"新增" ✓ —— 与指标同口径 ✓

    # ★ 2026-10-11 修口径 ✗：上面那个 `first_mark` 是"**这一轮里曾经**变致命的格"（并集 ✓，
    #   含"被清掉又重标"的 ✓）⇒ 比指标的 `n_added` 大得多 ✗。
    #   ★ 指标口径 = **末拍致命 ∧ 起始拍不致命** ✓ ⇒ 这里必须**取交集** ✓（否则两个数对不上 ✓）。
    b_end = open(maps[seq_of(ticks[-1])], 'rb').read()
    L = lambda v: 99 <= v <= 100
    target = {k for k in range(len(b_end)) if L(b_end[k]) and not L(b0[k])}
    first_mark = {k: s for k, s in first_mark.items() if k in target}

    # null-hypothesis control (reviewer 2026-10-11 (b)b): in 'random' mode each cell is
    # matched against an UNRELATED tick, same window width
    import random as _rnd
    _rng = _rnd.Random(20261011)
    seq_list = [seq_of(t) for t in ticks]
    matched, unmatched = [], 0
    for k, s in first_mark.items():
        m = idx.get(s) or {}
        # ★ ✗ 不做"附近凑一个"的兜底 ✗（那会把距离分布带偏 ✓）—— 只在**该拍**的索引里找 ✓
        cand = []
        for ds in range(-a.window, a.window + 1):
            base = s
            if a.control == 'random':
                base = seq_list[_rng.randrange(len(seq_list))]
            mm = idx.get(base + ds)
            if mm and k in mm:
                cand.append(mm[k])
        if cand:
            matched.append(min(cand))
        else:
            unmatched += 1
    matched.sort()

    # ★ 2026-10-11（reviewer）：把"**任何拍都匹配不上**"的那些格**落盘** ✓
    #   （✗ 只报个数不够 ✓ —— 下一步要问"它们落在图的哪里" ✓）
    if a.dump_unmatched:
        un = [k for k, s2 in first_mark.items() if not any(
            (idx.get(s2 + ds) or {}).get(k) is not None for ds in range(-a.window, a.window + 1))]
        if a.control == 'random':
            un = []
        json.dump({'probe_dir': os.path.abspath(P), 'window': a.window, 'control': a.control,
                   'n_unmatched': len(un), 'cells': un, 'width': W, 'resolution': res,
                   'origin': {'x': ox, 'y': oy},
                   'end_map': os.path.basename(maps[seq_of(ticks[-1])]),
                   'start_map': os.path.basename(maps[seq_of(ticks[0])]),
                   'traj_xy': [[round(x, 4), round(y, 4)] for x, y in [tuple(p2) for p2 in
                               [json.load(open(t, encoding='utf-8')).get('robot_xy_map') for t in ticks]
                               if p2]]},
                  open(a.dump_unmatched, 'w', encoding='utf-8'), ensure_ascii=False)
        print('  ★ 未匹配格已落盘：%d 个 ⇒ %s' % (len(un), a.dump_unmatched))

    def q(p):
        return matched[int(len(matched) * p)] if matched else None
    out = {
        'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'cloud': a.cloud, 'window_ticks': a.window, 'control': a.control,
        'n_first_mark_cells': len(first_mark), 'n_matched_to_a_return': len(matched),
        'n_unmatched': unmatched, 'match_m': a.match_m,
        'range_m': {'min': matched[0] if matched else None, 'p10': q(0.10), 'median': q(0.5),
                    'p90': q(0.90), 'max': matched[-1] if matched else None},
        'bins': {k: sum(1 for v in matched if lo <= v < hi) for k, (lo, hi) in
                 (('0-0.5', (0, 0.5)), ('0.5-1', (0.5, 1)), ('1-2', (1, 2)), ('2-4', (2, 4)),
                  ('4-6', (4, 6)), ('6-7.5', (6, 7.5)), ('>=7.5(填充)', (7.5, 1e9)))},
        'convention': ('★ 追到"该格**最早被标上**那一拍"的回波 ✓（✗ 不用末尾 ✗）；'
                       '★ 匹配半径 0.15 m ✓；★ 每格取该拍最近的那条回波 ✓'),
    }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 新增致命格 %d ⇒ 匹配到回波 %d（未匹配 %d）' % (len(first_mark), len(matched), unmatched))
    print('  ★ 回波距离：中位 %s m ｜ p10 %s ｜ p90 %s' % (out['range_m']['median'], out['range_m']['p10'], out['range_m']['p90']))
    for k, v in out['bins'].items():
        print('     %-12s %5d（%.1f%%）' % (k, v, 100.0 * v / max(1, len(matched))))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

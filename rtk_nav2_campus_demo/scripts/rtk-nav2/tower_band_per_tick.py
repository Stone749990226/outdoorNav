#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tower_band_per_tick.py —— ★ 主楼西立面那一带的 <b>逐拍</b>回波（★ 四轮＋空场 ✓）

★★ 为什么要"逐拍" ✗（2026-10-11 定式 ✓）★★
  ★ 绝对个数**分不开** ✗ —— ★ 它随"抽了几拍"变 ✓（cutS1 抽 61 拍 ⇒ 383 个 ✓；
    ★ cutS2 抽 66 拍 ⇒ 496 个 ✓）⇒ ★ 两个**本该完全不同**的场景给出**同量级**的绝对值 ✗。
  ★ 而归一成"每拍"后 ✓：★ **有楼 51.37／32.10** ✓ ⇒ ★ **无楼（只去掉楼）6.28** ✓ ⇒
    ★ **空场（只留地面）7.52** ✓ ⇒ ★★ **那 ~6–7.5/拍 是"地板"** ✓。
★★ 地板是什么：★ 激光**下倾通道打到地面**的回报 ✓（★ 与"铺满全场的地面盒子会让判据恒真"同族 ✓）——
  ★ 所以 ★ "楼没了 ⇒ 该带应为 **0**" 这个预期**是错的** ✗ ⇒ ★ 正确读法是**掉落量** ✓
  （★ −87.8%／−80.4% ✓，★ 而不是"归零" ✗）。
"""
import glob
import json
import os
import re
import sys


def seq(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def end_seq_of(run_dir):
    """★ 该轮的**右边界** ✓（= 指标自己的窗口末端 ✓）

    ★★ 为什么必须有 ✗：★ 探针（1500 s ✓）比这一轮活得久 ⇒ 直接扫**活的** `probe/` 会让
      ★ 抽样步长与取到的拍**随墙钟变** ✗ —— ★ 实测：★ 同一个 cutS2，★ 两次跑出 **8.06／9.48** ✗
      （★ 抽 72 拍 vs 61 拍 ✓）⇒ ★ 与"还在长的文件不能算整段统计"**同族** ✗（★ 我这轮栽第二次 ✓）。
    """
    for cand in ('probe_frozen/dynamic_lethal_delta.json', 'probe/dynamic_lethal_delta.json'):
        p = os.path.join(run_dir, cand)
        if os.path.isfile(p):
            return json.load(open(p, encoding='utf-8')).get('end_seq')
    return None


def band(run_dir, sample=60, max_seq=None):
    ticks = sorted(glob.glob(os.path.join(run_dir, 'probe/tick_*.json')), key=seq)
    if max_seq is not None:
        ticks = [t for t in ticks if seq(t) <= max_seq]
    ticks = ticks[::max(1, len(ticks) // sample)]
    facade = near = 0
    zs = []
    used = 0
    for t in ticks:
        f = os.path.join(run_dir, 'probe/scan_%04d.xyz' % seq(t))
        if not os.path.isfile(f):
            continue
        used += 1
        for line in open(f, encoding='utf-8'):
            v = line.split()
            if len(v) != 3:
                continue
            x, y, z = float(v[0]), float(v[1]), float(v[2])
            if -40.0 <= y <= -8.25:
                if 11.9 <= x <= 13.0:
                    facade += 1
                    zs.append(z)
                if 11.9 <= x <= 14.0:
                    near += 1
    zs.sort()
    return {'n_sampled_ticks': used, 'n_points_in_band': facade,
            'per_tick': (facade / used) if used else None,
            'z_median': (zs[len(zs) // 2] if zs else None),
            'points_x_11.9_14.0': near}


def main():
    base = sys.argv[1]
    runs = sys.argv[2:]
    out = {}
    for r in runs:
        d = os.path.join(base, r)
        if not os.path.isdir(d):
            continue
        ms = end_seq_of(d)
        out[r] = band(d, max_seq=ms)
        out[r]['right_boundary_max_seq'] = ms
        b = out[r]
        print('  ★ %-18s 右边界 %s ｜ 抽 %3d 拍 ｜ 带内 %6d 个 ⇒ ★ **%.2f 每拍** ✓ ｜ z 中位 %s'
              % (r, ms, b['n_sampled_ticks'], b['n_points_in_band'], b['per_tick'],
                 ('%.4f' % b['z_median']) if b['z_median'] is not None else '—'))
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())

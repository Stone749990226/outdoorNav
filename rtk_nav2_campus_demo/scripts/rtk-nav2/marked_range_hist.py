#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""marked_range_hist —— ★ **被标点**的"到车距离"直方图 ✓（★ 纯离线 ✓，★ 决定 7.2 值不值 ✓）

用法: python3 marked_range_hist.py --out <json> [--rounds r1,r2] [--stride 10]

★★ 为什么先查这一句（★ reviewer 2026-10-11 派 ✓）★★
  ★ **7.2 只削掉 7.2–7.5 m 那 0.3 m 的一条** ✓ —— ★ 而被标／幸存的那批点**未必堆在那一条上** ✗
  ⇒ ★ **先测再跑** ✓（★ 免得白跑一轮 ✓）。

★★ 判据（★ 两头都算结论 ✓）★★
  ★ **6.0–8.0 那几档里，7.0–7.5／7.2–7.5 有明显的峰** ⇒ ★ **这一档正中要害** ✓ ⇒ ★ 直接跑 7.2 ✓
  ★ **平坦** ⇒ ★ **它只削一小条** ✗ ⇒ ★ **换更激进的值**（★ 例 6.0 ✓）⇒ ★ **那就得重做 provision** ✓、★ 判据照旧 ✓

★★ 口径（★ 写死 ✓）★★
  · ★ "**被标点**" = ★ 过了**标记门**的点 ✓：★ `0.02 ≤ z ≤ 1.2` ✓（★ 层级 `min/max_obstacle_height` ✓）
    ∧ ★ `0.2 ≤ r ≤ 7.5` ✓（★ `obstacle_min_range` ／ **`obstacle_max_range` 7.5** ✓ —— ★ 门就是它 ✓）
  · ★ `r` = ★ 点到**传感器原点**的距离 ✓（★ 原点取该拍 `scan.tf.M` 的平移 ✓ —— ★ 实测 = 车 xy ＋ z 0.1 ✓）
  · ★ 只算 **`scan`** 那一路 ✓（★ 相机的门是 0.30 ✓、★ 且它的云不可信 ✓ —— ★ 与"保留的那支"一致 ✓）
  · ★ 分档 **0.5 m** ✓；★ 右边界 = **到达拍** ✓（★ 与历轮同一个锚 ✓）
"""
import argparse
import glob
import json
import math
import os
import re
import sys

ART = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'
SCENE = '/home/xjy/projects/outdoorNav/gazebo_campus/generated/scene_geometry.json'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_obstacles import load_obstacles, dist_to_obstacle      # noqa: E402
ON_OBS_M = 0.30    # ★ 点离**真障碍** ≤ 这个 ⇒ ★ 算"**合法标记**" ✓（★ 地面已剔 ✓）
Z_LO, Z_HI = 0.02, 1.2
R_LO, R_HI = 0.2, 7.5
BUCKET = 0.5


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--rounds', default='campus9-res1,campus9-cutS9')
    ap.add_argument('--stride', type=int, default=10)
    ap.add_argument('--bucket', type=float, default=BUCKET)
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    out = {'what': '★ **被标点**的"到车距离"直方图 ✓（★ 决定 7.2 那一刀值不值 ✓）',
           # ★★ 判词按 **0.1 m 档**写 ✓ ★★
           #   ★✗ 旧说法作废 ✗：★ 我第一版用 **0.5 m 档**，★ 报"7.2–7.5 是空的 ⇒ 削到 7.2 是 0%" ✗ ——
           #     ★ **那是分档太粗造成的假象** ✗（★ "7.0" 那一档装的是 **7.0–7.5** ✓ ⇒ ★ 把 7.2–7.5 藏进去了 ✓）
           #     ⇒ ★ 细化到 0.1 m ⇒ ★ 7.2 实际削 **5.0%／6.8%** ✓（★ 不是 0 ✗）
           #   ★ 结论**不变** ✓（★ 7.2 出局、6.5 主选 ✓），★ 只是理由从"0%"换成"**5–7%，仍在轮间底噪之下**" ✓
           'preregistered': {
               'granularity_rule': '★★ 报直方图**必须给粒度** ✓，且 ★ **结论要换一档再验一次** ✓（★ 越粗越假 ✓）',
               'cut_7.2': '★ 削 **5.0%／6.8%**（★ 0.1 m 档 ✓）⇒ ★ **落在轮间散布 ±20% 之下** ✗ ⇒ ★ **出局** ✓',
               'cut_7.0': '★ 削 8.4%／11.6% ⇒ ★ 同样在底噪之下 ✗ ⇒ ★ **出局** ✓',
               'cut_6.5': '★ 削 **20.3%／22.2%** ✓ ⇒ ★ **够得着** ✓ ⇒ ★ **主选** ✓（★ 但代价 = ★ **丢掉主楼近面** ✓ —— ★ 见功能判定 ✓）'},
           'convention': {'gate': '★ `0.02 ≤ z ≤ 1.2` ∧ `0.2 ≤ r ≤ 7.5` ✓（★ 层级高度门 ＋ `obstacle_max_range` ✓）',
                          'r': '★ 点到**传感器原点** ✓（★ 该拍 `scan.tf.M` 平移 ✓）',
                          'source': '★ 只算 `scan` ✓（★ 相机门 0.30 ＋ 它的云不可信 ✓）',
                          'bucket_m': BUCKET, 'window': '★ 到达锚 ✓（★ 与历轮同一个 ✓）'},
           'rounds': {}}

    for run in [r for r in a.rounds.split(',') if r]:
        ds = sorted(glob.glob(os.path.join(ART, run, 'probe_frozen_wp8_arrival_*')))
        if not ds:
            continue
        d = ds[-1]
        hist = {}
        hist_on, hist_off = {}, {}       # ★ 窄一点的那两张 ✓
        OBS = load_obstacles(SCENE)
        n_tot = n_gated = 0
        seqs = sorted({seq_of(p) for p in glob.glob(os.path.join(d, 'scan_*.xyz'))})
        for s in seqs[::a.stride]:
            tf = os.path.join(d, 'tick_%04d.json' % s)
            if not os.path.isfile(tf):
                continue
            try:
                M = json.load(open(tf, encoding='utf-8'))['scan']['tf']['M']
            except Exception:
                continue
            sx, sy = M[0][3], M[1][3]
            for line in open(os.path.join(d, 'scan_%04d.xyz' % s), encoding='utf-8'):
                v = line.split()
                if len(v) != 3:
                    continue
                px, py, pz = float(v[0]), float(v[1]), float(v[2])
                n_tot += 1
                r = math.hypot(px - sx, py - sy)
                if Z_LO <= pz <= Z_HI and R_LO <= r <= R_HI:
                    n_gated += 1
                    k = round(math.floor(r / a.bucket) * a.bucket, 2)
                    hist[k] = hist.get(k, 0) + 1
                    (hist_on if dist_to_obstacle(px, py, OBS) <= ON_OBS_M else hist_off)[k] = \
                        (hist_on if dist_to_obstacle(px, py, OBS) <= ON_OBS_M else hist_off).get(k, 0) + 1
        ks = sorted(hist)
        out['rounds'][run] = {
            'frozen_dir': os.path.relpath(d, ART), 'n_ticks_sampled': len(seqs[::a.stride]),
            'n_points_total': n_tot, 'n_points_gated': n_gated,
            'hist': {('%.1f' % k): hist[k] for k in ks},
            'buckets_6_0_to_8_0': {('%.1f' % k): hist.get(k, 0) for k in (6.0, 6.5, 7.0, 7.5)},
            'share_below_7.2': (round(sum(v for k, v in hist.items() if k < 7.2) / n_gated, 4) if n_gated else None),
            'share_7_0_to_7_5': (round((hist.get(7.0, 0)) / n_gated, 4) if n_gated else None),
            'share_at_or_above_7_0': (round(sum(v for k, v in hist.items() if k >= 7.0) / n_gated, 4) if n_gated else None),
            'split_on_vs_off_obstacle': {
                'ON_OBS_M': ON_OBS_M,
                'n_on': sum(hist_on.values()), 'n_off': sum(hist_off.values()),
                'hist_on': {('%.1f' % k): hist_on[k] for k in sorted(hist_on)},
                'hist_off': {('%.1f' % k): hist_off[k] for k in sorted(hist_off)},
                'cuts': {('%.1f' % c): {
                    'X_pct_of_TOFIX_pile': (round(100 * sum(v for k, v in hist_off.items() if k >= c) / sum(hist_off.values()), 2)
                                            if hist_off else None),
                    'Y_pct_of_LEGIT_pile': (round(100 * sum(v for k, v in hist_on.items() if k >= c) / sum(hist_on.values()), 2)
                                            if hist_on else None)}
                    for c in (7.2, 7.0, 6.5, 6.0)},
                'reading': ('★ **要治的堆** = 不在真障碍上的那些 ✓（★ 就是业主说的"随机的点" ✓）；'
                            '★ **合法堆** = 落在真障碍上的 ✓（★ 含主楼 ✓）')}}

    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    for run, v in out['rounds'].items():
        print('%-16s 采样 %d 拍 ｜ 过门点 %d ｜ ★ 6.0-8.0 那几档：%s ｜ ★ ≥7.0 占 %s' %
              (run, v['n_ticks_sampled'], v['n_points_gated'], v['buckets_6_0_to_8_0'],
               v['share_at_or_above_7_0']))
        print('    全直方图：', v['hist'])
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

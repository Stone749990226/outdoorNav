#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""true_return_cost —— ★ 把 `obstacle_max_range` 削到某一档，**丢多少真回波** ✓（★ 纯离线 ✓）

用法: python3 true_return_cost.py --out <json> [--rounds r1,r2] [--stride 10]

★★ 为什么（★ reviewer 2026-10-11 定 ✓）★★
  ★ 7.2 出局（★ 直方图那一档是 **0 个点** ✓）；★ 7.0 出局（★ 效应 8–12% 落在轮间散布 ±20% 之下 ✗）
  ★ **主选 6.5**（★ 削掉 **20–22%** 的过门点 ✓）★ 备选 6.0 ✓
  ⇒ ★★ 但 ★ **6.5 的"丢真回波"代价没测过** ✗ ⇒ ★ 本工具量它 ✓（★ ✗ 不许插值猜 ✗）

★★ 尺子怎么定 ＋ ★ **先拿文档 §9 的已知值标定** ✓ ★★
  · ★ "**丢真回波**" = ★ 在**真回波**里，★ **距离 > 该档** 的占比 ✓
  · ★ **真回波** = ★ 该拍的 `scan` 点里 ★ **`r < 7.99`** ✓（★ ≥7.99 是**填充值** ✓ —— ★ 本项目已定死的 artifact ✓）
  · ★ `r` = 点到**传感器原点** ✓（★ 该拍 `scan.tf.M` 平移 ✓）
  ★★ 标定 ✓（★ 文档 §9 第 267 行 ✓）：★ 7.2 ⇒ **1.14%** ✓／7.0 ⇒ **2.21%** ✓／6.0 ⇒ **11.10%** ✓／5.0 ⇒ **20.27%** ✓
    ⇒ ★ **本工具跑同几档，对得上才算尺子对** ✓；★ 对不上 ⇒ ★ 先查尺子 ✗、★ **不拿 6.5 那个数当结论** ✗。
"""
import argparse
import glob
import json
import math
import os
import re
import sys

ART = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'
FILL_MIN = 7.99
CUTS = (7.2, 7.0, 6.5, 6.0, 5.0)
KNOWN = {'7.2': 1.14, '7.0': 2.21, '6.0': 11.10, '5.0': 20.27}


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--rounds', default='campus9-res1,campus9-res2,campus9-cutS9')
    ap.add_argument('--stride', type=int, default=10)
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    out = {'what': '★ `obstacle_max_range` 各档的"**丢真回波**"代价 ✓（★ 量、✗ 不插值 ✓）',
           'ruler': '★ 真回波里「★ r > 该档」的占比 ✓；★ 真回波 = `r < %.2f` ✓（≥ 那个是**填充值** ✓）' % FILL_MIN,
           'calibration_against_docs_s9': KNOWN, 'rounds': {}}
    for run in [r for r in a.rounds.split(',') if r]:
        ds = sorted(glob.glob(os.path.join(ART, run, 'probe_frozen_wp8_arrival_*')))
        if not ds:
            continue
        d = ds[-1]
        rs = []
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
                r = math.hypot(float(v[0]) - sx, float(v[1]) - sy)
                if r < FILL_MIN:
                    rs.append(r)
        n = len(rs)
        out['rounds'][run] = {
            'n_true_returns': n, 'r_max_m': round(max(rs), 3) if rs else None,
            'cost_pct': {('%.1f' % c): (round(100 * sum(1 for x in rs if x > c) / n, 3) if n else None)
                         for c in CUTS}}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('%-16s %9s %8s | %s' % ('round', '真回波数', 'max_m', '  '.join('削到%.1f⇒丢' % c for c in CUTS)))
    for run, v in out['rounds'].items():
        print('%-16s %9d %8s | %s' % (run, v['n_true_returns'], v['r_max_m'],
                                      '  '.join('%s%%' % v['cost_pct']['%.1f' % c] for c in CUTS)))
    print('★ 文档 §9 的已知值（★ 用来标定）：', KNOWN)
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dynamic_lethal_delta.py —— ★ 把"**静态先验层**"贡献的那部分致命格**减掉** ✓（阶段十第 0 刀必需 ✓）

用法: dynamic_lethal_delta.py <probe目录> <起始seq> <末seq> [--out 新json]

★★ 为什么必需（★ 2026-10-11 实测，第 0 刀第一个大坑 ✗）★★
  ★ `lethal_per_meter.py` 直接数"末拍整幅图里的致命格" ✓ —— 这对**新版**成立 ✓（新版**没有**
    `static_layer` ✓，图开始时是空的 ✓，所以每一格致命都是**动态观测**来的 ✓）。
  ★ 但**旧版有 `static_layer`** ✗ ⇒ 光**静态先验**就先把整个校园的楼标成致命 ✓。
    实测：★ 开车**之前**（seq=2）就已经 **285,349** 个致命格 ✓；开车**之后**（seq=616）**284,505** ✗
    —— ★ 比开始时**还少 844 个** ✗。
  ⇒ ★ 拿 284,505 去和 1536／1676 比 ⇒ **差 170 倍** ✗，可这不是"残留"，是**楼** ✗✗。
  ⇒ ★ 可比量只能是"**开车过程中**新出现的致命格" ✓ =
        `#{末拍致命 ∧ 起始拍不致命}` ✓（★ 用**集合差**，✗ 不用计数差 ✗ —— 因为还有"被清掉"的 ✓）

★ 口径（写死 ✓）：
  · 「起始拍」= 探针**最早一拍**（车还没动 ✓）⇒ 它里面是**静态先验 ＋ 出生点附近的车自身标记** ✓
    （★ 后者一起被减掉 ⇒ 这个减法**偏保守** ✓）
  · 「末拍」= 开车结束那一拍（★ 由 `freeze_probe_snapshot.py` 定 ✓）
  · ★ 输出**三个数**（✗ 不给单一个净额 ✗）：`n_added` ✓／`n_removed` ✓／`n_lethal_end` ✓
    ＋ 每米的 `added_per_m` ✓（分母同样记 ✓）
"""
import argparse
import glob
import json
import math
import os
import re
import sys


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


LETHAL = lambda b: (b >= 99) if b < 128 else False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--min-seq', type=int, default=None, help='起始拍（默认 = 最早一拍 ✓）')
    ap.add_argument('--max-seq', type=int, default=None, help='末拍（默认 = 最后一拍 ✓）')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    maps = sorted(glob.glob(os.path.join(a.probe_dir, 'map_*.bin')), key=seq_of)
    if len(maps) < 2:
        print('✗ 图不够两张'); return 2
    m0 = next((m for m in maps if a.min_seq is None or seq_of(m) >= a.min_seq), maps[0])
    m1 = next((m for m in reversed(maps) if a.max_seq is None or seq_of(m) <= a.max_seq), maps[-1])
    b0 = open(m0, 'rb').read()
    b1 = open(m1, 'rb').read()
    if len(b0) != len(b1):
        print('✗ 两张图尺寸不同（%d vs %d）⇒ 栅格变了 ⇒ 拒绝出数 ✗' % (len(b0), len(b1)))
        return 2
    n_added = n_removed = n_end = 0
    for x, y in zip(b0, b1):
        lx = 99 <= x <= 100
        ly = 99 <= y <= 100
        if ly:
            n_end += 1
            if not lx:
                n_added += 1
        elif lx:
            n_removed += 1

    ticks = sorted(glob.glob(os.path.join(a.probe_dir, 'tick_*.json')), key=seq_of)
    traj = 0.0
    prev = None
    for f in ticks:
        if seq_of(f) > seq_of(m1):
            break
        d = json.load(open(f, encoding='utf-8'))
        r = d.get('robot_xy_map')
        if r and prev:
            traj += math.hypot(r[0] - prev[0], r[1] - prev[1])
        if r:
            prev = r

    out = {
        'probe_dir': os.path.abspath(a.probe_dir),
        'start_map': os.path.basename(m0), 'end_map': os.path.basename(m1),
        'start_seq': seq_of(m0), 'end_seq': seq_of(m1),
        'n_lethal_start': sum(1 for x in b0 if 99 <= x <= 100),
        'n_lethal_end': n_end,
        'n_added': n_added,                      # ★ 开车过程中**新出现**的致命格 ✓
        'n_removed': n_removed,                  # ★ 开车过程中**被清掉**的致命格 ✓
        'traj_len_m': round(traj, 3),            # ★ 分母 ✓
        'added_per_m': (round(n_added / traj, 4) if traj > 1e-9 else None),
        'metric': '★ added_per_m = n_added ÷ traj_len_m ✓（★ 静态先验层的贡献已被"起始拍"减掉 ✓）',
        'convention': ('★ 起始拍 = 探针最早一拍（车未动 ✓，含静态先验 ＋ 出生点附近的自标记 ✓）；'
                       '末拍 = 开车结束那一拍 ✓（由 freeze_probe_snapshot.py 定 ✓）；'
                       '★ 用集合差（逐格比）✗ 不用计数差 ✓'),
        'why': ('★ 旧版有 static_layer ⇒ 光楼就 285,349 个致命格 ✗ ⇒ 不减掉它，'
                '285k 与 1.5k 的对比是"楼 vs 残留"✗，不是"旧版 vs 新版" ✓'),
    }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 起始拍 %s：致命 %d ｜ 末拍 %s：致命 %d' % (out['start_map'], out['n_lethal_start'],
                                                        out['end_map'], out['n_lethal_end']))
    print('  ★ 开车中**新增**致命 = %d ✓ ｜ **被清掉** = %d ✓ ｜ 分母 traj = %.2f m ✓'
          % (n_added, n_removed, traj))
    print('  ★★ added_per_m = %s ✓（✗ 不是"末拍致命÷米"✗）' % out['added_per_m'])
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

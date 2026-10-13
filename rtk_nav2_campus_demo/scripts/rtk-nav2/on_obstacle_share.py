#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""on_obstacle_share.py —— ★ 新增的致命格里，**多少落在真障碍上** ✗（把"真观测"与"幻影"分开 ✓）

用法: on_obstacle_share.py <本轮probe目录> <本轮max_seq> <参照"静态先验"图>

★★ 为什么需要（★ 2026-10-11 第 1 刀实测 ✓）★★
  ★ 指标 `added_per_m` 数的是"开车中**新出现**的致命格" ✓ —— 它**同时包含两类** ✗：
    · **真障碍**被看到 ✓（合理 ✓）· **开阔地上的幻影** ✗（业主抱怨的正是这个 ✗）
  ⇒ ★ 换场景会**同时**改变两件事 ✗：① 幻影多不多 ② **真障碍本来有多少** ✓
  ⇒ ★ 所以"残留掉了一半"**不足以**说明幻影少了一半 ✗ —— ★ 必须把两类分开 ✓。
  ★ 分开的**便宜办法**（✗ 不用解析 SDF ✓）：★ 第 0 刀那张图里，
    **静态先验层**已经把 campus_100 的**楼**逐格标成致命 ✓ ⇒ 那份集合 ≈ "**真障碍**" ✓。
    ★ 而且第 0 刀与第 1 刀的栅格**逐字相同**（都 1200x1200 @0.10 origin(-60,-60) ✓）⇒
    ★ **可以直接逐格比** ✓✓。
  ★ 于是：`on_obstacle_share = |新增 ∩ 静态先验致命| ÷ |新增|` ✓
    ⇒ ★ 剩下那部分 = **开阔地上的新增致命** ✓ = 业主说的"残留" ✓。
"""
import glob
import json
import os
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    probe, max_seq, ref_map = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    maps = sorted(glob.glob(os.path.join(probe, 'map_*.bin')), key=seq_of)
    m0 = maps[0]
    m1 = next(m for m in reversed(maps) if seq_of(m) <= max_seq)
    b0 = open(m0, 'rb').read()
    b1 = open(m1, 'rb').read()
    ref = open(ref_map, 'rb').read()
    if not (len(b0) == len(b1) == len(ref)):
        print('✗ 三张图尺寸不同（%d/%d/%d）⇒ 拒绝出数 ✗' % (len(b0), len(b1), len(ref)))
        return 2
    L = lambda v: 99 <= v <= 100
    added = [k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])]
    on_obs = [k for k in added if L(ref[k])]
    ref_lethal = sum(1 for v in ref if L(v))
    hit_ref = sum(1 for k in range(len(ref)) if L(ref[k]) and L(b1[k]))
    out = {
        'probe_dir': os.path.abspath(probe),
        'start_map': os.path.basename(m0), 'end_map': os.path.basename(m1),
        'ref_static_map': os.path.abspath(ref_map),
        'n_added': len(added),
        'n_added_on_obstacle': len(on_obs),
        'frac_added_on_obstacle': (len(on_obs) / len(added)) if added else None,
        'n_added_in_open': len(added) - len(on_obs),
        'frac_added_in_open': ((len(added) - len(on_obs)) / len(added)) if added else None,
        'n_ref_static_lethal': ref_lethal,
        'n_ref_static_hit_by_this_round': hit_ref,
        'frac_ref_static_hit': (hit_ref / ref_lethal) if ref_lethal else None,
        'convention': ('★ "真障碍" ≈ 第 0 刀那张图的**静态先验致命格**（= campus_100 的楼 ✓，逐格 ✓）；'
                       '★ 两轮栅格逐字相同（1200x1200 @0.10 origin(-60,-60) ✓）⇒ 可直接比 ✓'),
        'note': ('★ 这条是**补充**，✗ 不改 `added_per_m` 的定义 ✗ —— 它把"新增"再分成'
                 '「落在真障碍上 ✓」与「落在开阔地 ✗（= 残留 ✓）」两部分 ✓'),
    }
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(probe)),
                                     'on_obstacle_share.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('  ★ 新增致命 %d ⇒ 落在**真障碍**上 %d（%.1f%% ✓）｜ 落在**开阔地** %d（%.1f%% ✗ = 残留 ✓）'
          % (len(added), len(on_obs), 100.0 * len(on_obs) / max(1, len(added)),
             len(added) - len(on_obs), 100.0 * (len(added) - len(on_obs)) / max(1, len(added))))
    print('  ★ 参照静态先验致命 %d 格 ⇒ 本轮覆盖到 %d（%.1f%%）' %
          (ref_lethal, hit_ref, 100.0 * hit_ref / max(1, ref_lethal)))
    print('  ★ 已落盘 %s/on_obstacle_share.json' % os.path.dirname(os.path.abspath(probe)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

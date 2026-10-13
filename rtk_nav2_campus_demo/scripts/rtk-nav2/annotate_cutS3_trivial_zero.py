#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""annotate_cutS3_trivial_zero.py —— ★ 把"第 3 刀是**平凡 0**"写进产物 ✓

★★ 为什么必须写 ✗（reviewer 2026-10-11 ✓）★★
  ★ 第 3 刀 = ★ 空场 ＋ 加回 26 根树干 ✓ ⇒ ★ 结果必然是 **≈0** ✓ —— ★ 但那个 0 **不是证据** ✗：
    ★ 26 根树干**全部 ≥19.31 m** ✓，★ 而激光 `raytrace_max_range` **8.0** ✓／`obstacle_max_range` **7.5** ✓
    ⇒ ★ **它们根本照不到** ✗ ⇒ ★ 残留为 0 是**几何上的必然** ✓，★ 与"树干必不必要"**无关** ✗。
  ★★ 不写这一句 ⇒ ★ 后人读到 `sticky_rate = 0.00%` 会读成"**树干不必要**" ✗✗
    （★ 这正是本项目那条：★ **平凡通过／恒过必须写清** ✓；★ 与"该拦的拦住"配套 ✓）。

用法: annotate_cutS3_trivial_zero.py <run目录> [--apply]
      默认 **只打印** ✓；--apply 才写 ✓（★ 两头标定：不写时不动任何东西 ✓）。
"""
import argparse
import json
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    R = a.run_dir
    note = {
        'verdict_on_this_round': '★★ **平凡 0 —— 不作为证据** ✗',
        'what_this_round_was': '★ 空场 ＋ 加回 26 根树干（★ `collision` 27 ＝ ground 1 ＋ tree 26 ✓）',
        'why_the_zero_is_trivial': (
            '★ 26 根树干**全部 ≥19.31 m** 离航线 ✓（★ 见 `bisect-offline/cutS3_tree_trunks_vs_route.json` ✓）；'
            '★ 而激光 `raytrace_max_range` **8.0** ✓／`obstacle_max_range` **7.5** ✓ ⇒ '
            '★ **没有任何一根够得着** ✗ ⇒ ★ 加回它们**不可能**产生回波 ✓ ⇒ ★ 残留为 0 是**几何必然** ✓'),
        'must_not_be_read_as': '★ ✗ **"树干不必要"** ✗ —— ★ 本刀对这个命题**没有分辨力** ✓'
                               '（★ 它连"照到过"都没发生过 ✓）',
        'what_it_is_good_for': ('★ 一条**有效负对照** ✓：★ 证实"**加回射程外的几何，残留不回来**" ✓ —— '
                                '★ 即"**加回式二分之前，先判那一组能不能被传感器看到**" ✓'),
        'methodology_entry': ('★★ 定式 ✓：★ **"加回式二分"之前，先算"那一组离航线最近多少米"** ✓，'
                              '★ 与传感器射程比 ✓；★ 够不着 ⇒ ★ 那一刀**必然是平凡 0** ✗ ⇒ ★ 别跑 ✗'
                              '（★ 每次都会白烧一轮共享容器 ✓）'),
        'source_artifacts': {
            'family_distance_to_route': 'bisect-offline/family_distance_to_route.json',
            'tree_trunks_vs_route': 'bisect-offline/cutS3_tree_trunks_vs_route.json',
            'scene_edit': 'campus9-cutS3/scene_edit.json',
        },
        'convention': '★ 本注**附着于这一轮的产物** ✓ —— ★ 引用 `sticky_rate` 时必须同时带本注 ✓',
    }
    gp = os.path.join(R, 'guardrails.json')
    if not os.path.isfile(gp):
        print('✗ 还没有 guardrails.json ⇒ 这一轮还没跑完 ⇒ 先不写 ✗'); return 2
    g = json.load(open(gp, encoding='utf-8'))
    if 'verdict_trivial_zero' in g:
        print('★ 已经写过 ✓ ⇒ 不重复覆盖 ✗'); return 3
    print('★ 将写入这些字段 ✓：')
    print(json.dumps(note, ensure_ascii=False, indent=2)[:1200])
    if not a.apply:
        print('\n★ 只是预演 ✓（★ 未写任何东西 ✓）—— 加 `--apply` 才写 ✓'); return 0
    g['verdict_trivial_zero'] = note
    json.dump(g, open(gp, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('\n★ 已写入 %s ✓' % gp)
    return 0


if __name__ == '__main__':
    sys.exit(main())

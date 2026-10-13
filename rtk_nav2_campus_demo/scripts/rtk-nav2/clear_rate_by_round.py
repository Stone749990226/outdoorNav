#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""clear_rate_by_round.py —— ★ 各轮"**清掉的格 / 米**" ✓（★ 纯离线 ✓，★ 用冻结 dump ✓）

用法: python3 clear_rate_by_round.py --out <json>

★★ 为什么（★ reviewer 2026-10-11 派 ✓）★★
  ★ 落点已经摆在那儿 ✓：★ **stock 批**（★ 残留**更低**那批 ✓）★ **更爱在航点附近磨**
    （★ 航点 1.5 m 内多走 **+7 m** ✓）＋ ★ **航段上鼓得更外**（★ 侧向偏移 p95 均 **2.17 m** vs **1.34 m** ✓）
  ⇒ ★★ 两样都是**走法形状** ✓ ⇒ ★ 最省的读法 ✓：★ **"多绕的那些路是多扫出来的"** ✓
    ★ 单元格被清掉的**唯一途径是某条射线扫过它** ✓ ⇒ ★ **多磨多鼓 ⇒ 扫的方向更多 ⇒ 幸存者更少** ✓✓
  ⇒ ★ 它顺带解释了 ★ **为什么"去掉地面回波"反而更糟** ✓（★ 射线变少 ⇒ 扫得少 ⇒ 幸存更多 ✓✓）

★★ 判据两头（★ 先写死 ✓）★★
  ★ **stock 的"清/米"显著更高** ⇒ ★★ **"扫过多少"坐实** ✓✓ ⇒ ★ 残留的**正解是"多扫"而不是"少标"** ✗
    ⇒ ★ **修法方向整个换** ✓（★ 例如 ★ 保持航点处的磨 ✓／★ 提高清除射线的覆盖 ✓）
  ★ **不高** ⇒ ★ 否 ✓

★★ 口径（★ 写死 ✓）★★
  · ★ **清事件** = 相邻两张 dump 之间，★ 某格 **上一拍 ∈ 致命 ∧ 这一拍 ∉ 致命** ✓（★ 逐格比 ✓，★ 累加 ✓）
    ★ 致命 = 值 ∈ [99, 127] ✓（★ 与 `dynamic_lethal_delta.py` **同一套** ✓；★ 255 **不是**致命 ✓）
  · ★ **加事件** = 反向那一条 ✓（★ 一并报 ✓ —— ★ 与先前 `clear/add` 同族 ✓）；★★ 主判据用**率**（÷ 米 ✓），✗ 不用比 ✗
  · ★ 右边界 = **到达拍** ✓（★ 与 `residue_at_wp8_anchor.py` **同一个锚** ✓）；★ 米用**探针口径**那一栏 ✓
    （★ 同一窗口 ✓；★ ✗ 不用轨迹口径 ✗ —— ★ 它跨尺有 +2.4…+3.3% 的系统差 ✓）
"""
import argparse
import glob
import json
import os
import re
import sys

ART = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'
LETHAL_LO, LETHAL_HI = 99, 127


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--anchor-json', default=ART + '/campus9-cutS9/residue_at_wp8_anchor.json')
    ap.add_argument('--rounds', default='campus9-res1,campus9-res2,campus9-cutS1,campus9-cutS5,'
                                        'campus9-cutS6,campus9-cutS7,campus9-cutS8,campus9-cutS9')
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    A = json.load(open(a.anchor_json, encoding='utf-8'))
    out = {'what': '★ 各轮 **清掉的格 ／ 米** ✓（★ 率，✗ 不是比 ✗）',
           'why': ('★ stock 批（残留更低）更爱**磨航点 +7 m** ＋ **航段外鼓 2.17 vs 1.34 m** ⇒ '
                   '★ 最省的读法 = **"多绕的那些路是多扫出来的"** ✓'),
           'preregistered': {
               'stock_clear_per_m_higher': '★★ **"扫过多少"坐实** ✓✓ ⇒ ★ 残留正解是"**多扫**"而不是"少标" ✗ ⇒ ★ 修法方向整个换 ✓',
               'not_higher': '★ 否 ✓'},
           'convention': {
               'clear_event': '★ 相邻两张 dump 之间，某格「上一拍致命 ∧ 这一拍不致命」✓（★ 逐格比、累加 ✓）',
               'add_event': '★ 反向那一条 ✓（★ 一并报 ✓）',
               'lethal': '★ 值 ∈ [%d, %d] ✓（★ 与 `dynamic_lethal_delta.py` 同一套 ✓；★ 255 不是致命 ✓）'
                         % (LETHAL_LO, LETHAL_HI),
               'window': '★ 探针最早一拍 → **到达拍** ✓（★ 与 `residue_at_wp8_anchor.py` 同一个锚 ✓）',
               'metres': '★ 用**探针口径**的 `traj_len_m` ✓（★ 同一窗口 ✓；★ ✗ 不用轨迹口径 ✗ —— ★ 它跨尺有 +2.4…+3.3% 系统差 ✓）'},
           'rounds': {}, 'notes': []}

    for run in [r for r in a.rounds.split(',') if r]:
        w = (A['rounds'].get(run, {}).get('windows') or {}).get('arrival') or {}
        cut = w.get('cut_seq'); traj = w.get('traj_len_m')
        if cut is None or not traj:
            out['notes'].append('✗ %s：锚里没有到达窗 ⇒ 跳过' % run)
            continue
        maps = [f for f in sorted(glob.glob(os.path.join(ART, run, 'probe_frozen', 'map_*.bin')), key=seq_of)
                if seq_of(f) <= cut]
        if len(maps) < 2:
            maps = [f for f in sorted(glob.glob(os.path.join(ART, run, 'probe', 'map_*.bin')), key=seq_of)
                    if seq_of(f) <= cut]
        if len(maps) < 2:
            out['notes'].append('✗ %s：窗口里少于两张图 ⇒ 跳过' % run)
            continue
        n_clear = n_add = 0
        prev = None
        for m in maps:
            cur = open(m, 'rb').read()
            if prev is not None:
                for x, y in zip(prev, cur):
                    lx = LETHAL_LO <= x <= LETHAL_HI
                    ly = LETHAL_LO <= y <= LETHAL_HI
                    if lx and not ly:
                        n_clear += 1
                    elif ly and not lx:
                        n_add += 1
            prev = cur
        out['rounds'][run] = {
            'cut_seq': cut, 'n_map_pairs': len(maps) - 1,
            'first_map': os.path.basename(maps[0]), 'last_map': os.path.basename(maps[-1]),
            'traj_len_m': traj,
            'n_clear_events': n_clear, 'n_add_events': n_add,
            'clear_per_m': round(n_clear / traj, 3), 'add_per_m': round(n_add / traj, 3),
            'net_added_from_anchor_json': w.get('n_added'),
            'clear_over_add': round(n_clear / n_add, 4) if n_add else None,
            'n_lethal_start_anchor_json': w.get('n_lethal_start')}

    STOCK = ['campus9-res1', 'campus9-res2', 'campus9-cutS1', 'campus9-cutS8']
    EDIT = ['campus9-cutS5', 'campus9-cutS6', 'campus9-cutS7', 'campus9-cutS9']
    cm = {}
    for k in ('clear_per_m', 'add_per_m', 'clear_over_add', 'n_clear_events'):
        s = [out['rounds'][r][k] for r in STOCK if r in out['rounds'] and out['rounds'][r].get(k) is not None]
        e = [out['rounds'][r][k] for r in EDIT if r in out['rounds'] and out['rounds'][r].get(k) is not None]
        if not s or not e:
            continue
        ms = sum(s) / len(s)
        margin = (min(e) - max(s)) if min(e) > max(s) else ((min(s) - max(e)) if min(s) > max(e) else None)
        cm[k] = {'stock_mean': round(ms, 4), 'edited_mean': round(sum(e) / len(e), 4),
                 'stock_range': [round(min(s), 4), round(max(s), 4)],
                 'edited_range': [round(min(e), 4), round(max(e), 4)],
                 'gap_pct_of_stock': round(100 * (sum(e) / len(e) - ms) / ms, 1) if ms else None,
                 'margin_abs': round(margin, 4) if margin is not None else None,
                 'margin_rel': (round(margin / ms, 4) if (margin is not None and ms) else None),
                 'separates_ge5pct': bool(margin is not None and ms and margin / ms >= 0.05)}
    out['cohort_compare'] = {'stock_rounds': STOCK, 'edited_rounds': EDIT, 'metrics': cm}
    v = cm.get('clear_per_m') or {}
    # ★★ 符号别搞反 ✗ ★★
    #   ★ `gap_pct_of_stock` = **（edited − stock）÷ stock** ✓ ⇒ ★ **负值 = stock 更高** ✓✓
    #   ★ 我第一版写成 `> 0` ⇒ ★ 把"stock 更高（−20.1% ✓）"读成了"没更高" ✗ ⇒ **判词整个反** ✗。
    gp = v.get('gap_pct_of_stock') or 0
    out['verdict'] = {
        'gap_definition': '★ `gap_pct_of_stock` = **（edited − stock）÷ stock** ✓ ⇒ ★ **负 = stock 更高** ✓',
        'stock_clear_per_m_higher': bool(gp < 0),
        'separates_le5pct': v.get('separates_ge5pct'),
        'reading': ('★★ **stock 的"清/米"更高** ✓（★ %.1f vs %.1f ⇒ ★ stock **%+.1f%%** ✓）'
                    '⇒ ★ 按预登记 ⇒ **"扫过多少"坐实** ✓✓ ⇒ ★ 残留正解是"**多扫**"而不是"少标" ✗ '
                    '⇒ ★ **修法方向整个换** ✓'
                    % (v.get('stock_mean', 0), v.get('edited_mean', 0), -gp)
                    if gp < 0 else
                    '★ **stock 的"清/米"没更高** ✗（★ %.1f vs %.1f ✓）⇒ ★ 否 ✓'
                    % (v.get('stock_mean', 0), v.get('edited_mean', 0)))}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)

    print('%-16s %9s %11s %11s %11s %9s' %
          ('round', 'traj_m', 'clear/m', 'add/m', 'clear_evt', 'clr/add'))
    for run, vv in out['rounds'].items():
        print('%-16s %9.3f %11.3f %11.3f %11d %9.4f' %
              (run, vv['traj_len_m'], vv['clear_per_m'], vv['add_per_m'],
               vv['n_clear_events'], vv['clear_over_add'] or -1))
    for k, vv in cm.items():
        print('  ★ %-16s stock %10.4f ／ edited %10.4f ⇒ %+6.1f%% ｜ 余量(相对) %s ｜ 分开≥5%%=%s'
              % (k, vv['stock_mean'], vv['edited_mean'], vv['gap_pct_of_stock'],
                 vv['margin_rel'], vv['separates_ge5pct']))
    print('★ 判词：%s' % out['verdict']['reading'])
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

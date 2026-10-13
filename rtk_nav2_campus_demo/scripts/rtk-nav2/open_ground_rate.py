#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""open_ground_rate.py —— ★ 「开阔地残留 个/米」这一行 ✓（★ 把两把现成的尺子拼起来 ✓）

用法: open_ground_rate.py <本轮probe目录> <参照静态先验图> --out <json>

★★ 为什么是"拼"而不是"自己算" ★★
  ★ 分子来自 `on_obstacle_share.py`（★ 把"新增致命格"分成**真障碍**与**开阔地** ✓），
    分母来自 `dynamic_lethal_delta.py`（★ 它定义 `traj_len_m` = 逐 `tick_*.json` 的
    `robot_xy_map` 位移和 ✓）—— ★ 两个都是**已经在用的尺子** ✓
  ⇒ ★ 本脚本**只调这两个工具** ✓、把它们的输出拼成一行 ✓
    （★ `on_obstacle_share.py` 落盘在 `probe/../on_obstacle_share.json` ✓，
       `dynamic_lethal_delta.py` 用 `--out` 指定 ✓）⇒ ✗ **不自己重算任何一格** ✗
    （★ 重算 = 换了一把尺子 ⇒ 与既有轮次不可比 ✗）。

★★ 报的时候**连分母一起报** ✓：`n_added_in_open` ／ `traj_len_m` ⇒ `open_per_m` ✓；
   ★ 同时报 `n_added` 与 `n_added_on_obstacle` ⇒ 复算方能自己核 ✓。
★ ★ 参照图的选取 ✓：同一棵树 ＋ 同一个场景的轮次之间比 ⇒ 参照图 = **本轮自己的 `map_0002.bin`**
   （★ 静态先验在里面 ✓）；★ 跨场景/跨树比 ⇒ ✗ 参照图必须同为那张 static 先验 ✗。
"""
import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def run(tool, args):
    p = subprocess.run([sys.executable, os.path.join(HERE, tool)] + args,
                       capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError('%s 退出码 %d：%s' % (tool, p.returncode, (p.stderr or p.stdout)[-400:]))
    return p.stdout.strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('ref_static_map')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    probe = os.path.abspath(a.probe_dir)
    parts = a.out + '.parts'
    os.makedirs(parts, exist_ok=True)
    d_out = os.path.join(parts, 'delta.json')

    print(run('dynamic_lethal_delta.py', [probe, '--out', d_out]))
    # ★ max_seq 给一个大数 = 取**最后一拍** ✓（✗ 给 0 会 StopIteration ✗ —— 实测栽过一次 ✓）
    print(run('on_obstacle_share.py', [probe, '999999999', os.path.abspath(a.ref_static_map)]))
    d = json.load(open(d_out, encoding='utf-8'))
    o = json.load(open(os.path.join(os.path.dirname(probe), 'on_obstacle_share.json'),
                       encoding='utf-8'))

    traj = d['traj_len_m']
    n_open = o['n_added_in_open']
    out = {
        'probe_dir': probe,
        'ref_static_map': os.path.abspath(a.ref_static_map),
        'window': {'start_map': d['start_map'], 'end_map': d['end_map'],
                   'start_seq': d['start_seq'], 'end_seq': d['end_seq']},
        'n_added': o['n_added'],
        'n_added_on_obstacle': o['n_added_on_obstacle'],
        'n_added_in_open': n_open,
        'traj_len_m': traj,
        'open_per_m': (n_open / traj) if traj else None,
        'added_per_m': d['added_per_m'],
        'n_lethal_end': d['n_lethal_end'],
        'n_removed': d['n_removed'],
        'ruler': ('★ 分子 = on_obstacle_share.py 的 n_added_in_open（开阔地上的新增致命格 ✓）；'
                  '★ 分母 = dynamic_lethal_delta.py 的 traj_len_m（逐 tick 的 robot_xy_map 位移和 ✓）'
                  '⇒ ★ 两个工具都是既有尺子 ✓，本脚本不重算 ✓'),
    }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('\n★ 开阔地残留 个/米 = %d ／ %.3f m = **%.4f**' % (n_open, traj, out['open_per_m']))
    print('  （★ 连分母一起报 ✓：新增致命 %d ⇒ 真障碍 %d ／ 开阔地 %d ✓；'
          '★ 整体 added_per_m = %.4f ✓）' % (o['n_added'], o['n_added_on_obstacle'], n_open,
                                              d['added_per_m']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""freeze_probe_snapshot.py —— 把探针产物**冻结到某一拍为止**（用软链，✗ 不复制、✗ 不删原件 ✓）

用法: freeze_probe_snapshot.py <probe目录> <新目录> <max_seq>

★★ 为什么需要（★ 2026-10-11 第 0 刀实测 ✓）★★
  ★ 探针是按**墙钟**跑 N 秒的 ✓，而**开车早就跑完了** ✗ ⇒ 车停在终点期间探针**继续拍** ✓。
  ★ 实测后果两条（都量过 ✓）：
    · **图本身不再变** ✓（`map_0554/0570/0581.bin` sha256 **完全相同** ✓）⇒ 分子稳定 ✓
    · 但 `robot_xy_map` **还在跳毫米级** ✗（-40.2456 → -40.2481 ⇒ 定位抖动 ✓）
      ⇒ ★ 而 `traj_len_m` 是**相邻拍位移之和** ✗ ⇒ 停着不动的 30 多分钟也会往上长 ✗
      ⇒ **分母被虚增** ⇒ 指标被压低 ✗（★ 跨轮比就失真了 ✓）。
  ★ 所以：**统一截到"图不再变"的那一拍为止** ✓ —— 那正是"**开车结束那一刻的图**" ✓。
  ★ 用**软链**而不是复制 ✓：① 不占空间 ✓ ② 原件一个字节不动 ✓ ③ 截到哪一拍**写在产物里** ✓。

★ 口径（写死，便于复算 ✓）：
  · 取 `map_*.bin` 的 sha256，从**末拍往前**找**最后一次变化**的那一拍 seq = `cut_seq` ✓
  · 快照 = 所有 `seq <= cut_seq` 的 `map_*/tick_*/scan_*/cam_*` ✓（软链 ✓）
"""
import glob
import hashlib
import json
import os
import re
import sys


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def main():
    probe, out, cut = sys.argv[1], sys.argv[2], int(sys.argv[3])
    maps = sorted(glob.glob(os.path.join(probe, 'map_*.bin')), key=seq_of)
    if not maps:
        print('✗ 读不到 map_*.bin')
        return 2
    # ★ 从末拍往前找"最后一次变化" ✓（= 图最后被更新的那一拍 ✓）
    last_change = seq_of(maps[0])
    prev = None
    for m in maps:
        h = hashlib.sha256(open(m, 'rb').read()).hexdigest()
        if prev is not None and h != prev:
            last_change = seq_of(m)
        prev = h
    os.makedirs(out, exist_ok=True)
    n = 0
    for pat in ('map_*.bin', 'tick_*.json', 'scan_*.xyz', 'cam_*.xyz', 'map_*.json'):
        for f in glob.glob(os.path.join(probe, pat)):
            s = seq_of(f)
            if s < 0 or s > cut:
                continue
            dst = os.path.join(out, os.path.basename(f))
            if os.path.islink(dst) or os.path.exists(dst):
                os.remove(dst)
            os.symlink(os.path.abspath(f), dst)
            n += 1
    meta = {'probe_dir': os.path.abspath(probe), 'snapshot_dir': os.path.abspath(out),
            'cut_seq': cut, 'n_links': n, 'n_map_files_total': len(maps),
            'last_map_change_seq': last_change,
            'convention': ('★ 软链 ✓（原件不动 ✓）；★ 只链 seq <= cut_seq ✓；'
                           '★ cut_seq 取"图最后一次变化的那一拍" ✓ = 开车结束那一刻 ✓'),
            'why': ('★ 探针按墙钟跑 ✓，车停后它还在拍 ✗；图不变但 robot_xy_map 仍抖毫米级 ✗ '
                    '⇒ traj_len_m 会被虚增 ⇒ 指标被压低 ✗ ⇒ 必须截到"图不再变"那一拍 ✓')}
    json.dump(meta, open(os.path.join(out, 'snapshot_meta.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('  ★ 快照 = %d 个软链（cut_seq=%d ✓；图最后一次变化在 seq=%d ✓，总拍数 %d ✓）'
          % (n, cut, last_change, len(maps)))
    print('  ★ cut_seq == last_map_change_seq ? %s' % (cut == last_change))
    return 0


if __name__ == '__main__':
    sys.exit(main())

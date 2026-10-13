#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""guardrail_map_variation.py —— ★ ⑤ 观测层的正确判据：★ **图的内容在变** ✓（✗ 不是"文件在、大小对" ✗）

用法: guardrail_map_variation.py <probe目录> --max-seq N --out json

★★ 为什么换掉旧的（★ reviewer 2026-10-11 ✓，★ 我那条是错的 ✗）★★
  ★ 我原来写 ✓："★ `map_*.bin` **1,775,000 字节** = 他们场景几何" ✗
  ★ 实测（★ 只读、不碰容器 ✓）：★ **三套完全不同的几何**（★ 他们完整 ✓／去掉主楼 ✓／只剩地面 ✓）
    ★ **大小逐字节相同 = 1,775,000** ✗ —— ★ 而 ★ **1250 × 1420 = 1,775,000** ✓ ⇒
    ★ 那就是一个**定尺寸的整张栅格 dump** ✓ ⇒ ★ **大小恒等于格子数 × 1 字节** ✓
    ⇒ ★★ **"字节数 = 场景几何"是**恒真**的** ✗ ⇒ ★ 对每一轮、每一拍都成立 ⇒ **没有分辨力** ✓
    （★ 本项目那条：★ **恒真的判据等于没有判据** ✓；★ 与"发出信号 ≠ 做成了"同族 ✓ ——
     ★ "文件在、大小对"是**信号** ✗）
★ **正确判据** ✓（★ 观测层要判的是"**图在更新**" ✓ ⇒ 落在**内容**上 ✓）：
  · ★ ① ★ **相邻两拍的致命格数**在变 ✓（★ 可比摘要 ✓）
  · ★ ② ★ **逐拍内容 sha256** 在变 ✓ —— ★ 且 ★ **要在窗口内变多次** ✓，✗ 一次不算 ✓
★ ★ **窗口留右边界** ✓：★ 只看到 `--max-seq` 为止 ✓（★ 否则拆栈后的拍会把它判死 ✓）。
"""
import argparse
import glob
import hashlib
import json
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
    ap.add_argument('--min-changes', type=int, default=3)
    a = ap.parse_args()
    P = a.probe_dir
    maps = sorted([f for f in glob.glob(os.path.join(P, 'map_*.bin')) if seq_of(f) <= a.max_seq],
                  key=seq_of)
    if not maps:
        print('✗ 读不到 map_*.bin ⇒ 记成未取到 ✗'); return 2
    hashes, sizes, lethal = [], [], []
    prev_h = None
    changes = 0
    for f in maps:
        b = open(f, 'rb').read()
        h = hashlib.sha256(b).hexdigest()
        hashes.append(h)
        sizes.append(len(b))
        lethal.append(sum(1 for v in b if 99 <= v <= 100))
        if prev_h is not None and h != prev_h:
            changes += 1
        prev_h = h
    n_lethal_changes = sum(1 for i in range(1, len(lethal)) if lethal[i] != lethal[i - 1])
    ok = (changes >= a.min_changes)
    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq,
           'window': {'first_map': os.path.basename(maps[0]), 'last_map': os.path.basename(maps[-1]),
                      'n_maps': len(maps), 'right_boundary': '★ 只看到 max_seq 为止 ✓'},
           'content_sha256_distinct': len(set(hashes)),
           'n_content_changes_adjacent': changes,
           'n_lethal_count_changes_adjacent': n_lethal_changes,
           'map_size_bytes_all_equal': len(set(sizes)) == 1,
           'map_size_bytes_value': sizes[0],
           'map_size_note': ('★ 大小 = 1250×1420 = 1775000 = **格子数 × 1 字节** ✓ ⇒ ★ **与"图里有什么"无关** ✗ '
                             '（★ 三套完全不同的几何大小逐字节相同 ✓，已实测 ✓）⇒ ★ **不能当判据** ✗'),
           'min_changes_required': a.min_changes,
           'verdict_content_is_changing': ok,
           'verdict_note': ('★ 判据 = ★ **窗口内相邻拍的内容哈希变化次数 ≥ %d** ✓（✗ 一次不算 ✓）'
                            % a.min_changes),
           'convention': ('★ 逐拍读整幅 map ✓；★ 变化计数只看**相邻两拍** ✓；★ 右边界 = max_seq ✓ '
                          '（★ 拆栈之后的拍不许进来 ✓）')}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 窗口 %s → %s（%d 张 ✓，右边界 max_seq=%d ✓）'
          % (out['window']['first_map'], out['window']['last_map'], len(maps), a.max_seq))
    print('  ★ ★ **内容哈希变化（相邻拍）%d 次** ✓ ｜ 去重后不同内容 %d 种 ｜ 致命格数变化 %d 次'
          % (changes, len(set(hashes)), n_lethal_changes))
    print('  ★ ★ 大小恒等？ %s（= %d 字节 ✓ —— ★ 那是格子数×1 ✓，**不能当判据** ✗）'
          % (out['map_size_bytes_all_equal'], sizes[0]))
    print('  ★ ★★ 判据（≥%d 次变化）⇒ %s' % (a.min_changes, '**成立：图确实在更新** ✓' if ok else '**不成立** ✗'))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

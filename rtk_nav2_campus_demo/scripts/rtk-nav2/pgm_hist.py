#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓完之后的自检：尺寸 ＋ 值域直方图（★ 用来一眼判"参数是否同源"）。

用法：pgm_hist.py <pgm 路径> <输出 json>

★ 判据（reviewer 2026-10-12 定，与交付轮那张对齐）：
  ① 尺寸同为 **1250 × 1420** ✓
  ② 值域只有 **{0, 205, 254} 三档** ✓（多出别的档 ⇒ --free/--occ 参数没同源 ⇒ 并排无效 ✗）
★ 注意别把两档"占据"混了（今天已栽过一次 ✗）：
  · pgm 的 **0 值格** ⇒ 与交付轮的 **1473** 并排（★ 也是 residue_split 的分母 ✓ ⇒ 95／1473 = 6.45%）
  · stats 的 occupied_gt0 ⇒ 与 **1877** 并排；stats 的 lethal ⇒ 与 **634** 并排
"""
import json
import sys
from collections import Counter


def read_pgm(path):
    data = open(path, 'rb').read()
    parts = []
    i = 0
    while len(parts) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        parts.append(data[i:j])
        i = j
    i += 1
    w, h = int(parts[1]), int(parts[2])
    return w, h, data[i:i + w * h]


pgm, out = sys.argv[1], sys.argv[2]
w, h, px = read_pgm(pgm)
c = Counter(px)
res = {
    'pgm': pgm,
    'size': [w, h],
    'size_ok_1250x1420': (w == 1250 and h == 1420),
    'distinct_values': sorted(c.keys()),
    'value_set_ok_subset_of_0_205_254': set(c.keys()) <= {0, 205, 254},
    'histogram': {str(k): v for k, v in sorted(c.items())},
    'zero_value_cells': c[0],
    'note': '★ pgm 的 0 值格 与交付轮的 1473 并排（也是 residue_split 的分母 ⇒ 95／1473 = 6.45%）；'
            '★ stats 的 occupied_gt0 对 1877、lethal 对 634 —— ✗ 两档别混 ✗',
}
print('★ %s' % pgm.split('/')[-1])
print('★ 尺寸 %d×%d ⇒ %s' % (w, h, '1250×1420 ✓' if res['size_ok_1250x1420'] else '✗ 不一致 ✗'))
print('★ 值域 %s ⇒ %s' % (res['distinct_values'],
                          '只有 {0,205,254} ✓ 同源' if res['value_set_ok_subset_of_0_205_254'] else '✗ 多出别的档 ⇒ 参数没同源 ✗'))
print('★ 直方图：%s' % res['histogram'])
print('★ 0 值格（＝可与 1473 并排的那一档）＝ %d' % res['zero_value_cells'])
json.dump(res, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print('★ 已落盘 %s' % out)

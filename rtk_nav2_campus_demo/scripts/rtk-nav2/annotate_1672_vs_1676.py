#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""★ 2026-10-11（reviewer 要求）：把 **1672 vs 1676** 这条口径差，**写进两边产物各一句** ✓

为什么：★ `arc_survivor.json` 报 res2 的 `n_lethal = 1672` ✓（**稳定残留集合内**的致命格 ✓）；
       ★ `lethal_per_meter.json` 报 res2 的 `n_lethal = 1676` ✓（**末拍整幅图**里的致命格 ✓）。
       ★ **两个都对、口径不同** ✓ —— 所以两边各写一句 ✓，
         免得以后有人拿 1672 与 1676 对不上、误当成"有一个算错了" ✗。

★ 只**新增**一个键 ✓（✗ 不改 `note` ✗、✗ 不改任何数字 ✗）⇒ 原产物**仍可复算** ✓；
  键名带日期 ⇒ 一眼看出是**事后注记**、不是工具产出 ✓。
"""
import json
import os
import sys

BASE = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'

ARC_NOTE = ('★ 与本轮 lethal_per_meter.json 的 n_lethal **差 4 格**（1672 vs 1676）⇒ **两个都对、口径不同** ✓：'
            '本文件的 1672 = **稳定残留集合内**的致命格（集合定义见 residue_cells.csv 的判定 ✓）；'
            'lethal_per_meter.json 的 1676 = **末拍整幅图**里的致命格 ✓ ⇒ **别把两个数对不上当成算错** ✗。')

LPM_NOTE = ('★ 与本轮 arc_survivor.json 的 n_lethal **差 4 格**（1676 vs 1672）⇒ **两个都对、口径不同** ✓：'
            '本文件的 1676 = **末拍整幅图**里的致命格 ✓；'
            'arc_survivor.json 的 1672 = **稳定残留集合内**的致命格 ✓ ⇒ **别把两个数对不上当成算错** ✗。')

TARGETS = [
    (os.path.join(BASE, 'campus9-res2/probe/arc_survivor.json'), ARC_NOTE),
    (os.path.join(BASE, 'campus9-res1/probe/lethal_per_meter.json'), LPM_NOTE),
    (os.path.join(BASE, 'campus9-res2/probe/lethal_per_meter.json'), LPM_NOTE),
]

rc = 0
for path, note in TARGETS:
    if not os.path.isfile(path):
        print('✗ 找不到 %s ⇒ 记成**未取到** ✗' % path)
        rc = 2
        continue
    with open(path, encoding='utf-8') as fh:
        d = json.load(fh)
    d['cross_reference_n_lethal_2026_10_11'] = note
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(d, fh, ensure_ascii=False, indent=2)
        fh.write('\n')
    print('✓ 已加注：%s' % path)
    print('   n_lethal=%s' % d.get('n_lethal'))
sys.exit(rc)

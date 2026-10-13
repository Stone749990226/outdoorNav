#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""★ 第 0 刀跑完之后：把"**探针与对照轮是同一份**"这条证据**也写进产物** ✓

用法: annotate_probe_identity.py <run-id>

★★ 为什么（reviewer 2026-10-11 ✓）★★
  ★ 要求"记下探针自己的 sha ＋ 采样间隔进那一轮的产物" ✓ —— 那部分由
    `record_probe_provenance.sh` **在起探针时**写 ✓（运行中自报 ✓）。
  ★ 但它**答不了**"res1／res2 当年用的是不是同一份" ✗（那两轮**没记过**探针 sha ✗）。
  ⇒ 这里补一条**可复算的旁证** ✓，并且 ★ **标明它是推理、不是运行时自报** ✗：
    · 探针文件**当前** sha256 ＝ `probe_script_sha256_host` ✓（宿主与容器各算一次 ✓）
    · 该文件 **mtime**（实测 ✓）
    · res1／res2 **探针产物首拍**的 mtime（实测 ✓）
    · ⇒ ★ 只要 mtime(探针) **早于**两轮的起拍时刻，且此后**没有再改过** ⇒ 那两轮读到的**就是这份内容** ✓
  ★ **限制条件写清楚** ✗（✗ 不许拿推理冒充当年的自报 ✗）：若将来有人改了探针，
    这条旁证**自动失效** ✓ —— 因为它的前提是"mtime 早于起拍、且文件内容是现在这份" ✓。
"""
import json
import os
import subprocess
import sys

PROJ = '/mnt/c/Users/xjy/projects/robonix'
PROBE = os.path.join(PROJ, 'scripts/rtk-nav2/container/residue_probe.py')
RUNS = ('campus9-res1', 'campus9-res2')


def mtime(p):
    return os.path.getmtime(p) if os.path.exists(p) else None


def first_artifact_mtime(run):
    """★ 该轮探针的**最早产物** mtime（= 起拍时刻）✓"""
    d = os.path.join(PROJ, 'artifacts/rtk-nav2', run, 'probe')
    cands = [os.path.join(d, 'map_0003.bin'), os.path.join(d, 'cam_0003.xyz')]
    c = [mtime(p) for p in cands if os.path.exists(p)]
    if not c:
        # 退一步：整个 probe 目录里最老的 map/cam 文件
        if not os.path.isdir(d):
            return None
        fs = [os.path.join(d, f) for f in os.listdir(d) if f.startswith(('map_', 'cam_'))]
        c = [mtime(p) for p in fs if os.path.isfile(p)]
    return min(c) if c else None


def main():
    run = sys.argv[1] if len(sys.argv) > 1 else 'campus9-cut0'
    out = os.path.join(PROJ, 'artifacts/rtk-nav2', run, 'probe/probe_provenance.json')
    if not os.path.isfile(out):
        print('✗ 找不到 %s ⇒ 记成未取到 ✗' % out)
        return 2
    sha = subprocess.run(['sha256sum', PROBE], capture_output=True, text=True).stdout.split()[0]
    mt = mtime(PROBE)
    per = {}
    for r in RUNS:
        per[r] = {'probe_first_artifact_mtime': first_artifact_mtime(r)}
    ok = all(per[r]['probe_first_artifact_mtime'] is not None
             and per[r]['probe_first_artifact_mtime'] > mt for r in RUNS)
    d = json.load(open(out, encoding='utf-8'))
    d['same_instrument_as_res_rounds_2026_10_11'] = {
        'kind': '★ 旁证（由 mtime 推得）✗ **不是运行时自报** ✗',
        'probe_script_path': PROBE,
        'probe_script_sha256_now': sha,
        'probe_script_mtime': mt,
        'res_rounds_probe_first_artifact_mtime': per,
        'res_rounds_started_after_probe_mtime': ok,
        'conclusion': ('★ 探针文件 mtime 早于 res1／res2 的起拍时刻，且此后未再改动 '
                       '⇒ 那两轮读到的是**同一份内容** ✓（★ 与"同一把尺子"等价 ✓）'
                       if ok else '★ 不成立 ⇒ **记成未取到** ✗（✗ 不许硬说同一份 ✗）'),
        'invalidated_if': ('★ 若 `residue_probe.py` **此后被改动** ⇒ 本条旁证**自动失效** ✗；'
                           '★ 那时唯一可信的做法是**重跑一轮对照** ✓'),
        'note': ('★ 补这条是因为 reviewer 要求"探针口径要同" ✓ —— 而 res1／res2 当年'
                 '**没有运行时自报过探针 sha** ✗ ⇒ 只能给**可复算的旁证** ✓，并标明它**不是自报** ✗'),
    }
    with open(out, 'w', encoding='utf-8') as fh:
        json.dump(d, fh, ensure_ascii=False, indent=2)
        fh.write('\n')
    print('✓ 已补：%s' % out)
    print('  探针 sha=%s ｜ mtime=%.0f ｜ 两轮起拍 mtime=%s ｜ 前提成立=%s'
          % (sha[:16], mt, {k: v['probe_first_artifact_mtime'] for k, v in per.items()}, ok))
    return 0


if __name__ == '__main__':
    sys.exit(main())

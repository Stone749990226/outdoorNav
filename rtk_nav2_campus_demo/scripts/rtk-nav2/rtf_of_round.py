#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rtf_of_round.py —— ★ 一轮的**实时率**（仿真时长 ÷ 墙钟时长 ✓）

用法: rtf_of_round.py <run_dir> [--out json]

★★ 为什么用**探针日志** ★★
  ★ `probe/probe.log` 每行**同时**有**墙钟时刻**（`[HH:MM:SS]` ✓）与**仿真时刻**（`sim_t=` ✓）
    ⇒ ★ 两者一比就是**那一段的实时率** ✓（★ 与 reviewer 用"起栈时刻 vs 产物末写"算的 0.80 同口径 ✓）
  ★ ★ 用"产物末写 mtime"算会把**收尾/静止**那段算进去 ✗（★ 那段仿真停着、墙钟照走 ⇒ RTF 被压低 ✗）
    ⇒ ★ 本脚本只用**探针自己覆盖的那段** ✓，★ 而且给**两点法**（首末两行 ✓）。

★ 判据 ✓：★ 报 `rtf` ＋ `sim_span` ＋ `wall_span` ＋ 取样行数 ✓（★ 复算方能自己核 ✓）。
"""
import argparse
import json
import os
import re
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    log = os.path.join(a.run_dir, 'probe', 'probe.log')
    if not os.path.isfile(log):
        print('✗ 没有 probe/probe.log ⇒ 取不到 ✗'); return 2
    pat = re.compile(r'^\[(\d+):(\d+):(\d+)\]\s+tick\s+(\d+)\s*｜\s*sim_t=([-\d.]+)')
    rows = []
    for ln in open(log, encoding='utf-8', errors='replace'):
        m = pat.match(ln.strip())
        if m:
            h, mi, s, tick, sim = m.groups()
            rows.append({'wall_s': int(h) * 3600 + int(mi) * 60 + int(s), 'tick': int(tick),
                         'sim_t': float(sim)})
    if len(rows) < 2:
        print('✗ 可用行太少（%d）⇒ 取不到 ✗' % len(rows)); return 2
    # ★ 两点法 ✓：首末两行（★ 跳过 sim 不前进的尾段 ✓ —— 那里栈已停、仿真不动 ✓）
    first = rows[0]
    last = rows[-1]
    # ★ 末点取"sim_t 还在前进"的最后一行 ✓（✗ 不用日志最后一行：栈停后仿真冻结、墙钟照走 ✗）
    last_move = rows[-1]
    for i in range(len(rows) - 1, 0, -1):
        if rows[i]['sim_t'] > rows[i - 1]['sim_t']:
            last_move = rows[i]
            break
    sim_span = last_move['sim_t'] - first['sim_t']
    wall_span = last_move['wall_s'] - first['wall_s']
    out = {'run_dir': os.path.abspath(a.run_dir), 'log': log,
           'n_lines': len(rows),
           'first': first, 'last_sim_advancing': last_move, 'last_line': last,
           'sim_span_s': sim_span, 'wall_span_s': wall_span,
           'rtf': (sim_span / wall_span) if wall_span > 0 else None,
           'rtf_naive_including_frozen_tail': ((last['sim_t'] - first['sim_t']) /
                                               (last['wall_s'] - first['wall_s'])
                                               if last['wall_s'] > first['wall_s'] else None),
           'note': ('★ rtf = 仿真时长 ÷ 墙钟时长 ✓（★ 只取"仿真还在前进"的那段 ✓ —— '
                    '★ 栈停后仿真冻结、墙钟照走 ⇒ 不剔掉会把 RTF 压低 ✗）；'
                    '★ 另给一个不剔尾段的朴素值作对照 ✓')}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('★ %s：sim %.1f s ÷ wall %.1f s ⇒ **RTF = %.3f**（朴素值 %.3f）'
          % (os.path.basename(os.path.abspath(a.run_dir)), sim_span, wall_span,
             out['rtf'] or -1, out['rtf_naive_including_frozen_tail'] or -1))
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

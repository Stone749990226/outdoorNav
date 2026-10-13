#!/usr/bin/env python3
"""活性自检（2026-10-08 reviewer 定案的新验收协议）：

**任何"没有标记 / 干净"的结论，必须同时给出"检测器当时活着"的证据** —— 否则"真干净"
和"瞎了"外观完全一样（本轮两种情形逐字段相同）。本脚本按分钟汇总一轮 `frames.jsonl`：

  frames / live(=upper_px>0) / edge_columns>0 / confirmed / floor−近场实测地面 的中位与极值

判据（任一窗口内必须成立，否则该窗口的"空"不能当证据）：
  · `live` 帧占比够高（默认 ≥50%）；或
  · 窗口内 `edge_columns>0`（真的产出过候选）；或
  · 有一个已知可检的对照物确实被检到（本轮 = 高台起点的崖线，98 列）。

另：`upper_px==0 且 edge_columns==0 且 rej_* 全 0` = **"瞎"的指纹**（判据一次都没轮到）。

用法：
  check_evidence_live.py <run_dir 或 frames.jsonl 路径> [--from HH:MM] [--to HH:MM]
"""
import argparse
import collections
import datetime
import json
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path')
    ap.add_argument('--from', dest='t_from', default=None, help='HH:MM')
    ap.add_argument('--to', dest='t_to', default=None, help='HH:MM')
    ap.add_argument('--min-live-frac', type=float, default=0.5)
    a = ap.parse_args()
    p = a.path
    if os.path.isdir(p):
        for cand in ('regime/frames.jsonl', 'regime_in_view/frames.jsonl',
                     'frames.jsonl'):
            if os.path.exists(os.path.join(p, cand)):
                p = os.path.join(p, cand)
                break
    if not os.path.exists(p):
        print('no frames.jsonl at %s' % p)
        return 1
    per = collections.OrderedDict()
    total = 0
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except Exception:
            continue
        if 'edge_columns' not in r:
            continue
        t = datetime.datetime.fromtimestamp(r['wall'])
        hhmm = t.strftime('%H:%M')
        if a.t_from and hhmm < a.t_from:
            continue
        if a.t_to and hhmm > a.t_to:
            continue
        d = per.setdefault(hhmm, {'n': 0, 'live': 0, 'ec': 0, 'conf': 0,
                                  'deltas': [], 'up0': 0, 'rej_all0_ec0': 0})
        d['n'] += 1
        total += 1
        if r.get('live'):
            d['live'] += 1
        if (r.get('upper_px') or 0) == 0:
            d['up0'] += 1
        if r['edge_columns'] > 0:
            d['ec'] += 1
        if r.get('confirmed'):
            d['conf'] += 1
        dl = r.get('floor_minus_near_ground')
        if dl is not None:
            d['deltas'].append(dl)
    print('%-6s %6s %6s %6s %6s  %-22s %s' % (
        'minute', 'frames', 'live', 'ec>0', 'conf', 'floor−近场(中位/极值)', '判读'))
    for k, v in per.items():
        frac = v['live'] / v['n'] if v['n'] else 0.0
        if v['deltas']:
            srt = sorted(v['deltas'])
            dtxt = '%.3f / [%.3f,%.3f]' % (srt[len(srt) // 2], srt[0], srt[-1])
        else:
            dtxt = 'n/a'
        alive = (frac >= a.min_live_frac) or v['ec'] > 0
        verdict = '有活性证据 ✓' if alive else '**无活性 ⇒ 空真，不能当证据**'
        print('%-6s %6d %6d %6d %6d  %-22s %s' % (k, v['n'], v['live'], v['ec'],
                                                  v['conf'], dtxt, verdict))
    print('总帧 %d；窗口内 `upper_px==0 且 edge_columns==0` 的帧 = %d（"瞎"的指纹）'
          % (total, sum(v['up0'] for v in per.values() if v['ec'] == 0)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

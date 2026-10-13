#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pitch_vs_residue.py —— ★ 把「车体俯仰」与「残留增量」放到**同一条时间轴**上 ✓

用法: pitch_vs_residue.py <run_dir> [--ref <静态先验图>] [--out <json>]

★★ 俯仰从哪来（★ 关键：**不用起重跑也有** ✓）★★
  ★ 探针逐拍的 `tick_*.json` 里，`scan.tf.M` 是 `map → 激光帧` 的 **3x4** ✓ ——
    ★ 它由 `tilt_frames` 按 **IMU 实测姿态**发布 ✓（★ 模块注释：`R_tilt = R_yaw⁻¹ · R_imu` ✓）
    ⇒ ★ **车体俯仰** = `acos(|M[2][2]|)` ✓ —— ★ 与 `arc_pitch_watcher.py` **同一把尺** ✓
  ★ （★ 注意：`robot_tf.M` 是**二维**的 ✗ —— 第三行恒 `[0,0,1,0]` ⇒ 它**没有**俯仰 ✓，
     ★ 别拿它算 ✗ —— 那是"定位给的那一份" ✓）

★★ 判据（★ 两头写死 ✓）★★
  ★ **停车事件**的定义 ✓：★ 该拍**速度 < 0.02 m/s** ∧ 之前 3 拍里出现过 **> 0.05 m/s** ✓
    （★ 速度 = 相邻两拍 `robot_xy_map` 的位移 ÷ 相邻 `sim_t` 差 ✓ —— ★ 探针 2 s 一拍 ✓）
  ★ 每个停车事件取 **±2 s**（★ 即 ±1 拍，探针 2 s/拍 ✓）内的俯仰峰值 ✓
  ★ 同时给出**全程**俯仰的 max/中位/分位 ✓ ⇒ ★ **全程 < 0.4° ⇒ 「俯仰导致残留」当场否掉** ✓
"""
import argparse
import glob
import json
import math
import os
import re
import statistics
import sys

LETHAL = lambda v: 99 <= v <= 100


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--ref', default=None, help='静态先验图（算开阔地增量用；默认用本轮首图）')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    probe = os.path.join(a.run_dir, 'probe')

    ticks = sorted(glob.glob(os.path.join(probe, 'tick_*.json')), key=seq_of)
    maps = sorted(glob.glob(os.path.join(probe, 'map_*.bin')), key=seq_of)
    recs = []
    for f in ticks:
        d = json.load(open(f, encoding='utf-8'))
        M = (d.get('scan') or {}).get('tf', {}).get('M')
        tilt = None
        if M and len(M) == 3:
            c = abs(M[2][2])
            c = max(-1.0, min(1.0, c))
            tilt = math.degrees(math.acos(c))
        recs.append({'seq': seq_of(f), 'sim_t': d.get('sim_t'),
                     'xy': d.get('robot_xy_map'), 'tilt_deg': tilt})
    recs = [r for r in recs if r['sim_t'] is not None]
    recs.sort(key=lambda r: r['sim_t'])

    # 速度（相邻两拍位移 ÷ Δt）
    for i, r in enumerate(recs):
        r['v'] = None
        if i and r['xy'] and recs[i - 1]['xy']:
            dt = r['sim_t'] - recs[i - 1]['sim_t']
            if dt > 0:
                r['v'] = math.hypot(r['xy'][0] - recs[i - 1]['xy'][0],
                                    r['xy'][1] - recs[i - 1]['xy'][1]) / dt

    # 停车事件：v < 0.02 且之前 3 拍里有 v > 0.05
    stops = []
    for i, r in enumerate(recs):
        if r['v'] is None or r['v'] >= 0.02:
            continue
        prev = [recs[j]['v'] for j in range(max(0, i - 3), i) if recs[j]['v'] is not None]
        if prev and max(prev) > 0.05:
            stops.append(i)

    # 残留：逐拍致命格数 + 与静态先验比出的「开阔地新增」
    ref_path = a.ref or (maps[0] if maps else None)
    ref = open(ref_path, 'rb').read() if ref_path and os.path.exists(ref_path) else None
    mrec = {}
    prev_lethal = None
    for m in maps:
        b = open(m, 'rb').read()
        n = sum(1 for v in b if LETHAL(v))
        n_open_new = None
        if ref is not None and len(ref) == len(b):
            n_open_new = sum(1 for i in range(len(b))
                             if LETHAL(b[i]) and not LETHAL(ref[i]))
        mrec[seq_of(m)] = {'n_lethal': n, 'n_open_new_vs_ref': n_open_new,
                           'd_lethal': (None if prev_lethal is None else n - prev_lethal)}
        prev_lethal = n

    for r in recs:
        r['map'] = mrec.get(r['seq'])

    tilts = [r['tilt_deg'] for r in recs if r['tilt_deg'] is not None]
    out = {
        'run_dir': os.path.abspath(a.run_dir),
        'ref_map': os.path.abspath(ref_path) if ref_path else None,
        'n_ticks': len(recs), 'n_maps': len(maps),
        'tilt_source': 'scan.tf.M 的 acos(|M[2][2]|)（＝ tilt_frames 按 IMU 发布的激光帧倾角；与 arc_pitch_watcher.py 同一把尺）',
        'tilt_overall_deg': {
            'n': len(tilts), 'max': max(tilts) if tilts else None,
            'median': statistics.median(tilts) if tilts else None,
            'p95': (sorted(tilts)[int(0.95 * (len(tilts) - 1))] if tilts else None),
            'frac_above_0p4': (sum(1 for x in tilts if x > 0.4) / len(tilts)) if tilts else None,
        },
        'stop_event_definition': 'v < 0.02 m/s 且之前 3 拍内 max(v) > 0.05 m/s（v = 相邻两拍 robot_xy_map 位移 ÷ Δsim_t）',
        'stop_events': [],
    }
    for i in stops:
        win = [recs[j] for j in range(max(0, i - 1), min(len(recs), i + 2))]
        wt = [w['tilt_deg'] for w in win if w['tilt_deg'] is not None]
        out['stop_events'].append({
            'seq': recs[i]['seq'], 'sim_t': recs[i]['sim_t'],
            'v': recs[i]['v'], 'tilt_deg_this': recs[i]['tilt_deg'],
            'tilt_peak_win2s_deg': max(wt) if wt else None,
            'd_lethal_next': (recs[i]['map'] or {}).get('d_lethal'),
        })
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)

    o = out['tilt_overall_deg']
    print('★ 俯仰（scan.tf.M，与 arc_pitch_watcher 同一把尺）：n=%d max=%.3f° 中位=%.3f° p95=%.3f°'
          % (o['n'], o['max'], o['median'], o['p95']))
    print('   ★ > 0.4° 的占比 = %.1f%%' % (100 * o['frac_above_0p4']))
    print('★ 停车事件 n=%d（定义：v<0.02 且前 3 拍曾有 v>0.05）' % len(out['stop_events']))
    for e in out['stop_events'][:12]:
        print('   sim %7.1f  seq %-5d v=%.4f  该拍 tilt=%.3f°  ±2s 峰值=%.3f°  下一拍 Δlethal=%s'
              % (e['sim_t'], e['seq'], e['v'], e['tilt_deg_this'] or -1,
                 e['tilt_peak_win2s_deg'] or -1, e['d_lethal_next']))
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arc_and_survivor.py —— ★ 钉住"每帧的弧" ＋ 追一个"幸存者"格 ✓（阶段十 ✓，★ 纯离线 ✓）

用法: arc_and_survivor.py <probe目录> [--ticks 5] [--far 7.9999] [--out 新json]

★★ 为什么改了研究对象（★ 业主 2026-10-11 的方法更正 ✓）★★
  ★ 我们前几轮一直在研究"**残留**"✗ —— ★ 而业主说的是 ★ "★ **小车附近、激光范围内、实时出现、
  整体类似弧形**的点集 ✗；★ 随后几帧大多被清除 ✓、★ 少数残留 ✓" ✓
  ⇒ ★★ **残留只是那个弧的幸存者** ✓ ⇒ ★ **先看弧** ✓（★ 缺这步 ⇒ ★ 连"在对齐什么"都不知道 ✗）。

★★ ① 弧（★ 车在走的 3–5 拍 ✓，每拍给）★★
  ★ a. ★ **相对**传感器**的半径分布** ✓（✗ 不是云质心 ✗；★ 原点从 `scan.tf.M` 的平移取 ✓）
  ★ b. 弧上点的 **z** ✓
  ★ c. ★★ **"真回波" vs "无回波被填的 `far`"各占多少** ✗ —— ★ 判据 = ★ **该点到传感器的距离 ≈ far** ✓
    （★ `scan_points.py` 把无回波填成 `range_max − 1e-4 = 7.9999` ✓ ⇒ ★ 那些点的**半径就是 7.9999** ✓）
  ★ d. ★ **该拍全局图新增的格** ✓（前后两拍做差 ✓）＋ ★ **新增格 ↔ 弧上点能不能对上** ✓

★★ ② 一个"幸存者"✓：★ 挑一个最终残留的格 ⇒ ★ 逐拍时间线（被标 ↔ 被清 ✓）＋ ★ 标上它的那个点 ✓
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import sys

FAR = 7.9999


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--ticks', type=int, default=5, help='★ 抽样拍数（★ 只在"车在走"的拍里挑 ✓）')
    ap.add_argument('--far', type=float, default=FAR)
    ap.add_argument('--residue', default=None, help='★ 残留 CSV（★ 用来挑幸存者 ✓）')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    ticks = []
    for f in sorted(glob.glob(os.path.join(a.probe_dir, 'tick_*.json'))):
        d = json.load(open(f, encoding='utf-8'))
        seq = d['seq']
        mp = os.path.join(a.probe_dir, 'map_%04d.bin' % seq)
        if not (d.get('map') and os.path.isfile(mp)):
            continue
        d['_mapfile'] = mp
        ticks.append(d)
    print('  ★ 可用 tick = %d ✓' % len(ticks))

    # ★ 只在"车在走"的拍里挑（★ 位移 > 0.02 m ✓）
    moving = []
    for i in range(1, len(ticks)):
        r0, r1 = ticks[i - 1].get('robot_xy_map'), ticks[i].get('robot_xy_map')
        if r0 and r1 and math.hypot(r1[0] - r0[0], r1[1] - r0[1]) > 0.02:
            moving.append(i)
    print('  ★ "车在走"的拍 = %d ✓（★ 从里面等距挑 %d 拍 ✓）' % (len(moving), a.ticks))
    step = max(1, len(moving) // a.ticks)
    picks = moving[::step][:a.ticks]

    def scan_pts(seq):
        p = os.path.join(a.probe_dir, 'scan_%04d.xyz' % seq)
        out = []
        if os.path.isfile(p):
            for line in open(p, encoding='utf-8'):
                v = line.split()
                if len(v) == 3:
                    out.append((float(v[0]), float(v[1]), float(v[2])))
        return out

    def sensor_org(t):
        M = ((t.get('scan') or {}).get('tf') or {}).get('M')
        return (M[0][3], M[1][3], M[2][3]) if M else None

    print()
    print('  ★★★ ① 弧（★ 半径一栏是**相对传感器** ✓，✗ 不是云质心 ✗）：')
    print('   拍    sim_t   ★ 半径: 中位 ｜ p90 ｜ max ｜ min   ★ z: 中位 ｜范围     ★ 填充值占比(≈%.4f)'
          % a.far)
    per_tick = []
    for i in picks:
        t = ticks[i]
        org = sensor_org(t)
        pts = scan_pts(t['seq'])
        if not org or not pts:
            continue
        rs = sorted(math.dist(p, org) for p in pts)
        zs = sorted(p[2] for p in pts)
        nf = sum(1 for r in rs if abs(r - a.far) < 0.05)
        med = rs[len(rs) // 2]
        print('   %4d  %6.1f   ★ %5.2f ｜ %5.2f ｜ %5.2f ｜ %5.2f   ★ %6.3f ｜%.3f–%.3f   ★ %5.1f%% (%d/%d)'
              % (t['seq'], t['sim_t'], med, rs[int(len(rs) * .9)], rs[-1], rs[0],
                 zs[len(zs) // 2], zs[0], zs[-1], 100.0 * nf / len(rs), nf, len(rs)))
        per_tick.append({'seq': t['seq'], 'sim_t': t['sim_t'], 'n': len(rs),
                         'r_median': med, 'r_p90': rs[int(len(rs) * .9)], 'r_max': rs[-1], 'r_min': rs[0],
                         'z_median': zs[len(zs) // 2], 'n_far_fill': nf,
                         'frac_far_fill': nf / float(len(rs))})

    # ★ d. 逐格做差：该拍全局图新增了哪些格 ✓ ＋ 与弧上点对得上吗 ✓
    print()
    print('  ★★★ ①d 该拍**全局图新增的格** ↔ **弧上的点**（★ "弧 → 图"的直接检验 ✓）：')
    arc_to_map = []
    for i in picks[:3]:
        t, tprev = ticks[i], ticks[i - 1]
        b1 = open(t['_mapfile'], 'rb').read()
        b0 = open(tprev['_mapfile'], 'rb').read()
        info = t['map']
        W, res, ox, oy = info['width'], info['resolution'], info['origin']['x'], info['origin']['y']
        new = [k for k in range(len(b1)) if b1[k] not in (255,) and b0[k] == 0 and b1[k] != 0]
        new = [k for k in new if b1[k] < 128]
        org = sensor_org(t)
        pts = scan_pts(t['seq'])
        hit = 0
        for k in new:
            gx, gy = k % W, k // W
            wx, wy = ox + (gx + .5) * res, oy + (gy + .5) * res
            if any(math.hypot(px - wx, py - wy) <= 0.15 for px, py, _ in pts):
                hit += 1
        print('     拍 %4d（sim_t %6.1f）⇒ ★ 新增格 **%d** ✓ ｜ ★ 其中与弧上点 ≤0.15 m 的 **%d（%.1f%%）** ✓'
              % (t['seq'], t['sim_t'], len(new), hit, 100.0 * hit / max(1, len(new))))
        arc_to_map.append({'seq': t['seq'], 'sim_t': t['sim_t'], 'new_cells': len(new),
                           'matched_within_0.15m': hit,
                           'frac': (hit / float(len(new))) if new else None})

    # ★★ ② 追一个幸存者 ✓
    if a.residue and os.path.isfile(a.residue):
        rows = list(csv.DictReader(open(a.residue, encoding='utf-8')))
        # ★ 挑"最早被标上"的那个 ✓（★ 时间线最长 ✓）
        rows = [r for r in rows if r.get('first_occ_sim_t')]
        rows.sort(key=lambda r: float(r['first_occ_sim_t']))
        if rows:
            v = rows[0]
            gx, gy = int(v['i']), int(v['j'])
            print()
            print('  ★★★ ② 幸存者：格 (%d, %d) ｜ 世界 (%.3f, %.3f) ｜ ★ 首标 sim_t=%s ｜ 末代价 %s ｜ 归因 %s'
                  % (gx, gy, float(v['wx']), float(v['wy']), v['first_occ_sim_t'], v['cost_end'], v['src']))
            hist = []
            for t in ticks:
                b = open(t['_mapfile'], 'rb').read()
                val = b[gy * W + gx]
                val = val - 256 if val >= 128 else val
                hist.append((t['seq'], t['sim_t'], val))
            # ★ 压缩成区间
            segs = []
            for s, st, val in hist:
                lab = 'free' if val == 0 else ('unknown' if val < 0 else ('lethal' if val >= 99 else 'inflated'))
                if segs and segs[-1][3] == lab:
                    segs[-1][1] = st
                else:
                    segs.append([st, st, val, lab])
            print('     ★ 逐拍时间线（★ 压缩成区间 ✓，★ %d 拍 / sim_t %.1f→%.1f ✓）：' % (len(hist), hist[0][1], hist[-1][1]))
            tl = []
            for s0, s1, val, lab in segs:
                print('        sim_t %7.1f → %7.1f  ★ %-8s (值 %d)' % (s0, s1, lab, val))
                tl.append({'sim_t_from': s0, 'sim_t_to': s1, 'state': lab, 'cost': val})
            survivor = {'cell': [gx, gy], 'world': [float(v['wx']), float(v['wy'])],
                        'first_mark_sim_t': float(v['first_occ_sim_t']),
                        'n_marked_events': sum(1 for x in segs if x[3] != 'free'),
                        'n_cleared_events': sum(1 for i2, x in enumerate(segs)
                                                if x[3] == 'free' and i2 > 0 and segs[i2 - 1][3] != 'free'),
                        'timeline': tl}
            # ★ 首次被标那一刻，最近的弧上点
            idx = next((i for i, (_, _, val) in enumerate(hist) if val > 0), None)
            if idx is not None:
                t = ticks[idx]
                org = sensor_org(t)
                pts = scan_pts(t['seq'])
                wx, wy = float(v['wx']), float(v['wy'])
                near = sorted(((math.hypot(px - wx, py - wy), px, py, pz) for px, py, pz in pts))[:1]
                if near and org:
                    d, px, py, pz = near[0]
                    print('     ★ **标上它的那条**: 拍 %d（sim_t %.1f）｜ ★ 最近的点 xy 距 %.3f m ｜ z=%.3f ｜'
                          ' ★ **到传感器 %.3f m**（★ ≈%.4f ⇒ ★ 是**填充值** ✗）'
                          % (t['seq'], t['sim_t'], d, pz, math.dist((px, py, pz), org), a.far))
                    survivor['nearest_scan_point_at_mark'] = {
                        'tick_seq': t['seq'], 'sim_t': t['sim_t'], 'xy_dist_m': d, 'z': pz,
                        'dist_to_sensor_m': math.dist((px, py, pz), org),
                        'looks_like_far_fill': abs(math.dist((px, py, pz), org) - a.far) < 0.05}
                    survivor['marker_identified'] = bool(d <= 0.15)

    # ★★★ 2026-10-11（reviewer ✓）：★ **晕 vs 致命**这一条**重新定义了整个问题的口径** ✗
    #   ⇒ ★ 必须进产物 ✓（★ 否则复算方只能采信转述 ✗）
    split = None
    if a.residue and os.path.isfile(a.residue):
        rr = list(csv.DictReader(open(a.residue, encoding='utf-8')))
        infl = [(float(x['wx']), float(x['wy']), int(float(x['cost_end'])))
                for x in rr if 0 < int(float(x['cost_end'])) < 99]
        leth = [(float(x['wx']), float(x['wy'])) for x in rr if int(float(x['cost_end'])) >= 99]
        ds = []
        if leth:
            for wx, wy, _ in infl:
                ds.append(min(math.hypot(wx - a2, wy - b2) for a2, b2 in leth))
        ds.sort()
        split = {'n_residue': len(rr), 'n_inflated': len(infl), 'n_lethal': len(leth),
                 'frac_inflated': len(infl) / float(max(1, len(rr))),
                 'halo_d_lethal_m': ({'min': ds[0], 'median': ds[len(ds) // 2],
                                      'p90': ds[int(len(ds) * .9)], 'max': ds[-1]} if ds else None),
                 'inflation_radius_m': 0.25,
                 'note': ('★ "残留格数"被这层晕撑着 ✗ ⇒ ★ 真要跨轮比，比'
                          '**致命格数** ✓（✗ 不比残留总数 ✗）')}
        print()
        print('  ★★★ 晕 vs 致命 ✓：★ 膨胀 **%d（%.1f%%）** ✗ ｜ ★ 致命 **%d（%.1f%%）** ✓ ｜'
              ' ★ 晕到最近致命格 中位 %s m ✓'
              % (len(infl), 100.0 * len(infl) / max(1, len(rr)), len(leth),
                 100.0 * len(leth) / max(1, len(rr)),
                 ('%.2f' % ds[len(ds) // 2]) if ds else '—'))

    out_obj = {'arc_per_tick': per_tick,
               'arc_to_map': arc_to_map,
               'residue_split_inflated_vs_lethal': split}
    if a.residue:
        out_obj['survivor'] = locals().get('survivor')
    json.dump(out_obj, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print()
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

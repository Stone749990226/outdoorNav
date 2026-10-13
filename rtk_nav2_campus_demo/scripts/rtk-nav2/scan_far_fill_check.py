#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scan_far_fill_check.py —— ★ 量"**没回波的方向被补成远裁剪面上的点**"的签名 ✓

★ 背景（reviewer 2026-10-12 ✓）：`depth_points.py` 的 `emit_far_points`（默认 true ✓）
  ⇒ ★ **没有回波的方向被补到远裁剪面（8.00 m ✓）** ⇒ ★ 激光是 **360°** ⇒ ★ **车后面也会被补** ✗
  ⇒ ★ 业主看到"**车后一大片障碍点**" ✓。
★ 签名（★ 一眼可辨 ✓）：★ **大量光束的值恰好落在 `range_max` 上（≈8.00 m ✓）** ✓
  ★ 而**真回波**是连续分布的 ✗ ⇒ ★ 所以量"**贴在 `range_max` 上的那一撮**" ✓✓
★ 判据（A/B ✓）：★ 开关 **false** ⇒ ★ 这一撮应当**基本消失** ✓；★ true ⇒ 它在 ✓

用法：scan_far_fill_check.py <bag目录> --t-sim 300 --half 60 --out x.json
"""
import argparse
import glob
import json
import os
import sqlite3
import statistics
import struct
import sys


def read_string(buf, off):
    n = struct.unpack_from('<I', buf, off)[0]
    off += 4
    return buf[off:off + n - 1].decode('utf-8', 'replace'), off + n


def align(off, n=4):
    return (off + n - 1) // n * n


def parse_scan(buf):
    off = 4
    sec, nsec = struct.unpack_from('<iI', buf, off)
    off = align(off + 8)
    _, off = read_string(buf, off)
    off = align(off)
    ang_min, ang_max, ang_inc, t_inc, scan_t = struct.unpack_from('<fffff', buf, off)
    off = align(off + 20)
    rmin, rmax = struct.unpack_from('<ff', buf, off)
    off = align(off + 8)
    n, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    if n == 0 or n > 5000:
        return None
    rng = struct.unpack_from('<%df' % n, buf, off)
    return (sec + nsec * 1e-9), ang_min, ang_inc, rng, rmin, rmax


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--t-sim', type=float, required=True)
    ap.add_argument('--half', type=float, default=60.0)
    ap.add_argument('--topic', default='/rtk/scan')
    ap.add_argument('--tol', type=float, default=0.02, help='"贴在 range_max 上"的容差（m）')
    ap.add_argument('--max-frames', type=int, default=400)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    got = []
    for db in sorted(glob.glob(os.path.join(a.bag_dir, '*.db3'))):
        try:
            con = sqlite3.connect('file:%s?mode=ro' % db, uri=True)
            rows = con.execute(
                "SELECT t.name, m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                "WHERE t.name = ?", (a.topic,)).fetchall()
            con.close()
        except sqlite3.Error:
            continue
        for _, data in rows:
            r = parse_scan(data)
            if r and abs(r[0] - a.t_sim) <= a.half:
                got.append(r)
    got.sort(key=lambda z: z[0])
    if not got:
        print('✗ 窗口里没有 scan 帧'); return 2
    step = max(1, len(got) // a.max_frames)
    got = got[::step]

    nbeam = len(got[0][3])
    rmax = got[0][5]
    fr_frac = []
    pers = [0] * nbeam
    n = 0
    for t, amin, ainc, rng, rmin, rmaxf in got:
        if len(rng) != nbeam:
            continue
        far = [1 if (v == v and abs(v - rmaxf) <= a.tol) else 0 for v in rng]
        fr_frac.append(sum(far) / float(nbeam))
        for i, v in enumerate(far):
            pers[i] += v
        n += 1
    fixed = [i for i in range(nbeam) if n and pers[i] / float(n) >= 0.9]
    out = {'bag_dir': os.path.abspath(a.bag_dir), 't_sim_center': a.t_sim, 'half_s': a.half,
           'range_max': rmax, 'tol_m': a.tol, 'n_frames': n, 'n_beams': nbeam,
           'far_frac_mean': round(statistics.mean(fr_frac), 5) if fr_frac else None,
           'far_frac_max': round(max(fr_frac), 5) if fr_frac else None,
           'n_fixed_far_beams': len(fixed),
           'rule': '★ "贴在 range_max（≈8.00 m）上的光束" —— 补远裁剪面的签名 ✓',
           'verdict': ('★ 有 %d 条**每帧都**贴在 range_max 上 ⇒ 补远点开着的签名 ✗' % len(fixed))
                      if len(fixed) > 20 else
                      ('★ 几乎没有固定贴 max 的光束（%d 条）⇒ 补远点**没开**或无效 ✓' % len(fixed))}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('★ 帧 %d ｜ range_max=%.2f ｜ ★ 贴 max 的光束占比 均值 %.5f 最大 %.5f ｜ ★ **固定贴 max 的光束 %d 条**'
          % (n, rmax, out['far_frac_mean'] or -1, out['far_frac_max'] or -1, len(fixed)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

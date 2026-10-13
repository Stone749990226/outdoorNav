#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""depth_row_ruler.py —— ★ "地面线行号"尺的**严版** ✓（把细条/杂点挡掉 ✓）

★ 与 `depth_sway_check.py` 的差别**只有一处** ✗：判据从"**该行有 ≥5 个有限回波**" ✓
  改成"**该行中央带里 ≥F 比例都是有限回波**"（默认 F=0.9 ✓）✓
★★ 为什么要改（实测 ✓）：★ 2026-10-12，业主那轮 campus_100：★ 画面里有一条 **7.4–7.9 m 远的细条**
  （一行只有 **2–3** 个有限列 ✗）一闪一闪 ⇒ 旧判据把"最上行"在 128 行／84–104 行之间**来回拽** ✗
  ⇒ 相邻帧跳 44 行（0.1 s 内 9° ✗ 非物理 ✓）⇒ **旧尺在该场景失效** ✗。
  ★ 新判据要求**整条带**都看得见地面 ✓ ⇒ 细条不算 ✓（地面本来就铺满下半 ✓：实测行 140–220 每行 20/20 ✓）。

★ 一并给出**两种换算** ✓：★ 行→度（`dθ/dy = 1/f` 在小角处 ⇒ 1 行 ≈ 0.207° ✓）
  和 ★ **行→度（既有标定）0.195–0.197 °/行** ✓（两者同量级 ⇒ 互证 ✓）。

用法：depth_row_ruler.py <bag目录> --t-sim 260 --half 12 --out x.json [--frac 0.9]
"""
import argparse
import glob
import json
import math
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


def parse_image(buf):
    if len(buf) < 8:
        return None
    off = 4
    sec, nsec = struct.unpack_from('<iI', buf, off)
    off = align(off + 8)
    _, off = read_string(buf, off)
    off = align(off)
    h, w = struct.unpack_from('<II', buf, off)
    off = align(off + 8)
    enc, off = read_string(buf, off)
    off = align(off + 1)
    step, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    n, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    return {'sec': sec, 'nsec': nsec, 'h': h, 'w': w, 'enc': enc,
            'step': step, 'n': n, 'data_off': off}


def row_finite_frac(data, info, y, x0, x1, stride=4):
    o0 = info['data_off'] + y * info['step']
    ncol = 0
    nfin = 0
    zs = []
    for x in range(x0, x1, stride):
        o = o0 + x * 4
        if o + 4 > len(data):
            break
        ncol += 1
        v, = struct.unpack_from('<f', data, o)
        if v != v or v <= 0.0 or v == float('inf'):
            continue
        nfin += 1
        zs.append(v)
    return nfin, ncol, (statistics.median(zs) if zs else None)


def ground_row(data, info, x0, x1, frac, min_z=None, max_z=None):
    """★ 最上面那一行"整条带 ≥frac 都是有限回波"的行 ✓（可选：要求该行中位深度落在 [min_z,max_z] ✓）"""
    for y in range(info['h']):
        nfin, ncol, z = row_finite_frac(data, info, y, x0, x1)
        if ncol == 0 or nfin < frac * ncol:
            continue
        if min_z is not None and (z is None or z < min_z):
            continue
        if max_z is not None and (z is None or z > max_z):
            continue
        return y, z
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--t-sim', type=float, required=True)
    ap.add_argument('--half', type=float, default=25.0)
    ap.add_argument('--topic', default='/rtk/camera/depth_image')
    ap.add_argument('--frac', type=float, default=0.9)
    ap.add_argument('--max-z', type=float, default=None,
                    help='★ 只认"中位深度 ≤ 这个值"的行 ✓（挡住远处的墙／细条 ✓）')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    f = (320 / 2.0) / math.tan(1.0472 / 2.0)          # 277.13 px
    deg_per_row = math.degrees(1.0 / f)
    x0, x1 = 160 - 40, 160 + 40
    dbs = sorted(glob.glob(os.path.join(a.bag_dir, '*.db3')))
    got = []
    for db in dbs:
        try:
            con = sqlite3.connect(db)
            rows = con.execute(
                "SELECT t.name, m.data FROM messages m JOIN topics t ON t.id=m.topic_id").fetchall()
            con.close()
        except sqlite3.Error:
            continue
        for name, data in rows:
            if name != a.topic:
                continue
            info = parse_image(data)
            if not info or info['n'] <= 0 or info['enc'] != '32FC1':
                continue
            t = info['sec'] + info['nsec'] * 1e-9
            if abs(t - a.t_sim) <= a.half:
                got.append((t, data, info))
    got.sort(key=lambda z: z[0])
    if not got:
        print('✗ 窗口内没有深度帧'); return 2

    fr = []
    for t, data, info in got:
        y, z = ground_row(data, info, x0, x1, a.frac, max_z=a.max_z)
        fr.append({'t_sim': round(t, 3), 'ground_row': y, 'row_median_z': (round(z, 3) if z else None)})
    ys = [x['ground_row'] for x in fr if x['ground_row'] is not None]
    span = (max(ys) - min(ys)) if ys else None
    out = {'bag_dir': os.path.abspath(a.bag_dir), 'topic': a.topic,
           't_sim_center': a.t_sim, 'half_s': a.half, 'frac': a.frac, 'max_z': a.max_z,
           'rule': '★ 地面线行 = **最上面一行**满足"中央带 ≥frac 的列有限"✓（可选 max_z 挡远处 ✓）',
           'n_frames': len(fr), 'n_rows_detected': len(ys),
           'ground_row_min': min(ys) if ys else None, 'ground_row_max': max(ys) if ys else None,
           'ground_row_span': span,
           'deg_per_row_intrinsics': round(deg_per_row, 4),
           'span_deg_via_intrinsics': (round(span * deg_per_row, 3) if span is not None else None),
           'span_deg_via_calib_0.196': (round(span * 0.196, 3) if span is not None else None),
           'frames': fr}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('★ %d 帧（检出 %d）｜ 行 min %s / max %s ｜ ★ **跨度 %s 行** ⇒ %.2f°（1行=%.3f°）'
          % (len(fr), len(ys), out['ground_row_min'], out['ground_row_max'], span,
             out['span_deg_via_intrinsics'] or 0, deg_per_row))
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sticky_two_buckets.py —— ★ 把"粘住格"分成**两个穷尽桶**并**落盘** ✓

用法: sticky_two_buckets.py <probe目录> --max-seq N --out json [--radii 0.105,0.50]

★★ 为什么要有这个文件（reviewer 2026-10-11 ✓）★★
  ★ 规矩：★ **"结论只由产物给"** ✓ —— ★ 我先前那两列（"从未被清"／"清掉过又被重标"）
    只在聊天正文里 ✗ ⇒ ★ 复算方**只能采信转述** ✗（★ "算出来了 ≠ 交出去了" ✓）⇒ ★ 落成产物 ✓。
★ 两个桶**穷尽** ✓（★ 论证：★ 粘住 ⇒ 末拍致命 ✓；★ 若它**曾被清过** ⇒ 桶 b ✓；★ 否则 ⇒ 桶 a ✓）✓。
★ 口径（写死 ✓）：
  · ★ "粘住" = ★ 格心到**加密轨迹(0.05 m)**任一点 ≤ R ✓ ∧ ★ 末拍致命 ∧ 起始拍不致命 ✓
  · ★ "被清掉过" = ★ **第一次被标上之后**，★ 存在某一拍该格**变回非致命** ✓
  · ★ 只取**离轨迹 ≤ R** 的格 ✓（★ 可比空间 ✓）
★ ★ 落盘里**带上输入文件名与各自 sha256** ✓（★ 复算方自己接回去 ✓）。
"""
import argparse
import glob
import hashlib
import json
import math
import os
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest() if os.path.isfile(p) else None


def densify(pts, step=0.05):
    out = []
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / step))
        for j in range(n):
            t = j / n
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
    if pts:
        out.append(pts[-1])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--radii', default='0.105,0.50')
    ap.add_argument('--inputs', default='', help='★ 逗号分隔的输入产物路径 ✓（记 sha256 ✓）')
    a = ap.parse_args()
    R = [float(x) for x in a.radii.split(',')]
    P = a.probe_dir

    ticks = sorted([t for t in glob.glob(os.path.join(P, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(P, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    meta = json.load(open(ticks[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    maps = {seq_of(t): os.path.join(P, 'map_%04d.bin' % seq_of(t)) for t in ticks}
    seqs = [seq_of(t) for t in ticks]
    b0 = open(maps[seqs[0]], 'rb').read()
    b1 = open(maps[seqs[-1]], 'rb').read()
    H = len(b1) // W
    L = lambda v: 99 <= v <= 100
    added = {k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])}
    pts = [tuple(json.load(open(t, encoding='utf-8'))['robot_xy_map']) for t in ticks
           if json.load(open(t, encoding='utf-8')).get('robot_xy_map')]
    dense = densify(pts, 0.05)
    xy = lambda k: (ox + ((k % W) + .5) * res, oy + ((k // W) + .5) * res)
    dtraj = lambda k: min(math.hypot(xy(k)[0] - px, xy(k)[1] - py) for px, py in dense)

    LETH = re.compile(rb'[\x63\x64]')
    sets = {s: {m.start() for m in LETH.finditer(open(maps[s], 'rb').read())} for s in seqs}

    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'radii_m': R,
           'probe_interval_s': 2.0, 'n_ticks': len(seqs),
           'window': {'first_seq': seqs[0], 'last_seq': seqs[-1], 'start_map': os.path.basename(maps[seqs[0]]),
                      'end_map': os.path.basename(maps[seqs[-1]])},
           'convention': ('★ "粘住" = 格心到加密轨迹(0.05 m)任一点 ≤ R ∧ 末拍致命 ∧ 起始拍不致命 ✓；'
                          '★ "被清掉过" = 第一次被标上之后，存在某一拍由致命变回非致命 ✓；'
                          '★ 只取离轨迹 ≤ R 的格 ✓（★ 可比空间 ✓）；★ 逐拍用同一份致命集合 ✓'),
           'buckets_are_exhaustive': ('★ 粘住 ⇒ 末拍致命 ✓；★ 曾被清过 ⇒ 桶 b ✓，否则 ⇒ 桶 a ✓ '
                                      '⇒ 两桶穷尽 ✓（★ 两个计数之和 = n_sticky ✓）'),
           'inputs_sha256': {}}
    for f in [f for f in a.inputs.split(',') if f]:
        out['inputs_sha256'][f] = sha(f)
    for nm in ('start_map', 'end_map'):
        fp = os.path.join(P, out['window'][nm])
        out['inputs_sha256'][fp] = sha(fp)

    for r in R:
        swept = set()
        for px, py in dense:
            i0, i1 = int((px - r - ox) / res), int((px + r - ox) / res)
            j0, j1 = int((py - r - oy) / res), int((py + r - oy) / res)
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    if 0 <= i < W and 0 <= j < H:
                        k = j * W + i
                        if math.hypot(xy(k)[0] - px, xy(k)[1] - py) <= r:
                            swept.add(k)
        sticky = sorted(swept & added)
        a_never, b_cleared = [], []
        for k in sticky:
            seen = was_cleared = False
            prev = None
            for s in seqs:
                cur = k in sets[s]
                if prev is not None:
                    if cur and not prev:
                        seen = True
                    if prev and not cur and seen:
                        was_cleared = True
                prev = cur
            (b_cleared if was_cleared else a_never).append(k)
        n = len(sticky)
        out['R=%.3f' % r] = {
            'n_sticky': n,
            'a_never_cleared': {'n': len(a_never), 'frac': (len(a_never) / n) if n else None},
            'b_cleared_then_remark': {'n': len(b_cleared), 'frac': (len(b_cleared) / n) if n else None},
            'sum_equals_sticky': (len(a_never) + len(b_cleared)) == n,
            'a_cells': a_never[:200], 'b_cells': b_cleared[:200],
        }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 拍 %d ｜ 窗口 %s→%s ✓ ｜ 输入 sha 记了 %d 条 ✓'
          % (len(seqs), seqs[0], seqs[-1], len(out['inputs_sha256'])))
    for r in R:
        v = out['R=%.3f' % r]
        print('     R=%.3f ⇒ 粘住 %4d ｜ ★ 从未被清 %3d（%.1f%%）｜ ★ 清掉过又被重标 %4d（%.1f%%）｜ 两桶之和=粘住? %s'
              % (r, v['n_sticky'], v['a_never_cleared']['n'], 100 * (v['a_never_cleared']['frac'] or 0),
                 v['b_cleared_then_remark']['n'], 100 * (v['b_cleared_then_remark']['frac'] or 0),
                 v['sum_equals_sticky']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

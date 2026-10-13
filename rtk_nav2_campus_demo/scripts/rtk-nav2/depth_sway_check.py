#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""depth_sway_check.py —— ★ 用**不经过 TF 的量**裁那把姿态尺 ✓（reviewer 2026-10-12 指派 ✓）

用法: depth_sway_check.py <bag目录> --t-sim 93.64 [--half 1.5] [--out json]

★★ 为什么是深度图 ★★
  ★ 姿态尺（`attitude.csv` / 探针 `scan.tf.M`）的倾角**来自 TF** ✓ ⇒ ★ **尺子坏了也会给出好看的曲线** ✗；
  ★ 而**相机图是渲染出来的原始像素** ✓ —— ★ **不经过 TF、不经过 IMU** ✓ ⇒ ★ 用它**独立裁** ✓✓

★ 量什么 ✓：★ 逐帧算"**地面线的行号**" ✓（★ 取中央一条竖带 ✓、★ 找"由近到远"的**跳变行** ✓ ⇒ 那是地平/地面边界 ✓）
　⇒ ★ 该行号随时间的**起伏** ＝ ★ 图里看到的"**上下摆**" ✓；★ 与同刻姿态的起伏对比 ✓
★ 不依赖 numpy ✓（★ 本机没有 ✓ —— ★ 纯 python ✓、★ 行方向抽样 ✓）

★ 判据两头 ✓（reviewer 给 ✓）：★ 图**也在摆**（幅度与 14° 相符 ✓）⇒ ★ 尺子对 ✓、★ 车身真的在摇 ✓；
  ★ 图**是静的** ⇒ ★ 尺子错 ✗ ⇒ ★ 那条关联整个标掉 ✓
"""
import argparse
import glob
import json
import math
import os
import re
import sqlite3
import struct
import sys


def read_string(buf, off):
    n = struct.unpack_from('<I', buf, off)[0]
    off += 4
    return buf[off:off + n - 1].decode('utf-8', 'replace'), off + n


def align(off, n=4):
    """★ CDR 要求 4 字节对齐 ✗ 我第一版漏了它 ⇒ 偏移一路错到 buffer 外 ✓（实测栽过 ✓）"""
    return (off + n - 1) // n * n


def parse_image(buf):
    """CDR(sensor_msgs/Image) ⇒ 各字段（★ 逐字段 4 字节对齐 ✓）"""
    if len(buf) < 8:
        return None
    off = 4                       # ★ 4 字节封装头（little-endian CDR ✓）
    sec, nsec = struct.unpack_from('<iI', buf, off)
    off = align(off + 8)
    _, off = read_string(buf, off)          # frame_id
    off = align(off)
    h, w = struct.unpack_from('<II', buf, off)
    off = align(off + 8)
    enc, off = read_string(buf, off)
    off = align(off + 1)                    # is_bigendian（1 字节 ✓）后再对齐 ✓
    step, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    n, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    return {'sec': sec, 'nsec': nsec, 'h': h, 'w': w, 'enc': enc,
            'step': step, 'n': n, 'data_off': off}


def frame_profile(buf, info, x_lo, x_hi, row_stride):
    """★ 行剖面 ✓：中央竖带里，每行取"第一个近距回波"的列位置的中位替代 ——
       ★ 这里更省：算每行的**最小深度**（近处地面会给出小值 ✓）⇒ 剖面 = 行 → 深度 ✓"""
    enc = info['enc']
    w, h, step = info['w'], info['h'], info['step']
    prof = []
    if enc in ('32FC1',):
        fmt, size = '<f', 4
    elif enc in ('16UC1',):
        fmt, size = '<H', 2
    else:
        return None
    base = info['data_off']
    for y in range(0, h, row_stride):
        row = base + y * step
        cnt = 0
        for x in range(x_lo, x_hi, 4):
            o = row + x * size
            if o + size > len(buf):
                break
            v, = struct.unpack_from(fmt, buf, o)
            if v != v or v <= 0 or v == float('inf'):
                continue
            cnt += 1
        prof.append((y, cnt))
    return prof


def ground_row(prof, min_valid=5):
    """★★ **地面边界行 = 最上面那一行有有限回波的行** ✓★
    ★ 朝向前的深度相机里：★ **上方是 `inf`（无回波 ✓）、下方是地面（有限 ✓）** ⇒
      ★ 边界行随相机俯仰**上下移动** ✓（★ 行号越小 ⇒ 相机越"抬头" ⇒ 倾角越大 ✓）。
    ★ ★ 我第一版取的是"**最下面**那行有近距回波" ✗ ⇒ ★ 恒等于画面底边（本例 236）✗
      ⇒ ★ 相邻帧互相关给出常数、整份量退化 ✗（★ reviewer 实测抓到这个 ✗）⇒ **改成"最上面"** ✓。"""
    for y, cnt in prof:
        if cnt >= min_valid:
            return y
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--t-sim', type=float, required=True)
    ap.add_argument('--half', type=float, default=1.5)
    ap.add_argument('--topic', default='/rtk/camera/depth_image')
    ap.add_argument('--row-stride', type=int, default=4)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    dbs = sorted(glob.glob(os.path.join(a.bag_dir, '*.db3')))
    if not dbs:
        print('✗ 没找到 db3'); return 2
    got = []
    for db in dbs:
        con = sqlite3.connect(db)
        try:
            rows = con.execute(
                "SELECT t.name, m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                "ORDER BY m.timestamp").fetchall()
        except sqlite3.Error as e:
            con.close(); continue
        con.close()
        for name, data in rows:
            if name != a.topic:
                continue
            info = parse_image(data)
            if not info or info['n'] <= 0:
                continue
            t = info['sec'] + info['nsec'] * 1e-9
            if abs(t - a.t_sim) <= a.half:
                got.append((t, data, info))
        # ★ 早停：窗口在更早的文件里已经过掉 ⇒ 但文件顺序按名字不保证 ⇒ 不早停 ✓
    got.sort(key=lambda z: z[0])
    if not got:
        print('✗ 窗口内没有深度帧（bag 里有没有这个话题？）'); return 2

    out = {'bag_dir': os.path.abspath(a.bag_dir), 'topic': a.topic,
           't_sim_center': a.t_sim, 'half_s': a.half,
           'n_frames': len(got), 'encoding': got[0][2]['enc'],
           'size': [got[0][2]['w'], got[0][2]['h']], 'frames': []}
    prev_prof = None
    for t, data, info in got:
        prof = frame_profile(data, info, info['w'] // 2 - 40, info['w'] // 2 + 40, a.row_stride)
        gr = ground_row(prof) if prof else None
        shift = None
        bs = None                      # ★ 第一版漏了初始化 ⇒ 第一帧就 UnboundLocalError ✗（实测栽过 ✓）
        if prev_prof and prof:
            # ★ 相邻帧行剖面的**互相关**（只算有限移位 ✓）⇒ 竖直位移（行） ✓
            m = {y: v for y, v in prev_prof}
            best, bs = None, None
            for s in range(-12, 13):
                num = den1 = den2 = 0.0
                for y, v in prof:
                    u = m.get(y - s)
                    if u is None or v is None:
                        continue
                    num += v * u; den1 += v * v; den2 += u * u
                if den1 > 0 and den2 > 0:
                    c = num / math.sqrt(den1 * den2)
                    if bs is None or c > bs:
                        bs, best = c, s
            shift = best
        out['frames'].append({'t_sim': round(t, 3), 'ground_row': gr, 'v_shift_rows_pp': shift,
                              'corr': None if bs is None else round(bs, 4)})
        prev_prof = prof

    grs = [f['ground_row'] for f in out['frames'] if f['ground_row'] is not None]
    shs = [f['v_shift_rows_pp'] for f in out['frames'] if f['v_shift_rows_pp'] is not None]
    out['summary'] = {
        'ground_row_min': min(grs) if grs else None, 'ground_row_max': max(grs) if grs else None,
        'ground_row_span_rows': (max(grs) - min(grs)) if grs else None,
        'per_frame_shift_min': min(shs) if shs else None,
        'per_frame_shift_max': max(shs) if shs else None,
        'per_frame_shift_abs_sum_rows': sum(abs(x) for x in shs) if shs else None,
        'note': '★ 行位移是"相邻帧互相关"给出的竖直位移 ✓；★ 累计摆动看 abs_sum ✓',
    }
    # ★★ 单峰量法（★ reviewer 2026-10-12 要求：**机械定义** ✗ 不许手挑窗口 ✗）★★
    #   ★ 定义 ✓：★ 取窗口内**地面线最小（＝最抬头）**的那一帧为极值 ✓ ⇒
    #     ★ 向两侧各找到**最近的局部极小／回归**（★ 即地面线**回到 ≥ 起始值 − 2 行** 的第一帧 ✓）
    #     ⇒ ★ 这一段的行跨度 ＝ **单次幅度** ✓；★ **剔饱和帧**（★ `ground_row == 0` ✗ 顶到画面顶端 ⇒ 再大也只读到 0 ✓）
    fr = out['frames']
    idx_min = None
    for i, f in enumerate(fr):
        if f['ground_row'] is None or f['ground_row'] == 0:
            continue
        if idx_min is None or f['ground_row'] < fr[idx_min]['ground_row']:
            idx_min = i
    if idx_min is not None:
        base = fr[idx_min]['ground_row']
        # 左侧：往回找第一次"回到 base + 2 行以上"（＝离开峰）
        i0 = idx_min
        while i0 > 0 and fr[i0 - 1]['ground_row'] is not None and fr[i0 - 1]['ground_row'] > base:
            i0 -= 1
        i1 = idx_min
        while i1 < len(fr) - 1 and fr[i1 + 1]['ground_row'] is not None and fr[i1 + 1]['ground_row'] > base:
            i1 += 1
        seg = [f['ground_row'] for f in fr[i0:i1 + 1] if f['ground_row'] is not None and f['ground_row'] != 0]
        out['single_peak'] = {
            'definition': ('★ 极值帧 = 窗口内 ground_row 最小且**非饱和**（≠0）者 ✓；★ 单峰段 = 从极值向两侧'
                           '扩展到"ground_row 不再 ≤ 极值"为止 ✓；★ 段内**剔除饱和帧**（ground_row==0）✓'),
            'peak_frame_t': fr[idx_min]['t_sim'], 'peak_ground_row': base,
            'span_frames': [fr[i0]['t_sim'], fr[i1]['t_sim']],
            'n_frames_in_segment': len(seg),
            'segment_row_max': max(seg) if seg else None,
            'segment_row_min': min(seg) if seg else None,
            'single_peak_span_rows': (max(seg) - min(seg)) if seg else None,
            'saturated_frames_in_window': sum(1 for f in fr if f['ground_row'] == 0),
        }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    if out.get('single_peak'):
        sp = out['single_peak']
        print('★ 单峰（机械定义 ✓）：极值帧 t=%.3f 行=%s ／ 段 %s…%s ／ ★ **单峰跨度 %s 行**（★ 剔饱和 ✓；★ 窗口内饱和帧 %d）'
              % (sp['peak_frame_t'], sp['peak_ground_row'], sp['span_frames'][0], sp['span_frames'][1],
                 sp['single_peak_span_rows'], sp['saturated_frames_in_window']))
    print('★ %d 帧（%s，%dx%d），窗口 %.2f±%.2f s'
          % (out['n_frames'], out['encoding'], out['size'][0], out['size'][1], a.t_sim, a.half))
    print('★ 地面线行号：min %s ／ max %s ／ 跨度 %s 行'
          % (out['summary']['ground_row_min'], out['summary']['ground_row_max'],
             out['summary']['ground_row_span_rows']))
    print('★ 相邻帧竖直位移：min %s ／ max %s ／ 累计 |位移| %s 行'
          % (out['summary']['per_frame_shift_min'], out['summary']['per_frame_shift_max'],
             out['summary']['per_frame_shift_abs_sum_rows']))
    for f in out['frames'][:14]:
        print('   t=%-8.3f 地面线行=%-5s 相邻帧竖移=%-5s' % (f['t_sim'], f['ground_row'], f['v_shift_rows_pp']))
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

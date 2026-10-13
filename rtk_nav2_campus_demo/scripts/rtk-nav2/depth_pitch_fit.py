#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""depth_pitch_fit.py —— ★ 用**地面平面**从深度图里解出**每帧的俯仰与相机高度** ✓（不经过 TF ✓）

★★ 为什么另起一把（不是 `depth_sway_check.py` 那把）★★
  ★ 那把量的是"**最上面一行有 ≥5 个有限回波的行**" ✓ —— 它的前提是"**画面之上全是 inf**" ✗。
  ★ 实测（2026-10-12，业主那一轮 campus_100，t=275–325 s）：★ **前提不成立** ✗ ——
    画面里有一条**约 7.4–7.9 m 远的细条**（少数几列 ✓，一行只有 2–3 个有限列 ✓）
    **一闪一闪** ⇒ "最上行"在 **128 行** 与 **84–104 行** 之间**来回跳** ✗✗
    ⇒ 相邻帧跳 44 行 = 0.1 s 内 9° ⇒ **物理上不可能** ⇒ ★ 那是**换了个特征**、不是摆动 ✗
    （实测同窗口 501 帧里 **347 帧**都读 128 ⇒ 已是"钉住"的退化读数 ✓）。
  ⇒ ★ 本脚本不找"行" ✗，而是**拟合地面平面** ✓ —— 地面**本来就铺满**画面下半 ✓（实测行 140–220
    每一行的中央带 **20/20 列全有限** ✓、均深从 2.9 m 单调降到 0.6 m ✓✓ ⇒ 那才是地面 ✓）。

★★ 模型（针孔 + 平地，两个未知量：俯仰 θ、相机离地高 h）★★
  ★ 行 y（主点在 cy）对应的**俯角** α(y) = atan((y-cy)/f) ✓（f = 160/tan(hfov/2) ✓ 由内参算 ✓）
  ★ 地面点沿光轴的距离 **Z(y) = h / tan(θ + α(y))** ✓ ⇒ 反解 **θ_y = atan(h/Z) - α(y)** ✓
  ★ **纯俯仰 ⇒ 各行的 θ_y 都相等** ✓；★ **纯升沉（h 变了）⇒ θ_y 随 y 系统性地散开** ✗
    ⇒ ★ 所以"**在 h 上搜一个值，使各行的 θ_y 最集中**" ⇒ **同时解出 (θ, h)** ✓✓
    ⇒ 这也是一道**自检** ✓：★ 残差散度小 ⇒ 平地模型成立、读数可信 ✓；★ 散度大 ⇒ 这一帧不算 ✓。
  ★ ★ 与"只取一行"相比：单行分不开俯仰与升沉 ✗（两者都改 Z ✓）；多行才分得开 ✓。

★ 与既有尺子的**互证** ✓：★ 行度数灵敏度 = `dθ/dy` ⇒ 取 θ*≈0、α≈20° 处：★ 角灵敏度 ≈ 1/f rad/px
  ⇒ ★ **1 行 ≈ 0.207°** ✓ —— 与另一把尺子标定出的 **0.195–0.197 °/行** 同量级 ✓（同一内参下应一致 ✓）。

用法：
  depth_pitch_fit.py <bag目录> --t-sim 300 --half 25 [--topic /rtk/camera/depth_image] --out x.json
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


def row_depth(data, info, y, x0, x1, stride=4):
    """★ 该行中央带的**有限回波中位数** ✓ ＋ 有限列数 ✓（饱和/NaN/inf 一律剔 ✗）"""
    o0 = info['data_off'] + y * info['step']
    vals = []
    n_col = 0
    for x in range(x0, x1, stride):
        o = o0 + x * 4
        if o + 4 > len(data):
            break
        n_col += 1
        v, = struct.unpack_from('<f', data, o)
        if v != v or v <= 0.0 or v == float('inf'):
            continue
        vals.append(v)
    if not vals:
        return None, 0, n_col
    return statistics.median(vals), len(vals), n_col


def fit_frame(rows, f, cy, h_grid):
    """★ 在 h 上搜 ⇒ 取"各行 θ_y 最集中"的那个 ⇒ 返回 (θ_deg, h, spread_deg, n_rows) ✓"""
    best = None
    for h in h_grid:
        ths = []
        for y, z in rows:
            a = math.atan((y - cy) / f)
            t = math.atan(h / z) - a
            ths.append(t)
        if len(ths) < 3:
            continue
        sp = statistics.pstdev(ths)
        if best is None or sp < best[0]:
            best = (sp, statistics.median(ths), h)
    if best is None:
        return None
    return math.degrees(best[1]), best[2], math.degrees(best[0]), len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--t-sim', type=float, required=True)
    ap.add_argument('--half', type=float, default=25.0)
    ap.add_argument('--topic', default='/rtk/camera/depth_image')
    ap.add_argument('--band', default='150:230:10', help='行带 start:stop:step')
    ap.add_argument('--min-frac', type=float, default=0.9, help='该行有限列数占比下限')
    # ★★ 有效性闸（2026-10-12 加 ✓ —— 起因：port-x86-nav 那轮 h 顶到表边 0.60 ✗、残差最大 2.74° ✗）
    #   ★ 判据两条 ✓：① h 必须落在**表内**（✗ 不许贴着 0.10/0.60 的边界 ⇒ 那是"搜到头了" ✗）
    #                ② 各行 θ_y 的散度必须小（平地模型成立 ✓）
    #   ⇒ ★ 只有两条都过才算"有效帧" ✓；★ 报数**一律带有效率** ✓（✗ 不拿无效帧的极值当峰值 ✗）。
    ap.add_argument('--h-lo', type=float, default=0.13, help='有效 h 下界（表边 0.10 之上）')
    ap.add_argument('--h-hi', type=float, default=0.55, help='有效 h 上界（表边 0.60 之下）')
    ap.add_argument('--max-spread', type=float, default=0.3, help='有效帧的散度上限（度）')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    ys = list(range(*[int(v) for v in a.band.split(':')]))
    hfov = 1.0472                      # ★ 与 campus_nav2.yaml 的 horizontal_fov 一致 ✓
    W = 320
    f = (W / 2.0) / math.tan(hfov / 2.0)   # ⇒ 277.13 px ✓
    cy = 240 / 2.0 - 0.5                    # ★ 主点行（图像中心）✓
    h_grid = [0.10 + 0.002 * i for i in range(251)]   # 0.10 … 0.60 m，2 mm 一档

    x0, x1 = W // 2 - 40, W // 2 + 40
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

    frames = []
    for t, data, info in got:
        prof = []
        for y in ys:
            z, nfin, ncol = row_depth(data, info, y, x0, x1)
            if z is not None and ncol > 0 and nfin >= a.min_frac * ncol:
                prof.append((y, z))
        if len(prof) < 3:
            frames.append({'t_sim': round(t, 3), 'pitch_deg': None,
                           'n_rows_used': len(prof), 'why': '可用行不足'})
            continue
        r = fit_frame(prof, f, cy, h_grid)
        if r is None:
            frames.append({'t_sim': round(t, 3), 'pitch_deg': None,
                           'n_rows_used': len(prof), 'why': '拟合失败'})
            continue
        th, h, spread, nrow = r
        # ★ 有效性闸（两条 ✓）：h 落在表内 ∧ 散度小 ✓
        h_ok = (a.h_lo <= h <= a.h_hi)
        s_ok = (spread <= a.max_spread)
        frames.append({'t_sim': round(t, 3), 'pitch_deg': round(th, 4), 'h_cam_m': round(h, 4),
                       'spread_deg': round(spread, 4), 'n_rows_used': nrow,
                       'valid': bool(h_ok and s_ok),
                       'invalid_reason': None if (h_ok and s_ok) else
                       ('h 顶到表边' if not h_ok else '') + ('' if h_ok else '；') + ('' if s_ok else '散度超限')})

    ok = [fr for fr in frames if fr.get('pitch_deg') is not None]
    val = [fr for fr in ok if fr.get('valid')]
    ths = [fr['pitch_deg'] for fr in val] if val else [fr['pitch_deg'] for fr in ok]
    ths_all = [fr['pitch_deg'] for fr in ok]
    sps = [fr.get('spread_deg') for fr in ok if fr.get('spread_deg') is not None]
    out = {
        'bag_dir': os.path.abspath(a.bag_dir), 'topic': a.topic,
        't_sim_center': a.t_sim, 'half_s': a.half,
        'n_frames_in_window': len(frames), 'n_frames_fitted': len(ok),
        'n_frames_valid': len(val),
        'valid_coverage': (round(len(val) / float(len(frames)), 4) if frames else None),
        'validity_gate': {'h_lo': a.h_lo, 'h_hi': a.h_hi, 'max_spread_deg': a.max_spread,
                          'rule': '★ h 必须落在表内（不贴 0.10/0.60 边界）∧ 各行 θ_y 散度 ≤ 上限 ⇒ 才算有效帧 ✓'},
        'pitch_deg_peak_to_peak_all_frames': (max(ths_all) - min(ths_all)) if ths_all else None,
        'model': ('★ Z(y) = h / tan(θ + atan((y-cy)/f)) ✓；★ 逐帧在 h 上搜"各行 θ_y 最集中"的那个 ⇒ '
                  '同时解出 (θ, h) ✓；★ 纯俯仰 ⇒ 各行 θ_y 相等 ✓，纯升沉 ⇒ 散开 ✗ ⇒ 多行才分得开 ✓'),
        'params': {'f_px': round(f, 3), 'cy_px': cy, 'rows_used': ys, 'min_frac': a.min_frac,
                   'deg_per_row_at_20deg': round(math.degrees(1.0 / f), 4)},
        'pitch_deg_min': min(ths) if ths else None,
        'pitch_deg_max': max(ths) if ths else None,
        'pitch_deg_peak_to_peak': (max(ths) - min(ths)) if ths else None,
        'pitch_deg_mean': (sum(ths) / len(ths)) if ths else None,
        'spread_deg_median': statistics.median(sps) if sps else None,
        'spread_deg_max': max(sps) if sps else None,
        'h_cam_m_median': statistics.median([fr['h_cam_m'] for fr in ok]) if ok else None,
        'frames': frames,
    }
    # ★ 单峰（机械定义 ✓ 与另一把尺子同口径）：取俯仰**最大**（最抬头）那一帧 ✓ 向两侧扩到不再 ≤ 极值 ✓
    idx = None
    for i, fr in enumerate(frames):
        if fr.get('pitch_deg') is None:
            continue
        if idx is None or fr['pitch_deg'] > frames[idx]['pitch_deg']:
            idx = i
    if idx is not None:
        peak = frames[idx]['pitch_deg']
        i0 = idx
        while i0 > 0 and frames[i0 - 1].get('pitch_deg') is not None and frames[i0 - 1]['pitch_deg'] >= peak:
            i0 -= 1
        i1 = idx
        while i1 < len(frames) - 1 and frames[i1 + 1].get('pitch_deg') is not None and frames[i1 + 1]['pitch_deg'] >= peak:
            i1 += 1
        seg = [fr['pitch_deg'] for fr in frames[i0:i1 + 1] if fr.get('pitch_deg') is not None]
        out['single_peak'] = {
            'definition': '★ 极值帧 = 窗口内俯仰最大（最抬头）者 ✓；★ 段 = 向两侧扩到"不再 ≤ 极值"为止 ✓',
            'peak_frame_t': frames[idx]['t_sim'], 'peak_pitch_deg': peak,
            'span_frames': [frames[i0]['t_sim'], frames[i1]['t_sim']],
            'n_frames_in_segment': len(seg),
            'segment_deg_min': min(seg), 'segment_deg_max': max(seg),
            'single_peak_span_deg': max(seg) - min(seg),
        }
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('★ 拟合 %d/%d 帧 ｜ f=%.1f px ｜ 1 行 ≈ %.3f°' % (len(ok), len(frames), f, math.degrees(1.0 / f)))
    if ths:
        print('★ 俯仰：min %.3f° ／ max %.3f° ／ ★ **峰-峰 %.3f°** ｜ 中位残差散度 %.3f° ／ 最大 %.3f°'
              % (out['pitch_deg_min'], out['pitch_deg_max'], out['pitch_deg_peak_to_peak'],
                 out['spread_deg_median'] or -1, out['spread_deg_max'] or -1))
        print('★ 相机高度中位 %.3f m' % (out['h_cam_m_median'] or -1))
    if out.get('single_peak'):
        sp = out['single_peak']
        print('★ 单峰（机械定义 ✓）：t=%.3f 峰值 %.3f° ／ 段 %s…%s ／ ★ **单峰跨度 %.3f°**'
              % (sp['peak_frame_t'], sp['peak_pitch_deg'], sp['span_frames'][0], sp['span_frames'][1],
                 sp['single_peak_span_deg']))
    return 0


if __name__ == '__main__':
    sys.exit(main())

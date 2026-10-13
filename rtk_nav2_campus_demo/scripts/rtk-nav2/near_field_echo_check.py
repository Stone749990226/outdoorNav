#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""near_field_echo_check.py —— ★ 找"**车自己的回波**" ✓（渲染式传感器的自见问题 ✓）

★ 机理（reviewer 2026-10-12 提 ✓）：★ 相机／激光是**渲染式**传感器（看 `<visual>` ✓）
  ⇒ ★ **车被渲染出来之后**，传感器可能把**车体自己的面**当成障碍 ✗ ⇒ 近距伪回波 ⇒
  ★ 控制器反复"前方有障碍" ⇒ 走走停停 ⇒ 每次停都点头 ✓ = **~1 Hz 俯仰** ✓。
★ 判据（★ 便宜、只看已有 bag ✓）：
  ★ ① `scan`：★ **贴车**（range < 阈值 ✓）的光束占比 ✓ ＋ ★ 这些光束是不是**固定的那几个索引** ✓
    （★ 真障碍会动 ✓；★ **固定不动** = 车体的一部分 ✓✓）
  ★ ② `depth_image`：★ **贴脸**（z < 阈值 ✓）的像素占比 ✓ ＋ ★ 这些像素是不是**固定的一块** ✓

用法：near_field_echo_check.py <bag目录> --t-sim 300 --half 25 --out x.json
      [--scan-topic /rtk/scan] [--depth-topic /rtk/camera/depth_image]
      [--scan-near 0.5] [--depth-near 0.3]
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


def parse_header(buf, off):
    sec, nsec = struct.unpack_from('<iI', buf, off)
    off = align(off + 8)
    fid, off = read_string(buf, off)
    return sec + nsec * 1e-9, fid, align(off)


def parse_scan(buf):
    """CDR(sensor_msgs/LaserScan) ⇒ (t, angle_min, angle_inc, ranges, range_min, range_max)"""
    t, _, off = parse_header(buf, 4)
    ang_min, ang_max, ang_inc, t_inc, scan_t = struct.unpack_from('<fffff', buf, off)
    off = align(off + 20)
    rmin, rmax = struct.unpack_from('<ff', buf, off)
    off = align(off + 8)
    n, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    if n == 0 or n > 5000:
        return None
    rng = struct.unpack_from('<%df' % n, buf, off)
    return t, ang_min, ang_inc, rng, rmin, rmax


def parse_image(buf):
    if len(buf) < 8:
        return None
    t, _, off = parse_header(buf, 4)
    h, w = struct.unpack_from('<II', buf, off)
    off = align(off + 8)
    enc, off = read_string(buf, off)
    off = align(off + 1)
    step, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    n, = struct.unpack_from('<I', buf, off)
    off = align(off + 4)
    return {'t': t, 'h': h, 'w': w, 'enc': enc, 'step': step, 'n': n, 'data_off': off}


def collect(bag_dir, topic, t_center, half):
    out = []
    for db in sorted(glob.glob(os.path.join(bag_dir, '*.db3'))):
        try:
            con = sqlite3.connect('file:%s?mode=ro' % db, uri=True)
            # ★ 只取**这个话题**的行 ✓（原来取全库 ⇒ 把 1.3 G 的深度图全读进来 ✗ ⇒ 慢到跑不完 ✗）
            rows = con.execute(
                "SELECT t.name, m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                "WHERE t.name = ?", (topic,)).fetchall()
            con.close()
        except sqlite3.Error:
            continue
        for name, data in rows:
            if name != topic:
                continue
            if topic.endswith('scan'):
                r = parse_scan(data)
                if r and abs(r[0] - t_center) <= half:
                    out.append(r)
            else:
                info = parse_image(data)
                if info and info['n'] > 0 and abs(info['t'] - t_center) <= half:
                    out.append((info['t'], info, data))   # ★ 统一成"第一项是时刻" ✓（否则排序时 dict 比 dict ✗）
    out.sort(key=lambda z: z[0])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--t-sim', type=float, required=True)
    ap.add_argument('--half', type=float, default=25.0)
    ap.add_argument('--scan-topic', default='/rtk/scan')
    ap.add_argument('--depth-topic', default='/rtk/camera/depth_image')
    ap.add_argument('--scan-near', type=float, default=0.5)
    ap.add_argument('--depth-near', type=float, default=0.3)
    ap.add_argument('--max-scan-frames', type=int, default=400)
    ap.add_argument('--max-depth-frames', type=int, default=200)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    res = {'bag_dir': os.path.abspath(a.bag_dir), 't_sim_center': a.t_sim, 'half_s': a.half,
           'scan_near_m': a.scan_near, 'depth_near_m': a.depth_near}

    # ── ① scan：贴车的光束 ──
    sc = collect(a.bag_dir, a.scan_topic, a.t_sim, a.half)
    if sc:
        step = max(1, len(sc) // a.max_scan_frames)
        sc = sc[::step]
        nbeam = len(sc[0][3])
        pers = [0] * nbeam
        fr_frac = []
        for t, amin, ainc, rng, rmin, rmax in sc:
            if len(rng) != nbeam:
                continue
            near = [1 if (v == v and v > 0 and v < a.scan_near) else 0 for v in rng]
            fr_frac.append(sum(near) / float(nbeam))
            for i, v in enumerate(near):
                pers[i] += v
        nfr = len(fr_frac)
        fixed = [(i, round(pers[i] / float(nfr), 3)) for i in range(nbeam)
                 if nfr and pers[i] / float(nfr) >= 0.9]
        # 固定索引的角位置（相对车头的角度 ✓）
        angs = [round((sc[0][1] + i * sc[0][2]), 3) for i, _ in fixed[:12]]
        res['scan'] = {
            'n_frames': nfr, 'n_beams': nbeam,
            'near_frac_mean': round(statistics.mean(fr_frac), 4) if fr_frac else None,
            'near_frac_max': round(max(fr_frac), 4) if fr_frac else None,
            'n_fixed_near_beams': len(fixed),
            'fixed_near_beams': fixed[:20],
            'fixed_near_angles_rad': angs,
            'verdict': ('★ 有**固定不动**的贴车光束 %d 条 ⇒ 像"车自己的面" ✗' % len(fixed)
                        if fixed else '★ 没有固定的贴车光束 ✓'),
        }
    else:
        res['scan'] = {'n_frames': 0, 'verdict': '★ 窗口里没有 scan 帧'}

    # ── ② depth：贴脸像素 ──
    dp = collect(a.bag_dir, a.depth_topic, a.t_sim, a.half)
    if dp:
        step = max(1, len(dp) // a.max_depth_frames)
        dp = dp[::step]
        persist = None
        n = 0
        fr_frac = []
        for _t, info, data in dp:
            if info['enc'] != '32FC1':
                continue
            h, w, st, base = info['h'], info['w'], info['step'], info['data_off']
            mask = []
            cnt = 0
            for y in range(0, h, 2):
                row = base + y * st
                for x in range(0, w, 2):
                    o = row + x * 4
                    if o + 4 > len(data):
                        break
                    v, = struct.unpack_from('<f', data, o)
                    near = 1 if (v == v and 0 < v < a.depth_near) else 0
                    mask.append(near)
                    cnt += near
            fr_frac.append(cnt / float(len(mask)) if mask else 0.0)
            persist = mask if persist is None else [p + m for p, m in zip(persist, mask)]
            n += 1
        if persist and n:
            kept = sum(1 for p in persist if p / float(n) >= 0.9)
            res['depth'] = {
                'n_frames': n, 'near_frac_mean': round(statistics.mean(fr_frac), 5),
                'near_frac_max': round(max(fr_frac), 5),
                'n_pixels_static_near': kept,
                'n_pixels_sampled': len(persist),
                'static_near_frac': round(kept / float(len(persist)), 5),
                'verdict': ('★ 有 %d 个**固定**贴脸像素（占采样 %.3f%%）⇒ 像"车自己的面" ✗'
                            % (kept, 100.0 * kept / len(persist))) if kept > 20
                           else '★ 几乎没有固定贴脸像素 ✓（%d 个）' % kept,
            }
        else:
            res['depth'] = {'n_frames': n, 'verdict': '★ 没有可解析的深度帧'}
    else:
        res['depth'] = {'n_frames': 0, 'verdict': '★ 窗口里没有深度帧'}

    json.dump(res, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    if 'n_frames' in res.get('scan', {}):
        print('★ scan：%d 帧 ｜ 贴车光束占比均值 %.4f ｜ ★ 固定贴车光束 %d 条'
              % (res['scan']['n_frames'], res['scan']['near_frac_mean'] or -1,
                 res['scan']['n_fixed_near_beams']))
    if 'n_frames' in res.get('depth', {}):
        print('★ depth：%d 帧 ｜ 贴脸像素占比均值 %.5f ｜ ★ 固定贴脸像素 %s 个'
              % (res['depth']['n_frames'], res['depth']['near_frac_mean'] or -1,
                 res['depth'].get('n_pixels_static_near')))
    return 0


if __name__ == '__main__':
    sys.exit(main())

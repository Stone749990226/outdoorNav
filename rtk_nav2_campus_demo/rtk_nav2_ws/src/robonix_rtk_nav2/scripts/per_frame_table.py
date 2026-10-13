#!/usr/bin/env python3
"""★ reviewer 2026-10-08 要求：**可复算的逐帧表**。

结论要能被重算，不能只被相信 ⇒ 本脚本把每帧的原料**逐行摊开**，让 reviewer 自己切"运动帧"：
  每行 = wall(epoch) + sim t + 真值位姿(x,y,yaw) + **上一行到本行的时间间隔** + 本行的位移/速度
  + ec（edge_columns，未过滤口径）+ ptp_across + prefilter_ok + merged + drop_th + ground_ref + live

**速度列的口径（先声明）**：`speed_1s_mps` = 以本帧 wall 为终点、向前找 **≥1 s** 的最近位姿样本，
位移 / 实际时间间隔。**它只是一列便利值**；判"运动帧"请用你自己从 (wall, x, y) 复算的口径 ✓。

用法：per_frame_table.py --run-dir <run> [--csv out.csv]
"""
import argparse
import csv
import json
import math
import os


def load_pose(run):
    pa = []
    p = os.path.join(run, 'pose_attitude.csv')
    if os.path.exists(p):
        with open(p) as f:
            for d in csv.DictReader(f):
                try:
                    pa.append((float(d['wall']), float(d['gt_x']), float(d['gt_y']),
                               float(d.get('gt_yaw_deg') or 0.0)))
                except Exception:
                    pass
    pa.sort()
    return pa


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--csv', default='')
    # ★ reviewer 2026-10-08 要求：加"是否在观测窗内"列（可参数化）——
    #   否则"窗口内 ptp 全 <0.5"这个结论换个口径就不成立 ✗。
    ap.add_argument('--window', default='0.30,1.00,1.10,2.60',
                    help='观测窗 x0,x1,y0,y1（默认 = §7.19/§7.29 用的同姿态窗口）')
    a = ap.parse_args()
    run = a.run_dir
    pa = load_pose(run)
    if not pa:
        print('无 pose_attitude.csv'); return
    t0 = pa[0][0]

    def pose_at(w):
        b = None
        for q in pa:
            if q[0] <= w:
                b = q
            else:
                break
        return b

    def speed_1s(w):
        i = None
        for k, q in enumerate(pa):
            if q[0] <= w:
                i = k
            else:
                break
        if i is None:
            return None
        w0, x0, y0, _ = pa[i]
        for j in range(i - 1, -1, -1):
            if w0 - pa[j][0] >= 1.0:
                dt = w0 - pa[j][0]
                return math.hypot(x0 - pa[j][1], y0 - pa[j][2]) / dt
        return None

    w0, w1, w2, w3 = [float(v) for v in a.window.split(',')]
    rows = []
    with open(os.path.join(run, 'regime', 'frames.jsonl')) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except Exception:
                continue
            if 'edge_columns' not in r:
                continue
            w = r['wall']
            q = pose_at(w)
            sp = speed_1s(w)
            rows.append({
                'wall': round(w, 3), 't_rel_s': round(w - t0, 2),
                'gt_x': None if q is None else round(q[1], 4),
                'gt_y': None if q is None else round(q[2], 4),
                'gt_yaw_deg': None if q is None else round(q[3], 2),
                'speed_1s_mps': None if sp is None else round(sp, 5),
                'in_window': bool(q is not None and w0 <= q[1] <= w1 and w2 <= q[2] <= w3),
                'ec': r.get('edge_columns'), 'ptp_across': r.get('ptp_across'),
                'prefilter_ok': r.get('prefilter_ok'), 'merged': r.get('merged'),
                'drop_th': r.get('drop_th'), 'ground_ref': r.get('ground_ref'),
                'live': r.get('live'),
            })
    out = a.csv or os.path.join(run, 'per_frame_table.csv')
    with open(out, 'w', newline='') as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    nn = [r['ptp_across'] for r in rows if r['ptp_across'] is not None]
    print('逐帧表 %d 行 → %s' % (len(rows), out))
    print('列：%s' % ', '.join(rows[0].keys()))
    mv = [r for r in rows if r['ptp_across'] is not None and (r['speed_1s_mps'] or 0) >= 0.005]
    mvw = [r for r in mv if r['in_window']]
    def q(lst):
        v = sorted(r['ptp_across'] for r in lst)
        return 'n=%d 中位=%.3f 最大=%.3f  <0.5:%d  ≥0.5:%d' % (
            len(v), v[len(v)//2], v[-1], sum(1 for x in v if x < 0.5), sum(1 for x in v if x >= 0.5)) if v else 'n=0'
    print('观测窗 %s' % a.window)
    print('运动帧(整轮) 且 ptp 非空：%s' % q(mv))
    print('运动帧(窗内) 且 ptp 非空：%s' % q(mvw))


if __name__ == '__main__':
    main()

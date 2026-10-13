#!/usr/bin/env python3
"""第四步验收：从**真值轨迹**上判"到没到、是不是撞墙/卡住停在附近"。

为什么用真值而不是估计：判据 4/6 的原话是"以真值位置判到达（≤ 0.30 m）"。
`trajectory_logger` 写的 CSV 就是真值（Gazebo `/ground_truth/odom`）+ 估计两列，
它在**整个栈运行期间**持续追加（每 0.05 s 一行，立即 flush），所以可以边跑边读——
本工具因此同时是"等到车到点"的等待器（`wait`）和"这一段走得怎么样"的分析器（`leg`）。

真值的角色是**事后评估的尺子**：它不进 TF/定位/控制/代价地图（来源审计口径见
`docs/rtk-nav2-validation.md`），这里只用它判到达与几何。

子命令：
  wait    轮询 CSV，直到真值位置进入目标点的 tol 内（连续 2 个采样都满足才算），
          写 JSON（到达时刻/位置/距离/用时）。超时则 arrived=false —— **不粉饰**。
  leg     对一段窗口 [--from-t, --to-t] 做分析：
            走过距离、到目标的最小距离、末段位移（判"不是卡住"）、最大 1 s 位移、
            **沿真值轨迹的穿墙核对**（与判据 7 同一套场地几何）、到最近真实表面的最小距离、
            以及"最后是不是停在开阔处"。
  render  把全局图 npz 快照渲染成 PNG（未知/空闲/膨胀/致命 四色）+ 真值轨迹 + 目标点，
          给"图上看得见效果"留一份直观产物（不依赖 RViz/GUI）。

用法：
  nav-truth.py wait artifacts/rtk-nav2/<run>/trajectory.csv --target -2.6 2.6 \
      --timeout 240 --json artifacts/rtk-nav2/<run>/nav4/leg_A_arrival.json
  nav-truth.py leg  artifacts/rtk-nav2/<run>/trajectory.csv --target -2.6 2.6 \
      --from-t 0 --json artifacts/rtk-nav2/<run>/nav4/leg_A_report.json
  nav-truth.py render artifacts/rtk-nav2/<run>/nav2/global_after_A.npz \
      --csv artifacts/rtk-nav2/<run>/trajectory.csv --png out.png --goal -2.6 2.6
"""

import argparse
import csv
import json
import math
import os
import sys
import time

# 场地几何（穿墙核对/最近表面）与第三阶段共用同一份常量
GEOM_DIR = os.environ.get(
    'NAV4_GEOM_DIR',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                 'rtk_nav2_ws', 'src', 'robonix_rtk_nav2', 'scripts'))
sys.path.insert(0, os.path.abspath(GEOM_DIR))
from map_geometry import nearest_surface, segment_hits_obstacle  # noqa: E402


def read_rows(path):
    rows = []
    try:
        with open(path, newline='', encoding='utf-8') as fh:
            for row in csv.DictReader(fh):
                try:
                    rows.append((float(row['t']), float(row['gt_x']), float(row['gt_y']),
                                 float(row['gt_yaw'])))
                except (TypeError, ValueError):
                    continue
    except FileNotFoundError:
        return []
    return rows


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def travelled(rows):
    return sum(dist((a[1], a[2]), (b[1], b[2])) for a, b in zip(rows, rows[1:]))


def window(rows, t0, t1):
    return [r for r in rows if (t0 is None or r[0] >= t0) and (t1 is None or r[0] <= t1)]


def analyse(rows, target, tol):
    """对一段轨迹做判据 4/6 需要的全部几何量。"""
    out = {'samples': len(rows), 'target': list(target), 'tol_m': tol}
    if not rows:
        out['error'] = '这段窗口里没有真值采样'
        return out
    d = [dist((r[1], r[2]), target) for r in rows]
    imin = min(range(len(d)), key=lambda i: d[i])
    out['t_from'] = rows[0][0]
    out['t_to'] = rows[-1][0]
    out['start_pos'] = [round(rows[0][1], 3), round(rows[0][2], 3)]
    out['end_pos'] = [round(rows[-1][1], 3), round(rows[-1][2], 3)]
    out['min_dist_to_target_m'] = round(d[imin], 3)
    out['min_dist_at_t'] = round(rows[imin][0], 2)
    out['reached_within_tol'] = bool(d[imin] <= tol)
    out['travelled_m'] = round(travelled(rows), 3)

    def disp_over(sec):
        t_end = rows[-1][0]
        sub = [r for r in rows if r[0] >= t_end - sec]
        if len(sub) < 2:
            return None
        return round(dist((sub[0][1], sub[0][2]), (sub[-1][1], sub[-1][2])), 3)

    out['displacement_last_5s_m'] = disp_over(5.0)
    out['displacement_last_10s_m'] = disp_over(10.0)
    # 最大 1 s 位移（≈ 峰值速度，用于说明"车真的在动"以及动得有多慢）
    peak = 0.0
    j = 0
    for i, r in enumerate(rows):
        while rows[j][0] < r[0] - 1.0:
            j += 1
        peak = max(peak, dist((rows[j][1], rows[j][2]), (r[1], r[2])))
    out['max_displacement_per_1s_m'] = round(peak, 3)
    out['mean_speed_mps'] = round(travelled(rows) / max(1e-6, rows[-1][0] - rows[0][0]), 3)

    # 穿墙核对：沿**真值**轨迹逐段求交（与判据 7 用同一套几何）
    cross = {}
    for a, b in zip(rows, rows[1:]):
        hit = segment_hits_obstacle((a[1], a[2]), (b[1], b[2]), margin=0.05)
        if hit:
            cross.setdefault(hit, 0)
            cross[hit] += 1
    out['wall_crossings_along_truth'] = cross
    out['crosses_wall_along_truth'] = bool(cross)
    # 到最近真实表面的最小距离：贴墙走会逼近 0，撞上就 < 0 不可能，但最小距离能说明"停在哪"
    clr = min(nearest_surface(r[1], r[2]) for r in rows)
    out['min_clearance_to_surface_m'] = round(clr, 3)
    out['clearance_at_end_m'] = round(nearest_surface(rows[-1][1], rows[-1][2]), 3)
    out['note'] = ('displacement_last_*s 用来判"不是撞墙/卡住停在附近"：'
                   '卡住的表现是末段位移 ~0；wall_crossings_along_truth 用场地几何'
                   '（中央方块/隔墙/立柱/4 段独立墙；**外墙已于 2026-09-21 删除**）'
                   '逐段求交，非空就是真值轨迹穿墙了。')
    return out


def cmd_wait(args):
    start = time.time()
    tol = args.tol
    consecutive = 0
    last = None
    while time.time() - start < args.timeout:
        rows = read_rows(args.csv)
        if rows:
            last = rows[-1]
            d = dist((last[1], last[2]), (args.target[0], args.target[1]))
            if d <= tol:
                consecutive += 1
                if consecutive >= 2:
                    out = {'arrived': True,
                           'arrival_sim_t': round(last[0], 2),
                           'arrival_pos': [round(last[1], 3), round(last[2], 3)],
                           'dist_m': round(d, 3),
                           'tol_m': tol,
                           'wall_elapsed_s': round(time.time() - start, 1),
                           'dist_from_start_m': round(dist(
                               (rows[0][1], rows[0][2]), (last[1], last[2])), 3)}
                    report(args, out)
                    return 0
            else:
                consecutive = 0
        time.sleep(1.0)
    rows = read_rows(args.csv)
    pos = [round(rows[-1][1], 3), round(rows[-1][2], 3)] if rows else None
    out = {'arrived': False,
           'timeout_s': args.timeout,
           'last_pos': pos,
           'dist_m': (round(dist((pos[0], pos[1]), tuple(args.target)), 3)
                      if pos else None),
           'tol_m': tol,
           'dist_from_start_m': (round(dist((rows[0][1], rows[0][2]), (pos[0], pos[1])), 3)
                                 if rows and pos else None),
           'note': '超时没到：这个数就是要照实写进结论里的'}
    report(args, out)
    return 1


def report(args, out):
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)


def cmd_now(args):
    """打印 CSV 最后一条有效采样（sim 时刻 + 真值位置）。

    用法：给目标之前先取一次，得到"这一段从哪一刻开始"，`leg` 的窗口就按它切——
    真值 CSV 的 t 是**仿真时间**，与墙钟对不上，事后没法靠墙钟把两段分开。
    """
    rows = read_rows(args.csv)
    if not rows:
        print(json.dumps({'error': '还没有真值采样'}))
        return 1
    out = {'sim_t': round(rows[-1][0], 3),
           'gt': [round(rows[-1][1], 3), round(rows[-1][2], 3)],
           'samples': len(rows)}
    print(json.dumps(out, ensure_ascii=False))
    return 0


def cmd_leg(args):
    rows = window(read_rows(args.csv), args.from_t, args.to_t)
    out = analyse(rows, (args.target[0], args.target[1]), args.tol)
    out['csv'] = args.csv
    out['window'] = {'from_t': args.from_t, 'to_t': args.to_t}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
    return 0


# ------------------------------------------------------------------ PNG 渲染
def _png(path, rgb):
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))

    blob = (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 6))
            + chunk(b'IEND', b''))
    with open(path, 'wb') as fh:
        fh.write(blob)


def _mark(rgb, row, col, value, rad=0):
    h, w, _ = rgb.shape
    for dr in range(-rad, rad + 1):
        for dc in range(-rad, rad + 1):
            rr, cc = row + dr, col + dc
            if 0 <= rr < h and 0 <= cc < w:
                rgb[rr, cc] = value


def cmd_render(args):
    import numpy as np
    z = np.load(args.npz)
    data = z['data']
    res = float(z['resolution'])
    ox = float(z['origin_x'])
    oy = float(z['origin_y'])
    h, w = data.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[:, :] = (205, 205, 205)                 # 未知
    rgb[data == 0] = (255, 255, 255)            # 空闲
    rgb[(data > 0) & (data < 50)] = (255, 214, 170)   # 膨胀带
    rgb[data >= 50] = (170, 60, 0)              # 占据（含膨胀到 >=50 的档）
    rgb[data >= 100] = (0, 0, 0)                # 致命格 = 真实障碍表面

    # 真值轨迹（红）：只画 px 上没画过的点，省时间
    if args.csv and os.path.exists(args.csv):
        for t, x, y, yaw in read_rows(args.csv):
            col = int((x - ox) / res)
            row = h - 1 - int((y - oy) / res)
            if 0 <= row < h and 0 <= col < w:
                rgb[row, col] = (220, 0, 0)
    for gx, gy in (args.goal or []):
        col = int((gx - ox) / res)
        row = h - 1 - int((gy - oy) / res)
        _mark(rgb, row, col, (0, 90, 255), rad=3)   # 目标点（蓝）
    png = args.png
    os.makedirs(os.path.dirname(os.path.abspath(png)), exist_ok=True)
    _png(png, rgb)
    print(json.dumps({'png': png, 'size': [w, h], 'resolution': res,
                      'origin': [ox, oy], 'markers': '蓝=目标点，红=真值轨迹',
                      'colors': {'未知': '(205,205,205)', '空闲': '白',
                                 '膨胀带': '(255,214,170)',
                                 '占据>=50': '(170,60,0)', '致命>=100': '黑'}},
                     ensure_ascii=False, indent=2))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('wait')
    p.add_argument('csv')
    p.add_argument('--target', nargs=2, type=float, required=True, metavar=('X', 'Y'))
    p.add_argument('--tol', type=float, default=0.30)
    p.add_argument('--timeout', type=float, default=240.0)
    p.add_argument('--json', default='')
    p.set_defaults(func=cmd_wait)

    p = sub.add_parser('now')
    p.add_argument('csv')
    p.set_defaults(func=cmd_now)

    p = sub.add_parser('leg')
    p.add_argument('csv')
    p.add_argument('--target', nargs=2, type=float, required=True, metavar=('X', 'Y'))
    p.add_argument('--tol', type=float, default=0.30)
    p.add_argument('--from-t', type=float, default=None)
    p.add_argument('--to-t', type=float, default=None)
    p.add_argument('--json', default='')
    p.set_defaults(func=cmd_leg)

    p = sub.add_parser('render')
    p.add_argument('npz')
    p.add_argument('--csv', default='')
    p.add_argument('--png', required=True)
    p.add_argument('--goal', nargs=2, type=float, action='append', metavar=('X', 'Y'))
    p.set_defaults(func=cmd_render)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())

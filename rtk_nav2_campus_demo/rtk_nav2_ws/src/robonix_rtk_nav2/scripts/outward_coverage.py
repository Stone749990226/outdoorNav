#!/usr/bin/env python3
"""**朝外方向的覆盖核对**：从真值轨迹算"这次行驶到底朝哪些方向看过"，并据此判断
"幽灵格工况"到底有没有被走到。

为什么需要它：TASK A 的 ablation 要求"**跑到会产生幽灵的那种工况**"。
**注意（2026-09-21 修正）**：本脚本原来按"幽灵来自相机看到地面方板板边时的上飘像素"写，
**那条归属已被 ablation 反证**（板边 ≥7.9 m、出 6 m 量程之外时幽灵照样出）。
本脚本本身仍然是**有效的覆盖度工具**：它量的就是"当次行驶有没有把'原板边那个方向'
在 4.4~6.0 m 视距内扫过"，用来证明"那个方向确实看过"；**它不代表幽灵的成因**。
板半边长用 `--plate-half` 传（默认新板 11.35；板 10.70 时传 5.35）。
所以"有没有覆盖到那个工况"不是"走了几圈"，而是**可判定的几何问题**：

  对真值轨迹上的每个采样点（真值 x, y, yaw），在相机水平视场 ±FOV/2 内逐度投一条射线，
  求它与**板边方框**的交点距离 d。**若存在一条 4.4 m <= d <= 6.0 m 的射线**，
  就说明该时刻相机能看到板边那条轮廓线（4.4~6.0 是实测会写字的那段距离：
  抓到过的现行是 4.98 m 与 5.15 m；上界 6.0 是 `obstacle_max_range`）。

  该判据只依赖世界几何与真值位姿，**与地图、阈值无关**，所以可以拿它当 ablation 的
  "工况覆盖度"证据：覆盖到（或没覆盖到）都是可复核的数字，不是"走了一圈"这种含糊说法。

用法：
  outward_coverage.py <trajectory.csv> [--json 结果.json]
  trajectory.csv 的表头：t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw
"""

import argparse
import csv
import json
import math
import os
import sys

# 相机水平视场：world 文件里写死 1.047 rad（60°）
FOV_DEG = 60.0
# 幽灵可写字的距离窗（实测 + obstacle_max_range）
GHOST_NEAR = 4.4
GHOST_FAR = 6.0
# 旧世界板边半边长（去掉外墙之前那份世界，见 docs 的几何记录）
OLD_PLATE_HALF = 5.35
# 新世界板边半边长：2 * (GROUND_HALF 5.0 + GROUND_MARGIN 6.15) + 0.4 = 22.70
NEW_PLATE_HALF = 11.35

SIDES = {'east': 0.0, 'north': 90.0, 'west': 180.0, 'south': -90.0}


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def yaw_to_deg(raw):
    """`trajectory.csv` 的 `gt_yaw` 是**弧度**（实测取值 −3.142..3.142）。

    这里按量程自动判定：全都落在 ±π 内就当弧度，否则当度。写成自适应是因为
    "单位看错"在这台机器上已经骗过好几次（第一列停更、内外层 timeout），
    而单位错了**不会报错**，只会把结论整体转错一个量纲。
    """
    if abs(raw) <= math.pi + 1e-6:
        return math.degrees(raw)
    return raw


def ray_box_exit(x, y, yaw_deg, half):
    """从 (x, y) 沿 yaw 方向投射线，求它与 |x|<=half 且 |y|<=half 方框的边界交点距离。

    返回 (距离, 命中的边名) 或 (None, None)。
    **从方框内部**出发时取正向出口；从外部出发时取**最近的**正向交点
    （车开到板外时仍能算出"板边在多远"）。
    """
    th = math.radians(yaw_deg)
    dx, dy = math.cos(th), math.sin(th)
    best = None
    best_side = None
    for side, (px, py, axis, sign) in (
            ('east', (half, 0.0, 'x', 1)),
            ('west', (-half, 0.0, 'x', -1)),
            ('north', (0.0, half, 'y', 1)),
            ('south', (0.0, -half, 'y', -1))):
        if axis == 'x':
            if abs(dx) < 1e-9:
                continue
            t = (px - x) / dx
        else:
            if abs(dy) < 1e-9:
                continue
            t = (py - y) / dy
        if t <= 0:
            continue
        hx, hy = x + t * dx, y + t * dy
        if abs(hx) > half + 1e-6 or abs(hy) > half + 1e-6:
            continue
        if best is None or t < best:
            best, best_side = t, side
    return best, best_side


def _tf(row):
    try:
        return float(row['t'])
    except (KeyError, TypeError, ValueError):
        return -1.0


def analyse(rows, half):
    """对一套板边半边长，统计轨迹上的覆盖情况。"""
    half_fov = FOV_DEG / 2.0
    samples = 0
    ghost_times = []          # 存在"能看到板边"射线的时刻
    per_side = {k: {'samples': 0, 't_min': None, 't_max': None, 'd_min': None, 'd_max': None}
                for k in ('east', 'north', 'west', 'south')}
    heading_bins = {}
    ray_d_min = None          # 全场最小"到板边"射线距离（不限 4.4~6.0）
    robot_d_min = None        # 全场机器人体心到板边的最近距离
    for row in rows:
        try:
            x, y = float(row['gt_x']), float(row['gt_y'])
            yaw = yaw_to_deg(float(row['gt_yaw']))
            t = float(row['t'])
        except (KeyError, TypeError, ValueError):
            continue
        samples += 1
        b = int((wrap180(yaw) + 180.0) // 15.0) * 15 - 180
        heading_bins[b] = heading_bins.get(b, 0) + 1
        hit_any = False
        for k in range(-int(half_fov), int(half_fov) + 1):
            d, side = ray_box_exit(x, y, yaw + k, half)
            if d is None:
                continue
            ray_d_min = d if ray_d_min is None else min(ray_d_min, d)
            if GHOST_NEAR <= d <= GHOST_FAR:
                hit_any = True
                e = per_side[side]
                e['samples'] += 1
                e['t_min'] = t if e['t_min'] is None else min(e['t_min'], t)
                e['t_max'] = t if e['t_max'] is None else max(e['t_max'], t)
                e['d_min'] = d if e['d_min'] is None else min(e['d_min'], d)
                e['d_max'] = d if e['d_max'] is None else max(e['d_max'], d)
        if hit_any:
            ghost_times.append(t)
        # 体心到板边的"最近距离"（方框内取到四条边的最近一条；方框外取到方框的最近点）
        dc = min(half - abs(x), half - abs(y)) if (abs(x) <= half and abs(y) <= half) \
            else math.hypot(max(abs(x) - half, 0.0), max(abs(y) - half, 0.0))
        robot_d_min = dc if robot_d_min is None else min(robot_d_min, dc)
    return {
        'plate_half_m': half,
        'samples': samples,
        'robot_min_dist_to_plate_edge_m': round(robot_d_min, 3) if robot_d_min is not None else None,
        'fov_min_ray_dist_to_plate_edge_m': round(ray_d_min, 3) if ray_d_min is not None else None,
        'ghost_capable_samples': len(ghost_times),
        'ghost_capable_fraction': round(len(ghost_times) / samples, 4) if samples else None,
        'ghost_capable_t_range_s': ([round(min(ghost_times), 2), round(max(ghost_times), 2)]
                                    if ghost_times else None),
        'by_side': per_side,
        'heading_hist_15deg': {str(k): v for k, v in sorted(heading_bins.items())},
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    ap.add_argument('--json', default=None)
    ap.add_argument('--plate-half', type=float, default=NEW_PLATE_HALF,
                    help='当前世界的板半边长（默认 %.2f）。做"板铺多大"的对照实验时必须显式传，'
                         '否则算出来的"新板边"不是当次实际跑的那块板。' % NEW_PLATE_HALF)
    ap.add_argument('--outward-tol', type=float, default=FOV_DEG / 2.0,
                    help='判"朝某个外方向看过"的角容差（默认半视场 30°）')
    ap.add_argument('--legs', default=None,
                    help='分段文件（ablation-drive.sh 写的 ablation-legs.txt）：'
                         '逐段给朝向范围与"幽灵可写射线"条数')
    args = ap.parse_args(argv)

    with open(args.trajectory, newline='', encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))

    out = {
        'trajectory': os.path.abspath(args.trajectory),
        'fov_deg': FOV_DEG,
        'ghost_window_m': [GHOST_NEAR, GHOST_FAR],
        'old_world': analyse(rows, OLD_PLATE_HALF),
        'new_world': analyse(rows, args.plate_half),
        'outward_heading_coverage': {},
    }
    # "朝某个外方向看过"：横摆角落在该方向的 ±tol 内
    for name, deg in SIDES.items():
        n = 0
        for row in rows:
            try:
                yaw = yaw_to_deg(float(row['gt_yaw']))
            except (KeyError, TypeError, ValueError):
                continue
            if abs(wrap180(yaw - deg)) <= args.outward_tol:
                n += 1
        out['outward_heading_coverage'][name] = {
            'direction_deg': deg, 'tol_deg': args.outward_tol, 'samples': n}

    # ---- 分段：逐段给"朝向范围 + 幽灵可写射线条数"（reviewer 要的是这个粒度）----
    if args.legs and os.path.exists(args.legs):
        marks = []
        for line in open(args.legs, encoding='utf-8'):
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'start_sim_t':
                marks.append(('start', float(parts[1]), None))
            elif parts[0] == 'end_sim_t':
                marks.append(('end', float(parts[1]), None))
            elif parts[0] == 'leg':
                # leg <n> goal <x> <y> arrived=<0|1> arr_sim_t <t>
                try:
                    t = float(parts[parts.index('arr_sim_t') + 1])
                    gx, gy = parts[3], parts[4]
                    ok = parts[5].split('=')[1]
                except (ValueError, IndexError):
                    continue
                marks.append(('leg', t, '%s(%s,%s) arrived=%s' % (parts[1], gx, gy, ok)))
        out['legs'] = []
        prev_t, prev_label = None, 'start'
        for kind, t, label in marks:
            if kind in ('start', 'end'):
                if kind == 'end' and prev_t is not None:
                    seg = [r for r in rows if prev_t < _tf(r) <= t]
                    a = analyse(seg, OLD_PLATE_HALF)
                    out['legs'].append({
                        'label': prev_label, 't_range': [prev_t, t],
                        'samples': a['samples'],
                        'old_ghost_capable_samples': a['ghost_capable_samples'],
                        'old_by_side': {s: v['samples'] for s, v in a['by_side'].items() if v['samples']},
                        'heading_hist_15deg': a['heading_hist_15deg'],
                    })
                prev_t = t
                continue
            if prev_t is not None:
                seg = [r for r in rows if prev_t < _tf(r) <= t]
                a = analyse(seg, OLD_PLATE_HALF)
                out['legs'].append({
                    'label': prev_label, 't_range': [prev_t, t],
                    'samples': a['samples'],
                    'old_ghost_capable_samples': a['ghost_capable_samples'],
                    'old_by_side': {s: v['samples'] for s, v in a['by_side'].items() if v['samples']},
                    'heading_hist_15deg': a['heading_hist_15deg'],
                })
            prev_t, prev_label = t, label

    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
        print('已写入 %s' % args.json, file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())

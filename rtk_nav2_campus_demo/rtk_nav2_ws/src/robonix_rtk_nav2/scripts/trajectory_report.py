#!/usr/bin/env python3
"""离线误差评估：定位精度 + 路线跟随的横向误差。

两类指标要分开看，混在一起会得出错误结论：

1. **定位精度**（est vs gt）：这条链路是 RTK + IMU 的融合结果，误差应当很小且**不随距离累积**。
   它是"绝对定位"这个卖点的直接体现。真值在这里只作为事后的尺子。
2. **横向跟踪误差**（轨迹到路线折线的垂距）：这是控制器 + 整车表现，不是定位精度。
   默认用**真值轨迹**来算（估计轨迹可能自己跟自己一致、看不出真实偏离），
   同时也给出用估计位姿算的一份，两者对比能看出定位误差在跟踪误差里占多少。

统计窗口默认取任务窗口（route_report.json 的 mission_start_sim_s / mission_end_sim_s）——
任务结束后栈还会继续跑，不设窗口会把静止段算进去，指标被稀释。

用法：
  trajectory_report.py <trajectory.csv> [--report route_report.json] [--output out.json]
"""

import argparse
import csv
import json
import math
import os
import sys


def load_csv(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as handle:
        for raw in csv.DictReader(handle):
            row = {}
            ok = True
            for key in ('t', 'gt_x', 'gt_y', 'gt_yaw'):
                try:
                    row[key] = float(raw[key])
                except (TypeError, ValueError, KeyError):
                    ok = False
                    break
            if not ok or not math.isfinite(row['gt_x']):
                continue
            for key in ('est_x', 'est_y', 'est_yaw'):
                try:
                    row[key] = float(raw[key])
                except (TypeError, ValueError, KeyError):
                    row[key] = None
            rows.append(row)
    return rows


def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    count = len(ordered)
    return {
        'samples': count,
        'mean': round(sum(ordered) / count, 4),
        'rmse': round(math.sqrt(sum(v * v for v in ordered) / count), 4),
        'max': round(ordered[-1], 4),
        'p95': round(ordered[min(count - 1, int(0.95 * count))], 4),
    }


def distance_to_polyline(point, route):
    """点到整条折线的最短距离（米）。用于横向跟踪误差。

    **必须把首尾接上**：路线是一个闭环（0→1→2→3→0），只遍历 len-1 段会漏掉
    收口那一段，最后一段直线上的偏差会被算成"离别的边多远"，误差虚高到米级
    （实测：真实横向误差 0.02~0.17 m，漏掉收口段时报告出 0.72 m RMSE / 2.60 m 最大）。
    """
    px, py = point
    best = None
    n = len(route)
    for i in range(n):
        ax, ay = route[i]
        bx, by = route[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        if denom < 1e-12:
            dist = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
            dist = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        best = dist if best is None else min(best, dist)
    return best


def main(argv=None):
    parser = argparse.ArgumentParser(description='RTK 版路线跟随误差评估')
    parser.add_argument('csv_path')
    parser.add_argument('--report', default='',
                        help='route_report.json；默认在 CSV 同级目录找，用它界定任务窗口与路线')
    parser.add_argument('--output', default='')
    args = parser.parse_args(argv)

    report_path = args.report
    if not report_path:
        candidate = os.path.join(os.path.dirname(os.path.abspath(args.csv_path)),
                                 'route_report.json')
        report_path = candidate if os.path.exists(candidate) else ''

    route = None
    window = None
    if report_path and os.path.exists(report_path):
        with open(report_path, encoding='utf-8') as handle:
            mission = json.load(handle)
        route = mission.get('route')
        start = mission.get('mission_start_sim_s')
        end = mission.get('mission_end_sim_s')
        if start is not None and end is not None and end > start:
            window = (float(start), float(end))

    rows = load_csv(args.csv_path)
    if not rows:
        print('CSV 没有可用样本', file=sys.stderr)
        return 1

    used_window = False
    if window:
        kept = [r for r in rows if window[0] <= r['t'] <= window[1]]
        if len(kept) >= 10:
            rows = kept
            used_window = True

    out = {
        'samples': len(rows),
        'time_range_s': [round(rows[0]['t'], 3), round(rows[-1]['t'], 3)],
        'window': ({
            'mission_start_sim_s': round(window[0], 3),
            'mission_end_sim_s': round(window[1], 3),
            'source': report_path or 'route_report.json（同级目录）',
        } if used_window else {
            'source': '整段 CSV',
            'note': '没有可用的任务窗口，指标含任务前后的静止段，仅供参考',
        }),
    }

    # ---- 1) 定位精度（est vs gt）----
    paired = [r for r in rows if r['est_x'] is not None]
    if paired:
        errs = [math.hypot(r['est_x'] - r['gt_x'], r['est_y'] - r['gt_y']) for r in paired]
        yaw_errs = [abs(math.atan2(math.sin(r['est_yaw'] - r['gt_yaw']),
                                   math.cos(r['est_yaw'] - r['gt_yaw'])))
                    for r in paired]
        out['localization'] = {
            'samples': len(paired),
            'position_error_m': stats(errs),
            'yaw_error_rad': stats(yaw_errs),
            'note': 'RTK(模拟)+IMU 融合位姿与真值之差；绝对定位，误差不随距离累积',
        }
    else:
        out['localization'] = {'error': 'no_estimate_column'}

    # ---- 1b) 真值里程（最硬的口径：真值不受估计噪声影响）----
    if len(rows) >= 2:
        gt_len = sum(math.hypot(rows[i]['gt_x'] - rows[i - 1]['gt_x'],
                                rows[i]['gt_y'] - rows[i - 1]['gt_y'])
                     for i in range(1, len(rows)))
        out['path_length_truth_m'] = round(gt_len, 3)
        if paired:
            est_len = 0.0
            for i in range(1, len(paired)):
                est_len += math.hypot(paired[i]['est_x'] - paired[i - 1]['est_x'],
                                      paired[i]['est_y'] - paired[i - 1]['est_y'])
            out['path_length_estimate_m'] = round(est_len, 3)
            out['path_length_note'] = ('estimate 那条是逐帧求和的原始值，会因定位噪声虚涨，'
                                       '只作对照；要报里程请用 truth 那条')

    # ---- 2) 横向跟踪误差 ----
    if route and len(route) >= 2:
        gt_lat = [distance_to_polyline((r['gt_x'], r['gt_y']), route) for r in rows]
        out['lateral_error_truth'] = dict(stats(gt_lat) or {},
                                          note='用**真值**轨迹算的横向偏差（推荐口径）')
        if paired:
            est_lat = [distance_to_polyline((r['est_x'], r['est_y']), route)
                       for r in paired]
            out['lateral_error_estimate'] = dict(stats(est_lat) or {},
                                                 note='用估计位姿算的横向偏差，供与真值那份对比')
    else:
        out['lateral_error_truth'] = {'error': 'no_route_in_report'}

    text = json.dumps(out, ensure_ascii=False, indent=2)
    print(text)
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)) or '.', exist_ok=True)
        with open(args.output, 'w', encoding='utf-8') as handle:
            handle.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

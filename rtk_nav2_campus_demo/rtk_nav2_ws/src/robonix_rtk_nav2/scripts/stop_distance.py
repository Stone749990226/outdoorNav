#!/usr/bin/env python3
"""从真值轨迹算"障碍停车距离"（判据 6 里"障碍停车"这一条的量化口径）。

为什么不用"最后一帧的位姿"：那样会把"停住之后又蹭了一点"或"最后几帧是静止的"
算进来，而且一旦 CSV 末尾有异常帧，距离就错了。这里取**整段轨迹里到障碍表面的最小距离**
——那才是"这台车最近到了离障碍多远"，再附上最小值出现的时刻与末帧值作为对照。

用法：stop_distance.py <trajectory.csv> --obstacle X Y [--side 0.4] [--json 结果.json]
"""

import argparse
import csv
import json
import math
import sys


def dist_to_surface(x, y, ox, oy, side):
    dx = max(abs(x - ox) - side / 2.0, 0.0)
    dy = max(abs(y - oy) - side / 2.0, 0.0)
    return math.hypot(dx, dy)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv_path')
    parser.add_argument('--obstacle', nargs=2, type=float, required=True, metavar=('X', 'Y'))
    parser.add_argument('--side', type=float, default=0.4)
    parser.add_argument('--json', default='')
    args = parser.parse_args(argv)

    ox, oy = args.obstacle
    rows = []
    with open(args.csv_path, encoding='utf-8') as handle:
        for r in csv.DictReader(handle):
            try:
                rows.append({'t': float(r.get('t') or r.get('stamp') or 0.0),
                             'gt_x': float(r['gt_x']), 'gt_y': float(r['gt_y']),
                             'est_x': float(r['est_x']) if r.get('est_x') not in (None, '') else None,
                             'est_y': float(r['est_y']) if r.get('est_y') not in (None, '') else None})
            except (KeyError, ValueError):
                continue
    if not rows:
        print(json.dumps({'error': 'CSV 没有可用行', 'columns_hint': 'gt_x/gt_y'}, ensure_ascii=False))
        return 1

    gt = [(dist_to_surface(r['gt_x'], r['gt_y'], ox, oy, args.side), r) for r in rows]
    d_min, r_min = min(gt, key=lambda p: p[0])
    est = [(dist_to_surface(r['est_x'], r['est_y'], ox, oy, args.side), r) for r in rows
           if r['est_x'] is not None]
    out = {
        'csv': args.csv_path,
        'samples': len(rows),
        'obstacle': {'center': [ox, oy], 'side_m': args.side,
                     'surface': '边长 %.2f m 的方块表面' % args.side},
        'truth': {
            'min_distance_to_surface_m': round(d_min, 4),
            'at_time_s': round(r_min['t'], 3),
            'last_frame_distance_m': round(gt[-1][0], 4),
            'note': '整段真值轨迹里到障碍表面的**最小**距离——"最近到了多远"；'
                    '机器人前缘约在 base_footprint 前 0.07 m 处，'
                    '所以实际间隙约等于该值减 0.07',
        },
    }
    if est:
        e_min, e_row = min(est, key=lambda p: p[0])
        out['estimate'] = {'min_distance_to_surface_m': round(e_min, 4),
                           'at_time_s': round(e_row['t'], 3)}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""把 `obstacle_map_check.py` 的产物**分项**读出来：判据 ③ 的"在场性"。

为什么要单独一个脚本：`obstacle_map_check.json` 里的键是
`present_obstacles_should_exist`（每项含 rect/lethal/free/unknown/max_cost）与
`old_outer_wall_lethal_total`——第一轮我用临时 python 读时把键名写成了
`old_wall_lethal_total`，于是**分项在场性这一条没跑出来**（数据其实一直在产物里，
只是没被读出来）。所以把它固定成一个可复跑的小工具。

用法：
  obstacle_present_report.py <obstacle_map_check.json> [--json 结果.json]
"""

import argparse
import json
import sys


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('check_json')
    ap.add_argument('--json', default='')
    args = ap.parse_args(argv)

    with open(args.check_json, encoding='utf-8') as fh:
        j = json.load(fh)

    present = j['present_obstacles_should_exist']
    far = j['lethal_far_from_real_surface']
    counts = j['counts']
    out = {
        'source': j.get('source'),
        'map_stamp_s': j.get('meta', {}).get('stamp_s'),
        'counts': counts,
        'old_outer_wall_lethal_total': j.get('old_outer_wall_lethal_total'),
        'present': {k: {'rect': v['rect'], 'lethal': v['lethal'], 'free': v['free'],
                        'unknown': v['unknown'], 'max_cost': v['max_cost']}
                    for k, v in present.items()},
        'ghost_lethal': {'threshold_m': far['threshold_m'], 'count': far['count'],
                         'max_distance_m': far['max_distance_m'], 'bbox_xy': far['bbox_xy']},
    }
    out['present_lethal_total'] = sum(v['lethal'] for v in present.values())
    out['present_missing'] = [k for k, v in present.items() if v['lethal'] <= 0]

    print('判据 ③ 分项在场性（来源 %s，图时间戳 %s）'
          % (out['source'], out['map_stamp_s']))
    print('  全图：cells=%d free=%d lethal=%d ge50=%d unknown=%d'
          % (counts['cells'], counts['free'], counts['lethal'],
             counts['occupied_ge50'], counts['unknown']))
    print('  旧外墙带（应已删除）：致命格合计 = %s' % out['old_outer_wall_lethal_total'])
    for name, v in sorted(present.items()):
        flag = '' if v['lethal'] > 0 else '   ← ** 该障碍没进图 **'
        print('  %-14s rect=%-38s lethal=%3d free=%3d unknown=%3d max_cost=%s%s'
              % (name, str(v['rect']), v['lethal'], v['free'], v['unknown'],
                 v['max_cost'], flag))
    print('  在场障碍里的致命格合计 = %d / %d 项有致命格'
          % (out['present_lethal_total'], len(present)))
    if out['present_missing']:
        print('  ** 以下障碍在图里没有致命格：%s **' % ', '.join(out['present_missing']))
    print('  幽灵格判据：离任何真实障碍表面 >%.2f m 的致命格 = %d（最远 %s m，bbox %s）'
          % (far['threshold_m'], far['count'], far['max_distance_m'], far['bbox_xy']))

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())

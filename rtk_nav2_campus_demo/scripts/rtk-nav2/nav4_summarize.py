#!/usr/bin/env python3
"""第四步验收：把旁路记录器（`nav4_recorder.py`）的产物汇总成**按判据**的一页数据。

产物口径见 `scripts/rtk-nav2/container/nav4_recorder.py` 的文档串。这里只做汇总与
**独立重算**（路径穿墙核对会从记录器存下的完整点列再算一遍，用来确认记录时的结论不是
一时的读数错误）。

子命令：
  plans     重规划序列：每一次被接受的路径（点数/长度/未知占比/穿墙）+ 与上一条的差异
  rosout    goal_navigator / controller_server 的日志汇总（重规划计数与触发原因、到达事件）
  coverage  全局图覆盖曲线（边走边建）与末态未知占比
  rates     每秒 cmd_vel / 点云 / 全局图计数 —— 判断"车有没有在被控制"、有没有停住
  all       以上全部 + 写一份 summary 文本

用法：
  python3 scripts/rtk-nav2/nav4_summarize.py all artifacts/rtk-nav2/<run>/nav4rec
"""

import argparse
import json
import math
import os
import re
import sys

# 与判据 7 同一套场地几何（独立重算穿墙用）
GEOM_DIR = os.environ.get(
    'NAV4_GEOM_DIR',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                 'rtk_nav2_ws', 'src', 'robonix_rtk_nav2', 'scripts'))
sys.path.insert(0, os.path.abspath(GEOM_DIR))
from map_geometry import segment_hits_obstacle  # noqa: E402

GOAL_A = (-2.6, 2.6)
GOAL_B = (2.6, 2.6)
GOAL_S = (2.6, -2.6)


def load_jsonl(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _dist_to_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def max_lateral_diff(new_pts, old_pts):
    """新路径各点到旧路径折线的最大距离（"路径被改过多少"的度量）。"""
    if not new_pts or len(old_pts) < 2:
        return None
    worst = 0.0
    for p in new_pts:
        d = min(_dist_to_segment(p, a, b) for a, b in zip(old_pts, old_pts[1:]))
        worst = max(worst, d)
    return round(worst, 4)


def which_goal(last_pt):
    best, name = None, None
    for nm, g in (('A(-2.6,2.6)', GOAL_A), ('B(2.6,2.6)', GOAL_B), ('S(2.6,-2.6)', GOAL_S)):
        d = math.hypot(last_pt[0] - g[0], last_pt[1] - g[1])
        if best is None or d < best:
            best, name = d, nm
    return name, round(best, 3)


def cmd_plans(args):
    rows = load_jsonl(os.path.join(args.dir, 'plan_paths.jsonl'))
    seq = []
    prev_pts = None
    prev_len = None
    crossings_total = 0
    crossing_paths = 0
    recheck_mismatch = []
    for r in rows:
        pts = [tuple(p) for p in r['pts']]
        lat = max_lateral_diff(pts, prev_pts) if prev_pts else None
        dlen = round(r['length_m'] - prev_len, 4) if prev_len is not None else None
        # **独立重算**：不信任记录时算的 crossing_count，从点列再算一遍
        recheck = 0
        for a, b in zip(pts, pts[1:]):
            if segment_hits_obstacle(a, b, margin=0.05):
                recheck += 1
        if recheck != r['crossing_count']:
            recheck_mismatch.append({'seq': r['seq'], 'recorded': r['crossing_count'],
                                     'recomputed': recheck})
        crossings_total += recheck
        if recheck:
            crossing_paths += 1
        goal_name, goal_d = which_goal(r['last_pt']) if r.get('last_pt') else (None, None)
        seq.append({'seq': r['seq'], 't_wall': r['t_wall'], 'n_poses': r['n_poses'],
                    'length_m': r['length_m'],
                    'unknown_frac': r['unknown_frac'], 'free_frac': r['free_frac'],
                    'crossings_recomputed': recheck,
                    'max_lateral_vs_prev_m': lat, 'length_delta_vs_prev_m': dlen,
                    'goal_hint': goal_name, 'goal_hint_dist_m': goal_d,
                    'changed_vs_prev': bool(lat is not None and (lat > 0.10
                                                                 or abs(dlen or 0) > 0.10))})
        prev_pts, prev_len = pts, r['length_m']

    first = seq[0] if seq else None
    # 给 B 的**第一条**路径（判据 5）：按终点归属筛
    leg_b = [s for s in seq if s['goal_hint'] == 'B(2.6,2.6)']
    leg_a = [s for s in seq if s['goal_hint'] == 'A(-2.6,2.6)']
    out = {
        'paths_recorded': len(seq),
        'first_path': first,
        'first_path_of_leg_A': leg_a[0] if leg_a else None,
        'first_path_of_leg_B': leg_b[0] if leg_b else None,
        'paths_in_leg_A': len(leg_a),
        'paths_in_leg_B': len(leg_b),
        'any_path_changed_vs_prev': any(s['changed_vs_prev'] for s in seq[1:]),
        'crossing_paths': crossing_paths,
        'crossings_total_recomputed': crossings_total,
        'recheck_mismatch': recheck_mismatch,
        'sequence': seq,
        'note': '判据 2 看 first_path_of_leg_A.unknown_frac（开局路径大部分落在未知里）；'
                '判据 3 看 any_path_changed_vs_prev 与每条的 max_lateral_vs_prev_m；'
                '判据 5 看 first_path_of_leg_B.unknown_frac/free_frac/crossings_recomputed；'
                '判据 7 看 crossing_paths（必须是 0）。crossings 是从记录器存下的'
                '**完整点列**独立重算的（与记录时的数逐条比对，mismatch 为空才采信）。',
    }
    _emit(args, 'plans', out)
    return 0


REPLAN_RE = re.compile(r'重规划 #(\d+)（([^）]+)）@([\d.]+)：横向差 ([\d.]+) m、长度差 ([\d.]+) m')
SUPPRESS_RE = re.compile(r'最小重发间隔\*\*压住：距上次 ([\d.]+) s')
FIRST_RE = re.compile(r'首次规划（([^）]+)）@([\d.]+)：(\d+) 点 / ([\d.]+) m')
ARRIVE_RE = re.compile(r'\*\*到达\*\* 目标 \(([-\d.]+), ([-\d.]+)\)：本段规划 (\d+) 次、'
                       r'重发 (\d+) 次、抖动抑制跳过 (\d+) 次、最小间隔压住 (\d+) 次')
FAIL_RE = re.compile(r'FollowPath 未成功（status=(\d+)')


def cmd_rosout(args):
    rows = load_jsonl(os.path.join(args.dir, 'rosout.jsonl'))
    nav = [r for r in rows if r['node'].endswith('goal_navigator')]
    ctl = [r for r in rows if r['node'].endswith('controller_server')]
    reasons = {}
    replans = []
    suppressed = 0
    arrivals = []
    duplicates = 0
    fails = {}
    empty_paths = 0
    for r in nav:
        m = r['msg']
        mm = REPLAN_RE.search(m)
        if mm:
            replans.append({'n': int(mm.group(1)), 'reason': mm.group(2),
                            't': r['t_wall'], 'lateral_m': float(mm.group(4)),
                            'length_delta_m': float(mm.group(5))})
            reasons[mm.group(2)] = reasons.get(mm.group(2), 0) + 1
        if SUPPRESS_RE.search(m):
            suppressed += 1
        ma = ARRIVE_RE.search(m)
        if ma:
            arrivals.append({'goal': [float(ma.group(1)), float(ma.group(2))],
                             't': r['t_wall'], 'plans': int(ma.group(3)),
                             'resends': int(ma.group(4)),
                             'jitter_skips': int(ma.group(5)),
                             'interval_throttled': int(ma.group(6))})
        if '重复目标' in m:
            duplicates += 1
        mf = FAIL_RE.search(m)
        if mf:
            fails[mf.group(1)] = fails.get(mf.group(1), 0) + 1
        if '规划器返回空路径' in m:
            empty_paths += 1
    ctl_count = {'received_goal': 0, 'canceled': 0, 'succeeded': 0, 'failed_progress': 0}
    for r in ctl:
        m = r['msg']
        if 'Received a goal' in m:
            ctl_count['received_goal'] += 1
        if 'Goal was canceled' in m:
            ctl_count['canceled'] += 1
        if 'Goal succeeded' in m or 'Reached the goal' in m:
            ctl_count['succeeded'] += 1
        if 'Failed to make progress' in m:
            ctl_count['failed_progress'] += 1
    states = [s['state'] for s in load_jsonl(os.path.join(args.dir, 'nav_states.jsonl'))]
    out = {
        'goal_navigator': {
            'first_plans': len([r for r in nav if FIRST_RE.search(r['msg'])]),
            'replans_resend': len(replans),
            'replan_reasons': reasons,
            'replans_throttled_by_min_interval': suppressed,
            'duplicate_goals_ignored': duplicates,
            'arrivals': arrivals,
            'follow_fail_status_counts': fails,
            'empty_path_warnings': empty_paths,
            'replan_timeline': replans,
        },
        'controller_server': ctl_count,
        'navigator_states': states,
        'note': '判据 3 的"重规划计数与触发原因"就是 replan_reasons + replans_resend；'
                'follow_fail_status_counts 里 status=5 是**被自己取消**（重发时先取消），'
                'status=6 才是控制器真的失败。',
    }
    _emit(args, 'rosout', out)
    return 0


def cmd_coverage(args):
    path = os.path.join(args.dir, 'global_map_timeline.csv')
    rows = []
    if os.path.exists(path):
        with open(path, encoding='utf-8') as fh:
            head = fh.readline()
            for line in fh:
                p = line.strip().split(',')
                if len(p) != 7:
                    continue
                rows.append({'frame': int(p[0]), 'stamp_s': float(p[1]),
                             'free': int(p[2]), 'occupied': int(p[3]),
                             'lethal': int(p[4]), 'unknown': int(p[5]),
                             'cells': int(p[6])})
    first = rows[0] if rows else None
    last = rows[-1] if rows else None
    peak_free = max((r['free'] for r in rows), default=0)
    out = {
        'timeline_rows': len(rows),
        'first': first,
        'last': last,
        'peak_free_cells': peak_free,
        'free_growth_cells': (last['free'] - first['free']) if (first and last) else None,
        'unknown_frac_first': (round(first['unknown'] / first['cells'], 4) if first else None),
        'unknown_frac_last': (round(last['unknown'] / last['cells'], 4) if last else None),
        'lethal_growth_cells': (last['lethal'] - first['lethal']) if (first and last) else None,
        'note': '判据 9 的第一半（边走边建）看 free 的增长与末态 unknown 占比：'
                '没去过的地方必须仍然是未知（unknown_frac_last 应当很高）。'
                'CSV 只记"计数发生变化"的帧（源图 2 Hz，全记会有几十万行）。',
    }
    _emit(args, 'coverage', out)
    return 0


def cmd_rates(args):
    path = os.path.join(args.dir, 'rates.csv')
    rows = []
    if os.path.exists(path):
        with open(path, encoding='utf-8') as fh:
            fh.readline()
            for line in fh:
                p = line.strip().split(',')
                if len(p) != 5:
                    continue
                rows.append({'s': int(p[0]), 'cmd_vel': int(p[1]), 'points': int(p[2]),
                             'global_map': int(p[3]), 'paths': int(p[4])})
    stalls = []
    run_start = None
    for r in rows:
        if r['cmd_vel'] == 0:
            if run_start is None:
                run_start = r['s']
        else:
            if run_start is not None and r['s'] - run_start >= 10:
                stalls.append({'from_s': run_start, 'to_s': r['s'] - 1,
                               'duration_s': r['s'] - run_start})
            run_start = None
    if run_start is not None and rows and rows[-1]['s'] - run_start >= 10:
        stalls.append({'from_s': run_start, 'to_s': rows[-1]['s'],
                       'duration_s': rows[-1]['s'] - run_start + 1})
    total = sum(r['cmd_vel'] for r in rows)
    out = {
        'seconds': len(rows),
        'cmd_vel_messages_total': total,
        'cmd_vel_active_seconds': sum(1 for r in rows if r['cmd_vel'] > 0),
        'points_seconds_active': sum(1 for r in rows if r['points'] > 0),
        'global_map_seconds_active': sum(1 for r in rows if r['global_map'] > 0),
        'paths_seconds_active': sum(1 for r in rows if r['paths'] > 0),
        'long_cmd_vel_gaps': stalls,
        'note': 'cmd_vel_active_seconds 明显小于总时长、且有 long_cmd_vel_gaps，'
                '就是"导航循环某段时间完全没有在控制车"。点云列不归零说明相机一直在看'
                '（用来区分"传感器死了"与"控制循环停了"）。',
    }
    _emit(args, 'rates', out)
    return 0


def _emit(args, name, out):
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
    if not args.quiet:
        print(json.dumps(out, ensure_ascii=False, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)
    for name, fn in (('plans', cmd_plans), ('rosout', cmd_rosout),
                     ('coverage', cmd_coverage), ('rates', cmd_rates)):
        p = sub.add_parser(name)
        p.add_argument('dir')
        p.add_argument('--json', default='')
        p.add_argument('--quiet', action='store_true')
        p.set_defaults(func=fn)

    p = sub.add_parser('all')
    p.add_argument('dir')
    p.add_argument('--out-dir', default='')
    args = parser.parse_args(argv)

    if args.cmd == 'all':
        out_dir = args.out_dir or args.dir
        for name, fn in (('plans', cmd_plans), ('rosout', cmd_rosout),
                         ('coverage', cmd_coverage), ('rates', cmd_rates)):
            ns = argparse.Namespace(dir=args.dir,
                                    json=os.path.join(out_dir, 'summary_%s.json' % name),
                                    quiet=True)
            fn(ns)
        print(json.dumps({'written': [os.path.join(out_dir, 'summary_%s.json' % n)
                                      for n in ('plans', 'rosout', 'coverage', 'rates')]},
                         ensure_ascii=False, indent=2))
        return 0
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())

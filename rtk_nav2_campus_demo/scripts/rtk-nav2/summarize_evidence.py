#!/usr/bin/env python3
"""把一次运行的产物汇总成"按判据"的一张表（宿主侧脚本，不连 ROS）。

为什么要它：产物散在 nav2/、coverage/、spin/、behavior/、perf/ 五个目录里十几个文件，
复验的人要一项项对判据翻。这个脚本把 brief §5 的八条判据各自的关键数字读出来，
**判据通过与否只按数据说**，缺产物就明确写"缺产物"，不猜。

用法：summarize_evidence.py <run-id 目录> [--json 汇总.json]
"""

import argparse
import csv
import json
import os
import sys


def load(path):
    if not os.path.exists(path):
        return None
    try:
        with open(path, encoding='utf-8') as handle:
            return json.load(handle)
    except Exception as exc:
        return {'_load_error': str(exc)}


def load_first(*paths):
    """按候选顺序取第一个存在的（同一份证据在不同 run 里文件名可能不同）。"""
    for path in paths:
        got = load(path)
        if got is not None:
            return got, path
    return None, None


def load_state(path):
    out = {}
    if not os.path.exists(path):
        return out
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            line = line.split('#')[0].strip()
            if '=' in line:
                key, _, value = line.partition('=')
                out[key.strip()] = value.strip()
    return out


def coverage_curve(path):
    if not os.path.exists(path):
        return None
    rows = []
    with open(path, encoding='utf-8') as handle:
        for row in csv.DictReader(handle):
            rows.append(row)
    if not rows:
        return None
    first, last = rows[0], rows[-1]
    cells = int(last['cells'])
    return {
        'samples': len(rows),
        'span_s': [first['t_s'], last['t_s']],
        'sim_span_s': [first['stamp_s'], last['stamp_s']],
        'free': [int(first['free']), int(last['free'])],
        'occupied': [int(first['occupied']), int(last['occupied'])],
        'unknown_fraction_first': round(int(first['unknown']) / cells, 4),
        'unknown_fraction_last': round(int(last['unknown']) / cells, 4),
        'free_area_m2': [round(int(first['free']) * 0.0025, 2),
                         round(int(last['free']) * 0.0025, 2)],
        'occupied_area_m2': [round(int(first['occupied']) * 0.0025, 2),
                             round(int(last['occupied']) * 0.0025, 2)],
    }


def route_summary(path):
    d = load(path)
    if not d or '_load_error' in d:
        return None
    return {
        'exit_code': d.get('exit_code'),
        'abort_reason': d.get('abort_reason'),
        'reached': len(d.get('reached') or []),
        'route_points': len(d.get('route') or []),
        'loop_count': d.get('loop_count'),
        'stop_events': d.get('stop_events'),
        'slow_events': d.get('slow_events'),
        'mission_window_s': [d.get('mission_start_sim_s'), d.get('mission_end_sim_s')],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir')
    parser.add_argument('--json', default='')
    args = parser.parse_args(argv)
    root = args.run_dir

    state = load_state(os.path.join(root, 'run_state.txt'))
    out = {'run_dir': os.path.abspath(root), 'run_state': state, 'criteria': {}}

    # ---------------- 判据 1：地图正确性 ----------------
    # 几何核对的文件名在不同 run 里不同（evidence-run 用 _after_lap，早期单圈用手工名），
    # 按候选顺序取，并把**实际用的那个文件**写进结论（免得复验的人找不到）。
    g, g_path = load_first(os.path.join(root, 'nav2', 'geometry_after_lap.json'),
                           os.path.join(root, 'nav2', 'geometry_lethal.json'),
                           os.path.join(root, 'nav2', 'geometry.json'))
    c1 = {'evidence': g_path or 'nav2/geometry_after_lap.json'}
    if g and '_load_error' not in g:
        dist = g.get('distance_to_nearest_real_surface_m', {})
        # **键名 2026-09-21 改了**：nav2_probe.py 的 `surface_samples_per_wall`
        # 在四周围墙删掉之后名不副实（值本来就不含外墙了），改叫
        # `surface_samples_per_obstacle`。老产物 JSON 里还是旧键名，**两个都读**，
        # 否则复验旧 run 会静默拿到空 dict（near_n = 0，判据直接变成"需人工看"）。
        samples = g.get('surface_samples_per_obstacle',
                        g.get('surface_samples_per_wall', {}))
        near = {w: v['route_near'] for w, v in samples.items()
                if v['route_near']['n'] > 0}
        near_free = sum(v['free'] for v in near.values())
        near_n = sum(v['n'] for v in near.values())
        c1.update({
            'meta': g.get('meta'),
            'counts': g.get('counts'),
            'lethal_cells': g.get('lethal_cells'),
            'lethal_within_tolerance_ratio': dist.get('within_ratio'),
            'lethal_distance_mean_m': dist.get('mean'),
            'lethal_distance_max_m': dist.get('max'),
            'route_near_obstacles': near,
            'route_near_free_ratio': (round(near_free / near_n, 4) if near_n else None),
            'verdict': ('通过' if (dist.get('within_ratio') == 1.0 and near_n and
                                   near_free / near_n < 0.02)
                        else '需人工看'),
            'note': '占据格用 **lethal（代价 == 100）** 算，不是 >=50（后者含膨胀带，'
                    '会把"障碍位置"虚报大约一圈）。近墙回归：路线 2 m 内的墙表面上'
                    '被写成空闲的比例必须接近 0（第二阶段 §0d 那个 bug 的判据）',
        })
    else:
        c1['verdict'] = '缺产物'
    out['criteria']['1_地图正确性'] = c1

    # ---------------- 判据 2：清除语义 ----------------
    infl = load_state(os.path.join(root, 'run_state.txt')).get('injection', '')
    c2 = {'evidence': 'nav2/obstacle_present_stats.json + nav2/geometry_after_removal_lap.json'}
    present = load(os.path.join(root, 'nav2', 'obstacle_present_stats.json'))
    after = load(os.path.join(root, 'nav2', 'geometry_after_removal_lap.json'))
    if present and after and '_load_error' not in present and '_load_error' not in after:
        c2.update({
            'lethal_with_obstacle': present.get('global', {}).get('counts', {}).get('lethal'),
            'lethal_after_removal_lap': after.get('counts', {}).get('lethal'),
            'unknown_with_obstacle': present.get('global', {}).get('counts', {}).get('unknown'),
            'unknown_after_removal_lap': after.get('counts', {}).get('unknown'),
            'injection_marked': bool(infl),
            'note': '判据是"移走障碍后再经过 → 对应格被清掉"；'
                    '两个 lethal 计数是**整张 40x40 m 图**的总数，不是障碍那几格——'
                    '精确结论要看 behavior/obstacle/ 下的前后快照 npz 与逐格 diff',
        })
    else:
        c2['verdict'] = '缺产物'
    out['criteria']['2_清除语义'] = c2

    # ---------------- 判据 3：边跑边建 ----------------
    cov = coverage_curve(os.path.join(root, 'coverage', 'coverage_lap.csv'))
    c3 = {'evidence': 'coverage/coverage_lap.csv'}
    if cov:
        c3.update(cov)
        c3['verdict'] = ('通过' if (cov['free'][1] > cov['free'][0] * 2
                                    and cov['unknown_fraction_last'] > 0.5)
                         else '需人工看')
        c3['note'] = 'free 面积随时间增长 = 边跑边建；最后一帧 unknown 占比仍 > 50% '
        c3['note'] += '= 没去过的区域没被写成空闲'
    else:
        c3['verdict'] = '缺产物'
    out['criteria']['3_边跑边建'] = c3

    # ---------------- 判据 4：规划可用 ----------------
    c4 = {}
    # 文件名按 plan-checks.sh 的产出（plan_before_lap / plan_after_lap），
    # 也兼容早期 evidence-run 的 plan_unknown_goal。
    for tag, f in (('next_waypoint', 'plan_next_waypoint.json'),
                   ('unknown_goal_before_lap', 'plan_before_lap.json'),
                   ('unknown_goal_after_lap', 'plan_after_lap.json'),
                   ('unknown_goal', 'plan_unknown_goal.json')):
        p = load(os.path.join(root, 'nav2', f))
        if p and '_load_error' not in p:
            c4[tag] = {
                'planned': p.get('planned'), 'error': p.get('error'),
                'poses': p.get('poses'), 'length_m': p.get('path_length_m'),
                'crosses_wall': p.get('crosses_wall'),
                'crossings': p.get('crossings'),
                'cell_class_on_path': p.get('cell_class_on_path'),
                'start_cell_class': p.get('start_cell_class'),
                'goal_cell_class': p.get('goal_cell_class'),
            }
        else:
            c4[tag] = '缺产物'
    cmp = load(os.path.join(root, 'nav2', 'plan_map_update_comparison.json'))
    if cmp and '_load_error' not in cmp:
        c4['map_update_changed_path'] = cmp.get('path_changed')
        c4['map_update_detail'] = {'before': cmp.get('before'), 'after': cmp.get('after')}
    c4['evidence'] = 'nav2/plan_*.json + plan_map_update_comparison.json'
    out['criteria']['4_规划可用'] = c4

    # ---------------- 判据 5：存读 ----------------
    rt = load(os.path.join(root, 'nav2', 'map_roundtrip.json'))
    sl = load(os.path.join(root, 'nav2', 'save_load_check.json'))
    conv196 = load(os.path.join(root, 'nav2', 'pgm_convention_free0196.json'))
    conv025 = load(os.path.join(root, 'nav2', 'pgm_convention_free025.json'))
    c5 = {'evidence': 'nav2/map_roundtrip.json + save_load_check.json + pgm_convention_*.json'}
    if rt and '_load_error' not in rt:
        c5.update({'roundtrip': rt.get('roundtrip'), 'loaded': rt.get('loaded', {}).get('counts'),
                   'file_decoded': rt.get('file_decoded'),
                   'vs_live': rt.get('vs_live'),
                   'verdict': ('通过' if all((rt.get('roundtrip') or {}).get(k)
                                             for k in ('geometry_match', 'origin_match',
                                                       'counts_match')) else '需人工看')})
    else:
        c5['verdict'] = '缺产物'
    if sl and '_load_error' not in sl:
        c5['pure_file_readback'] = {k: v for k, v in (sl.get('vs_live') or {}).items()
                                    if isinstance(v, bool)}
        c5['pixel_histogram'] = sl.get('pixel_histogram')
    c5['pgm_convention'] = {
        'free_0.196': (conv196 or {}).get('mapping'),
        'free_0.25': (conv025 or {}).get('mapping'),
        'note': 'map_saver 写未知用像素 205；配它默认的 free_thresh 0.25，'
                'nav2 读回会判成**空闲**（未知丢失）。存图时显式 --free 0.196 才对得上',
    }
    out['criteria']['5_存读'] = c5

    # ---------------- 判据 6：不回归 ----------------
    c6 = {'evidence': 'route_report.json / behavior/ / nav2/trajectory_report_lap2.json'}
    c6['lap1'] = route_summary(os.path.join(root, 'lap1_report.json'))
    c6['lap2_after_removal'] = route_summary(os.path.join(root, 'lap2_after_removal_report.json'))
    c6['trajectory_lap2'] = load(os.path.join(root, 'nav2', 'trajectory_report_lap2.json'))
    obs_exit = os.path.join(root, 'behavior', 'obstacle', 'exit_code.txt')
    if os.path.exists(obs_exit):
        c6['obstacle_exit_code'] = open(obs_exit).read().strip()
    c6['stop_distance'] = load(os.path.join(root, 'behavior', 'obstacle', 'stop_distance.json'))
    cam_exit = os.path.join(root, 'behavior', 'camera_lost', 'exit_code.txt')
    if os.path.exists(cam_exit):
        c6['camera_lost_exit_code'] = open(cam_exit).read().strip()
    c6['odom_deviation'] = load(os.path.join(root, 'behavior', 'odom_deviation.json'))
    c6['note'] = ('退出码约定（第一阶段原样沿用）：0 正常 / 3 感知失效 / 5 前方障碍停车 / '
                  '6 超时 / 7 定位失效')
    out['criteria']['6_不回归'] = c6

    # ---------------- 判据 7：性能 ----------------
    perf = load(os.path.join(root, 'perf', 'perf.json'))
    c7 = {'evidence': 'perf/perf.json'}
    if perf and '_load_error' not in perf:
        c7.update({'rates': perf.get('rates'),
                   'container_cpu_total_pct': perf.get('container_cpu_total_pct'),
                   'container_cpu_top': (perf.get('container_cpu_top') or [])[:8],
                   'host_loadavg': perf.get('host_loadavg_1_5_15')})
    else:
        c7['verdict'] = '缺产物'
    out['criteria']['7_性能'] = c7

    # ---------------- 判据 8：局部图旋转帧 ----------------
    spin = load(os.path.join(root, 'spin', 'spin_summary.json'))
    c8 = {'evidence': 'spin/spin_summary.json + spin_samples.csv'}
    if spin and '_load_error' not in spin:
        c8.update(spin)
    else:
        c8['verdict'] = '缺产物'
    out['criteria']['8_局部图旋转帧'] = c8

    # ---------------- 控制权归属 ----------------
    cvp = os.path.join(root, 'nav2', 'cmd_vel_publishers_final.txt')
    if os.path.exists(cvp):
        text = open(cvp, encoding='utf-8', errors='replace').read()
        pubs = [line.split(':', 1)[1].strip() for line in text.splitlines()
                if 'Node name' in line]
        out['cmd_vel_publishers'] = sorted(set(pubs))

    text = json.dumps(out, ensure_ascii=False, indent=2)
    print(text)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            handle.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""rev8_beats_in_tolerance.py —— 统计**每个点"在容差内待了几拍"** ✓（第 2 步的判据量）

用法：
    rev8_beats_in_tolerance.py <run目录>              # 单轮
    rev8_beats_in_tolerance.py <runA> --pair <runB>   # 成对（旧行为 vs 新行为 ✓）

数据来源（**都是驱动自己落的产物** ✓，不重跑仿真 ✓）：
    <run>/geo_route_records.json    —— 点位清单、tol/v_still（口径值 ✓）
    <run>/geo_route_point_<名字>.csv —— 逐样本：t,map_x,map_y,gt_x,gt_y,d_gt,d_map

★ 判据量 = `d_gt < tol` 的样本条数（"进容差之后还在采样多少拍"）✓ ——
  改动前每个点要**等真停稳**才走 ⇒ 这个数很大（实测 `wp2` = **108 拍** ✗）；
  改动后**进容差即发下一点** ⇒ 这个数应当掉到 **1~2 拍** ✓。
★ 阈值（tol/v_still/quiet_s）**从产物里读**，✗ 不在这里写死 ✓（口径变了这里要跟着变 ✓）。
退出码：0 = 全部点都读到 ｜ 1 = 有点的逐样本 CSV 缺失 ｜ 2 = 用法/读文件错
"""
import csv
import json
import os
import sys


def load(run_dir):
    rec_path = os.path.join(run_dir, 'geo_route_records.json')
    if not os.path.isfile(rec_path):
        raise SystemExit('✗ 找不到 %s ⇒ 这一轮没开过车' % rec_path)
    with open(rec_path, encoding='utf-8') as fh:
        rec = json.load(fh)
    tol = rec.get('tol')
    rows = []
    missing = []
    for p in rec.get('points', []):
        name = p.get('name')
        csv_path = os.path.join(run_dir, 'geo_route_point_%s.csv' % name)
        if not os.path.isfile(csv_path):
            missing.append(name)
            continue
        d = []
        with open(csv_path, encoding='utf-8') as fh:
            for r in csv.DictReader(fh):
                d.append((float(r['t']), float(r['d_gt'])))
        beats = [x for x in d if x[1] < tol]
        rows.append({
            'name': name,
            'samples': len(d),
            'beats_in_tol': len(beats),
            'first_t': (d[0][0] if d else None),
            'first_beat_t': (beats[0][0] if beats else None),
            'first_beat_d': (beats[0][1] if beats else None),
            'last_t': (d[-1][0] if d else None),
            'last_d': (d[-1][1] if d else None),
            'tolerance_entered': bool(p.get('tolerance_entered')),
            'arrived_strict': bool(p.get('arrived')),
        })
    return {'tol': tol, 'v_still': rec.get('v_still'), 'points': rows, 'missing': missing,
            'route': rec.get('route'), 'calib_key': str(rec.get('calib_key'))[:16]}


def show(run_dir, info, title):
    print('=== %s ===' % title)
    print('  目录: %s' % run_dir)
    print('  口径: tol=%s m  v_still=%s m/s' % (info['tol'], info['v_still']))
    print('  航线: %s' % info['route'])
    print('  %-8s %8s %12s %12s %10s   %s' %
          ('点', '样本数', '容差内拍数', '首次进容差', '末拍 d_gt', '进过容差/严判到达'))
    tot = 0
    for r in info['points']:
        tot += r['beats_in_tol']
        fb = '%.2fs' % r['first_beat_t'] if r['first_beat_t'] is not None else '—'
        print('  %-8s %8d %12d %12s %10s   %s / %s' %
              (r['name'], r['samples'], r['beats_in_tol'], fb,
               '%.4f' % r['last_d'] if r['last_d'] is not None else '—',
               '✓' if r['tolerance_entered'] else '✗',
               '✓' if r['arrived_strict'] else '✗'))
    print('  ----')
    print('  样本总数 = %d ｜ **容差内拍数合计 = %d** ｜ 最大单点 = %d' %
          (sum(r['samples'] for r in info['points']), tot,
           max([r['beats_in_tol'] for r in info['points']] or [0])))
    entered = sum(1 for r in info['points'] if r['tolerance_entered'])
    print('  进过容差的点 = %d / %d（★ 新口径的"到达"判据 ✓）' % (entered, len(info['points'])))
    # ★ 全程耗时（★ reviewer 要的那一维 ✓）：首个点的首拍 → 末点的末拍
    ft = [r['first_t'] for r in info['points'] if r['first_t'] is not None]
    lt = [r['last_t'] for r in info['points'] if r['last_t'] is not None]
    if ft and lt:
        info['total_s'] = max(lt) - min(ft)
        print('  ★ **全程耗时 = %.1f s**（仿真时间；%.1f → %.1f）' % (info['total_s'], min(ft), max(lt)))
    else:
        info['total_s'] = None
    if info['missing']:
        print('  ✗ 缺逐样本 CSV 的点: %s' % '、'.join(info['missing']))
    return tot


def main():
    args = [a for a in sys.argv[1:]]
    if not args or args[0] in ('-h', '--help'):
        print(__doc__)
        return 2
    a_dir = args[0]
    b_dir = None
    if '--pair' in args:
        i = args.index('--pair')
        if i + 1 >= len(args):
            print('✗ --pair 需要目录')
            return 2
        b_dir = args[i + 1]

    ia = load(a_dir)
    ta = show(a_dir, ia, 'A')

    if b_dir:
        ib = load(b_dir)
        tb = show(b_dir, ib, 'B')
        print()
        print('=== 成对比较（★ 同一台机、同一条航线 ✓）===')
        print('  %-8s %14s %14s %10s' % ('点', 'A 容差内拍数', 'B 容差内拍数', '变化'))
        ma = {r['name']: r for r in ia['points']}
        for r in ib['points']:
            x = ma.get(r['name'])
            if not x:
                continue
            d = r['beats_in_tol'] - x['beats_in_tol']
            print('  %-8s %14d %14d %10s' % (r['name'], x['beats_in_tol'], r['beats_in_tol'],
                                             ('%+d' % d) if d else '0'))
        print('  ----')
        print('  合计: A=%d 拍 ｜ B=%d 拍 ｜ 变化 %+d' % (ta, tb, tb - ta))
        if ia.get('total_s') is not None and ib.get('total_s') is not None:
            print('  ★ 全程耗时: A=%.1f s ｜ B=%.1f s ｜ 变化 %+.1f s'
                  % (ia['total_s'], ib['total_s'], ib['total_s'] - ia['total_s']))
        na = sum(1 for r in ia['points'] if r['tolerance_entered'])
        nb = sum(1 for r in ib['points'] if r['tolerance_entered'])
        print('  ★ 进过容差的点: A=%d/%d ｜ B=%d/%d（★ 这一维两轮应当**相同** ✓）'
              % (na, len(ia['points']), nb, len(ib['points'])))

    return 1 if (ia['missing'] or (b_dir and load(b_dir)['missing'])) else 0


if __name__ == '__main__':
    sys.exit(main())

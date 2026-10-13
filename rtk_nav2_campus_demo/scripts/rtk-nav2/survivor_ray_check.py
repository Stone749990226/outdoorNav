#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""survivor_ray_check.py —— ★ 把"那些格不再落在任何清除射线上"这句**断言**变成**量** ✓

用法: python3 survivor_ray_check.py --out <json> [--rounds r1,r2]

★★ 要回答什么（★ reviewer 2026-10-11 派；★ 源头是业主给的机制 ✓）★★
  ★ 业主说 ✓：★ **大部分点在姿态恢复后被清掉** ✓；★ 残留的是 ★ **"小车一边移动、离开激光雷达范围的点"** ✓
  ★ reviewer 核了记录 ✓：★ 幸存格到**轨迹**的距离 ★ p10 **0.64** ／ 中位 **2.81** ／ p75 5.60 ／ max **7.75 m** ✓
    ⇒ ★ 中位 2.81 m **远在 8 m 量程之内** ✗ ⇒ ★ **"离开量程"只解释了最远那批** ✓
  ⇒ ★★ 而记录里那句"**不再落在任何清除射线上**" ✓ **是断言、✗ 没测过** ✗ ⇒ ★ 本工具直接测它 ✓✓

★★ 判据（★ 两头都算结论 ✓）★★
  ★ 对每个幸存格问一句 ✓：★ **它被（第一次）标上之后，有没有哪一拍的某个回波，
    落在它后面（★ 同一方位 ✓、★ 更远 ✓、★ 在量程内 ✓）？**
    ★ **没有** ⇒ ★ **没有任何后来的射线能穿过它** ✓ ⇒ ★★ **业主那条机制成立** ✓✓（★ 量程只是它的一个特例 ✓）
    ★ **有** ⇒ ★ 那条射线本该清它、★ 它却活着 ✗ ⇒ ★ **机制不充分** ✗ ⇒ ★ 另有东西护着它 ✓
      ★ 且 ★ **如实报** ✓：★ 这批的**共同点**（★ 离轨迹多远 ✓／★ 在不在曲面上 ✓）

★★ 口径（★ 写死 ✓）★★
  · ★ 幸存格 = ★ **到达拍那张图里致命 ∧ 第一张图里不致命** ✓（★ 与 `dynamic_lethal_delta.py` 同一套 ✓）
  · ★ **第一次被标上** = ★ 逐张 dump 往前扫、★ 第一次致命的那一拍 ✓
    （★ ✗ 不用末拍归因 ✗ —— ★ "归因取该格第一次被标上那一刻" ✓ 本项目有档 ✓）
  · ★ 云 = `scan_<seq>.xyz` ✓ —— ★ **实测在 map 帧** ✓（★ 该拍 360 点均值 ≈ 车位姿 ✓，✗ 不是传感器系 ✗）
  · ★ 传感器原点 = ★ 该拍 `scan.tf.M` 的**平移** ✓（★ 实测 = 车 xy ＋ z **0.1** ✓）
  · ★ 方位容差 **1.0°** ✓（★ 360 束/360° ⇒ 一束就是一档 ✓）；★ 距离上限 ★ **8.0 m** ✓
    （★ = `raytrace_max_range` ✓ —— ★ **清除**吃的是它 ✓，✗ 不是 `obstacle_max_range` 7.5 ✗）
  · ★ 抽样 ✓：★ 每轮**最多 200 格** ✓（★ 等距抽 ✓）＋ ★ 第一次标上之后**每 20 拍**取一拍 ✓
"""
import argparse
import glob
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ART = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'
SCENE = '/home/xjy/projects/outdoorNav/gazebo_campus/generated/scene_geometry.json'
sys.path.insert(0, HERE)
from scene_obstacles import load_obstacles, dist_to_obstacle      # noqa: E402

LETHAL_LO, LETHAL_HI = 99, 127
ANG_TOL_DEG = 1.0
RANGE_CAP = 8.0
FILL_MIN = 7.99   # ★ 无回波被填成远裁剪面 **8.0** ✓（★ 本项目已定死的 artifact ✓）⇒ ★ ≥7.99 记成**填充** ✓
MOVE_EPS = 0.05
SAMPLE_MAX = 200
TICK_STRIDE = 20
ON_OBST_M = 0.30


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def read(p):
    return open(p, 'rb').read()


def poly_dist(px, py, poly):
    best = float('inf')
    for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
        best = min(best, math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--rounds', default='campus9-res1,campus9-cutS9')
    ap.add_argument('--nominal-from', default='campus9-res1')
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    # ★ 轨迹折线（★ 用探针逐拍位姿 ✓，与"到轨迹距离"那条代理量同源 ✓）
    obs = load_obstacles(SCENE)
    out = {'what': '★ 幸存格"**标上之后还有没有射线能穿过它**"的直接检验 ✓（★ 纯离线 ✓）',
           'preregistered': {
               'no_later_ray': '★★ **业主那条机制成立** ✓✓（★ "离开量程"只是它的一个特例 ✓）',
               'ray_existed_but_survived': '★ **机制不充分** ✗ ⇒ ★ 另有东西护着它 ✓（★ 并报这批判的共同点 ✓）'},
           'convention': {'lethal': '[%d, %d] ✓' % (LETHAL_LO, LETHAL_HI),
                          'cloud_frame': '★ `scan_*.xyz` **实测在 map 帧** ✓（★ 该拍 360 点均值 ≈ 车位姿 ✓）',
                          'sensor_origin': '★ 该拍 `scan.tf.M` 的平移 ✓（★ 实测 = 车 xy ＋ z 0.1 ✓）',
                          'ang_tol_deg': ANG_TOL_DEG, 'range_cap_m': RANGE_CAP,
                          'range_cap_why': '★ = `raytrace_max_range` 8.0 ✓ —— ★ **清除**吃的是它 ✓，✗ 不是 7.5 ✗',
                          'sampling': '★ 每轮最多 %d 格 ✓ ＋ ★ 第一次标上之后每 %d 拍取一拍 ✓' % (SAMPLE_MAX, TICK_STRIDE),
                          'first_mark': '★ 逐张 dump 往前扫、第一次致命的那一拍 ✓（★ ✗ 不用末拍归因 ✗）'},
           'rounds': {}, 'notes': []}

    for run in [r for r in a.rounds.split(',') if r]:
        ds = sorted(glob.glob(os.path.join(ART, run, 'probe_frozen_wp8_arrival_*')))
        if not ds:
            out['notes'].append('✗ %s：没有到达锚的冻结目录' % run)
            continue
        d = ds[-1]
        maps = sorted(glob.glob(os.path.join(d, 'map_*.bin')), key=seq_of)
        if len(maps) < 3:
            out['notes'].append('✗ %s：图太少' % run)
            continue
        cur_end = read(maps[-1]); cur_start = read(maps[0])
        surv = [i for i in range(len(cur_end))
                if (LETHAL_LO <= cur_end[i] <= LETHAL_HI) and not (LETHAL_LO <= cur_start[i] <= LETHAL_HI)]
        step = max(1, len(surv) // SAMPLE_MAX)
        sample = surv[::step][:SAMPLE_MAX]

        # ★ 每个被抽到的幸存格的"第一次被标上"那一拍 ✓ ＋ ★ **逐拍致命时间线** ✓
        #   ★ 时间线在同一遍里记 ✓（✗ 别为了一句新问题把 266 MB 再读一遍 ✗）
        first = {}
        timeline = {i: [] for i in sample}          # ★ [(seq, 是否致命), ...] ✓
        remaining = set(sample)
        for m in maps[1:]:
            buf = read(m); s = seq_of(m); done = []
            for i in sample:
                timeline[i].append((s, LETHAL_LO <= buf[i] <= LETHAL_HI))
            if not remaining:
                continue
            for i in remaining:
                if LETHAL_LO <= buf[i] <= LETHAL_HI:
                    first[i] = s; done.append(i)
            for i in done:
                remaining.discard(i)
        meta = json.load(open(os.path.join(d, 'tick_%04d.json' % seq_of(maps[1])), encoding='utf-8'))
        res_ = meta['map']['resolution']; W = meta['map']['width']; H = meta['map']['height']
        ox = meta['map']['origin']['x']; oy = meta['map']['origin']['y']

        # ★ 轨迹折线（★ 探针逐拍位姿 ✓）
        traj = []
        for f in sorted(glob.glob(os.path.join(d, 'tick_*.json')), key=seq_of):
            r = json.load(open(f, encoding='utf-8')).get('robot_xy_map')
            if r and len(r) >= 2:
                traj.append((float(r[0]), float(r[1])))

        avail = {seq_of(p) for p in glob.glob(os.path.join(d, 'scan_*.xyz'))}
        rows = []
        for i in sample:
            fm = first.get(i)
            if fm is None:
                continue
            ix, iy = i % W, i // W
            cx = ox + (ix + 0.5) * res_; cy = oy + (iy + 0.5) * res_
            hit = None
            for m in maps:
                s = seq_of(m)
                if s <= fm or s not in avail or (s - fm) % TICK_STRIDE:
                    continue
                try:
                    M = json.load(open(os.path.join(d, 'tick_%04d.json' % s), encoding='utf-8'))['scan']['tf']['M']
                except Exception:
                    continue
                sx, sy = M[0][3], M[1][3]
                dc = math.hypot(cx - sx, cy - sy)
                if dc <= 1e-9 or dc > RANGE_CAP:
                    continue
                bc = math.atan2(cy - sy, cx - sx)
                for line in open(os.path.join(d, 'scan_%04d.xyz' % s), encoding='utf-8'):
                    v = line.split()
                    if len(v) != 3:
                        continue
                    px, py = float(v[0]), float(v[1])
                    dp = math.hypot(px - sx, py - sy)
                    if dp <= dc + MOVE_EPS or dp > RANGE_CAP:
                        continue
                    dth = abs((math.atan2(py - sy, px - sx) - bc + math.pi) % (2 * math.pi) - math.pi)
                    if math.degrees(dth) <= ANG_TOL_DEG:
                        hit = {'tick': s, 'ray_len_m': round(dp, 3), 'd_cell_m': round(dc, 3),
                               'is_fill': bool(dp >= FILL_MIN)}
                        break
                if hit:
                    break
            # ★★ 射线之后：★ 有没有先被清掉、又被重新标上？ ✓（★ 用上面那条时间线 ✓）★★
            re_mark = None
            if hit:
                tl = [x for x in timeline[i] if x[0] > hit['tick']]
                cleared_at = next((sq for sq, l in tl if not l), None)
                if cleared_at is not None:
                    remark_at = next((sq for sq, l in tl if l and sq > cleared_at), None)
                    re_mark = {'cleared_at_tick': cleared_at, 're_marked_at_tick': remark_at,
                               'story': ('★ **清了又被重新标上** ✓（★ 另一个故事 ✗ —— ★ 与"没人清"不同 ✓）'
                                         if remark_at else '★ 被清之后**没再被标上** ✗（★ 但它现在是致命的 ⇒ 异常 ✓）')}
                else:
                    re_mark = {'cleared_at_tick': None, 're_marked_at_tick': None,
                               'story': ('★ **射线之后从未被清掉** ✗ ⇒ ★ 那条射线**没起作用** ✓ '
                                         '⇒ ★ 要看是谁在同期又标了它 ✓（★ 下面查源 ✓）')}
            rows.append({'cell': i, 'xy': [round(cx, 3), round(cy, 3)], 'first_mark_tick': fm,
                         'd_to_traj_m': round(poly_dist(cx, cy, traj), 3),
                         'd_to_obstacle_m': round(dist_to_obstacle(cx, cy, obs), 3),
                         'later_ray': hit, 'after_ray': re_mark,
                         'who_now': None})
        # ★★ "谁在标它" ✓：★ 那些格**第一次被标上**那一拍，`scan` 与 `cam` 各自有没有点落进这一格 ✓
        def who_marks(r, tick=None):
            # ★ "谁在标它" ✓ —— ★ 默认问**第一次被标上**那一拍 ✓；
            # ★ 而 ★ 对"清了又被重新标上"那批 ✓ ⇒ ★ 要问的是**重新标上**那一拍 ✓（★ reviewer 的措辞 ✓）
            s0 = tick if tick is not None else r['first_mark_tick']
            out_ = {}
            zs = []
            # ★★ 窗口要放宽 ✓ ★★
            #   ★ 实测 ✗：★ 只查**那一拍**、★ 半径 0.1 m ⇒ ★ 26/28 报"都没有" ✗
            #     —— ★ 那不一定是"没有源" ✗，★ 多半是**窗口太窄** ✗（★ 标记是逐拍连续写进去的 ✓，
            #     ★ 而且点落在**格心 0.1 m 外**也算压到这一格 ✓）
            #   ⇒ ★ 取 **±2 拍 ＋ 半径 0.15 m** ✓；★ 仍取不到 ⇒ ★ 记成**未取到** ✗（✗ 不落成"没有" ✗）。
            for src in ('scan', 'cam'):
                n = 0
                seen = False
                # ★ ① ★ 窗口再放宽一档 ✓：★ ±2 拍报"未取到"的有 11／8 格 ✗
                #   ⇒ ★ 逐拍连续写 ⇒ ★ ±2 可能还是窄 ✓ ⇒ ★ 取 **±4 拍** ✓（★ 仍取不到 ⇒ ★ 如实留"未取到" ✓）
                for ds_ in (-4, -3, -2, -1, 0, 1, 2, 3, 4):
                    fp = os.path.join(d, '%s_%04d.xyz' % (src, s0 + ds_))
                    if not os.path.isfile(fp):
                        continue
                    seen = True
                    for line in open(fp, encoding='utf-8'):
                        v = line.split()
                        if len(v) != 3:
                            continue
                        if math.hypot(float(v[0]) - r['xy'][0], float(v[1]) - r['xy'][1]) <= 0.15:
                            n += 1
                            zs.append(float(v[2]))
                out_[src] = n if seen else None
            # ★ ② ★ 这些点的 **z** ✓ —— ★ 用来看"清除有没有高度带" ✓
            out_['z_min'] = round(min(zs), 3) if zs else None
            out_['z_max'] = round(max(zs), 3) if zs else None
            return out_

        no_ray = [r for r in rows if not r['later_ray']]
        has_ray = [r for r in rows if r['later_ray']]
        # ★★ 区分"**真回波**在更远处"与"**填充点**在 8.0" ✓（★ 后者是 artifact ✓）★★
        has_real = [r for r in has_ray if not r['later_ray']['is_fill']]
        has_fill_only = [r for r in has_ray if r['later_ray']['is_fill']]

        def pct(xs, p):
            if not xs:
                return None
            s = sorted(xs)
            return round(s[min(len(s) - 1, int(round(p * (len(s) - 1))))], 3)

        out['rounds'][run] = {
            'frozen_dir': os.path.relpath(d, ART), 'n_survivors': len(surv), 'n_sampled': len(rows),
            'n_no_later_ray': len(no_ray), 'n_ray_existed_but_survived': len(has_ray),
            'n_ray_REAL_return_beyond': len(has_real),
            'n_ray_only_FILL_at_8m': len(has_fill_only),
            'fill_note': ('★ `scan` 云里**无回波**的像素被填成远裁剪面 **8.0 m** ✓（★ 本项目已定死的 artifact ✓）'
                          '⇒ ★ 只看**真回波**（★ < 7.99 ✓）那一栏才是"**有真的射线穿过它**" ✓；'
                          '★ 而"只被填充点挡着"那一栏：★ 填充点**在 nav2 里也算一条射线的端点** ✓'
                          '（★ 它会清到 7.9999 ✓）⇒ ★ 两栏分开报 ✓，✗ 不混 ✗'),
            'never_cleared_group': {
                'note': ('★ "真有射线却从未被清"那批 ✓：★ 查两句 —— ★ **① 它们的 z** ✓；'
                         '★ **② 射线终点离它多近** ✓（★ 终点若就贴在格上 ⇒ ★ 它是**端点格** ⇒ '
                         '★ nav2 的 raytrace 清的是**端点之前**的格 ⇒ ★ 端点格**不被清** ✓）'),
                'n': len([r for r in has_real if r['after_ray'] and not r['after_ray']['cleared_at_tick']]),
                'margin_m': sorted(round(r['later_ray']['ray_len_m'] - r['later_ray']['d_cell_m'], 3)
                                   for r in has_real
                                   if r['after_ray'] and not r['after_ray']['cleared_at_tick']),
                'n_margin_le_0.3m': sum(1 for r in has_real
                                        if r['after_ray'] and not r['after_ray']['cleared_at_tick']
                                        and (r['later_ray']['ray_len_m'] - r['later_ray']['d_cell_m']) <= 0.3),
                # ★ 在**汇总这一层**调 who_marks ✓（★ 它定义在循环之后 ⇒ ★ 循环里取不到 ✓）
                'z_of_marking_points': [(who_marks(r) or {}).get('z_min') for r in has_real
                                        if r['after_ray'] and not r['after_ray']['cleared_at_tick']][:20]},
            'remark_analysis': {
                'note': ('★ 只对"**有真回波在更远**"那批问 ✓：★ 那条射线之后，它有没有**先被清、又被重新标上** ✓'),
                'n_cleared_then_remarked': sum(1 for r in has_real if r['after_ray'] and r['after_ray']['re_marked_at_tick']),
                'n_cleared_never_remarked': sum(1 for r in has_real if r['after_ray'] and r['after_ray']['cleared_at_tick'] and not r['after_ray']['re_marked_at_tick']),
                'n_never_cleared_after_ray': sum(1 for r in has_real if r['after_ray'] and not r['after_ray']['cleared_at_tick']),
                'who_marked_them': {
                    'when': '★ 对"清了又被重新标上"那批 ⇒ ★ 问**重新标上**那一拍 ✓；★ 其余 ⇒ 问第一次被标上那一拍 ✓',
                    'scan_hits': sum(1 for r in has_real
                                     for t in [(r['after_ray'] or {}).get('re_marked_at_tick') or r['first_mark_tick']]
                                     if (who_marks(r, t) or {}).get('scan')),
                    'cam_hits': sum(1 for r in has_real
                                    for t in [(r['after_ray'] or {}).get('re_marked_at_tick') or r['first_mark_tick']]
                                    if (who_marks(r, t) or {}).get('cam')),
                    'both': sum(1 for r in has_real
                                for t in [(r['after_ray'] or {}).get('re_marked_at_tick') or r['first_mark_tick']]
                                if (who_marks(r, t) or {}).get('scan') and (who_marks(r, t) or {}).get('cam')),
                    'neither': sum(1 for r in has_real
                                   for t in [(r['after_ray'] or {}).get('re_marked_at_tick') or r['first_mark_tick']]
                                   if not (who_marks(r, t) or {}).get('scan') and not (who_marks(r, t) or {}).get('cam')),
                    'n': len(has_real)}},
            'has_real_group': {'d_to_traj_p50': pct([r['d_to_traj_m'] for r in has_real], 0.50),
                               'frac_near_obstacle_le0.3m': (round(sum(1 for r in has_real if r['d_to_obstacle_m'] <= ON_OBST_M) / len(has_real), 4) if has_real else None),
                               'examples': has_real[:6]},
            'frac_no_later_ray': round(len(no_ray) / len(rows), 4) if rows else None,
            'no_ray_group': {'d_to_traj_p10': pct([r['d_to_traj_m'] for r in no_ray], 0.10),
                             'd_to_traj_p50': pct([r['d_to_traj_m'] for r in no_ray], 0.50),
                             'd_to_traj_p75': pct([r['d_to_traj_m'] for r in no_ray], 0.75),
                             'd_to_traj_max': max((r['d_to_traj_m'] for r in no_ray), default=None),
                             'frac_near_obstacle_le0.3m': (round(sum(1 for r in no_ray if r['d_to_obstacle_m'] <= ON_OBST_M) / len(no_ray), 4) if no_ray else None)},
            'has_ray_group': {'d_to_traj_p50': pct([r['d_to_traj_m'] for r in has_ray], 0.50),
                              'd_to_traj_min': min((r['d_to_traj_m'] for r in has_ray), default=None),
                              'frac_near_obstacle_le0.3m': (round(sum(1 for r in has_ray if r['d_to_obstacle_m'] <= ON_OBST_M) / len(has_ray), 4) if has_ray else None),
                              'examples': has_ray[:8]},
            'all_sampled': {'d_to_traj_p10': pct([r['d_to_traj_m'] for r in rows], 0.10),
                            'd_to_traj_p50': pct([r['d_to_traj_m'] for r in rows], 0.50),
                            'd_to_traj_p75': pct([r['d_to_traj_m'] for r in rows], 0.75),
                            'd_to_traj_max': max((r['d_to_traj_m'] for r in rows), default=None)}}

    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    for run, v in out['rounds'].items():
        print('%-16s 幸存 %5d ｜ 抽 %3d ｜ ★ 无射线 %4d (%.1f%%) ｜ ★ 真回波在更远 %4d ｜ ★ 只有填充点 %4d' %
              (run, v['n_survivors'], v['n_sampled'], v['n_no_later_ray'],
               100 * (v['frac_no_later_ray'] or 0), v['n_ray_REAL_return_beyond'],
               v['n_ray_only_FILL_at_8m']))
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

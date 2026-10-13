#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wp4_stuck_probe.py —— ★ 逐拍查"卡住那一段"的全局图 ✓（✗ 不看节点日志 ✗）

用法: wp4_stuck_probe.py <run_dir> <wp4_x> <wp4_y> [--out <json>]

★★ 要回答的一句（reviewer 2026-10-11 指定 ✓）★★
  ★ **当时规划器给了路径没有** ✓ —— 载体两件 ✓：
    ① `bag/` 的 `/plan` ✓ —— ★ 本轮的 bag **只录了 4 个话题** ✗（`/rtk/scan`、
       `/rtk/camera/depth_image`、`/rtk/camera/camera_info`、`/clock`）⇒
       ★ **这一项未取到** ✗（✗ 不许当成"没有路径" ✗）；
    ② `probe/` 的**全局图逐拍快照** ✓ —— ★ 路径存在的前提是**目标可达** ✓ ⇒
       ★ 若图里那段被挡住 ⇒ ★ "没路径"那一支 ✓。★ 这就是本脚本量的东西 ✓。

★★ 量什么（★ 逐拍 ✓，车道与目标两条 ✓）★★
  · **车→wp4 的走廊**：直线上 ±`--tol` m 内的格，分类计数 ✓
    （★ 自由 0 ✓／★ 膨胀 1–98 ✓／★ **致命 99–100** ✓／★ 未知 255 ✓）
  · **wp4 自己那一格** ＋ 它周围 `--goal-r` m 的盘 ✓
  · **车自己周围** `--tol` m 的盘 ✓（★ 已知：车会把自己那格标脏 ✓）
  · ★ 最近致命格到车的距离 ✓（★ "是不是被自己围住了" ✓）

★ 判词规矩 ✓：✗ 不把"取不到"当"缺失" ✗；★ 本脚本只报**图里看到什么** ✓，
  ✗ 不替规划器下结论 ✗。
"""
import json
import math
import os
import re
import sys

LETHAL = lambda v: 99 <= v <= 100
FREE = lambda v: v == 0
UNKNOWN = lambda v: v == 255
INFLATED = lambda v: 1 <= v <= 98


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def load_meta(path):
    m = json.load(open(path, encoding='utf-8'))
    return {
        'stamp': float(m['stamp']), 'res': float(m['resolution']),
        'W': int(m['width']), 'H': int(m['height']),
        'ox': float(m['origin']['x']), 'oy': float(m['origin']['y']),
    }


def cell_of(x, y, meta):
    ix = int(math.floor((x - meta['ox']) / meta['res']))
    iy = int(math.floor((y - meta['oy']) / meta['res']))
    if 0 <= ix < meta['W'] and 0 <= iy < meta['H']:
        return iy * meta['W'] + ix
    return None


def xy_of(k, meta):
    return (meta['ox'] + ((k % meta['W']) + 0.5) * meta['res'],
            meta['oy'] + ((k // meta['W']) + 0.5) * meta['res'])


def disc(meta, x, y, r):
    r_ = int(math.ceil(r / meta['res']))
    c0 = cell_of(x, y, meta)
    if c0 is None:
        return []
    ix, iy = c0 % meta['W'], c0 // meta['W']
    out = []
    for j in range(iy - r_, iy + r_ + 1):
        for i in range(ix - r_, ix + r_ + 1):
            if 0 <= i < meta['W'] and 0 <= j < meta['H']:
                if math.hypot((i - ix) * meta['res'], (j - iy) * meta['res']) <= r + 1e-9:
                    out.append(j * meta['W'] + i)
    return out


def corridor(meta, p0, p1, tol):
    """★ 线段 p0→p1 上每 `res` 取一点，铺成 ±tol 的盘并集 ✓"""
    d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    n = max(1, int(math.ceil(d / meta['res'])))
    cells = set()
    for t in range(n + 1):
        a = t / n
        x = p0[0] + (p1[0] - p0[0]) * a
        y = p0[1] + (p1[1] - p0[1]) * a
        cells.update(disc(meta, x, y, tol))
    return sorted(cells)


def tally(blob, cells):
    c = {'free': 0, 'inflated': 0, 'lethal': 0, 'unknown': 0}
    for k in cells:
        v = blob[k]
        if LETHAL(v):
            c['lethal'] += 1
        elif UNKNOWN(v):
            c['unknown'] += 1
        elif FREE(v):
            c['free'] += 1
        elif INFLATED(v):
            c['inflated'] += 1
    c['n'] = len(cells)
    return c


def traj(run_dir):
    p = os.path.join(run_dir, 'trajectory.csv')
    rows = []
    with open(p, encoding='utf-8') as f:
        next(f)
        for ln in f:
            a = ln.strip().split(',')
            if len(a) < 3 or not a[1] or not a[2]:
                continue
            try:
                rows.append((float(a[0]), float(a[1]), float(a[2])))
            except ValueError:
                continue
    return rows


def stuck_window(rows):
    """★ 定义：从某拍起，**其后所有拍**相对该拍的位移都 ≤ 0.5 m ⇒ 该拍 = 停住起点 ✓"""
    n = len(rows)
    for i in range(n):
        x0, y0 = rows[i][1], rows[i][2]
        mx = 0.0
        for j in range(i, n, 20):
            mx = max(mx, math.hypot(rows[j][1] - x0, rows[j][2] - y0))
            if mx > 0.5:
                break
        if mx <= 0.5:
            return rows[i][0], i, n - 1
    return None, None, None


def main():
    run_dir = sys.argv[1]
    wx, wy = float(sys.argv[2]), float(sys.argv[3])
    tol, goal_r = 0.35, 0.5
    out_path = None
    if '--out' in sys.argv:
        out_path = sys.argv[sys.argv.index('--out') + 1]

    rows = traj(run_dir)
    t_stop, i_stop, i_end = stuck_window(rows)
    probe = os.path.join(run_dir, 'probe')
    js = sorted([f for f in os.listdir(probe) if re.match(r'map_\d+\.json$', f)], key=seq_of)
    dumps = []
    for j in js:
        meta = load_meta(os.path.join(probe, j))
        binp = os.path.join(probe, j.replace('.json', '.bin'))
        if not os.path.exists(binp):
            continue
        dumps.append((seq_of(j), meta, binp))

    # 车在每拍的位置：取 trajectory 里 ≤ stamp 的最后一拍 ✓
    def car_at(t):
        lo, hi = 0, len(rows) - 1
        if t < rows[0][0]:
            return rows[0][1], rows[0][2]
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if rows[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        return rows[lo][1], rows[lo][2]

    recs = []
    for seq, meta, binp in dumps:
        blob = open(binp, 'rb').read()
        if len(blob) != meta['W'] * meta['H']:
            recs.append({'seq': seq, 'error': 'size %d != %d' % (len(blob), meta['W'] * meta['H'])})
            continue
        cx, cy = car_at(meta['stamp'])
        d_wp = math.hypot(wx - cx, wy - cy)
        cor = corridor(meta, (cx, cy), (wx, wy), tol)
        gw = cell_of(wx, wy, meta)
        r = {
            'seq': seq, 'stamp': meta['stamp'],
            'car': [round(cx, 3), round(cy, 3)], 'd_to_wp4': round(d_wp, 3),
            'corridor': tally(blob, cor),
            'goal_disc': tally(blob, disc(meta, wx, wy, goal_r)),
            'car_disc': tally(blob, disc(meta, cx, cy, tol)),
            'goal_cell_value': (blob[gw] if gw is not None else None),
            'goal_cell_in_map': gw is not None,
        }
        # ★ 车到最近致命格的距离 ✓（★ 1.5 m 内找 ✓，找不到给 None ✓）
        near = disc(meta, cx, cy, 1.5)
        dl = [math.hypot(xy_of(k, meta)[0] - cx, xy_of(k, meta)[1] - cy)
              for k in near if LETHAL(blob[k])]
        r['nearest_lethal_m'] = round(min(dl), 3) if dl else None
        recs.append(r)

    # ★ 全局计数（★ 图有没有被淹 ✓）
    for r in recs:
        if 'error' in r:
            continue
        binp = os.path.join(probe, 'map_%04d.bin' % r['seq'])
        blob = open(binp, 'rb').read()
        r['global'] = {
            'lethal': sum(1 for v in blob if LETHAL(v)),
            'inflated': sum(1 for v in blob if INFLATED(v)),
            'unknown': sum(1 for v in blob if UNKNOWN(v)),
            'free': sum(1 for v in blob if FREE(v)),
        }

    out = {
        'run_dir': os.path.abspath(run_dir),
        'wp4': [wx, wy],
        'tol_m': tol, 'goal_r_m': goal_r,
        'stuck_onset': {'t': t_stop, 'traj_row': i_stop,
                        'last_row': i_end,
                        'car': [rows[i_stop][1], rows[i_stop][2]] if i_stop is not None else None},
        'n_dumps': len(recs),
        'lethal_convention': '99<=v<=100 致命 / 1..98 膨胀 / 0 自由 / 255 未知',
        'plan_topic_availability': ('★ 本轮 bag 只录了 /rtk/scan、/rtk/camera/depth_image、'
                                    '/rtk/camera/camera_info、/clock ⇒ /plan **未取到** ✗'
                                    '（✗ 不等于"没有路径" ✗）'),
        'records': recs,
    }
    if out_path:
        with open(out_path, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('wrote %s' % out_path)

    # ★ 时间线（★ 停住前后各打一行 ✓）
    print('★ 停住起点 t=%.2f（车 %s）｜本段 wp4 距离 %.3f m'
          % (t_stop, out['stuck_onset']['car'],
             math.hypot(wx - out['stuck_onset']['car'][0], wy - out['stuck_onset']['car'][1])))
    print('%-5s %-9s %-8s | %-28s | %-22s | %-6s %s'
          % ('seq', 'stamp', 'd_wp4', 'corridor(f/i/L/u)', 'car_disc(f/i/L/u)', 'goalV', 'nearL'))
    prev = None
    for r in recs:
        if 'error' in r:
            print('%s  ✗ %s' % (r['seq'], r['error']))
            continue
        c, cd = r['corridor'], r['car_disc']
        mark = ''
        if prev is not None and c['lethal'] > 0 and prev == 0:
            mark = '  ◀ 走廊出现致命'
        if t_stop is not None and r['stamp'] >= t_stop - 1.0 and (prev is None or prev == 0 or True):
            pass
        print('%-5d %-9.2f %-8.3f | %-28s | %-22s | %-6s %s%s'
              % (r['seq'], r['stamp'], r['d_to_wp4'],
                 '%d/%d/%d/%d' % (c['free'], c['inflated'], c['lethal'], c['unknown']),
                 '%d/%d/%d/%d' % (cd['free'], cd['inflated'], cd['lethal'], cd['unknown']),
                 r['goal_cell_value'], r['nearest_lethal_m'], mark))
        prev = c['lethal']


if __name__ == '__main__':
    main()

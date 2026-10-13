#!/usr/bin/env python3
"""从 `trajectory.csv` 判 **"到达并停稳"这个事件** —— 停稳必须**带量纲、按采样间隔折成速度**。

★★ 为什么要有这个脚本（2026-10-09 被 reviewer 抓到的真缺陷）★★
我先是把停稳判成"**单拍位移 < 5 mm**"。在 20 Hz 采样下这等于 **0.1 m/s** ✗ —— 实测放过了这一段：
车在"到达事件"之后**又飘出去 16.6 cm、到第 9 秒才真停**，而判据说它"从此停稳"。
这正是本项目被罚过三次的那条（[[arrival-criteria-must-require-standstill]]）：
**"单拍位移小于某个毫米数"不是一个判据，是一个随采样率漂移的数**。

⇒ 本脚本把口径**钉死成速度**（reviewer 2026-10-09 定，已同步 §4.1）：
  · **停稳** = 速度 < `--v-still`（默认 **0.02 m/s**，与 `geo_route` 的停稳判据同值）；
  · **到达事件** = 进入容差 ∧ **从此**速度 < `v_still`；
  · **到达读数取"停住后的位置"**（事件之后**运动真正停止**那一段的均值），**不取事件那一拍**
    —— 事件那一拍是"刚开始停"，读它会把"还在走"的瞬时值当成落点 ✗。
  · **停稳窗口由数据定义**：从"速度从此 < v_still"那一刻起，到**记录结束**（不是"记录器何时收工"）。

★★ 2026-10-09 第二轮修正（reviewer 又抓到一处会复发的残留）★★
`trajectory.csv` **是边跑边写的** ⇒ "记录结束"= **你读文件的那一刻** ⇒
同一条数据我读到 558.0 s、reviewer 读到 651.0 s ⇒ **同一个量报出两个窗口长度（309.6 vs 402.6 s）** ✗。
**落点读数本身稳健**（窗口从 558 拉到 651，`d_gt` 仍是 0.0513 —— 它读的是"停住后的平台"），
但**窗口长度不可复算** ✗ ⇒ 违反"读数有效性"第 4 维（**前提**）。
⇒ 规则（reviewer 给的一句话）：**凡会被"什么时候读"改变的量，要么按规则截断、要么把读取时刻写进产物** ✓。
本脚本两条都做：① 另给一个**按规则截断**的窗口 `rule_window_30s`（停住后固定 30 s，**与何时读无关**）；
② 把**读取时刻 + 文件末时间戳**一并写进产物（`file_read_at` / `source_last_t`）✓。

用法（宿主机上就能跑，不需要 ROS）：
  arrival_event.py <trajectory.csv> --goal X Y [--tol 0.25] [--v-still 0.02] [--json out.json]
"""

import argparse
import csv
import os
import json
import math
import sys
import time


def load(path):
    rows = []
    for r in csv.DictReader(open(path, encoding='utf-8')):
        if not r.get('est_x'):
            continue
        rows.append({'t': float(r['t']),
                     'gx': float(r['gt_x']), 'gy': float(r['gt_y']),
                     'mx': float(r['est_x']), 'my': float(r['est_y'])})
    return rows


def speeds(rows, key_x, key_y):
    """逐拍速度（m/s）：位移 / 相邻时间差。**带量纲**，不随采样率漂移 ✓。"""
    out = []
    for i in range(len(rows) - 1):
        dt = rows[i + 1]['t'] - rows[i]['t']
        d = math.hypot(rows[i + 1][key_x] - rows[i][key_x],
                       rows[i + 1][key_y] - rows[i][key_y])
        out.append(d / dt if dt > 1e-9 else float('inf'))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('trajectory')
    # ★ 用 `X,Y` 单参数、不用 `nargs=2`：**值以 `-` 开头**（目标 x 常是负数）时
    #   argparse 会把它当选项 ⇒ `--goal -40 25` 直接报错 ✗（本项目的 `stop_margin_check.py`
    #   踩过同一个坑，那里也是改成 `--stop=X,Y`）✓
    ap.add_argument('--goal', required=True, help='目标 X,Y（例：--goal=-40,25 或 --goal -40,25）')
    ap.add_argument('--tol', type=float, default=0.25, help='到位容差 (m)')
    ap.add_argument('--v-still', type=float, default=0.02, help='停稳速度门 (m/s)')
    ap.add_argument('--json', default='')
    a = ap.parse_args(argv)

    read_at = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime())
    rows = load(a.trajectory)
    if len(rows) < 10:
        print('!! 样本太少', file=sys.stderr)
        return 1
    gx, gy = [float(v) for v in a.goal.split(',')[:2]]
    dg = [math.hypot(r['gx'] - gx, r['gy'] - gy) for r in rows]
    dm = [math.hypot(r['mx'] - gx, r['my'] - gy) for r in rows]
    v = speeds(rows, 'gx', 'gy')          # 速度按**真值**算（真值才是"车真在哪"）

    # 事件：进入容差 ∧ 从此速度 < v_still
    ev = None
    for i in range(len(rows)):
        if dg[i] >= a.tol:
            continue
        if all(v[j] < a.v_still for j in range(i, len(v))):
            ev = i
            break
    if ev is None:
        print('!! 没找到"进入容差 且 从此停稳"的事件（容差 %.2f / 停稳 %.3f m/s）' % (a.tol, a.v_still),
              file=sys.stderr)
        return 2

    # "真正停住"那一段：从事件起找**最后一次速度 >= v_still** 的下一拍
    last_moving = ev
    for j in range(ev, len(v)):
        if v[j] >= a.v_still:
            last_moving = j + 1
    stop_rows = rows[last_moving:]
    stop_t0 = stop_rows[0]['t']
    gx_s = sum(r['gx'] for r in stop_rows) / len(stop_rows)
    gy_s = sum(r['gy'] for r in stop_rows) / len(stop_rows)
    mx_s = sum(r['mx'] for r in stop_rows) / len(stop_rows)
    my_s = sum(r['my'] for r in stop_rows) / len(stop_rows)

    # ★★ 2026-10-10（裁定 ④ 的全库扫查）：这里原来是 `rows[-31:]`，字段名却叫
    #   `tail_30_max_step_m`（暗示 30 s）—— 可 `trajectory.csv` 是 ~20 Hz ⇒
    #   **31 个样本 ≈ 1.5 s，不是 30 s** ✗。**与 geo_route 那个是同族**（按样本数量时间）。
    #   ⇒ 改成**按时间**取尾部窗口 ✓。它只是**诊断量、不是判据**，所以不影响任何裁决 ✓，
    #     但字段名会骗人，一样要修 ✓。
    _t_end = rows[-1]['t']
    tail = [r for r in rows if r['t'] >= _t_end - 30.0]
    tail_max = max((math.hypot(tail[j + 1]['gx'] - tail[j]['gx'],
                               tail[j + 1]['gy'] - tail[j]['gy'])
                    for j in range(len(tail) - 1)), default=0.0)
    win_v = v[last_moving:]
    res = {
        'trajectory': a.trajectory, 'goal': [gx, gy],
        'tolerance_m': a.tol, 'v_still_mps': a.v_still,
        'samples': len(rows), 't_range_s': [rows[0]['t'], rows[-1]['t']],
        'file_read_at': read_at,
        'file_mtime': time.strftime('%Y-%m-%dT%H:%M:%S',
                                    time.localtime(os.path.getmtime(a.trajectory))),
        # ★ 事件那一拍（"刚开始停"）—— **只作过程量，不是落点读数**
        'event_row': {'t': rows[ev]['t'],
                      'gt': [rows[ev]['gx'], rows[ev]['gy']], 'd_gt': dg[ev],
                      'map': [rows[ev]['mx'], rows[ev]['my']], 'd_map': dm[ev]},
        # ★ **落点读数**：真正停住之后那一段的均值
        'stopped_pose': {'t_from': stop_t0, 't_to': rows[-1]['t'],
                         'gt_mean': [gx_s, gy_s],
                         'map_mean': [mx_s, my_s],
                         'd_gt_mean_m': math.hypot(gx_s - gx, gy_s - gy),
                         'd_map_mean_m': math.hypot(mx_s - gx, my_s - gy),
                         'map_minus_gt_m': math.hypot(mx_s - gx_s, my_s - gy_s)},
        'standstill': {
                       # ① **与何时读无关**的窗口：停住后固定 30 s（0 ≤ 30 s 时退化为可用长度）
                       'rule_window_30s': {'t_from': stop_t0,
                                           't_to': min(stop_t0 + 30.0, rows[-1]['t']),
                                           'rule': '停住那一刻起固定 30 s —— **不随"何时读文件"变** ✓'},
                       # ② 到"文件末尾"的窗口 —— **会被读取时刻改变**，故连同读取时刻一起记
                       'window_s': rows[-1]['t'] - stop_t0,
                       'tail_30_max_step_m': tail_max,
                       'window_max_speed_mps': max(win_v) if win_v else 0.0,
                       'source_last_t': rows[-1]['t'],
                       'file_read_at': read_at,
                       'note': '★ `window_s` 的右端 = **读这份文件的那一刻**（`trajectory.csv` 是边跑边写）'
                               '⇒ **同一个量在不同时刻读会得到不同的窗口长度**（实测 309.6 vs 402.6 s）✗。'
                               '落点读数不受影响（它读的是停住后的平台 ✓），但窗口长度要报就用 '
                               '`rule_window_30s` ✓；要用 `window_s` 就必须连 `file_read_at` 一起报 ✓'},
    }
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(res, fh, ensure_ascii=False, indent=2)
    print(json.dumps(res, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())

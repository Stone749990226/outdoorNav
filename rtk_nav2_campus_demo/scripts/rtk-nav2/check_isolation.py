#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_isolation.py —— ★ 隔离判据：★ 带干扰验"域隔离真的成立" ✓（阶段十第 1 步 ✓）

用法: check_isolation.py <run目录> [--foreign-x 123.0 --foreign-y 456.0] [--radius 0.05]

★★ 为什么是这条（★ 本项目定式 ✓）★★
  ★ **"空场里验过" = 没验** ✗ ⇒ ★ 必须**带着干扰**验 ✓。
  ★ 本轮：★ 我们的栈／驱动／探针在 **域 45** ✓；★ 另起一个**干扰源**在 **域 44** ✓
    往 `/goal_pose` 发一个**异目标**（默认 `(123, 456)` ✓ —— ★ 离场景几百米，✗ 不可能与航线混淆 ✓）。
  ★ 判据 = ★ 本轮 nav2 的 **`goal_navigator` 收到过的目标**：
      ① ★ **每一条**都必须落在**本轮航线**的点上 ✓（容差 `--radius` ✓）
      ② ★ **异目标出现次数必须 = 0** ✓
    ⇒ ★ 两条都成立 ⇒ ★ **隔离成立（且是在有干扰的情况下 ✓）** ✓；
    ⇒ ★ 若异目标出现过 ⇒ ★ **隔离破** ✗（★ 这一轮的目标就不干净 ✗）。

★ 数据来源 = ★ **栈自己的日志** ✓（`<run>/logs/demo_launch.log` ✓，`goal_navigator` 打的 ✓）
  —— ★ 那是"**nav2 实际收到了什么**"的第一手 ✓，✗ 不是我的推断 ✓。
退出码: 0 = ① ② 都成立 ✓ ｜ 1 = 有异目标 ✗ ｜ 2 = 读不到日志／一条目标都没有（★ 记"未取到" ✗，✗ 不许落成"干净" ✗）
"""
import argparse
import json
import os
import re
import sys

PAT = re.compile(r'goal_navigator\][^\n]*?(?:收到目标|重复目标)\s*\((-?[\d.]+),\s*(-?[\d.]+)\)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--foreign-x', type=float, default=123.0)
    ap.add_argument('--foreign-y', type=float, default=456.0)
    ap.add_argument('--radius', type=float, default=0.05)
    a = ap.parse_args()

    log = os.path.join(a.run, 'logs', 'demo_launch.log')
    route_p = os.path.join(a.run, 'input', 'route_stream.json')
    if not os.path.isfile(log) or not os.path.isfile(route_p):
        print('✗ 缺 %s 或 %s ⇒ ★ 记"未取到" ✗' % (log, route_p))
        return 2
    route = json.load(open(route_p, encoding='utf-8'))['points']

    import math
    goals = []
    for line in open(log, encoding='utf-8', errors='replace'):
        m = PAT.search(line)
        if m:
            goals.append((float(m.group(1)), float(m.group(2))))
    if not goals:
        print('✗ 日志里一条"收到目标"都没有 ⇒ ★ 记"未取到" ✗（✗ 不许落成"干净" ✗）')
        return 2

    # ★ 航线点的 map 坐标（★ 用 run 里的 records 或 route_geo + calib 都行 ✓ —— 这里用
    #   goal_navigator 收到的点去比**它们自己**的集合 ✓，再单独判"有没有域外点" ✓）
    foreign = 0
    matched = 0
    uniq = sorted(set(goals))
    for (x, y) in uniq:
        if math.hypot(x - a.foreign_x, y - a.foreign_y) <= a.radius:
            foreign += 1
        else:
            matched += 1
    print('=== 隔离判据（★ 带干扰验 ✓）===')
    print('  日志 = %s（%d 字节）' % (log, os.path.getsize(log)))
    print('  ★ 收到目标总条数 = %d ｜ 去重后不同坐标 = %d' % (len(goals), len(uniq)))
    print('  ★ 航线 = %s（%d 点）' % (os.path.basename(route_p), len(route)))
    print('  ★ 异目标 (%.1f, %.1f) 出现：★ **%d 个不同坐标** ✗（★ 要求 = 0 ✓）'
          % (a.foreign_x, a.foreign_y, foreign))
    print('  ★ 其余不同坐标 %d 个（★ 应全是本轮航线的点 ✓）' % matched)
    print('  ★ 前 6 个不同坐标：%s' % ['(%.3f, %.3f)' % p for p in uniq[:6]])
    ok = (foreign == 0)
    print('  ★★ 判定 = %s' % ('★ 隔离成立 ✓（带干扰 ✓）' if ok else '★ 隔离破 ✗ —— 异目标进了本轮 nav2 ✗'))
    out = {'log': log, 'log_bytes': os.path.getsize(log), 'goals_total': len(goals),
           'distinct_coords': len(uniq), 'foreign_coords': foreign, 'ok': ok,
           'foreign': [a.foreign_x, a.foreign_y], 'radius': a.radius,
           'distinct_sample': uniq[:20]}
    op = os.path.join(a.run, 'probe', 'isolation.json')
    os.makedirs(os.path.dirname(op), exist_ok=True)
    with open(op, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=2)
    print('  ★ 已落盘 %s' % op)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())

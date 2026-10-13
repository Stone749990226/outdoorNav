#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""survivor_z_hist —— ★ **全部幸存格**的"标记点 z"直方图 ✓（★ 纯离线 ✓）

★★ 回答业主那个疑问（★ reviewer 2026-10-11 派 ✓）★★
  ★ 业主问 ✓：★ "干净族随俯仰单调 **0→0→141**、★ 几何下限以下**精确为 0** ⇒ 为什么又说**压住俯仰没让残留变少**？" ✓
  ★ 说法 ✓：★ **那是两族** ✓ —— ★ 我测的是 **z < 0.05**（★ A 族 ✓）；★ **构成残留的是 z 0.06–0.14 那族**（★ B 族 ✓）
  ⇒ ★★ 但 ★ 这条此前**只靠一个子集**（★ "从未被清"那 14／3 格 ✓）撑 ✗ ⇒ ★ **本工具把它做成全量的** ✓✓

★★ 判据（★ 两头都算结论 ✓）★★
  ★ **主峰在 0.06–0.14** ⇒ ★★ **"压俯仰没用"解释成立** ✓✓（★ 链子自洽 ✓）
  ★ **主峰在 <0.05** ✗ ⇒ ★★ **那才是真矛盾** ✓ ⇒ ★ **立刻报 reviewer** ✓（★ 不绕过去 ✓）

★★ 口径（★ 写死 ✓）★★
  · ★ **幸存格 = 每轮全部** ✓（★ 末拍致命 ∧ 第一张图不致命 ✓ —— ★ ✗ 不是那 200 抽样 ✗）
  · ★ **标记点** = ★ 该格**第一次致命那一拍**里、★ 落在它附近（★ **±2 拍 ／ 0.15 m** ✓）的 **`scan`** 点 ✓
  · ★ 分档 **0.01 m** ✓，★ 覆盖 **−0.05 … 0.20** ✓
  · ★ **取不到就单列"未取到"** ✓（★ 这条纪律本项目有档 ✓）
"""
import argparse
import glob
import json
import math
import os
import re
import sys

ART = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'
SCENE = '/home/xjy/projects/outdoorNav/gazebo_campus/generated/scene_geometry.json'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_obstacles import load_obstacles, dist_to_obstacle      # noqa: E402
ON_OBS_M = 0.30      # ★ ≤ 这个距离 ⇒ ★ 算"**在障碍上**" ✓
LETHAL_LO, LETHAL_HI = 99, 127
NEAR_M = 0.15
TICK_WIN = 2
Z_LO, Z_HI, ZBIN = -0.05, 0.20, 0.01


def poly_dist(px, py, poly):
    """★ 点到折线的最近距离 ✓（★ "离轨迹多远" ✓）"""
    best = float('inf')
    for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
        best = min(best, math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)))
    return best


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--rounds', default='campus9-res1,campus9-cutS9')
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    out = {'what': '★ **全部幸存格**的"标记点 z"直方图 ✓（★ 0.01 m 一档 ✓）',
           'preregistered': {'peak_006_014': '★★ **"压俯仰没用"解释成立** ✓（★ 链子自洽 ✓）',
                             'peak_below_005': '★★ **真矛盾** ✗ ⇒ ★ **立刻报** ✓'},
           'convention': {'survivors': '★ 每轮**全部** ✓（末拍致命 ∧ 第一张图不致命 ✓）',
                          'mark_points': '★ 第一次致命那一拍 ±%d 拍 ／ %.2f m 内的 `scan` 点 ✓' % (TICK_WIN, NEAR_M),
                          'bins': '★ %.2f m 一档 ✓，覆盖 %.2f…%.2f ✓' % (ZBIN, Z_LO, Z_HI)},
           'rounds': {}}

    for run in [r for r in a.rounds.split(',') if r]:
        ds = sorted(glob.glob(os.path.join(ART, run, 'probe_frozen_wp8_arrival_*')))
        if not ds:
            continue
        d = ds[-1]
        maps = sorted(glob.glob(os.path.join(d, 'map_*.bin')), key=seq_of)
        if len(maps) < 3:
            continue
        end = open(maps[-1], 'rb').read(); start = open(maps[0], 'rb').read()
        surv = [i for i in range(len(end))
                if LETHAL_LO <= end[i] <= LETHAL_HI and not (LETHAL_LO <= start[i] <= LETHAL_HI)]
        meta = json.load(open(os.path.join(d, 'tick_%04d.json' % seq_of(maps[1])), encoding='utf-8'))
        W = meta['map']['width']; res_ = meta['map']['resolution']
        ox = meta['map']['origin']['x']; oy = meta['map']['origin']['y']

        # ★ 全部幸存格 → 第一次致命那一拍 ✓（一趟流式 ✓）
        first = {}
        remaining = set(surv)
        for m in maps[1:]:
            if not remaining:
                break
            buf = open(m, 'rb').read(); s = seq_of(m); done = []
            for i in remaining:
                if LETHAL_LO <= buf[i] <= LETHAL_HI:
                    first[i] = s; done.append(i)
            for i in done:
                remaining.discard(i)

        OBS = load_obstacles(SCENE)
        TRAJ = []
        for f in sorted(glob.glob(os.path.join(d, 'tick_*.json')), key=seq_of):
            r = json.load(open(f, encoding='utf-8')).get('robot_xy_map')
            if r and len(r) >= 2:
                TRAJ.append((float(r[0]), float(r[1])))
        cache = {}
        hist = {}
        n_pts = 0
        no_z = 0
        # ★★ 在不在障碍上 ✗／✓ ＋ ★ 按族交叉 ✓ ★★
        cross = {'on_obs': {'lt_0.05': 0, '0.06_0.14': 0, 'other': 0, 'no_z': 0},
                 'off_obs': {'lt_0.05': 0, '0.06_0.14': 0, 'other': 0, 'no_z': 0}}
        off_d = []
        FIRSTKEY = {}
        # ★ 先按"在不在障碍上"分堆 ✓（★ 与 z 无关 ⇒ 每个幸存格都有这一栏 ✓）
        for i in first:
            ix, iy = i % W, i // W
            cx = ox + (ix + 0.5) * res_; cy = oy + (iy + 0.5) * res_
            k_ = 'on_obs' if dist_to_obstacle(cx, cy, OBS) <= ON_OBS_M else 'off_obs'
            FIRSTKEY[i] = k_

        for i, fm in first.items():
            ix, iy = i % W, i // W
            cx = ox + (ix + 0.5) * res_; cy = oy + (iy + 0.5) * res_
            d_obs = dist_to_obstacle(cx, cy, OBS)
            d_traj = poly_dist(cx, cy, TRAJ)
            got = False
            zs_i = []
            for ds_ in range(-TICK_WIN, TICK_WIN + 1):
                s = fm + ds_
                if s not in cache:
                    fp = os.path.join(d, 'scan_%04d.xyz' % s)
                    pts = []
                    if os.path.isfile(fp):
                        for line in open(fp, encoding='utf-8'):
                            v = line.split()
                            if len(v) == 3:
                                pts.append((float(v[0]), float(v[1]), float(v[2])))
                    cache[s] = pts
                for px, py, pz in cache[s]:
                    if math.hypot(px - cx, py - cy) <= NEAR_M:
                        got = True
                        zs_i.append(pz)
                        n_pts += 1
                        k = round(math.floor(pz / ZBIN) * ZBIN, 2)
                        hist[k] = hist.get(k, 0) + 1
            if not got:
                no_z += 1
            # ★ 族 ＋ 在不在障碍 的交叉 ✓（★ 取到的点里最小的那个 z 定族 ✓）
            fam = ('no_z' if not got else
                   ('lt_0.05' if (min(zs_i) < 0.05) else
                    ('0.06_0.14' if (0.06 <= min(zs_i) <= 0.14) else 'other')))
            cross[FIRSTKEY[i]][fam] += 1
            if FIRSTKEY[i] == 'off_obs':
                off_d.append((round(d_traj, 3), round(d_obs, 3)))
        n_on = sum(cross['on_obs'].values()); n_off = sum(cross['off_obs'].values())
        ks = sorted(hist)
        tot = sum(hist.values())
        out['rounds'][run] = {
            'frozen_dir': os.path.relpath(d, ART), 'n_survivors': len(surv),
            'n_with_mark_point': len(surv) - no_z, 'n_NO_MARK_POINT_unresolved': no_z,
            'n_marking_points': n_pts,
            'hist': {('%.2f' % k): hist[k] for k in ks},
            'peak_bin': (('%.2f' % max(ks, key=lambda k: hist[k])) if ks else None),
            'share_lt_0.05': (round(sum(v for k, v in hist.items() if k < 0.05) / tot, 4) if tot else None),
            'share_0.06_to_0.14': (round(sum(v for k, v in hist.items() if 0.06 <= k <= 0.14) / tot, 4) if tot else None),
            'share_lt_0.00': (round(sum(v for k, v in hist.items() if k < 0.0) / tot, 4) if tot else None),
            'on_vs_off_obstacle': {
                'ON_OBS_M': ON_OBS_M,
                'n_on_obstacle': n_on, 'n_off_obstacle': n_off,
                'share_on_obstacle': round(n_on / (n_on + n_off), 4) if (n_on + n_off) else None,
                'cross_by_family': cross,
                'off_obstacle_d_to_traj': {
                    'p10': (sorted(x[0] for x in off_d)[len(off_d) // 10] if off_d else None),
                    'p50': (sorted(x[0] for x in off_d)[len(off_d) // 2] if off_d else None),
                    'max': (max((x[0] for x in off_d), default=None)),
                    'd_obs_p50': (sorted(x[1] for x in off_d)[len(off_d) // 2] if off_d else None),
                    'd_obs_max': (max((x[1] for x in off_d), default=None))},
                'reading': ('★ **多数不在障碍上** ⇒ "随机的点"确是多数 ✓ ⇒ 量程那一刀方向是对的 ✓'
                            if n_off > n_on else
                            '★ **多数在障碍上** ⇒ 残留大部分是**合法标记** ✓ ⇒ 业主可能根本不想动 ✓')}}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    for run, v in out['rounds'].items():
        print('%-16s 幸存 %5d ｜ 取到 %5d ／ 未取到 %4d ｜ ★ 主峰 %s ｜ ★ <0.05 占 %s ｜ ★ 0.06–0.14 占 %s' %
              (run, v['n_survivors'], v['n_with_mark_point'], v['n_NO_MARK_POINT_unresolved'],
               v['peak_bin'], v['share_lt_0.05'], v['share_0.06_to_0.14']))
        print('   直方图：', v['hist'])
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

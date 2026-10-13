#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""expiry_curves —— ★ "标记按时间过期" 的**收益／代价两条曲线** ✓（★ 纯离线 ✓，★ 不写码 ✓）

用法: python3 expiry_curves.py --out <json> [--rounds r1,r2] [--sample 200]

★★ 要回答什么（★ reviewer 2026-10-11 派 ✓）★★
  ★ 机制链已经说清 ✓：★ **残留 = 被标上以后没人再扫到的格** ✓（★ 70–82% ✓）
    ⇒ ★ **按时间过期正中它** ✓✓（★ 不必碰量程、不必碰标记门 ✓）
  ⇒ ★★ 但 ★ **主楼在 7.07 m** ✓ ⇒ ★ **车开走后它也"没人再看"** ✗ ⇒ ★ **它也会过期** ✗
    ⇒ ★ 所以必须把 **收益曲线** 和 **代价曲线** 一并报 ✓ ⇒ ★ 业主按"T 取多少 ⇒ 治多少 vs 误清多少"定 ✓

★★ 两条曲线怎么算 ✓ ★★
  · ★ 对每个幸存格求 ★ **`t_last`** = ★ **最后一次"有射线能穿过它"的那一刻** ✓
    （★ 口径与 `survivor_ray_check.py` **同一套** ✓：★ 同方位 ✓、★ 更远 ✓、★ 上限 = `raytrace_max_range` 8.0 ✓；
     ★ 只是方向相反 —— ★ 它找**第一条** ✓、★ 这里找**最后一条** ✓）
  · ★ **收益** ✓：★ `T` 时，★ **`t_end − t_last ≥ T`** 的格 ⇒ ★ **会过期被清** ✓
  · ★ **代价** ✓：★ 同一批里 ★ **落在真障碍上**（★ ≤0.3 m ✓，★ 地面已剔 ✓）的格 ⇒ ★ **被误清** ✓
  · ★ 时间用 ★ **`sim_t`** ✓（★ 过期是仿真／ROS 时间 ✓，✗ 不是墙钟 ✗）
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
ANG_TOL_DEG, RANGE_CAP, MOVE_EPS = 1.0, 8.0, 0.05
ON_OBS_M = 0.30
TICK_STRIDE = 20
TS = (20.0, 40.0, 80.0, 160.0, 320.0)


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--rounds', default='campus9-res1,campus9-cutS9')
    ap.add_argument('--sample', type=int, default=200)
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    OBS = load_obstacles(SCENE)
    out = {'what': '★ "标记按时间过期"的**收益／代价两条曲线** ✓（★ 全离线 ✓、★ 不写码 ✗）',
           'mechanism': '★ 残留 = 被标上以后没人再扫到的格 ✓（★ 70–82% ✓）⇒ ★ 按时间过期正中它 ✓',
           'criterion': '★ 格在 T 时过期 ⇔ ★ `t_end − t_last ≥ T` ✓；★ `t_last` = 最后一次"有射线穿过它"的那一刻 ✓',
           't_last_ruler': '★ 与 `survivor_ray_check.py` 同一套 ✓（★ 同方位 ±1° ✓、★ 更远 ✓、★ 上限 8.0 = `raytrace_max_range` ✓）',
           'time': '★ 用 `sim_t` ✓（★ 过期是仿真时间 ✓）', 'tick_stride': TICK_STRIDE, 'rounds': {}}

    for run in [r for r in a.rounds.split(',') if r]:
        ds = sorted(glob.glob(os.path.join(ART, run, 'probe_frozen_wp8_arrival_*')))
        if not ds:
            continue
        d = ds[-1]
        maps = [m for m in sorted(glob.glob(os.path.join(d, 'map_*.bin')), key=seq_of)]
        if len(maps) < 3:
            continue
        end = open(maps[-1], 'rb').read(); start = open(maps[0], 'rb').read()
        surv = [i for i in range(len(end))
                if LETHAL_LO <= end[i] <= LETHAL_HI and not (LETHAL_LO <= start[i] <= LETHAL_HI)]
        step = max(1, len(surv) // a.sample)
        sample = surv[::step][:a.sample]
        meta = json.load(open(os.path.join(d, 'tick_%04d.json' % seq_of(maps[1])), encoding='utf-8'))
        W = meta['map']['width']; res_ = meta['map']['resolution']
        ox = meta['map']['origin']['x']; oy = meta['map']['origin']['y']

        # ★ 每拍 sim_t ✓ ＋ 云（按需读、缓存 ✓）
        simt = {}
        for f in glob.glob(os.path.join(d, 'tick_*.json')):
            try:
                j = json.load(open(f, encoding='utf-8'))
                if j.get('sim_t') is not None:
                    simt[seq_of(f)] = float(j['sim_t'])
            except Exception:
                pass
        t_end = max(simt.values()) if simt else None
        avail = {seq_of(p) for p in glob.glob(os.path.join(d, 'scan_*.xyz'))}
        cache = {}

        def cloud(s):
            if s not in cache:
                pts = []
                fp = os.path.join(d, 'scan_%04d.xyz' % s)
                if os.path.isfile(fp):
                    for line in open(fp, encoding='utf-8'):
                        v = line.split()
                        if len(v) == 3:
                            pts.append((float(v[0]), float(v[1])))
                cache[s] = pts
            return cache[s]

        def tfl(s):
            try:
                return float(json.load(open(os.path.join(d, 'tick_%04d.json' % s), encoding='utf-8'))['scan']['tf']['M'][0][3]), \
                       float(json.load(open(os.path.join(d, 'tick_%04d.json' % s), encoding='utf-8'))['scan']['tf']['M'][1][3])
            except Exception:
                return None, None

        seqs = [seq_of(m) for m in maps if seq_of(m) in avail]
        rows = []
        for i in sample:
            ix, iy = i % W, i // W
            cx = ox + (ix + 0.5) * res_; cy = oy + (iy + 0.5) * res_
            on_obs = dist_to_obstacle(cx, cy, OBS) <= ON_OBS_M
            t_last = None
            for s in reversed(seqs):
                if s not in simt:
                    continue
                sx, sy = tfl(s)
                if sx is None:
                    continue
                dc = math.hypot(cx - sx, cy - sy)
                if dc <= 1e-9 or dc > RANGE_CAP:
                    continue
                bc = math.atan2(cy - sy, cx - sx)
                hit = False
                for px, py in cloud(s):
                    dp = math.hypot(px - sx, py - sy)
                    if dp <= dc + MOVE_EPS or dp > RANGE_CAP:
                        continue
                    dth = abs((math.atan2(py - sy, px - sx) - bc + math.pi) % (2 * math.pi) - math.pi)
                    if math.degrees(dth) <= ANG_TOL_DEG:
                        hit = True
                        break
                if hit:
                    t_last = simt[s]
                    break
            rows.append({'on_obs': on_obs,
                         'since_last_sweep_s': (round(t_end - t_last, 1) if (t_last is not None and t_end) else None)})
        n = len(rows)
        n_on = sum(1 for r in rows if r['on_obs']); n_off = n - n_on
        curves = {}
        for T in TS:
            exp = [r for r in rows if r['since_last_sweep_s'] is not None and r['since_last_sweep_s'] >= T]
            e_on = sum(1 for r in exp if r['on_obs']); e_off = len(exp) - e_on
            curves['T=%.0f' % T] = {
                'benefit_pct_of_TOFIX': (round(100 * e_off / n_off, 2) if n_off else None),
                'cost_pct_of_LEGIT': (round(100 * e_on / n_on, 2) if n_on else None),
                'n_expired_off': e_off, 'n_expired_on': e_on}
        out['rounds'][run] = {
            'frozen_dir': os.path.relpath(d, ART), 'n_survivors': len(surv), 'n_sampled': n,
            'n_on_obstacle': n_on, 'n_off_obstacle': n_off,
            'window_sim_s': (round(t_end - min(simt.values()), 1) if (t_end and simt) else None),
            'curves': curves}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    for run, v in out['rounds'].items():
        print('★ %s：幸存 %d ｜ 抽 %d ｜ 要治 %d ／ 合法 %d ｜ 窗口 %.0f s' %
              (run, v['n_survivors'], v['n_sampled'], v['n_off_obstacle'], v['n_on_obstacle'], v['window_sim_s'] or -1))
        for k, c in v['curves'].items():
            print('   %-7s ⇒ ★ 治掉要治堆 %s%% ｜ ★ 误清合法堆 %s%%' % (k, c['benefit_pct_of_TOFIX'], c['cost_pct_of_LEGIT']))
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

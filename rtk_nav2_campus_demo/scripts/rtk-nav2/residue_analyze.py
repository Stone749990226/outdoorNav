#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""residue_analyze.py —— ★ 全局图"残留障碍点"归因（阶段十第 1 步 ✓）

用法: residue_analyze.py <run目录> [--radius 0.20] [--stable-from 0.5]

做四件事（★ 全部由产物给，✗ 不推断 ✓）：
  ① **两时刻差分** ＋ **"一直没被清掉"** 的残留格集合 ✓ —— 后者用**整条 tick 序列**判 ✓
     （★ "中间没变回自由"**两个快照判不出来** ✗ ⇒ 必须用序列 ✓；两时刻那一份照样单独报 ✓）
  ② **归因到源**：把残留格投到 `/rtk/scan_points` 与 `/rtk/camera/points` 的点上 ✓ ⇒ 各占多少 ✓
  ③ **"自身还是地面"**：★ 匹配点的**高度**分布 ✓ ＋ ★ 车**静止**那段还在不在**新增**格 ✓
  ④ ★ **正向对照**（✗ 只标帧换算不够 ✓）：★ 点云的点落在全局图各档**代价**上的分布 ✓ ——
     ★ "该有数据的地方真的有"＝★ 传感器端点大部分落在**占用格**上 ✓（✗ 若几乎全落 free/unknown
     ⇒ ★ 是**对齐错了** ✗，不是"没有残留" ✗ —— 本项目 dumpdiff 转置那次就是这么栽的 ✓）。

★ `self-check`：本工具会在输出里显式写"对齐是否成立" ✓ —— ✗ 不成立时**不许**继续给归因结论 ✗。
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys


def load_ticks(run):
    ticks = []
    for f in sorted(glob.glob(os.path.join(run, 'probe', 'tick_*.json'))):
        d = json.load(open(f, encoding='utf-8'))
        seq = d['seq']
        mp = os.path.join(run, 'probe', 'map_%04d.bin' % seq)
        if d.get('map') and os.path.isfile(mp):
            blob = open(mp, 'rb').read()
            d['_blob'] = blob
            # ★★ 2026-10-11：★ 别再把 178 万格逐格转 int ✗（315 拍 ⇒ 5.6 亿次 ⇒ 跑不完 ✗）。
            #   ★ 占用 = 字节值 1..100 ✓（未知 -1 在 wire 上是 **255** ✓ ⇒ 不在这个区间 ✓）
            #   ⇒ ★ 用正则一次扫出**占用格下标集合** ✓（C 速度 ✓、且占用是**稀疏**的 ~0.3% ✓）。
            d['_occ'] = {m.start() for m in re.finditer(rb'[\x01-\x64]', blob)}
        d['_file'] = f
        ticks.append(d)
    return ticks


def load_xyz(run, seq, kind):
    p = os.path.join(run, 'probe', '%s_%04d.xyz' % (kind, seq))
    out = []
    if os.path.isfile(p):
        for line in open(p, encoding='utf-8'):
            parts = line.split()
            if len(parts) == 3:
                out.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return out


def cell_of(info, wx, wy):
    res = info['resolution']
    gx = int((wx - info['origin']['x']) / res)
    gy = int((wy - info['origin']['y']) / res)
    return gx, gy


def world_of(info, gx, gy):
    res = info['resolution']
    return (info['origin']['x'] + (gx + 0.5) * res,
            info['origin']['y'] + (gy + 0.5) * res)


def val(blob, k):
    """★ 取该格**带符号**的值 ✓（wire 上 -1 = 255 ✓）—— ★ 只在对少数格取值时用 ✓"""
    b = blob[k]
    return b - 256 if b >= 128 else b


def band(v):
    if v < 0:
        return 'unknown'
    if v == 0:
        return 'free'
    if v >= 99:
        return 'lethal'
    return 'inflated'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--radius', type=float, default=0.20, help='残留格与点云的匹配半径（米）')
    ap.add_argument('--stable-from', type=float, default=0.5,
                    help='"稳定残留"从**整段的这个比例**开始算（默认后半段）')
    ap.add_argument('--overwrite', action='store_true',
                    help='★ 允许覆盖已存在的 analysis.json／residue_cells.csv ✓（默认**拒绝** ✗）')
    ap.add_argument('--mark-window', type=int, default=1,
                    help='★ 归因时取"第一次标上"那一拍 **±N 拍**的点云（默认 1 ⇒ ±2 s ✓）')
    a = ap.parse_args()
    run = a.run
    ticks = load_ticks(run)
    if len(ticks) < 4:
        print('✗ tick 太少（%d）⇒ ★ 记成"未取到"，✗ 不记成"没有残留" ✗' % len(ticks))
        return 2
    info = ticks[-1]['map']
    W, H = info['width'], info['height']
    N = W * H

    L = []
    skipped = 0
    for t in ticks:
        if '_occ' not in t:             # ★ 探针刚起、还没收到图的那几拍 ⇒ ✗ 跳过（别崩 ✓）
            skipped += 1
            continue
        L.append({'seq': t['seq'], 'sim_t': t['sim_t'],
                  'robot': t.get('robot_xy_map'),
                  'scan_n': (t.get('scan') or {}).get('n_kept', 0),
                  'cam_n': (t.get('cam') or {}).get('n_kept', 0),
                  'o': t['_occ'], 'b': t['_blob'],
                  'org': {k: (((t.get(k) or {}).get('tf') or {}).get('M') and
                              tuple((t.get(k) or {})['tf']['M'][r][3] for r in range(3)))
                          for k in ('scan', 'cam')}})
    if skipped:
        print('  ★ 跳过 %d 拍（★ 那时还没收到全局图 ⇒ ✗ 不是"没有残留" ✗）' % skipped)

    i1 = int(len(L) * 0.33)
    i2 = len(L) - 1
    # ★★ 2026-10-11 实测修正 ✗：★ "33% 那拍"可能落在**图还是空的**时刻上 ✗ ★
    #   （实测 `campus9-res1`：第 27 拍 sim_t=139.5 时**占用 = 0** ✗ ⇒ ★ 拿它当 t1
    #    ⇒ "两时刻都在占用 = 0" ✗ ⇒ ★ 会被读成"没有持久障碍" ✗ —— ★ 而真相是**那一拍是空图** ✓）
    #   ⇒ ★ 两条：★ ① 每拍占用数**印出来** ✓（空图可见 ✓）；
    #     ★ ② t1 取"33% 之后**第一拍非空**" ✓，★ 并把"改过没改过"明说 ✓。
    occ_series = [len(t['o']) for t in L]
    empties = [t['seq'] for t, o in zip(L, occ_series) if o == 0]
    while i1 < i2 and occ_series[i1] == 0:
        i1 += 1
    # ★★★ 2026-10-11（reviewer 要求**落成断言** ✓）：★ **冷启动段占用恒为 0** ✗ ★★★
    #   ★ 依据：★ 这一步我**栽过两次** ✗（★ 第 1 步：t1 落在空图拍 ⇒ 报"两时刻都在占用 = 0" ✗；
    #     ★ 第 2 步：用"从起点起每拍都占用"⇒ ★ 两轮**都恒为 0** ✗ ⇒ ★ 空读数被当成"没残留" ✗）。
    #   ★ 光写进 note **不够** ✓（★ 第二次照样踩 ✓）⇒ ★ 落成**断言** ✓：
    #     · 任何"起点类"窗口**必须从第 `first_nonempty` 拍之后开始** ✓
    #     · 若窗口起点落在冷启动段里 ⇒ ★ **直接报错退出** ✗（✗ 不给一个恒 0 的读数 ✗）
    #     · 且把冷启动段长度**写进产物** ✓（下游一眼看得出 ✓）
    i_start = int(len(L) * a.stable_from)
    first_nonempty = next((i for i, o in enumerate(occ_series) if o > 0), None)
    if first_nonempty is None:
        print('✗ **全程占用恒为 0** ✗ ⇒ ★ 记成"未取到"（✗ 不许落成"没有残留" ✗）')
        return 3
    if i_start < first_nonempty:
        print('✗ **"起点类"窗口落在了冷启动段里** ✗：i_start=%d < first_nonempty=%d（sim_t %.1f）'
              % (i_start, first_nonempty, L[first_nonempty]['sim_t']))
        print('  ⇒ ★ 这样算出来的"持久量"**必然恒 0** ✗（★ 那是**空图拍**，✗ 不是"没有障碍" ✗）')
        print('  ⇒ ★ 拒绝给数 ✓（★ 请把窗口起点挪到第 %d 拍之后 ✓）' % first_nonempty)
        return 3
    t1, t2 = L[i1], L[i2]
    print('=== ① 全局图残留 ===')
    print('  tick 数 = %d（sim_t %.1f → %.1f）｜ 栅格 %d×%d @%.3f m'
          % (len(L), L[0]['sim_t'], L[-1]['sim_t'], W, H, info['resolution']))
    print('  ★ 每拍占用格数：首 %d ｜ 末 %d ｜ ★ **空图拍（占用=0）%d 个**%s'
          % (occ_series[0], occ_series[-1], len(empties),
             ('（★ 它们是**空图**，✗ 不是"没有障碍" ✗ ⇒ t1 已避开：' +
              '第 %d 拍 sim_t=%.1f ✓）' % (t1['seq'], t1['sim_t'])) if empties else ' ✓'))

    # ①a 两时刻差分
    two = sorted(t1['o'] & t2['o'])
    # ①b 稳定残留：从 i_start 起，**每一拍都 >0**（一次都没回到 free/unknown）
    resid = set(L[i_start]['o'])
    for i in range(i_start + 1, len(L)):
        resid &= L[i]['o']            # ★ 交集 = "**每一拍都占用**" ✓
    resid = sorted(resid)
    print('  ★ 两时刻都在占用 = **%d** 格（t1=第 %d 拍 sim_t=%.1f；t2=末拍 sim_t=%.1f）'
          % (len(two), t1['seq'], t1['sim_t'], t2['sim_t']))
    print('  ★ **稳定残留**（后半段每拍都占用）= **%d** 格（从第 %d 拍 sim_t=%.1f 起）'
          % (len(resid), L[i_start]['seq'], L[i_start]['sim_t']))
    # ★★ 2026-10-11 加：★ **停留时长本身是关键量** ✗ ★
    #   （实测：★ 末 40 拍稳定格 = 0 ✗、末 20 拍 = 399 ✓、末 10 拍 = 582 ✓ ⇒
    #    ★ 只看"后半段全占用"那一个口径会把"能活几十秒的格"全判掉 ✗）
    #   ⇒ ★ 按**多个窗口**分别报 ✓，★ 并报"末拍占用格的**寿命分布**" ✓。
    print('  ★ 按窗口分别看（末 N 拍里**每拍都占用**的格数 ✓）：')
    win_rows = []
    for w in (3, 5, 10, 20, 40, 60):
        if len(L) < w:
            continue
        acc = set(L[-w]['o'])
        for t in L[-w + 1:]:
            acc &= t['o']
        n = len(acc)
        win_rows.append({'window_ticks': w, 'window_s': w * 2, 'cells': n})
        print('     末 %2d 拍（≈%3d s）：★ **%d** 格' % (w, w * 2, n))
    # ★ 末拍占用格的"寿命"（★ 从末拍往前连续占用的拍数 ✓）
    life = {}
    for k in t2['o']:
        c = 0
        for t in reversed(L):
            if k in t['o']:
                c += 1
            else:
                break
        life[k] = c
    if life:
        ls = sorted(life.values())
        print('  ★ 末拍占用 %d 格 ｜ ★ 连续占用拍数：min %d ｜ 中位 %d ｜ max %d ｜ '
              '★ ≥5 拍(10 s) 的 %d 个 ｜ ★ ≥60 拍(120 s) 的 %d 个'
              % (len(ls), ls[0], ls[len(ls) // 2], ls[-1],
                 sum(1 for v in ls if v >= 5), sum(1 for v in ls if v >= 60)))
    else:
        print('  ★ 末拍占用 = 0（★ 那是**空图** ⇒ ✗ 不许落成"没有残留" ✗）')
    print('    其中末拍致命(>=99) = %d、膨胀(1..98) = %d'
          % (sum(1 for k in resid if val(t2['b'], k) >= 99),
             sum(1 for k in resid if 0 < val(t2['b'], k) < 99)))

    # ④ 正向对照：点云落在各档代价上的分布
    print()
    print('=== ④ 正向对照（★ 点云的点落在全局图各档上的分布 ✓）===')
    ctrl = {}
    for kind in ('scan', 'cam'):
        agg = {'free': 0, 'inflated': 0, 'lethal': 0, 'unknown': 0, 'outside': 0}
        npts = 0
        for t in L[-6:]:
            pts = load_xyz(run, t['seq'], kind)
            npts += len(pts)
            for (x, y, z) in pts:
                gx, gy = cell_of(info, x, y)
                if not (0 <= gx < W and 0 <= gy < H):
                    agg['outside'] += 1
                    continue
                agg[band(val(t['b'], gy * W + gx))] += 1
        ctrl[kind] = agg
        tot = max(1, npts)
        occ = agg['inflated'] + agg['lethal']
        print('  /rtk/%-17s 点 %6d ｜ ★ 落**占用格** %5d（%.1f%%）｜ free %5d（%.1f%%）｜ unknown %5d（%.1f%%）｜ 出图 %d'
              % ({'scan': 'scan_points', 'cam': 'camera/points'}[kind], npts, occ,
                 100.0 * occ / tot, agg['free'], 100.0 * agg['free'] / tot,
                 agg['unknown'], 100.0 * agg['unknown'] / tot, agg['outside']))
    ok_align = (ctrl['scan']['inflated'] + ctrl['scan']['lethal']) > 0 and \
               (ctrl['scan']['free'] + ctrl['scan']['unknown']) > 0
    print('  ★ 对齐自检 = %s（★ 两类都要出现才算"尺子两头都动过" ✓）'
          % ('✓ 成立' if ok_align else '✗ 不成立 ⇒ ★ 下面的归因不算数 ✗'))

    # ② 归因 ＋ ③ 高度
    print()
    print('=== ② 归因 + ③ 高度（★ 匹配半径 %.2f m，xy）===' % a.radius)
    # ★★ 2026-10-11 实测修正 ✗★★：★ 归因必须用"**该格第一次被标上**的那一刻"的点云 ✓
    #   ★ 我第一版拿**最后 4 拍**去配 ✗ ⇒ ★ 车早开走了 ⇒ 那些"很久前标上、一直没清"的格
    #     必然配不到任何点 ✗ ⇒ ★ 报出 **none 100%** ✗ —— ★ 那是**方法错**，✗ 不是"没有源" ✗
    #     （★ 实测 `campus9-res1`：1997 个稳定残留格全被报成 none ✗，而正向对照明明成立 ✓）。
    #   ⇒ ★ 改成：★ 每格取它**第一次出现**的那一拍（±1 拍 ✓）的点云 ✓；
    #     ★ 并同时记"**那一刻车在哪**" ✓（★ 用来分"自身还是地面" ✓）。
    pts_cache = {}
    first_occ = {}
    for idx, t in enumerate(L):
        for k in t['o']:
            if k not in first_occ:
                first_occ[k] = idx

    def cloud(idx, kind):
        key = (L[idx]['seq'], kind)
        if key not in pts_cache:
            # ★★ 2026-10-11：★ 必须建**空间索引** ✗ —— 第一版是"2000 格 × 3 拍 × 23 万点"
            #   ⇒ ★ 14 亿次比较 ⇒ **跑不完** ✗（实测 6 组敏感性**卡死** ✓）。
            #   ★ 按 0.2 m 分桶 ✓ ⇒ 每格只查邻近 9 个桶 ✓。
            buckets = {}
            for (x, y, z) in load_xyz(run, L[idx]['seq'], kind):
                buckets.setdefault((int(x / 0.2), int(y / 0.2)), []).append((x, y, z))
            pts_cache[key] = buckets
        return pts_cache[key]

    # ★★ 2026-10-11：★ 归因必须**按该源自己的 z 门与距离门过滤** ✗ ★★
    #   ★ 依据（★ 运行时参数 ✓，✗ 不猜 ✓）：★ `scan` `min_obstacle_height 0.02`／`obstacle_max_range 7.5`
    #     ／`max_obstacle_height 1.2`；★ `camera_cloud` `min_obstacle_height **0.3**`／`obstacle_max_range 6.0` ✓。
    #   ★ 为什么必须过滤 ✗：★ 不过滤的话，"附近有个点"**不等于**"那个点把这个格标脏了" ✗ ——
    #     ★ 实测：★ cam 命中点 z 中位 **0.016 m** ✗ 而它自己的门是 **0.3 m** ⇒ ★ 那些点**进不了图** ✗
    #     ⇒ ★ 不过滤就会把"相机"报成来源 ✗（★ 邻近 ≠ 因果 ✓）。
    GATE = {'scan': (0.02, 1.2, 7.5), 'cam': (0.3, 1.2, 6.0)}

    def nearest(buckets, wx, wy, kind, org):
        zmin, zmax, rmax = GATE[kind]
        bx, by = int(wx / 0.2), int(wy / 0.2)
        best, bz = 1e9, None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for (x, y, z) in buckets.get((bx + dx, by + dy), ()):
                    if not (zmin <= z <= zmax):
                        continue                      # ★ 过不了 z 门 ⇒ 它标不了这个格 ✓
                    if org is not None and math.dist((x, y, z), org) > rmax:
                        continue                      # ★ 过不了距离门 ✓
                    d = math.hypot(x - wx, y - wy)
                    if d < best:
                        best, bz = d, z
        return best, bz

    rows = []
    agg = {'scan': 0, 'cam': 0, 'both': 0, 'none': 0}
    zs = {'scan': [], 'cam': []}
    dcar = {'scan': [], 'cam': []}
    for k in resid:
        gx, gy = k % W, k // W
        wx, wy = world_of(info, gx, gy)
        idx = first_occ.get(k, len(L) - 1)
        hit = {}
        for kind in ('scan', 'cam'):
            best, bz = 1e9, None
            for j in {max(0, min(len(L) - 1, idx + d))
                      for d in range(-a.mark_window, a.mark_window + 1)}:
                db, zb = nearest(cloud(j, kind), wx, wy, kind, L[j]['org'][kind])
                if db < best:
                    best, bz = db, zb
            hit[kind] = (best, bz)
        s_ok = hit['scan'][0] <= a.radius
        c_ok = hit['cam'][0] <= a.radius
        cls = 'both' if (s_ok and c_ok) else ('scan' if s_ok else ('cam' if c_ok else 'none'))
        agg[cls] += 1
        rob = L[idx].get('robot')
        dc = (math.hypot(wx - rob[0], wy - rob[1]) if rob else None)
        for kind, ok in (('scan', s_ok), ('cam', c_ok)):
            if ok:
                zs[kind].append(hit[kind][1])
                if dc is not None:
                    dcar[kind].append(dc)
        rows.append({'i': gx, 'j': gy, 'wx': round(wx, 3), 'wy': round(wy, 3),
                     'cost_end': val(t2['b'], k), 'cost_t1': val(t1['b'], k),
                     'first_occ_tick': L[idx]['seq'], 'first_occ_sim_t': round(L[idx]['sim_t'], 1),
                     'd_car_at_mark': None if dc is None else round(dc, 3),
                     'src': cls,
                     'd_scan': None if hit['scan'][0] > 1e8 else round(hit['scan'][0], 3),
                     'z_scan': None if hit['scan'][1] is None else round(hit['scan'][1], 3),
                     'd_cam': None if hit['cam'][0] > 1e8 else round(hit['cam'][0], 3),
                     'z_cam': None if hit['cam'][1] is None else round(hit['cam'][1], 3)})
    tot = max(1, len(resid))
    print('  ★ 归因（★ 取**该格第一次被标上**那一刻的点云 ✓）：'
          'scan %d（%.1f%%）｜ cam %d（%.1f%%）｜ **both** %d（%.1f%%）｜ none %d（%.1f%%）'
          % (agg['scan'], 100.0 * agg['scan'] / tot, agg['cam'], 100.0 * agg['cam'] / tot,
             agg['both'], 100.0 * agg['both'] / tot, agg['none'], 100.0 * agg['none'] / tot))
    for kind in ('scan', 'cam'):
        z = sorted(v for v in zs[kind] if v is not None)
        d = sorted(dcar[kind])
        if z:
            print('  ★ %s 命中点**高度**（map 帧 z，米）：min %.3f ｜ 中位 %.3f ｜ max %.3f ｜ n=%d'
                  % (kind, z[0], z[len(z) // 2], z[-1], len(z)))
            print('       ★ 标上那一刻**离车**：min %.2f ｜ 中位 %.2f ｜ max %.2f m'
                  % (d[0], d[len(d) // 2], d[-1]) if d else '')
        else:
            print('  ★ %s 命中点：n=0（★ 记成"未命中"，✗ 不记成"高度为 0" ✗）' % kind)

    # ③a 车静止时还生不生成新格
    print()
    print('=== ③a 车静止时还会不会**新增**占用格 ===')
    moves = []
    for i in range(1, len(L)):
        r0, r1 = L[i - 1]['robot'], L[i]['robot']
        if r0 and r1:
            moves.append((L[i]['seq'], math.hypot(r1[0] - r0[0], r1[1] - r0[1])))
    still = [s for s, d in moves if d < 0.02]
    print('  ★ 位移 <0.02 m 的"静止拍" = %d 拍（总 %d 拍）' % (len(still), len(moves)))
    if len(still) >= 3:
        idx = [idx for idx, (s, d) in enumerate(moves) if d < 0.02][-6:]
        print('  ★ 末段静止拍里，**每拍新增**的占用格数：')
        for i in idx:
            new = sorted(L[i + 1]['o'] - L[i]['o'])     # ★ 集合差 = **新变占用的格** ✓
            r = L[i + 1].get('robot')
            dm = min((math.hypot(world_of(info, k % W, k // W)[0] - r[0],
                                 world_of(info, k % W, k // W)[1] - r[1]) for k in new),
                     default=None) if (new and r) else None
            print('     拍 %d（sim_t %.1f）：新增 %d 格 ｜ ★ 最近的一格离车 %s m'
                  % (L[i + 1]['seq'], L[i + 1]['sim_t'], len(new),
                     '—' if dm is None else '%.2f' % dm))
    else:
        print('  ★ 静止拍不足 ⇒ ★ 记成"未取到" ✗（✗ 不许落成"静止时不生成" ✗）')

    # 残留格离车轨迹多远
    traj = [(t['robot'][0], t['robot'][1]) for t in L if t.get('robot')]
    dmins = []
    for k in resid:
        wx, wy = world_of(info, k % W, k // W)
        dmins.append(min(math.hypot(wx - x, wy - y) for (x, y) in traj))
    track_stats = None
    if dmins:
        d = sorted(dmins)
        n = len(d)
        q = lambda p: d[min(n - 1, int(n * p))]
        track_stats = {'n': n, 'min': round(d[0], 3), 'p10': round(q(0.10), 3),
                       'p25': round(q(0.25), 3), 'median': round(d[n // 2], 3),
                       'p75': round(q(0.75), 3), 'max': round(d[-1], 3),
                       'frac_ge_3m': round(sum(1 for v in d if v >= 3.0) / float(n), 4),
                       'frac_lt_1m': round(sum(1 for v in d if v < 1.0) / float(n), 4)}
        print('  ★ 残留格到**车轨迹**的距离：min %.2f ｜ p10 %.2f ｜ ★ **中位 %.2f** ｜ p75 %.2f ｜ max %.2f m'
              % (d[0], q(0.10), d[n // 2], q(0.75), d[-1]))
        print('     ★ 其中 ★ **≥3 m 的占 %.1f%%** ✓ ｜ ★ **<1 m 的占 %.1f%%** ✗'
              % (100.0 * track_stats['frac_ge_3m'], 100.0 * track_stats['frac_lt_1m']))
        print('     ★ （★ 这一条是"**清除射线照不到 ⇒ 清不掉**"那条结论的**主证据** ✓ ⇒ ★ 必须进产物 ✓）')

    # ★★★ 2026-10-11（reviewer 报的真缺陷 ✗）：★ 本工具是"分析"✗，却会**覆盖已验收产物** ✗ ★★★
    #   ★ 实测踩到 ✓：★ reviewer 为做标定跑了一次 ✓ ⇒ ★ 把 `res2/probe/` 那两份**覆盖掉** ✗
    #     （★ `c0118a71…` → `ab084a69…` ✗），★ 而"我验过的那一版"就此**对不上产物** ✗。
    #   ★ 同族 ✓：★ "**产物先算完再落盘、别截断已有证据**" ✓、★ "处置类动作要挂结构护栏" ✓。
    #   ⇒ ★ 默认**拒绝覆盖** ✓（要覆盖必须显式 `--overwrite` ✓）—— ★ 落盘前先看目标 ✓。
    out_dir = os.path.join(run, 'probe')
    _targets = [os.path.join(out_dir, 'residue_cells.csv'),
                os.path.join(out_dir, 'analysis.json')]
    _exist = [t for t in _targets if os.path.exists(t)]
    if _exist and not a.overwrite:
        print('✗ **拒绝覆盖已存在的产物** ✗（★ 本工具是分析器，✗ 不该悄悄改写证据 ✓）：')
        for t in _exist:
            print('    ★ 已存在：%s（%d 字节）' % (t, os.path.getsize(t)))
        print('  ⇒ ★ 要看新结果：★ 加 `--overwrite` ✓（★ 或先把它挪走 ✓）')
        print('  ⇒ ★ 依据：★ "我验过的那一版"必须能**与产物对上** ✓（✗ 覆盖了就永远对不上 ✗）')
        return 4
    with open(os.path.join(out_dir, 'residue_cells.csv'), 'w', newline='', encoding='utf-8') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()) if rows else
                           ['i', 'j', 'wx', 'wy', 'cost_end', 'cost_t1', 'src',
                            'd_scan', 'z_scan', 'd_cam', 'z_cam'])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    summary = {'ticks': len(L), 'grid': info,
               'two_time_cells': len(two), 'stable_residue_cells': len(resid),
               'attribution': agg, 'positive_control': ctrl, 'align_ok': ok_align,
               'radius': a.radius, 'stable_from_frac': a.stable_from,
               'mark_window_ticks': a.mark_window,
               'track_distance_m': track_stats,
               # ★★ 2026-10-11（reviewer ①）：★ **两个距离是不同的量** ✗，别混 ✓ ——
               #   ★ `d_car_at_mark` = **标上该格那一刻**车离它多远 ✓（★ 用来分"自身还是地面" ✓）
               #   ★ `track_distance_m` = 该格到**整条轨迹**的最近距离 ✓（★ 用来判"车是否已开过" ✓）
               'd_car_at_mark_m': (lambda v: None if not v else {
                   'n': len(v), 'min': round(v[0], 3), 'p10': round(v[int(len(v) * .10)], 3),
                   'median': round(v[len(v) // 2], 3), 'p75': round(v[int(len(v) * .75)], 3),
                   'max': round(v[-1], 3),
                   'frac_ge_3m': round(sum(1 for x in v if x >= 3.0) / float(len(v)), 4),
                   'frac_lt_1m': round(sum(1 for x in v if x < 1.0) / float(len(v)), 4)})(
                       sorted(float(r['d_car_at_mark']) for r in rows if r['d_car_at_mark'])),
               'attribution_z': {k: {'n': len(v), 'median': (sorted(x for x in v if x is not None)[len(v) // 2]
                                                             if v else None)}
                                 for k, v in zs.items()},
               'attribution_d_car_m': {k: {'n': len(v), 'median': (sorted(v)[len(v) // 2] if v else None)}
                                       for k, v in dcar.items()}}
    with open(os.path.join(out_dir, 'analysis.json'), 'w', encoding='utf-8') as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=2)
    print()
    print('  ★ 已落盘 probe/residue_cells.csv（%d 行）＋ probe/analysis.json' % len(rows))
    return 0


if __name__ == '__main__':
    sys.exit(main())

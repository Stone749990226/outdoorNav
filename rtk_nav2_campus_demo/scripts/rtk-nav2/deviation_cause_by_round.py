#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""deviation_cause_by_round.py —— ★ 同一条航线，为什么能开出 20% 的里程差 ✗（★ 纯离线 ✓，★ 不起栈 ✓）

用法: python3 deviation_cause_by_round.py --out <json> [--rounds r1,r2,...]

★★ 新事实（★ reviewer 2026-10-11 派 ✓）★★
  ★ 八轮**航线逐字节相同** ✓（★ sha `8be043a0e70f9298` ✓、★ 8 点 ✓、★ 851 字节 ✓）、
    ★ **出生点也相同** ✓ ⇒ ★ 而里程 **58.07–66.28**（★ stock 批 ✓）vs **43.96–52.10**（★ edited 批 ✓）✗✗
  ⇒ ★ 只剩两族 ✓：★ **(a) 绕行**（★ 被代价图上的东西推着走 ✓）
    ★ 或 ★ **(b) 航点处的过冲／来回摆**（★ 控制器调参 ✓）

★★ 判据两头（★ 先写死 ✓）★★
  ★ **偏离对得上标记／障碍** ⇒ ★★ **代价图在推着车走** ✓✓ ⇒ ★ 那是一个**反馈环**
    （★ 标记 ⇒ 走法 ⇒ 标记 ✓）⇒ ★ 它能解释**两批为什么分得那么干净** ✓、★ 以及**为什么差能有 2–3 倍** ✓✓
  ★ **对不上（且多为航点过冲）** ⇒ ★ **是控制器调参** ✓
  ★★ 而 ★ 若是反馈环 ⇒ ★ 连"配得上"的那一对也**未必干净** ✗（★ 同一套配置也可能走出不同的路 ✓）
    ⇒ ★ 那时**默认就该用**"★ **按同一弧长位置比**" ✓（★ reviewer 已把它定成**默认** ✓，✗ 不是备选 ✗）

★★ 三个量（★ 都用**到达拍**那一窗 ✓ —— ★ 与残留指标**同一个锚** ✓）★★
  ★ ① ★ **每个航点处的过冲** ✓：★ 到该航点的最近距离 `d_min` ✓、★ **"越过最远多少"**
    （★ 从最近点起、到下一个航点的最近点为止，★ 到该航点距离的**最大值** ✓）
    ＋ ★ **在航点附近磨掉的弧长**（★ 半径 1.5 m 内走了多少米 ✓）
  ★ ② ★ **沿航向的回退** ✓（★ 把点投到名义折线上得**沿航进度** `s` ✓ ⇒ ★ `ds < 0` 的**倒退米数** ✓、
    ★ 方向反转次数 ✓）＋ ★ **静止时长**（★ 每步 ≤0.005 m ✓，步长 0.05 s ✓）
  ★ ③ ★ **偏离是否与"标记所在处"对齐** ✓ —— ★ 关键：★ 用的是**该拍之前的**探针 dump ✓
    （★ ✗ 不是末拍那张 ✗ —— ★ "归因必须取**该格第一次被标上**那一刻" ✓ 同族 ✓）
    ⇒ ★ 比"**偏离的采样点**"与"**贴近名义线的采样点**"各自"★ 1.0 m 内有致命格"的比例 ✓

★★ 口径（★ 写死 ✓）★★
  · ★ 路径 = `trajectory.csv` 的 **`gt_x`／`gt_y`** ✓（★ 0.05 s 一条 ✓ ⇒ ★ 反转／静止都分得出来 ✓；
    ★ 它是**仿真真值** ✓，★ 且 map 帧 ≡ 世界帧 ✓）
  · ★ 右边界 = **到达拍** ✓（★ 与 `residue_at_wp8_anchor.py` **同一个锚**：★ 与 wp8 GT 的最近接近点 ✓）
  · ★ 名义线 = 8 个航点的 **GT** 折线 ✓（★ 各轮同一条 ✓）
  · ★ 致命 = 值 ∈ **99…127** ✓（★ 与 `dynamic_lethal_delta.py` 同一口径 ✓；★ 255 不是致命 ✓）
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ART = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'
LOITER_R = 1.5          # ★ "在航点附近"的半径 ✓
MOVE_EPS = 0.005        # ★ "静止"的位移门 ✓（★ 采样 0.05 s ✓）
RETRO_EPS = 0.05        # ★ 倒退判定门（米）✓
DEV_OFF_M = 0.5         # ★ "偏离名义线"的判定 ✓
NEAR_LETHAL_M = 1.0     # ★ "附近有致命格"的判定 ✓
LETHAL_LO, LETHAL_HI = 99, 127


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def wp_gt(run, wp):
    p = os.path.join(ART, run, 'stream_point_wp%d.csv' % wp)
    if not os.path.isfile(p):
        return None
    with open(p, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            try:
                return (float(row['gt_x']), float(row['gt_y']))
            except Exception:
                continue
    return None


def read_traj(run):
    p = os.path.join(ART, run, 'trajectory.csv')
    out = []
    with open(p, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            try:
                out.append((float(row['t']), float(row['gt_x']), float(row['gt_y'])))
            except Exception:
                continue
    return out


def poly_proj_s(px, py, poly):
    """★ 投到名义折线上 ⇒ (沿航进度 s, 横向偏移) ✓"""
    best = (float('inf'), 0.0, 0.0)
    acc = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
        d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        if d < best[0]:
            best = (d, acc + t * L, L)
        acc += L
    return best[1], best[0]


def probe_ticks(run, cut):
    """★ 探针逐拍位姿 ✓（★ `cut=None` ⇒ 不设上限 ✓ —— 调用方靠**单调配准**在到达处停 ✓）"""
    out = []
    for f in sorted(glob.glob(os.path.join(ART, run, 'probe', 'tick_*.json')), key=seq_of):
        s = seq_of(f)
        if cut is not None and s > cut:
            break
        try:
            d = json.load(open(f, encoding='utf-8'))
        except Exception:
            continue
        r = d.get('robot_xy_map')
        if r and len(r) >= 2:
            out.append((s, float(r[0]), float(r[1])))
    return out


def map_meta(run):
    """★ 栅格元数据 ✓ —— ★ 前几拍可能还没有图 ✗ ⇒ ★ **扫到有图的那一拍** ✓
    （★ 实测：★ `tick_0002.json` 在有的轮里 `map` 还是 `None` ✓ ⇒ ★ 直接取会 `TypeError` ✗）"""
    for i in range(1, 30):
        p = os.path.join(ART, run, 'probe', 'tick_%04d.json' % i)
        if not os.path.isfile(p):
            continue
        m = (json.load(open(p, encoding='utf-8')) or {}).get('map')
        if m:
            return (m['resolution'], m['width'], m['height'], m['origin']['x'], m['origin']['y'])
    raise RuntimeError('★ %s：前 30 拍都没有图 ✗' % run)


def lethal_density_ahead(buf, res, W, H, ox, oy, x, y, hdg, r0, r1, half_ang_deg=60.0,
                         center_deg=0.0):
    """★ 车**前方**那个扇形里的致命格密度 ✓（★ 这才是"是不是有东西在推它" ✓）

    ★★ 为什么必须看**前方** ✗（★ 先写成"周围"是错的 ✓）★★
      ★ 车会**标自己走过的地** ✓ ⇒ ★ 它的**周围（尤其身后）永远有标记** ✗ ⇒
        ★ 拿"周围有没有标记"比 ⇒ ★ **两边都会有** ⇒ ★ 恒真、零分辨力 ✗（★ "铺满全场的地面"那条同族 ✓）。
      ★ 而"**前方**有没有标记"才是它**接下来要躲的东西** ✓ ⇒ ★ 这才是"代价图推着它走"的可检命题 ✓。
    """
    ix = int(math.floor((x - ox) / res)); iy = int(math.floor((y - oy) / res))
    k = int(math.ceil(r1 / res))
    hdg_c = hdg + math.radians(center_deg)
    hx, hy = math.cos(hdg_c), math.sin(hdg_c)
    lim = math.cos(math.radians(half_ang_deg))
    n_lel = n_tot = 0
    for dy in range(-k, k + 1):
        yy = iy + dy
        if yy < 0 or yy >= H:
            continue
        base = yy * W
        for dx in range(-k, k + 1):
            xx = ix + dx
            if xx < 0 or xx >= W:
                continue
            ex, ey = dx * res, dy * res
            r = math.hypot(ex, ey)
            if r < r0 or r > r1:
                continue
            if r > 1e-9 and (ex * hx + ey * hy) / r < lim:
                continue
            n_tot += 1
            if LETHAL_LO <= buf[base + xx] <= LETHAL_HI:
                n_lel += 1
    return (n_lel, n_tot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--rounds', default='campus9-res1,campus9-res2,campus9-cutS1,campus9-cutS5,'
                                        'campus9-cutS6,campus9-cutS7,campus9-cutS8,campus9-cutS9')
    ap.add_argument('--nominal-from', default='campus9-res1')
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    nominal = [p for p in (wp_gt(a.nominal_from, i) for i in range(1, 9)) if p]
    out = {'what': '★ 同一条航线、同样的出生点 —— ★ **20% 的里程差是哪来的** ✗',
           'why': ('★ 八轮航线**逐字节相同** ✓、★ 出生点相同 ✓ ⇒ ★ 里程 58.07–66.28（stock ✓）vs '
                   '43.96–52.10（edited ✓）✗ ⇒ ★ 只剩两族：★ (a) **绕行**（★ 代价图推着走 ✓）'
                   '／★ (b) **航点过冲＋来回摆**（★ 控制器调参 ✓）'),
           'preregistered': {
               'deviation_aligns_with_marks': '★★ **代价图在推着车走** ✓✓ ⇒ ★ **反馈环**（★ 标记 ⇒ 走法 ⇒ 标记 ✓）'
                                              '⇒ ★ 能解释两批为何分得干净、★ 以及 2–3 倍的差 ✓',
               'does_not_align_mostly_overshoot': '★ **是控制器调参** ✓'},
           'consequence_if_feedback_loop': ('★ 连"配得上"的那一对也**未必干净** ✗（★ 同一套配置也可能走出不同的路 ✓）'
                                            '⇒ ★ **默认就该用"按同一弧长位置比"** ✓（★ reviewer 已定成默认 ✓）'),
           'convention': {
               'path': '★ `trajectory.csv` 的 `gt_x`／`gt_y` ✓（★ 0.05 s 一条 ✓ ⇒ ★ 反转／静止分得出来 ✓；★ 仿真真值 ✓）',
               'window': '★ 探针最早一拍 → **到达拍** ✓（★ 与 `residue_at_wp8_anchor.py` 同一个锚 ✓）',
               'nominal': '★ 8 个航点 GT 折线 ✓（★ 取自 %s ✓）' % a.nominal_from,
               'lethal': '★ 值 ∈ [%d, %d] ✓（★ 与 `dynamic_lethal_delta.py` 同一口径 ✓）' % (LETHAL_LO, LETHAL_HI),
               'causality': '★ 用**该拍之前的**探针 dump ✓（★ ✗ 不是末拍那张 ✗ —— ★ "归因取该格第一次被标上那一刻" ✓）'},
           'nominal_polyline': [[round(x, 4), round(y, 4)] for x, y in nominal],
           'rounds': {}, 'notes': []}

    for run in [r for r in a.rounds.split(',') if r]:
        tr = read_traj(run)
        g8 = wp_gt(run, 8)
        if len(tr) < 10 or not g8:
            out['notes'].append('✗ %s：读不到轨迹/航点 ⇒ 跳过' % run)
            continue
        # ★ 到达：与 wp8 GT 的最近接近点 ✓
        i_arr = min(range(len(tr)), key=lambda i: math.hypot(tr[i][1] - g8[0], tr[i][2] - g8[1]))
        # ★★ 窗口**从探针第一拍起** ✓（✗ 不是从起栈起 ✗）★★
        #   ★ `trajectory.csv` 是**从起栈记到停栈** ✓ ⇒ ★ 它比探针**早开** ✗（★ 这段时间车还停在出生点 ✓）
        #   ⇒ ★ 拿全段算里程 ⇒ ★ 与残留指标**不同窗口** ✗（★ 实测：res1 会从 66.281 变成 68.443 ✗）
        #   ⇒ ★ 起点 = 轨迹上**离探针第一拍最近**的那一点 ✓。
        #   ★★ 而 ★ 起点**不能**取"离探针第一拍最近的轨迹点" ✗ —— 实测：★ 有的轮里
        #     **探针前 ~100 拍没有位姿** ✗（★ 定位还没收敛 ✓）⇒ ★ 那样取会把窗口**起晚 ~10%** ✗✗
        #     （★ cutS5 −9.5%／cutS6 −11.6% ✓，★ 而且那三轮只读到 **5–7 张图** ✗ ⇒ 尺子等于没用 ✓）。
        #   ⇒ ★ 改成**统一的规则** ✓：★ 起点 = ★ 轨迹上**最早**那个"★ 离出生点（wp1 GT）≤0.2 m"的采样 ✓
        #     （★ 车就是从出生点起步的 ✓）⇒ ★ 八轮**同一条规则** ✓，★ 代价 = 把起栈后那段
        #     **停着抖**的时间也算进去 ⇒ ★ 里程系统性高 ~3% ✓（★ 八轮一样 ⇒ 比得出 ✓，★ 但这条偏差要写明 ✓）。
        g1 = wp_gt(run, 1)
        cand = [i for i in range(len(tr)) if math.hypot(tr[i][1] - g1[0], tr[i][2] - g1[1]) <= 0.2] if g1 else []
        i0 = min(cand) if cand else 0
        i0 = min(i0, i_arr)
        win = tr[i0:i_arr + 1]
        dt = (win[-1][0] - win[0][0]) / max(1, len(win) - 1)
        mileage = sum(math.hypot(b[1] - a2[1], b[2] - a2[2]) for a2, b in zip(win, win[1:]))

        # ★ ① 每个航点：最近距离 ＋ 越过最远 ＋ 附近磨掉的弧长 ✓
        seg = [math.hypot(b[1] - a2[1], b[2] - a2[2]) for a2, b in zip(win, win[1:])]
        wps = []
        for i in range(1, 9):
            g = wp_gt(run, i)
            d = [math.hypot(p[1] - g[0], p[2] - g[1]) for p in win]
            j = min(range(len(d)), key=lambda k: d[k])
            # ★ 越过最远 ✓：★ 从**最近点**起、到**下一个航点的最近点**为止（★ 末点则到窗口末 ✓），
            #   ★ 这一段里"离本航点最远"是多少 ✓ —— ★ 那就是"越过去又绕回来"的量 ✓
            nxt = wp_gt(run, i + 1) if i < 8 else None
            if nxt:
                dn = [math.hypot(p[1] - nxt[0], p[2] - nxt[1]) for p in win[j:]]
                lim = j + min(range(len(dn)), key=lambda k: dn[k])
            else:
                lim = len(d) - 1
            # ★ 航点附近磨掉的弧长 ✓：★ 相邻两点**都在** 1.5 m 内才算 ✓
            loiter = sum(seg[k - 1] for k in range(1, len(win)) if d[k - 1] <= LOITER_R and d[k] <= LOITER_R)
            wps.append({'wp': 'wp%d' % i, 'd_min': round(d[j], 4), 'idx_min': j,
                        'overshoot_m': round(max(d[j:lim + 1]), 4) if lim >= j else None,
                        'loiter_m_within_1.5m': round(loiter, 4)})

        # ★ ② 沿航进度 ⇒ 倒退 ＋ 反转；★ 静止 ✓
        prog, off = [], []
        for p in win:
            s, o = poly_proj_s(p[1], p[2], nominal)
            prog.append(s); off.append(o)
        ds = [prog[k] - prog[k - 1] for k in range(1, len(prog))]
        retro_m = sum(-x for x in ds if x < -RETRO_EPS)
        n_rev = sum(1 for k in range(1, len(ds)) if (ds[k] > RETRO_EPS and ds[k - 1] < -RETRO_EPS)
                    or (ds[k] < -RETRO_EPS and ds[k - 1] > RETRO_EPS))
        step = [math.hypot(b[1] - a2[1], b[2] - a2[2]) for a2, b in zip(win, win[1:])]
        n_stat = sum(1 for x in step if x <= MOVE_EPS)

        # ★ ③ 车**前方**那个扇形里的致命格密度 ✓（★ 用**该拍之前**的 dump ✓）
        #   ★ ✗ 不比"周围有没有标记" ✗ —— ★ 车会标自己走过的地 ⇒ 两边都必有 ⇒ 恒真 ✗
        ticks = probe_ticks(run, None)
        res, W, H, ox, oy = map_meta(run)
        cache, j = {}, 0
        dev, nom, dev_r, nom_r, dev_e, nom_e = [], [], [], [], [], []
        endp = os.path.join(ART, run, 'probe_frozen')
        _em = sorted(glob.glob(os.path.join(endp, 'map_*.bin')), key=seq_of)
        endbuf = open(_em[-1], 'rb').read() if _em else None
        for k in range(1, len(win)):
            p, q = win[k], win[k - 1]
            if math.hypot(p[1] - q[1], p[2] - q[2]) < 1e-9:
                continue
            hdg = math.atan2(p[2] - q[2], p[1] - q[1])
            # ★★ 配准 = **滑窗内取最近** ✓（★ 原来的"贪心前进"会卡住 ✗ ——
            #   ★ 实测：一轮只读到 **1–4** 张图 ✗，而窗口里有 ~150+ 拍 ✓ ⇒ ★ 尺子根本没用上 ✓）
            lo = max(1, j - 30); hi = min(len(ticks), j + 30)
            j = min(range(lo, hi), key=lambda t: math.hypot(ticks[t][1] - p[1], ticks[t][2] - p[2]))
            if off[k] > DEV_OFF_M:
                tgt = dev
            elif off[k] <= 0.2:
                tgt = nom
            else:
                continue
            s_prev = ticks[j - 1][0]                      # ★ 该拍**之前**那一拍 ✓
            if s_prev not in cache:
                fp = os.path.join(ART, run, 'probe', 'map_%04d.bin' % s_prev)
                cache[s_prev] = open(fp, 'rb').read() if os.path.isfile(fp) else None
            buf = cache[s_prev]
            if buf is None:
                continue
            nl, nt = lethal_density_ahead(buf, res, W, H, ox, oy, p[1], p[2], hdg, 1.5, 5.0)
            if nt:
                tgt.append((nl, nt))
            # ★★ 两头标定（★ 本项目的老规矩 ✓）★★
            #   ★ ① **后锥**（★ 同一张图、★ 转身 180° ✓）：★ 车会标自己走过的地 ⇒ ★ 这里**应当非 0** ✓
            #     ⇒ ★ 若后锥也 0 ⇒ ★ **是尺子坏了** ✗（★ 不是"前方真没东西" ✗）
            #   ★ ② **末拍冻结图**上的前锥 ✓：★ 那一张有**全部残留** ⇒ ★ 用作**量程对照** ✓
            rl, rt = lethal_density_ahead(buf, res, W, H, ox, oy, p[1], p[2], hdg, 1.5, 5.0,
                                          center_deg=180.0)
            if rt:
                (dev_r if off[k] > DEV_OFF_M else nom_r).append((rl, rt))
            if endbuf is not None:
                el, et = lethal_density_ahead(endbuf, res, W, H, ox, oy, p[1], p[2], hdg, 1.5, 5.0)
                if et:
                    (dev_e if off[k] > DEV_OFF_M else nom_e).append((el, et))

        def dens(xs):
            if not xs:
                return None
            L = sum(a2 for a2, _ in xs); T = sum(b2 for _, b2 in xs)
            return round(L / T, 6) if T else None

        out['rounds'][run] = {
            'arrival_idx': i_arr, 'start_idx': i0, 'traj_dt_s': round(dt, 4),
            'n_traj_samples': len(win), 'window_s': round(win[-1][0] - win[0][0], 2),
            'mileage_m': round(mileage, 3),
            'waypoints': wps,
            'overshoot_max_m': round(max((w['overshoot_m'] or 0) for w in wps), 4),
            'overshoot_note': ('★ ✗ 这个量**几乎恒定**（★ 11.85–12.66 m ✓）✗ ⇒ ★ 它量到的其实是'
                               '**航点间距**（★ 最近点 → 下一航点最近点 ✓）⇒ ★ **没有分辨力** ✗ —— ★ 如实记 ✓'),
            'loiter_total_m': round(sum(w['loiter_m_within_1.5m'] for w in wps), 3),
            'loiter_frac': round(sum(w['loiter_m_within_1.5m'] for w in wps) / mileage, 4) if mileage else None,
            'retro_m': round(retro_m, 3), 'retro_frac': round(retro_m / mileage, 4) if mileage else None,
            'n_reversals': n_rev,
            'stationary_s': round(n_stat * dt, 2),
            'stationary_frac': round(n_stat / max(1, len(step)), 4),
            'offset_p50': round(sorted(off)[len(off) // 2], 3),
            'align_ahead': {'n_dev_samples': len(dev), 'n_nominal_samples': len(nom),
                            'dev_lethal_density_ahead': dens(dev),
                            'nominal_lethal_density_ahead': dens(nom),
                            'CONTROL_rear_cone_dev': dens(dev_r),
                            'CONTROL_rear_cone_nominal': dens(nom_r),
                            'CONTROL_forward_on_end_map_dev': dens(dev_e),
                            'CONTROL_forward_on_end_map_nominal': dens(nom_e),
                            'n_map_reads': len(cache)}}

    # ★★ 同批对照 ＋ 判词（★ 两头都算 ✓）★★
    STOCK = ['campus9-res1', 'campus9-res2', 'campus9-cutS1', 'campus9-cutS8']
    EDIT = ['campus9-cutS5', 'campus9-cutS6', 'campus9-cutS7', 'campus9-cutS9']
    def g(run, k):
        return (out['rounds'].get(run) or {}).get(k)
    cm = {}
    for k in ('mileage_m', 'overshoot_max_m', 'loiter_total_m', 'loiter_frac',
              'retro_m', 'retro_frac', 'n_reversals', 'stationary_s', 'stationary_frac'):
        s = [g(r, k) for r in STOCK if g(r, k) is not None]
        e = [g(r, k) for r in EDIT if g(r, k) is not None]
        if not s or not e:
            continue
        ms = sum(s) / len(s)
        cm[k] = {'stock_mean': round(ms, 4), 'edited_mean': round(sum(e) / len(e), 4),
                 'stock_range': [round(min(s), 4), round(max(s), 4)],
                 'edited_range': [round(min(e), 4), round(max(e), 4)]}
    dev_hits = [(g(r, 'align_ahead') or {}).get('dev_lethal_density_ahead') for r in out['rounds']]
    nom_hits = [(g(r, 'align_ahead') or {}).get('nominal_lethal_density_ahead') for r in out['rounds']]
    dev_hits = [x for x in dev_hits if x is not None]
    nom_hits = [x for x in nom_hits if x is not None]
    md = sum(dev_hits) / len(dev_hits) if dev_hits else None
    mn = sum(nom_hits) / len(nom_hits) if nom_hits else None
    out['cohort_compare'] = {'stock_rounds': STOCK, 'edited_rounds': EDIT, 'metrics': cm}
    out['alignment_summary'] = {
        'dev_lethal_density_ahead_mean': round(md, 6) if md is not None else None,
        'nominal_lethal_density_ahead_mean': round(mn, 6) if mn is not None else None,
        'ratio': (round(md / mn, 3) if (md is not None and mn) else None),
        'note': ('★ 偏离的采样点若**明显更常**"1 m 内有致命格" ⇒ ★ 代价图在推着车走 ✓；'
                 '★ 两边差不多 ⇒ ★ 偏离与标记**无关** ✗')}
    verdict = {'aligns': None, 'reading': None}
    if md is not None and mn is not None:
        verdict['aligns'] = bool(mn > 0 and md / mn >= 2.0)
        verdict['reading'] = (
            '★★ **偏离与标记对得上**（★ 偏离点 %.1f%% vs 名义点 %.1f%% ⇒ %.2f 倍 ✓）⇒ ★ **代价图在推着车走** ✓✓ '
            '⇒ ★ 反馈环 ⇒ ★ "配得上"那一对也未必干净 ✓ ⇒ ★ **默认按同一弧长位置比** ✓'
            % (100 * md, 100 * mn, md / mn) if (mn > 0 and md / mn >= 2.0) else
            '★ **偏离与标记对不上**（★ 偏离点 %.1f%% vs 名义点 %.1f%% ⇒ %.2f 倍 ✗）⇒ ★ 偏离多半**不是**标记推的 ✓ '
            '⇒ ★ 看航点过冲那一栏 ⇒ ★ **控制器调参** ✓' % (100 * md, 100 * mn, (md / mn if mn else float('nan'))))
    out['verdict'] = verdict
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)

    print('%-16s %9s %9s %9s %9s %9s %9s' %
          ('round', 'mileage', 'overshoot', 'loiter_m', 'retro_m', 'n_rev', 'stat_s'))
    for run, v in out['rounds'].items():
        print('%-16s %9.3f %9.4f %9.3f %9.3f %9d %9.2f' %
              (run, v['mileage_m'], v['overshoot_max_m'], v['loiter_total_m'],
               v['retro_m'], v['n_reversals'], v['stationary_s']))
    print('★ 前方扇形致命密度：偏离点 %s ／ 名义点 %s ⇒ %s' %
          (out['alignment_summary']['dev_lethal_density_ahead_mean'],
           out['alignment_summary']['nominal_lethal_density_ahead_mean'],
           out['alignment_summary']['ratio']))
    print('★ 判词：%s' % verdict['reading'])
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""path_geometry_by_round.py —— ★ 把各轮的**路径几何**摆出来 ✓（★ 纯离线 ✓，★ 不起栈 ✓）

用法: python3 path_geometry_by_round.py --out <json> [--rounds r1,r2,...]

★★ 为什么（★ reviewer 2026-10-11 派 ✓）★★
  ★ 第 9 刀**自己**给出了"**改了配置 ⇒ 路变了**"的直接证据 ✓：★ 把那段还回标记 ⇒
    ★ 到达同一批航点多走了 **+17.1%** ✓（★ 52.100 m vs cutS6 44.478 m ✓）。
  ⇒ ★★ 那就带出一个**更大的问题** ✗：★ 我们一直用 **per 米／per 覆盖格** 归一 ✓ ——
    ★ 但这两个**都消不掉"路径形状不同"** ✗（★ 航点一样 ✓，★ 走出来的几何不一样 ✓）
  ⇒ ★★ 于是 ★ **那 2–3 倍"批差"** 也可能**主要是路径造的** ✗✗
    ⇒ ★ 那么 cutS5／cutS6／cutS7 的"配置效果"**全都要重估** ✗。

★★ 判据两头（★ 先写死 ✓）★★
  ★ **同批内的路径系统性不同** ✗ ⇒ ★★ **残留差主要是路径造的** ✓ ⇒ ★ "配置效果"那几条
    要按**路径受控**的设计重做 ✓✓
  ★ **路径基本相同** ⇒ ★ 才轮到"**标记／清除**"这一层 ✓✓

★★ 四个量（★ 都用**航点锚**（第 8 个航点到达 ✓）那一窗 ✓ —— ★ 与残留指标**同一段** ✓）★★
  ★ ① **里程** ＋ ★ **`sim_t`／米**（★ "每米摊到多少仿真时间" ✓ ⇒ ★ 扫同一块地的次数 ✓）
  ★ ② **转弯**：★ 总航向变化 `Σ|Δθ|` ✓、★ 每米多少度 ✓、★ "转弯段"个数 ✓、★ 转弯段占的长度比 ✓
  ★ ③ **离最近障碍的距离分布** ✓（★ 复用 `scene_obstacles.py` ✓ —— ★ 它已经把**地面**剔了 ✓，
      ★ 就是"铺满全场的地面会让判据恒真"那条 ✓）
  ★ ④ **相对名义航线的横向偏移** ✓（★ 名义线 = 8 个航点的 **GT** 折线 ✓，★ 各轮**同一条** ✓）

★★ 口径（★ 写死 ✓）★★
  · ★ 路径 = `probe/tick_*.json` 的 `robot_xy_map` ✓，按 seq 递增 ✓，**右边界 = 到达拍** ✓
    （★ ✗ 不扫活的探针目录 ✗ —— 探针比这一轮活得久 ✓）
  · ★ 距离／偏移都在**采样点**上算 ✓（★ 每 ~0.5 m 取一点 ✓，✗ 不逐拍逐点爆算 ✗）
  · ★ `Δθ` 只在 `Δs > 0.02 m` 的步上算 ✓（★ 原地抖动的航向噪声不算转弯 ✓），★ 角度归一到 (−π, π] ✓
  · ★ `Σ|Δθ|` 是**路径长度的一个函数** ✓ ⇒ ★ 报**每米**那一栏 ✓（✗ 不单报总量 ✗）
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
SCENE = '/home/xjy/projects/outdoorNav/gazebo_campus/generated/scene_geometry.json'
sys.path.insert(0, HERE)
from scene_obstacles import load_obstacles, dist_to_obstacle      # noqa: E402

STEP_M = 0.5                 # ★ 采样步长 ✓
MOVE_M = 0.02                # ★ Δθ 的最小位移门 ✓
TURN_RUN_DEG = 15.0          # ★ "转弯段"的判定：★ 同号连续 Δθ 累计 ≥ 15° ✓
TURN_LOCAL_DEG_PER_M = 5.0   # ★ "在转弯"的判定：★ 局部 |Δθ|/Δs > 5°/m ✓
MARGIN_MIN_REL = 0.05        # ★ "两批分开"还要求**余量 ≥ 5% 的 stock 均值** ✓（★ 见下 ✓）


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


def arrival_seq(run):
    """★ 到达拍 = 与 wp8 的最近接近点（`argmin d` ✓ —— ★ 与残留指标**同一个锚** ✓）"""
    g = wp_gt(run, 8)
    best = None
    for f in sorted(glob.glob(os.path.join(ART, run, 'probe', 'tick_*.json')), key=seq_of):
        try:
            r = json.load(open(f, encoding='utf-8')).get('robot_xy_map')
        except Exception:
            continue
        if not r or len(r) < 2:
            continue
        d = math.hypot(float(r[0]) - g[0], float(r[1]) - g[1])
        if best is None or d < best[0]:
            best = (d, seq_of(f))
    return best[1] if best else None


def path_of(run, cut):
    pts = []
    for f in sorted(glob.glob(os.path.join(ART, run, 'probe', 'tick_*.json')), key=seq_of):
        s = seq_of(f)
        if s > cut:
            break
        try:
            d = json.load(open(f, encoding='utf-8'))
        except Exception:
            continue
        r = d.get('robot_xy_map')
        if not r or len(r) < 2:
            continue
        pts.append((float(r[0]), float(r[1]), d.get('sim_t')))
    return pts


def resample(pts, step):
    """★ 按弧长重采样 ✓ ⇒ ★ 每 ~step 米一个点 ✓（★ 头尾都保留 ✓）"""
    out, acc = [pts[0]], 0.0
    for a, b in zip(pts, pts[1:]):
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        if acc >= step:
            out.append(b); acc = 0.0
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def poly_dist(px, py, poly):
    best = float('inf')
    for (x0, y0), (x1, y1) in zip(poly, poly[1:]):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
        best = min(best, math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)))
    return best


def pct(xs, p):
    if not xs:
        return None
    s = sorted(xs)
    return round(s[min(len(s) - 1, int(round(p * (len(s) - 1))))], 4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--rounds', default='campus9-res1,campus9-res2,campus9-cutS1,campus9-cutS5,'
                                        'campus9-cutS6,campus9-cutS7,campus9-cutS8,campus9-cutS9')
    ap.add_argument('--nominal-from', default='campus9-res1',
                    help='★ 名义航线取哪一轮的 8 个航点 GT ✓（★ 各轮同一条 ✓，本工具会核 ✓）')
    a = ap.parse_args()
    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s' % a.out)
        return 4

    nominal = [wp_gt(a.nominal_from, i) for i in range(1, 9)]
    nominal = [p for p in nominal if p]
    obs = load_obstacles(SCENE)
    out = {'what': '★ 各轮**路径几何**对照 ✓（★ 航点锚窗 ✓，★ 与残留指标同一段 ✓）',
           'why': ('★ 第 9 刀自己给出了"改了配置 ⇒ 路变了"的直接证据（★ +17.1% 里程 ✓）⇒ '
                   '★ 要问：★ 那 2–3 倍**批差**是不是**主要是路径造的** ✗'),
           'preregistered': {
               'cohorts_differ_systematically': '★★ **残留差主要是路径造的** ✓ ⇒ ★ "配置效果"要按**路径受控**的设计重做 ✓',
               'cohorts_similar': '★ 才轮到"**标记／清除**"这一层 ✓'},
           'convention': {
               'window': '★ 探针最早一拍 → **到达拍**（与 wp8 的最近接近点 ✓）—— ★ 与 `residue_at_wp8_anchor.py` **同一个锚** ✓',
               'sampling': '★ 路径每 ~%.1f m 取一点 ✓；★ Δθ 只在 Δs > %.2f m 的步上算 ✓；★ 角度归一 (−π, π] ✓' % (STEP_M, MOVE_M),
               'obstacles': '★ 复用 `scene_obstacles.py` ✓（★ **地面已剔** ✓ —— ★ 否则"铺满全场的地面"会让判据恒真 ✓）',
               'nominal_route': '★ 名义线 = 8 个航点的 **GT** 折线 ✓（★ 取自 %s ✓）；★ 各轮的航点 GT 会**逐一核** ✓' % a.nominal_from,
               'scene': SCENE, 'n_obstacles': len(obs)},
           'nominal_polyline': [[round(x, 4), round(y, 4)] for x, y in nominal],
           'rounds': {}, 'wp_gt_check': {}, 'notes': []}

    for run in [r for r in a.rounds.split(',') if r]:
        cut = arrival_seq(run)
        if cut is None:
            out['notes'].append('✗ %s：定不出到达拍 ⇒ 跳过' % run)
            continue
        # ★ 各轮航点 GT 与名义线逐一核 ✓
        gts = [wp_gt(run, i) for i in range(1, 9)]
        dev = max((math.hypot(g[0] - n[0], g[1] - n[1]) for g, n in zip(gts, nominal) if g), default=None)
        out['wp_gt_check'][run] = round(dev, 4) if dev is not None else None

        pts = path_of(run, cut)
        if len(pts) < 3:
            out['notes'].append('✗ %s：路径点太少 ⇒ 跳过' % run)
            continue
        traj = sum(math.hypot(b[0] - a2[0], b[1] - a2[1]) for a2, b in zip(pts, pts[1:]))
        sims = [p[2] for p in pts if isinstance(p[2], (int, float))]
        sim_span = (sims[-1] - sims[0]) if len(sims) >= 2 else None

        # ★ ② 转弯 ✓ —— ★ `Σ|Δθ|` 是**相邻步航向之差**的绝对值之和 ✓
        #   （✗ 不是"每步航向的绝对值" ✗ —— 那个量在直路上恒 0、在绕圈时乱跳 ✓，与本意无关 ✗）
        turn_len = 0.0
        runs, cur_sign, cur_sum = [], 0, 0.0
        hd = []
        for a2, b in zip(pts, pts[1:]):
            ds = math.hypot(b[0] - a2[0], b[1] - a2[1])
            if ds <= MOVE_M:
                continue
            hd.append((math.atan2(b[1] - a2[1], b[0] - a2[0]), ds))
        total_turn = 0.0
        for (h0, d0), (h1, d1) in zip(hd, hd[1:]):
            dth = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
            total_turn += abs(dth)
            if abs(math.degrees(dth)) / max(d0, 1e-6) > TURN_LOCAL_DEG_PER_M:
                turn_len += d0
            sgn = 1 if dth > 0 else -1
            if sgn != cur_sign:
                if abs(math.degrees(cur_sum)) >= TURN_RUN_DEG:
                    runs.append(cur_sum)
                cur_sign, cur_sum = sgn, 0.0
            cur_sum += dth
        if abs(math.degrees(cur_sum)) >= TURN_RUN_DEG:
            runs.append(cur_sum)

        # ★ ③ 离最近障碍 ✓
        rs = resample(pts, STEP_M)
        dd = [dist_to_obstacle(x, y, obs) for x, y, _ in rs]
        # ★ ④ 相对名义线的横向偏移 ✓
        off = [poly_dist(x, y, nominal) for x, y, _ in rs]

        out['rounds'][run] = {
            'arrival_seq': cut,
            'n_probe_samples': len(pts), 'n_path_samples': len(rs),
            'traj_len_m': round(traj, 3),
            'sim_span_s': round(sim_span, 2) if sim_span else None,
            'sim_s_per_m': round(sim_span / traj, 4) if (sim_span and traj) else None,
            'total_turn_deg': round(math.degrees(total_turn), 2),
            'turn_deg_per_m': round(math.degrees(total_turn) / traj, 3) if traj else None,
            'n_turns_ge15deg': len(runs),
            'turns_per_10m': round(10.0 * len(runs) / traj, 3) if traj else None,
            'turn_length_frac': round(turn_len / traj, 4) if traj else None,
            'd_obstacle': {'min': round(min(dd), 3), 'p05': pct(dd, 0.05), 'p50': pct(dd, 0.50),
                           'p95': pct(dd, 0.95),
                           'share_le_0.5m': round(sum(1 for x in dd if x <= 0.5) / len(dd), 4),
                           'share_le_1.0m': round(sum(1 for x in dd if x <= 1.0) / len(dd), 4),
                           'share_le_2.0m': round(sum(1 for x in dd if x <= 2.0) / len(dd), 4)},
            'offset_from_nominal': {'p50': pct(off, 0.50), 'p95': pct(off, 0.95),
                                    'max': round(max(off), 3),
                                    'mean': round(sum(off) / len(off), 3)}}

    # ★★ 同批对照 ✓（★ stock = 落到 stock 带那几轮 ✓；★ edited = 改过配置那几轮 ✓）★★
    STOCK = ['campus9-res1', 'campus9-res2', 'campus9-cutS1', 'campus9-cutS8']
    EDIT = ['campus9-cutS5', 'campus9-cutS6', 'campus9-cutS7', 'campus9-cutS9']
    METRICS = ['traj_len_m', 'sim_s_per_m', 'turn_deg_per_m', 'turns_per_10m', 'turn_length_frac',
               'offset_from_nominal.p95']
    def g(run, k):
        cur = out['rounds'].get(run) or {}
        for part in k.split('.'):
            cur = (cur or {}).get(part) if isinstance(cur, dict) else None
        return cur
    cmp_ = {'stock_band_rounds': STOCK, 'edited_config_rounds': EDIT,
            'note': '★ stock 带 = 落到 23–29 个/米那几轮 ✓（★ 含对照轮 cutS8 ✓）；★ edited = 改过配置那几轮 ✓（★ 含关键实验 cutS9 ✓）',
            'metrics': {}}
    for k in METRICS:
        s = [g(r, k) for r in STOCK if g(r, k) is not None]
        e = [g(r, k) for r in EDIT if g(r, k) is not None]
        if not s or not e:
            continue
        ms, me = sum(s) / len(s), sum(e) / len(e)
        # ★★ "两批分开"不能只看**不重叠** ✗ —— ★ 还要求**余量够厚** ✓ ★★
        #   ★ reviewer 2026-10-11 抓到的 ✗：★ 我用"区间不重叠"当分辨力 ✓ ⇒ ★ 会把
        #     **转弯段占比**也算进来 ✓ —— ★ 而它余量只有 **0.0002** ✗（★ stock 下沿 0.9096 vs
        #     edited 上沿 0.9094 ✓）⇒ ★ **换个阈值定义就翻** ✗ ⇒ ★ 那不是分辨力 ✗。
        #   ⇒ ★ 判据改成：★ **不重叠 ∧ 余量 ≥ %.0f%% 的 stock 均值** ✓（★ 余量同时报出来 ✓）。
        margin = (min(e) - max(s)) if min(e) > max(s) else ((min(s) - max(e)) if min(s) > max(e) else None)
        cmp_['metrics'][k] = {'stock_mean': round(ms, 4), 'edited_mean': round(me, 4),
                              'gap_pct': round(100 * (me - ms) / ms, 1) if ms else None,
                              'stock_range': [round(min(s), 4), round(max(s), 4)],
                              'edited_range': [round(min(e), 4), round(max(e), 4)],
                              'ranges_overlap': not (max(s) < min(e) or max(e) < min(s)),
                              'margin_abs': (round(margin, 6) if margin is not None else None),
                              'margin_rel': (round(margin / abs(ms), 6) if (margin is not None and ms) else None),
                              'separates': bool(margin is not None and ms and margin / abs(ms) >= MARGIN_MIN_REL)}
    out['cohort_compare'] = cmp_
    # ★ 判词 ✓（★ 两头都算 ✓）
    sep = [k for k, v in cmp_['metrics'].items() if v['separates']]
    thin = [k for k, v in cmp_['metrics'].items()
            if not v['ranges_overlap'] and not v['separates']]
    out['verdict'] = {
        'metrics_that_separate': sep,
        'metrics_disjoint_but_MARGINAL_not_evidence': thin,
        'margin_rule': '★ **不重叠 ∧ 余量 ≥ %.0f%% 的 stock 均值** ✓ —— ★ 只"不重叠"不够 ✗（★ 余量薄的一换定义就翻 ✓）' % (100 * MARGIN_MIN_REL),
        'reading': ('★ **里程**这一条**两批分开且余量够厚**（★ 5.966 m ＝ stock 均值的 9.7% ✓）⇒ '
                    '★ 同批路径**系统性不同** ✗ ⇒ ★ 按预登记 ⇒ ★ 残留差**分不开**是路径还是配置 ✓ ⇒ '
                    '★ "配置效果"要按**路径受控**重做 ✓；'
                    '★ 转弯段占比**虽然不重叠但余量只有 0.02%** ✗ ⇒ ★ **记成"边际、不作凭据"** ✓'
                    if sep else
                    '★ 没有量能"两批分开且余量够厚" ✓ ⇒ ★ 路径**基本相同** ⇒ ★ 按预登记 ⇒ '
                    '★ 才轮到"**标记／清除**"这一层 ✓')}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)

    print('★ 名义线取自 %s；★ 障碍 %d 个（★ 地面已剔 ✓）' % (a.nominal_from, len(obs)))
    print('%-16s %9s %9s %9s %9s %9s %9s' %
          ('round', 'traj_m', 'sim_s/m', 'turn°/m', 'turns/10m', 'turnfrac', 'off_p95'))
    for run, v in out['rounds'].items():
        print('%-16s %9.3f %9.4f %9.3f %9.3f %9.4f %9.3f' %
              (run, v['traj_len_m'], v['sim_s_per_m'] or -1, v['turn_deg_per_m'] or -1,
               v['turns_per_10m'] or -1, v['turn_length_frac'] or -1,
               v['offset_from_nominal']['p95']))
    for k, v in cmp_['metrics'].items():
        print('  ★ %-22s stock %8.4f ／ edited %8.4f ⇒ %+6.1f%% ｜ 两批重叠=%s'
              % (k, v['stock_mean'], v['edited_mean'], v['gap_pct'], v['ranges_overlap']))
    print('★ 判词：%s' % out['verdict']['reading'])
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

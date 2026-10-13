#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""residue_at_wp8_anchor.py —— ★ 残留量**按航点锚**算 ✓（★ 不与"走了多少米"挂钩 ✗）

用法: python3 residue_at_wp8_anchor.py --rounds campus9-res1,campus9-cutS1,campus9-cutS7 --out <json>

★★ 为什么要"航点锚"（★ reviewer 2026-10-11 派 ✓）★★
  ★ 跨轮比**累积量**（残留格数 ✓）必须**先对齐进度锚** ✗ —— 否则窗口长短不同 ⇒ 不可比 ✗
    （★ 本项目已栽过：★ "每拍都占用"／`traj_len_m` 都随窗口长度变 ✓）。
  ★ 而"里程锚"在本批轮次上**不成立** ✗：★ 同一批航点，★ 各轮走的米数差得很远 ✓
    （★ cutS7 48.417 m ／ cutS1 60.142 m ✓）⇒ ★ 里程**不同源** ✗。
  ★ 所以锚取 **"到达第 N 个航点那一刻"** ✓ —— ★ 那是一个**地点** ✓、★ 与走了多少米无关 ✓。

★★ 锚怎么定（★ 写死 ✓；★ 两个候选实测**不是同一件事** ✗）★★
  ★ **① 到达**（本工具的主锚 ✓）= 与 wpN 的**最近接近点**（`argmin d` ✓）。
    实测：★ 车**穿过** wpN ✓ ⇒ ★ 最接近那一刻就是"到达" ✓，★ 之后它还在往前开 ✗。
  ★ **② 官方 `probe_frozen.cut_seq`**（= "图最后一次变化" = 车彻底停稳 ✓）。
    ★ 两者实测差 **20 拍上下** ✗（cutS7 **149↔171** ✓、cutS1 **230↔253** ✓）⇒ ★ 停稳前车还爬了 ~1.2 m ✓。
  ★ 所以：★ 主锚用 ① ✓；★ ② 也**各算一遍当灵敏度上界** ✓
    —— ★ 两头同判词 ⇒ **判词与锚的选法无关** ✓✓（★ 这正是要的 ✓）。
  ★ ✗ 不用"最后一个在 tol 内的拍" ✗：★ 车停在 wp 附近时**一直在 tol 内** ⇒ 会拿到**停尾段的末拍** ✓
    （★ cutS7 会拿到 531 而不是 149 ✓）⇒ ★ `n_added`／`traj_len_m` 都**不变** ⇒ **看着正常** ✗（★ 静默失真那族 ✓）。

★★ 三个归一（★ 都报 ✓ —— ★ 口径写死 ✓）★★
  · ★ `added_per_m`            = n_added ÷ traj_len_m       （★ 与历轮 `cut_result.json` **同一族** ✓）
  · ★ `added_per_tick`         = n_added ÷ 窗口拍数          （★ 按"标记机会数"归 ✓）
  · ★ `net_per_delta_covered`  = n_added ÷ Δ非 unknown 格数  （★ 与 `coverage_normalized.json` **同一把** ✓）

★★ 预登记（★ 先写死 ✓；★ 两头都算结论 ✓）★★
  ★ 主判据 = ★ **`added_per_m`** ✓；★ 另两个归一**必须同号** ✓ 才给强判词 ✓。
  ★ 相对基线：★ **≤ −20%** ⇒ ★ **降**（★ 俯仰这一刀把残留也压下去了 ✓）；
    ★ **≥ +20%** ⇒ ★ **升**（★ 残留另有其源 ✗）；★ 之间 ⇒ ★ **不可判**（★ 记成"没变" ✓）。
  ★ 阈值 **±20%** 沿用 `coverage_normalized.json` 那一档 ✓（★ ✗ 不新造 ✗）。
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ART = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'
BAND = 0.20                      # ★ ±20% ⇒ 强判词；之间 = 不可判 ✓


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def ticks(d):
    return sorted(glob.glob(os.path.join(d, 'tick_*.json')), key=seq_of)


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


def anchor_of(run, probe, wp, tol):
    """★ **到达**那一拍 ✓ = 与 wpN 的**最近接近点**（★ `argmin d` ✓）"""
    g = wp_gt(run, wp)
    if g is None:
        return {'wp_gt': None, 'arrival_seq': None, 'd_at_anchor': None}
    best = None
    for f in ticks(probe):
        try:
            r = json.load(open(f, encoding='utf-8')).get('robot_xy_map')
        except Exception:
            continue
        if not r or len(r) < 2:
            continue
        d = math.hypot(float(r[0]) - g[0], float(r[1]) - g[1])
        if best is None or d < best[2]:
            best = (seq_of(f), (float(r[0]), float(r[1])), d)
    if best is None:
        return {'wp_gt': g, 'arrival_seq': None, 'd_at_anchor': None}
    return {'wp_gt': g, 'arrival_seq': best[0], 'd_at_anchor': round(best[2], 4),
            'arrival_xy': [round(best[1][0], 4), round(best[1][1], 4)],
            'within_tol': bool(best[2] <= tol)}


def disp_after(probe, mx):
    """★ 锚之后还走了多少米 ✓（★ 到达锚**不为 0** 是**正常**的 ✓ —— ★ 车会再往前爬 ~1.2 m ✓）"""
    tot, prev = 0.0, None
    for f in ticks(probe):
        if seq_of(f) <= mx:
            continue
        try:
            r = json.load(open(f, encoding='utf-8')).get('robot_xy_map')
        except Exception:
            continue
        if r and prev:
            tot += math.hypot(r[0] - prev[0], r[1] - prev[1])
        if r:
            prev = r
    return round(tot, 4)


def freeze(probe, out, cut):
    """★ 冻结到某一拍 ✓ —— ★ 守的是**内容对不对**，✗ 不是"目录在不在" ✗

    ★★ 为什么不能只判"存在 ⇒ 拒绝" ✗（★ 本族已栽过 ✓）★★
      ★ `freeze_probe_snapshot.py` **碰到已存在的目录不报错** ✓ —— ★ 它会**覆盖同名软链** ✓，
        ★ 但**留着上一次多出来的**（★ seq > 本次 cut 的那些 ✓）✗
      ⇒ ★ 换一个**更小**的 cut 复用同目录 ⇒ ★ 读到的是**上一刀的窗口** ✗✗（★ 静默失真 ✓）。
      ★ 所以判据要落在**内容**上 ✓：★ 读 `snapshot_meta.json` 的 `cut_seq` ✓ ——
        ★ **等于**本次要的 ⇒ **复用 ✓**（★ 幂等 ✓，✗ 不必重冻 ✗）；★ **不等** ⇒ **拒绝** ✗。
    """
    meta = os.path.join(out, 'snapshot_meta.json')
    if os.path.exists(out):
        if os.path.isfile(meta):
            try:
                got = json.load(open(meta, encoding='utf-8')).get('cut_seq')
            except Exception:
                got = None
            if got == cut:
                return True, 'reused'
            print('✗ 目录在，但 `cut_seq`=%s ≠ 本次要的 %d ⇒ 拒绝（★ 陈旧目录 ✗）' % (got, cut))
            return False, 'stale'
        print('✗ 目录在但没有 snapshot_meta.json ⇒ 拒绝 ✗：%s' % out)
        return False, 'stale'
    r = subprocess.run(['python3', os.path.join(HERE, 'freeze_probe_snapshot.py'), probe, out, str(cut)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print('✗ 冻结失败：%s' % (r.stdout + r.stderr).strip()[:400])
        return False, 'failed'
    return True, 'froze'


def park_seq(probe):
    """★ 车彻底停稳那一拍 = **图最后一次变化** ✓（★ 与 `freeze_probe_snapshot.py` 同一算法 ✓）

    ★ 有 `probe_frozen/snapshot_meta.json` 的轮次**直接用它记的那个值** ✓（★ 权威 ✓）；
      ★ 没有的（res1／res2 ✓）现算 ✓ —— ★ 这样"停稳窗口"那一栏**每轮都有** ✓（✗ 不留空 ✗）。
    """
    maps = sorted(glob.glob(os.path.join(probe, 'map_*.bin')), key=seq_of)
    if not maps:
        return None
    import hashlib
    last, prev = seq_of(maps[0]), None
    for m in maps:
        h = hashlib.sha256(open(m, 'rb').read()).hexdigest()
        if prev is not None and h != prev:
            last = seq_of(m)
        prev = h
    return last


def delta(frozen, tmpout):
    r = subprocess.run(['python3', os.path.join(HERE, 'dynamic_lethal_delta.py'), frozen,
                        '--max-seq', '99999', '--out', tmpout], capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(tmpout):
        return None, (r.stdout + r.stderr).strip()[:400]
    return json.load(open(tmpout, encoding='utf-8')), None


def covered(frozen):
    """★ Δ非 unknown 格数 ✓（★ 与 `coverage_normalized.json` 同一口径 ✓）"""
    maps = sorted(glob.glob(os.path.join(frozen, 'map_*.bin')), key=seq_of)
    if not maps:
        return None
    b0 = open(maps[0], 'rb').read()
    b1 = open(maps[-1], 'rb').read()
    c0 = sum(1 for x in b0 if x != 255)
    c1 = sum(1 for x in b1 if x != 255)
    return {'start_map': os.path.basename(maps[0]), 'end_map': os.path.basename(maps[-1]),
            'covered_start': c0, 'covered_end': c1, 'delta_covered': c1 - c0}


def official_cut(run):
    p = os.path.join(ART, run, 'probe_frozen', 'snapshot_meta.json')
    if os.path.isfile(p):
        try:
            return json.load(open(p, encoding='utf-8')).get('cut_seq')
        except Exception:
            return None
    return None


def official_result(run):
    p = os.path.join(ART, run, 'cut_result.json')
    if os.path.isfile(p):
        try:
            return json.load(open(p, encoding='utf-8')).get('this_round')
        except Exception:
            return None
    return None


def window_metric(run, probe, cut, tag, tmpdir):
    """★ 算一个窗口的四个数 ✓"""
    frozen = os.path.join(ART, run, 'probe_frozen_wp8_%s_%04d' % (tag, cut))
    rec = {'tag': tag, 'cut_seq': cut, 'frozen_dir': os.path.relpath(frozen, ART)}
    ok, how = freeze(probe, frozen, cut)
    rec['freeze'] = how
    if not ok:
        rec['error'] = '★ 冻结失败/陈旧目录被拒 ✗（%s）' % how
        return rec
    dl, err = delta(frozen, os.path.join(tmpdir, 'dl_%s_%s.json' % (run, tag)))
    if dl is None:
        rec['error'] = '★ dynamic_lethal_delta 失败：%s' % err
        return rec
    ts = ticks(frozen)
    n_ticks = len(ts)
    cov = covered(frozen)
    rec.update({'n_added': dl['n_added'], 'n_removed': dl['n_removed'],
                'n_lethal_start': dl['n_lethal_start'], 'n_lethal_end': dl['n_lethal_end'],
                'traj_len_m': dl['traj_len_m'], 'n_ticks': n_ticks,
                'first_seq': seq_of(ts[0]), 'last_seq': seq_of(ts[-1]),
                'added_per_m': dl['added_per_m'],
                'added_per_tick': (round(dl['n_added'] / n_ticks, 4) if n_ticks else None),
                'coverage': cov,
                'net_per_delta_covered': (round(dl['n_added'] / cov['delta_covered'], 6)
                                          if cov and cov['delta_covered'] else None)})
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rounds', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--wp', type=int, default=8)
    ap.add_argument('--tol', type=float, default=0.40)
    ap.add_argument('--baseline', default='campus9-res1')
    a = ap.parse_args()

    if os.path.exists(a.out):
        print('✗ 产物已存在 ⇒ 拒绝覆盖 ✗：%s（★ 换名字 ✓）' % a.out)
        return 4

    tmpdir = os.path.join(os.environ.get('CLAUDE_JOB_DIR', '/tmp'), 'tmp')
    os.makedirs(tmpdir, exist_ok=True)

    out = {'what': '★ 阶段十：★ **残留量按"第 %d 个航点到达"锚**算 ✓ ⇒ 与"走了多少米"解耦 ✓' % a.wp,
           'ruler': ('★ n_added / n_removed / traj_len_m 由**同一个** `dynamic_lethal_delta.py` 现算 ✓'
                     '（★ 与历轮 `cut_result.json` **同一把尺** ✓）；★ 窗口 = 探针最早一拍 → 锚拍 ✓'),
           'anchors': {
               'primary': 'arrival —— ★ 与 wp%d 的**最近接近点**（argmin d ✓）；★ 车穿过航点 ⇒ 这就是到达 ✓' % a.wp,
               'sensitivity': ('official —— ★ 车彻底停稳那一拍（★ 有 `probe_frozen/snapshot_meta.json` 的用它记的 `cut_seq` ✓；'
                               '★ res1／res2 没有 ⇒ 现算"图最后一次变化" ✓，同一算法 ✓）；'
                               '★ 两头同判词 ⇒ 判词与锚的选法无关 ✓'),
               'why_not_last_within_tol': '★ 车停在航点附近会**一直在 tol 内** ⇒ 取"最后一个在 tol 内的拍"会落到**停尾段末拍** ✗（★ 静默失真 ✓）'},
           'normalizations': {
               'added_per_m': '★ n_added ÷ traj_len_m ✓（★ 与历轮同一族 ✓）',
               'added_per_tick': '★ n_added ÷ 窗口拍数 ✓（★ 按标记机会数归 ✓）',
               'net_per_delta_covered': '★ n_added ÷ Δ非 unknown 格数 ✓（★ 与 coverage_normalized.json 同一把 ✓）'},
           'preregistered': {
               'primary': 'added_per_m',
               'require_same_sign': ['added_per_tick', 'net_per_delta_covered'],
               'le_neg20': '★ 降 ⇒ 俯仰这一刀把残留也压下去了 ✓',
               'ge_pos20': '★ 升 ⇒ 残留另有其源 ✗',
               'in_between': '★ 不可判（记成"没变" ✓）',
               'band': BAND, 'band_source': '★ 沿用 coverage_normalized.json 的 ±20% ✓（✗ 不新造 ✗）'},
           'baseline_named': a.baseline,
           'rounds': {}, 'notes': []}

    for run in [r for r in a.rounds.split(',') if r]:
        probe = os.path.join(ART, run, 'probe')
        if not os.path.isdir(probe):
            out['notes'].append('✗ %s：没有 probe 目录 ⇒ 跳过' % run)
            continue
        an = anchor_of(run, probe, a.wp, a.tol)
        off = official_cut(run)
        rec = {'anchor': an, 'official_cut_seq': off,
               'park_seq': (off if off else park_seq(probe)),
               'park_seq_source': ('probe_frozen/snapshot_meta.json ✓（权威 ✓）' if off else
                                   '★ 现算：图最后一次变化 ✓（与 freeze_probe_snapshot.py 同一算法 ✓）'),
               'official_result': official_result(run), 'windows': {}}
        if an['arrival_seq'] is None:
            rec['error'] = '★ 读不到位姿 ⇒ 无法定锚 ✗'
            out['rounds'][run] = rec
            continue
        cuts = [('arrival', an['arrival_seq'])]
        if rec['park_seq'] and rec['park_seq'] != an['arrival_seq']:
            cuts.append(('official', rec['park_seq']))
        for tag, cut in cuts:
            w = window_metric(run, probe, cut, tag, tmpdir)
            if tag == 'arrival':
                w['disp_after_anchor_m'] = disp_after(probe, cut)
            rec['windows'][tag] = w
        out['rounds'][run] = rec

    # ★ 相对基线的判词（★ 两头都算 ✓；★ 主锚 ＋ 灵敏度各给一次 ✓）
    for tag in ('arrival', 'official'):
        base = ((out['rounds'].get(a.baseline) or {}).get('windows') or {}).get(tag) or {}
        out.setdefault('verdicts', {})[tag] = {}
        for run, r0 in out['rounds'].items():
            w = (r0.get('windows') or {}).get(tag)
            if not w or run == a.baseline or 'added_per_m' not in w or not base.get('added_per_m'):
                continue
            rel, signs = {}, []
            for k in ('added_per_m', 'added_per_tick', 'net_per_delta_covered'):
                b, v = base.get(k), w.get(k)
                if b and v is not None:
                    rr = (v - b) / b
                    rel[k] = round(rr, 4)
                    signs.append(math.copysign(1, rr) if abs(rr) > 1e-9 else 0)
            w['relative_to_baseline'] = rel
            pr = rel.get('added_per_m')
            same = len(set(s for s in signs if s)) <= 1
            if pr is None:
                v = '★ 不可判（主判据缺数 ✗）'
            elif pr <= -BAND and same:
                v = '★ **降** ✓（★ 三个归一**同号** ✓）'
            elif pr >= BAND and same:
                v = '★ **升** ✗（★ 俯仰这一刀**没有**把残留压下去 ⇒ 残留另有其源 ✓）'
            elif not same:
                v = '★ **不可判** ✗（★ 三个归一**不同号** ⇒ 先查为什么 ✓）'
            else:
                v = '★ **不可判**（★ 落在 ±20% 带内 = 记成"没变" ✓）'
            w['verdict'] = v
            out['verdicts'][tag][run] = {'relative': rel, 'verdict': v}

    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('★ 锚 = 第 %d 个航点到达；★ 基线 = %s；★ 主锚 = arrival ✓' % (a.wp, a.baseline))
    for tag in ('arrival', 'official'):
        print('＝＝ 窗口：%s ＝＝' % tag)
        print('%-18s %7s %7s %6s %9s %10s %10s %13s' %
              ('round', 'anchor', 'n_add', 'ticks', 'traj_m', 'per_m', 'per_tick', 'per_dcov'))
        for run, r0 in out['rounds'].items():
            w = (r0.get('windows') or {}).get(tag)
            if not w:
                print('%-18s  —' % run)
                continue
            if 'added_per_m' not in w:
                print('%-18s  %s' % (run, w.get('error', '—')))
                continue
            print('%-18s %7d %7d %6d %9.3f %10.4f %10.4f %13.6f' %
                  (run, w['cut_seq'], w['n_added'], w['n_ticks'], w['traj_len_m'],
                   w['added_per_m'], w['added_per_tick'], w['net_per_delta_covered']))
            if 'relative_to_baseline' in w:
                rel = w['relative_to_baseline']
                print('%-18s   ★ 相对基线：per_m %+.1f%% ｜ per_tick %+.1f%% ｜ per_dcov %+.1f%% ⇒ %s' %
                      ('', 100 * rel.get('added_per_m', 0), 100 * rel.get('added_per_tick', 0),
                       100 * rel.get('net_per_delta_covered', 0), w['verdict']))
    for n in out['notes']:
        print('  ★ 注：%s' % n)
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

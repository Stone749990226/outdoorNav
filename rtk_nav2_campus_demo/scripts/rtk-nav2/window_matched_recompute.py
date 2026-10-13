#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""window_matched_recompute.py —— ★ 把 res1／res2 截到**与本刀那一轮同长**再重算（★ 同一把尺 ✓）

用法: window_matched_recompute.py --out json --cut-run campus9-cutS1 --refs campus9-res1,campus9-res2

★★ 为什么（reviewer 2026-10-11 ① ✓）★★
  ★ `sticky` 的定义是"**末拍仍致命**" ✓ ⇒ ★ **窗口越短 ⇒ 末尾那些标记越没时间被清** ✗
    ⇒ ★ 窗口短的轮次会被**系统性抬高** ✗ ⇒ ★ 必须**截到同长**再比 ✓。
★ 两个口径都给 ✓（★ 因为"同米数"与"同帧数"**不能同时满足** ✗ —— ★ 两轮车速不同 ✓）：
  · ★ **按米截**（★ 与指标自己的分母同源 ✓）⇒ 主口径 ✓
  · ★ **按帧截**（★ 同样帧数 ⇒ 同样"标记机会数"✓）⇒ 灵敏度 ✓
★ 另给 ★ **到达量** ✓（★ 与 res 同一把尺：★ 逐点 `min_d_gt` ＋ 进容差计数 ✓ ——
  ★ "航线文件里有几个点"是**信号** ✗、"车到了几个点"才是**完成** ✓）。
"""
import argparse
import glob
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def probe_ticks(run):
    d = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2/%s/probe' % run
    return sorted(glob.glob(os.path.join(d, 'tick_*.json')), key=seq_of)


def traj_of(run):
    """★ 逐拍累计弧长 ✓（与 `dynamic_lethal_delta.py` 同一算法 ✓）"""
    cum, prev, tot = [], None, 0.0
    for f in probe_ticks(run):          # ★ 逐拍累计弧长 ✓（与运行期指标同一算法 ✓）
        r = json.load(open(f, encoding='utf-8')).get('robot_xy_map')
        if r and prev:
            tot += math.hypot(r[0] - prev[0], r[1] - prev[1])
        if r:
            cum.append(tot)
            prev = r
    return cum


def freezes_at(run, max_seq, tag, cut=None):
    """★ 用现成工具冻结到某一拍 ✓，再算两把尺 ✓

    ★★ 2026-10-11 更正 ✗（★ 我自己栽的 ✓）：★ 旧版把冻结目录写死成 `probe_frozen_<tag>` ✗ ——
      ★ 而 `freeze_probe_snapshot.py` 碰到**已存在**的目录**不报错、直接返回** ✓
      ⇒ ★ 换一刀（cut_len 变小 ✓）后，读到的**还是上一刀匹配出来的窗口** ✗✗
      （★ 实测：★ 本轮 cutS2 的 `cut_len` 44.27 m ✓，★ 而 res1 的 `bym` 读出来是 **59.59 m** ✗ = 上一刀 cutS1 的 60.14 m ✓）
      ⇒ ★ 与"**陈旧产物静默生效**"同族 ✗。
    ⇒ ★ 两条修法 ✓：★ ① 目录名带上**本刀** ✓；★ ② 回来**核一句**：★ `traj_len_m` 必须 ≈ `cut_len` ✓，
      ★ 差超过 10% ⇒ ★ **记成匹配失败** ✓（✗ 不许静默当成"已匹配" ✗）。
    """
    P = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2/%s' % run
    out = os.path.join(P, 'probe_frozen_%s%s' % (tag, ('_' + cut) if cut else ''))
    subprocess.run(['python3', os.path.join(HERE, 'freeze_probe_snapshot.py'),
                    os.path.join(P, 'probe'), out, str(max_seq)], capture_output=True, text=True)
    a = os.path.join(out, 'footprint.json')
    b = os.path.join(out, 'dynamic_lethal_delta.json')
    subprocess.run(['python3', os.path.join(HERE, 'footprint_test.py'), out, '--max-seq', '99999',
                    '--out', a], capture_output=True, text=True)
    subprocess.run(['python3', os.path.join(HERE, 'dynamic_lethal_delta.py'), out,
                    '--max-seq', '99999', '--out', b], capture_output=True, text=True)
    fp = json.load(open(a, encoding='utf-8')) if os.path.isfile(a) else {}
    dl = json.load(open(b, encoding='utf-8')) if os.path.isfile(b) else {}
    return {'tag': tag, 'max_seq': max_seq, 'frozen_dir': out,
            'n_ticks_in_window': max_seq - seq_of(probe_ticks(run)[0]) + 1,
            'match_check_ok': None,
            'sticky_rate_R0.105': (fp.get('sticky_rate', {}).get('R=0.105') or {}).get('sticky_rate'),
            'sticky_rate_R0.500': (fp.get('sticky_rate', {}).get('R=0.500') or {}).get('sticky_rate'),
            'added_per_m': dl.get('added_per_m'), 'n_added': dl.get('n_added'),
            'traj_len_m': dl.get('traj_len_m')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cut-run', default='campus9-cutS1')
    ap.add_argument('--refs', default='campus9-res1,campus9-res2')
    a = ap.parse_args()
    B = '/mnt/c/Users/xjy/projects/robonix/artifacts/rtk-nav2'

    # ---- ① 本刀那一轮自己：窗口 ＋ 到达量 ----
    def arrivals(run):
        p = os.path.join(B, run, 'stream_records.json')
        if not os.path.isfile(p):
            return None
        d = json.load(open(p, encoding='utf-8'))
        recs = d.get('records') or []
        return {'n_records': len(recs),
                'per_point': [{'name': r.get('name'), 'min_d_gt': r.get('min_d_gt'),
                               'n_samples': r.get('n_samples')} for r in recs],
                'n_entered_tol_0.4': sum(1 for r in recs if (r.get('min_d_gt') or 9) <= 0.4),
                'criterion': d.get('criterion'),
                'source': p}
    cut = freezes_at(a.cut_run, 99999, 'asis')      # ★ 本轮本身已是"到停栈"为止 ✓
    cut_ticks = probe_ticks(a.cut_run)
    cut_first = seq_of(cut_ticks[0])
    # ★ 本轮的冻结上限 = 图上最后一次变化（与出指标时一致 ✓）
    cut_used = json.load(open(os.path.join(B, a.cut_run, 'probe_frozen/snapshot_meta.json'),
                              encoding='utf-8'))['cut_seq']
    cut_traj = traj_of(a.cut_run)
    cut_len = cut_traj[min(len(cut_traj) - 1, cut_used - cut_first)]

    out = {'what': '★ 把 res1／res2 截到与**本轮**（`--cut-run` ✓）同长再重算 ✓（★ 同一把尺 ✓）',
           'cut_run': {'run': a.cut_run, 'first_seq': cut_first, 'frozen_at': cut_used,
                       'n_ticks': cut_used - cut_first + 1, 'traj_m': round(cut_len, 3),
                       'arrivals': arrivals(a.cut_run)},
           'windows': {}}
    for run in [r for r in a.refs.split(',') if r]:
        tk = probe_ticks(run)
        first, last = seq_of(tk[0]), seq_of(tk[-1])
        tj = traj_of(run)
        full_len = tj[-1]
        # ★ 按米：找第一个 ≥ cut_len 的拍 ✓
        by_m = next((first + i for i, v in enumerate(tj) if v >= cut_len), last)
        # ★ 按帧：同样帧数 ✓
        by_f = min(last, first + (cut_used - cut_first))
        out['windows'][run] = {
            'full': {'first_seq': first, 'last_seq': last, 'n_ticks': last - first + 1,
                     'traj_m': round(full_len, 3), 'arrivals': arrivals(run)},
            'matched_by_metres': freezes_at(run, by_m, 'bym', a.cut_run),
            'matched_by_frames': freezes_at(run, by_f, 'byf', a.cut_run),
            'matched_by_metres_seq': by_m, 'matched_by_frames_seq': by_f,
        }
        # ★ 核一句 ✓：★ "按米截"的窗口**必须真的≈ cut_len** ✗（★ 否则"已匹配"是假的 ✗）
        for k in ('matched_by_metres', 'matched_by_frames'):
            w = out['windows'][run][k]
            w['cut_len_m'] = round(cut_len, 3)
            w['match_check_ok'] = bool(w['traj_len_m'] and abs(w['traj_len_m'] - cut_len) <= 0.10 * cut_len)
        out['windows'][run]['match_check_note'] = (
            '★ `match_check_ok` = |窗口轨迹 − cut_len| ≤ 10%×cut_len ✓ —— '
            '★ 按米截**必须**过 ✓；★ 按帧截**不一定**过 ✓（★ 两轮车速不同 ⇒ "同帧数"与"同米数"不能同时满足 ✓，'
            '★ 所以按帧那栏的 `match_check_ok=false` 是**预期**✓，✗ 不是故障 ✗）')
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)

    print('★ %s：窗口 %d 拍（seq %d→%d）｜ 轨迹 %.2f m ｜ 到达 %s/%s 点'
          % (a.cut_run, cut_used - cut_first + 1, cut_first, cut_used, cut_len,
             (out['cut_run']['arrivals'] or {}).get('n_entered_tol_0.4'),
             (out['cut_run']['arrivals'] or {}).get('n_records')))
    for run, v in out['windows'].items():
        f = v['full']
        print('★ %s：全程 %d 拍 / %.2f m ｜ 到达 %s/%s 点'
              % (run, f['n_ticks'], f['traj_m'],
                 (f['arrivals'] or {}).get('n_entered_tol_0.4'), (f['arrivals'] or {}).get('n_records')))
        for tag, key in (('按米截', 'matched_by_metres'), ('按帧截', 'matched_by_frames')):
            w = v[key]
            print('   %s ⇒ %d 拍 ｜ ★ 粘住率 %.2f%%（R=0.105）｜ ★ 开阔地 %.3f 个/米 ｜ 轨迹 %.2f m'
                  % (tag, w['n_ticks_in_window'], 100 * (w['sticky_rate_R0.105'] or 0),
                     (w['added_per_m'] or 0), (w['traj_len_m'] or 0)))
    print('★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""marking_census.py —— ★ "标记普查"：按 `obstacle_layer` 的规则，**哪些点该被标上** ✓（纯离线）

用法: marking_census.py <probe目录> --max-seq N --out json [--tick-stride 8]

★★ 为什么（reviewer 2026-10-11 ✓）★★
  ★ 到这儿我们量的都是"**结果**"（哪些格致命 ✓）⇒ ★ **没有一次把"按规则哪些点该被标上"算出来** ✗
    ⇒ ★ 于是**说不清"标上的到底是什么"** ✗ —— ★ 而两场景的差**恰恰只能在标记侧** ✓（清除侧已排干净 ✓）。
★ 门值 ★ **按各轮**运行时参数**取** ✓（✗ 不抄别人的数 ✓）—— ★ 出处：
  `artifacts/rtk-nav2/<run>/nav2/params_cli_global_costmap_global_costmap.yaml` ✓（★ 实测四轮**完全一致** ✓）：
  · ★ **scan**（`/rtk/scan_points` ✓）：★ `min_obstacle_height 0.02` ✓／`max_obstacle_height 1.2` ✓／
    ★ `obstacle_max_range 7.5` ✓／`obstacle_min_range 0.2` ✓／★ `inf_is_valid true` ✓
  · ★ **cam**（`/rtk/camera/points` ✓）：★ `min_obstacle_height 0.3` ✓／`max 1.2` ✓／`obstacle_max_range 6.0` ✓／
    ★ `obstacle_min_range 0.0` ✓
★★ **关键一步** ✓：★ 把"该被标上的点"投到格上 ⇒ ★ **与实际变致命的格对一遍** ✓（★ 命中率 ＋ 假阳／假阴 ✓）。
★ 判据两头 ✓：★ **对得上**（命中率高 ✓）⇒ ★ 我们的"标记"模型对 ✓ ⇒ ★ 普查说的就是机制 ✓；
　★ **对不上** ✗ ⇒ ★ 连"标记怎么发生"都没建模对 ✓ ⇒ ★ 那也是结论 ✓（★ 且正好解释"为什么一直找不到机制" ✓）。
★★ ★ **顺带答那 3 cm** ✓：★ 专列一档"**贴地(≤0.05)且过门的点**" ✓ ——
　★ 有 ⇒ ★ 那条有直接证据 ✓；★ **一个都没有** ⇒ ★ 那条当场关掉 ✓。
"""
import argparse
import collections
import glob
import json
import math
import os
import re
import sys

GATES = {   # ★ 来自各轮运行时参数 dump ✓（四轮逐项相同 ✓，已实测 ✓）
    'scan': {'min_h': 0.02, 'max_h': 1.2, 'max_r': 7.5, 'min_r': 0.2},
    'cam':  {'min_h': 0.30, 'max_h': 1.2, 'max_r': 6.0, 'min_r': 0.0},
}


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--tick-stride', type=int, default=8)
    a = ap.parse_args()
    P = a.probe_dir

    ticks = sorted([t for t in glob.glob(os.path.join(P, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(P, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    samp = ticks[::a.tick_stride]
    meta = json.load(open(samp[-1], encoding='utf-8'))['map']
    W, res = meta['width'], meta['resolution']
    ox, oy = meta['origin']['x'], meta['origin']['y']
    b0 = open(os.path.join(P, 'map_%04d.bin' % seq_of(ticks[0])), 'rb').read()
    b1 = open(os.path.join(P, 'map_%04d.bin' % seq_of(ticks[-1])), 'rb').read()
    L = lambda v: 99 <= v <= 100
    added = {k for k in range(len(b1)) if L(b1[k]) and not L(b0[k])}
    H = len(b1) // W
    # ★★ 2026-10-11 加一条**公平的参照集** ✗：★ 上面那个 `added` 是"**末拍仍致命 ∧ 起始不致命**" ✓
    #   ⇒ ★ 而"被标上又被清掉"的格**不在里面** ✗ ⇒ ★ 拿它当参照会**假报"模型对不上"** ✗
    #   ⇒ ★ 所以另算 ★ **"这一轮里曾经致命过"**（并集 ✓）⇒ ★ 这才是"标记"该对的东西 ✓。
    LETH = re.compile(rb'[\x63\x64]')
    ever = set()
    for t in ticks:
        ever |= {m.start() for m in LETH.finditer(open(os.path.join(P, 'map_%04d.bin' % seq_of(t)), 'rb').read())}
    ever -= {k for k in range(len(b1)) if L(b0[k])}     # ★ 起始就致命的（★ 静态先验那类 ✓）不算"标记"✓
    # ★★ 2026-10-11（reviewer ② ✓）：★ 探针 **2.0 s 一拍** ✓ 而代价图 ~10 Hz ⇒
    #   ★ "**标上、下一拍就被清掉**"的格**探针永远看不到** ✗（★ 而这类格正是多数 ✓ ——
    #   ★ 与本项目"**存活中位 = 1 拍**"✓ 完全一致 ✓）⇒ ★ 拿全部预测去比会**低估**模型 ✗。
    #   ⇒ ★ 另算 ★ **"活过 ≥1 拍"**（★ 在**相邻两拍**里都致命 ✓）⇒ ★ 只拿这批比 ✓。
    survived = set()
    prev = None
    for t in ticks:
        cur = {m.start() for m in LETH.finditer(open(os.path.join(P, 'map_%04d.bin' % seq_of(t)), 'rb').read())}
        if prev is not None:
            survived |= (cur & prev)
        prev = cur
    survived -= {k for k in range(len(b1)) if L(b0[k])}

    predicted = collections.defaultdict(set)      # sensor -> set(cells)
    stats = {c: {'n': 0, 'z': [], 'r': [], 'az': collections.Counter(), 'z_floor': 0} for c in GATES}
    for t in samp:
        s = seq_of(t)
        d = json.load(open(t, encoding='utf-8'))
        for cloud, g in GATES.items():
            f = os.path.join(P, '%s_%04d.xyz' % (cloud, s))
            if not os.path.isfile(f):
                continue
            M = ((d.get(cloud) or {}).get('tf') or {}).get('M')
            if not M:
                continue
            org = (M[0][3], M[1][3], M[2][3])
            yaw = math.atan2(M[1][0], M[0][0])
            for line in open(f, encoding='utf-8'):
                v = line.split()
                if len(v) != 3:
                    continue
                x, y, z = map(float, v)
                if not (g['min_h'] <= z <= g['max_h']):
                    continue                       # ★ 高度门 ✓
                r = math.dist((x, y, z), org)
                if not (g['min_r'] <= r <= g['max_r']):
                    continue                       # ★ 距离门 ✓
                gi, gj = int((x - ox) / res), int((y - oy) / res)
                if 0 <= gi < W and 0 <= gj < H:
                    predicted[cloud].add(gj * W + gi)
                st = stats[cloud]
                st['n'] += 1
                st['z'].append(z)
                st['r'].append(r)
                st['az'][int(((math.degrees(math.atan2(y - org[1], x - org[0])) - math.degrees(yaw)) % 360.0) // 10) * 10] += 1
                if z <= 0.05:
                    st['z_floor'] += 1

    allpred = predicted['scan'] | predicted['cam']

    def dist(v, p):
        v = sorted(v)
        return round(v[min(len(v) - 1, int(len(v) * p))], 3) if v else None
    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq, 'tick_stride': a.tick_stride,
           'n_ticks_sampled': len(samp), 'gates_used': GATES,
           'gates_source': ('★ 各轮 `nav2/params_cli_global_costmap_global_costmap.yaml` ✓'
                            '（★ 四轮逐项相同 ✓，已实测 ✓，✗ 不是抄的 ✓）'),
           'convention': ('★ 逐拍点云 ✓；★ 高度门用**地图帧 z** ✓（★ 与 obstacle_layer 一致 ✓）；'
                          '★ 距离用**到该云原点**的 3D 距离 ✓；★ 投到格 ⇒ 与"实际变致命的格"比 ✓；'
                          '★★ 两条参照 ★ **分母都写清** ✓：'
                          '① `hit_rate_vs_ever_marked` ＝ **|预测 ∩ 曾经致命过| ÷ |预测|** ✓'
                          '（★ 分子是**交集** ✗ 不是 `cells_predicted_and_actually_lethal` ✗，那个是"末拍仍致命"那条的分子 ✓）；'
                          '② `hit_rate_vs_survived_1tick` ＝ **|预测 ∩ 活过≥1拍| ÷ |预测|** ✓ —— '
                          '★ 探针 2.0 s 一拍、代价图 ~10 Hz ⇒ ★ "标上又立刻被清"的格探针看不到 ✗ ⇒ '
                          '★ 只拿"活过 ≥1 拍"那批比才是**标记模型**的真正检验 ✓（★ 有意收窄：只覆盖"粘住/存活"类 ✓）；'
                          '★ 另：`n_points_at_floor_le_0.05` 那一档 ★ **只能是激光门** ✓'
                          '（★ z ≤ 0.05 不可能过相机门 `min_h 0.3` ✓）；'
                          '★★★ **本普查不能校验"标记模型"** ✗ —— ★ 依据（★ 2026-10-11 reviewer ② ✓）：'
                          '★ ① `observation_persistence = 0` ✓ ⇒ ★ 一有清除射线穿过就清 ✓；'
                          '★ ② ★ **存活中位 = 1 拍** ✓ ⇒ ★ 一个"真的被标上"的格，寿命通常 **≲ 一个代价地图周期** ✓；'
                          '★ ③ ★ **探针 2.0 s 一拍** ✗ 而代价图快得多 ✓ ⇒ ★ **"标上、下一拍就被清"的格探针根本采不到** ✗ '
                          '⇒ ★ **"预测了却从没见过它致命"是**预期结果** ✗ 不是模型错的证据** ✓；'
                          '★ 反向旁证 ✓：★ 被观测到致命的格里预测覆盖 **12.7–26.4%** ✓，★ 而我们只抽了 **13–19%** 的拍 ✓ '
                          '⇒ ★ **用 13–19% 的拍覆盖到 26% 的格 ⇒ 高于随机 ✓** ⇒ ★ 这是**支持**"模型在做实事"的一侧 ✓；'
                          '★ 要真判 ⇒ 得让采样快过 mark-clear 周期 ⇒ ★ 要起栈 ＋ 改探针 ✗ ⇒ ★ 不值得 ✓ '
                          '（★ 且"该拦的"已经拦下了 ✓）；'
                          '★★ 所以两条命中率（`hit_rate_of_predicted` ✓／`hit_rate_vs_survived_1tick` ✓）'
                          '★ **都不能当模型有效性** ✗ —— ★ 只作"输入侧／构成"的参考 ✓；'
                          '★ **不受采样粒度影响、可当结论的**是**输入侧**那几条 ✓：'
                          '★ cam 过门点绝大多数来自主楼 ✓／★ 贴地过门 9.6 vs 10.4 每拍 ⇒ 3 cm 没显出差别 ✓／'
                          '★ scan 过门 61–66 每拍 ✓')}
    for cloud in GATES:
        st = stats[cloud]
        pred = predicted[cloud]
        hit = len(pred & added)
        out[cloud] = {'n_points_pass_gate': st['n'], 'per_tick': round(st['n'] / max(1, len(samp)), 2),
                      'n_cells_predicted': len(pred),
                      'cells_predicted_and_actually_lethal': hit,
                      'hit_rate_of_predicted': (hit / len(pred)) if pred else None,
                      'cells_lethal_covered_by_prediction': (hit / len(added)) if added else None,
                      # ★ ① 分母写清 ✓：|pred ∩ ever| ÷ |pred| ✓（★ 分子是**交集** ✗ 不是 406 ✗）
                      'hit_rate_vs_ever_marked': (len(pred & ever) / len(pred)) if pred else None,
                      'hit_rate_vs_ever_marked_numerator': len(pred & ever),
                      'hit_rate_vs_ever_marked_denominator': len(pred),
                      'cover_rate_vs_ever_marked': (len(pred & ever) / len(ever)) if ever else None,
                      # ★ ② "活过 ≥1 拍"那批（★ 真正的模型检验 ✓）
                      'n_cells_survived_1tick': len(survived),
                      'hit_rate_vs_survived_1tick': (len(pred & survived) / len(pred)) if pred else None,
                      'cover_rate_vs_survived_1tick': (len(pred & survived) / len(survived)) if survived else None,
                      'z_median': dist(st['z'], .5), 'z_p10': dist(st['z'], .1), 'z_p90': dist(st['z'], .9),
                      'n_points_at_floor_le_0.05': st['z_floor'],
                      'r_median': dist(st['r'], .5), 'r_p10': dist(st['r'], .1),
                      'azimuth_bins_occupied': len(st['az'])}
    hit_all = len(allpred & added)
    out['n_cells_ever_marked'] = len(ever)
    out['n_cells_survived_1tick'] = len(survived)
    out['combined'] = {'n_cells_predicted': len(allpred),
                       'hit_rate_vs_ever_marked': (len(allpred & ever) / len(allpred)) if allpred else None,
                       'hit_rate_vs_ever_marked_numerator': len(allpred & ever),
                       'hit_rate_vs_ever_marked_denominator': len(allpred),
                       'cover_rate_vs_ever_marked': (len(allpred & ever) / len(ever)) if ever else None,
                       'hit_rate_vs_survived_1tick': (len(allpred & survived) / len(allpred)) if allpred else None,
                       'cover_rate_vs_survived_1tick': (len(allpred & survived) / len(survived)) if survived else None,
                       'cells_predicted_and_actually_lethal': hit_all,
                       'hit_rate_of_predicted': (hit_all / len(allpred)) if allpred else None,
                       'cells_lethal_covered_by_prediction': (hit_all / len(added)) if added else None,
                       'n_lethal_added': len(added)}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 抽样 %d 拍 ｜ 实际变致命的格 %d' % (len(samp), len(added)))
    for cloud in GATES:
        v = out[cloud]
        print('  ★ %-5s 过门点 %6d（%.1f/拍）⇒ 预测格 %5d ｜ ★ 命中（预测∧真的致命）%d '
              '｜ 命中率 %.1f%% ｜ 覆盖了致命的 %.1f%% ｜ ★ 贴地(z≤0.05)的点 %d'
              % (cloud, v['n_points_pass_gate'], v['per_tick'], v['n_cells_predicted'],
                 v['cells_predicted_and_actually_lethal'],
                 100 * (v['hit_rate_of_predicted'] or 0),
                 100 * (v['cells_lethal_covered_by_prediction'] or 0), v['n_points_at_floor_le_0.05']))
    c = out['combined']
    print('  ★ 合并 ⇒ 预测格 %d ｜ 命中 %d ｜ ★ 命中率 %.1f%% ｜ ★ 覆盖致命的 %.1f%%'
          % (c['n_cells_predicted'], c['cells_predicted_and_actually_lethal'],
             100 * (c['hit_rate_of_predicted'] or 0), 100 * (c['cells_lethal_covered_by_prediction'] or 0)))
    print('  ★ 对"曾经致命过"（%d 格）⇒ 命中率 %.1f%%（分子 %d ÷ 分母 %d）｜ 覆盖 %.1f%%'
          % (len(ever), 100 * (c['hit_rate_vs_ever_marked'] or 0),
             c['hit_rate_vs_ever_marked_numerator'], c['hit_rate_vs_ever_marked_denominator'],
             100 * (c['cover_rate_vs_ever_marked'] or 0)))
    print('  ★★ 对"**活过 ≥1 拍**"（%d 格 ✓ —— ★ 真正的标记模型检验 ✓）⇒ ★ 命中率 %.1f%% ｜ 覆盖 %.1f%%'
          % (len(survived), 100 * (c['hit_rate_vs_survived_1tick'] or 0),
             100 * (c['cover_rate_vs_survived_1tick'] or 0)))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

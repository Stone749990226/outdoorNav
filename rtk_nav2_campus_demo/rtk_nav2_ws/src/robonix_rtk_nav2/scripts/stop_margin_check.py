#!/usr/bin/env python3
"""★ 跑前**解析**余量自检（reviewer 2026-10-09 定，② 类裁定后加）：观察停点离"致命带"还剩多少。

**来历（一次真事故）**：`ramp7-aq` 的观察停点 `x=-3.5` 离 `seal_west` 内侧面 `x=-4.00` 只有 **0.50 m**，
扣掉 `inflation_radius` 0.25 ⇒ 余量 **0.250 m** ✗ ⇒ 车一过冲（实测停到 **`x=-3.8013`**）
就进了致命带，nav2 循环 `collision ahead` ×6 → `Controller patience exceeded` → `Aborting handle`，
**起点落在致命格 ⇒ 规划必失败 ⇒ 永远出不来** ✗（整轮作废）。

**为什么必须"解析算"**：掩膜/代价格是**跑完才有**的 ✗ ⇒ 跑前只能从**世界几何 + 配置值**算 ✓
⇒ 这样才能把"刀尖"变成**先算再做**，下一轮不会再撞 ✓。

**判据**：把停点到**任一"高出平台面的箱体"**（= 封边块，从 SDF 现解析 ✓）的最近距离，
扣掉 `inflation_radius`，须 ≥ `--min-margin`（默认 **0.5 m** ✓，reviewer 指定）。
**正负样本**：`x=-3.0` 应 **PASS**（余量 0.75 ✓）、`x=-3.5` 应 **FAIL**（余量 0.250 ✗）✓。

用法：
  stop_margin_check.py --world worlds/depth_drop_ramp.sdf --stop=-3.0,-2.0 --stop=-3.0,0.0 \
      [--inflation-radius 0.25] [--min-margin 0.5] [--json out.json]

★ **两种写法都收** ✓：`--stop=-3.0,-2.0` 与 `--stop -3.0,-2.0` 都可以 ✓ ——
  后者在**部分** Python/argparse 上会被当成选项 ⇒ `error: argument --stop: expected one argument` ✗
  （reviewer 2026-10-09 在容器里踩到，而我本机同一条命令**能跑** ✗ ⇒ 同一个脚本两个行为 ⇒
   **"我这儿能跑"不能当"用法没问题"** ✗）⇒ 见 `_normalize_argv()` ✓。

退出码：0 = 全部达线 ✓｜2 = 有停点余量不足 ✗｜1 = 用法/解析失败
"""
import argparse
import json
import math
import os
import re
import sys

DEFAULT_SDF = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'worlds', 'depth_drop_ramp.sdf')


def parse_raised_boxes(path):
    """从 SDF 解析出**顶面高出平台面（z>0）的箱体** ⇒ 这些就是"封边块" ✓（不硬编码名单 ✗）。

    返回 [{'name','x0','x1','y0','y1','top'}, ...]。
    ★ 用"顶面 z > 0"当判据、而不是写死 `seal_*` 名字 ✗：
      `upper_platform` 顶恰好 z=0 ⇒ 不算 ✓；`lower_platform` 顶 -0.30 ⇒ 不算 ✓；
      `ramp_wedge` 顶 ≈ -0.15 ⇒ 不算 ✓ ⇒ **只剩两块封边** ✓ —— 名字改了也不会漏 ✓。
    """
    s = open(path, encoding='utf-8').read()
    out = []
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', s, re.S):
        nm, body = m.group(1), m.group(2)
        pm = re.search(r'<pose>([^<]+)</pose>', body)
        sm = re.search(r'<size>([^<]+)</size>', body)
        if not (pm and sm):
            continue                       # 坡是 wedge、没有 <box><size> 语义 ⇒ 跳过 ✓
        try:
            px, py, pz = [float(v) for v in pm.group(1).split()[:3]]
            sx, sy, sz = [float(v) for v in sm.group(1).split()[:3]]
        except ValueError:
            continue
        top = pz + sz / 2.0
        if top <= 0.0:
            continue                       # 不高出平台面 ⇒ 不是"墙"、不产生致命带 ✓
        out.append({'name': nm, 'x0': px - sx / 2, 'x1': px + sx / 2,
                    'y0': py - sy / 2, 'y1': py + sy / 2, 'top': round(top, 4)})
    return out


def dist_point_to_rect(px, py, r):
    """点到轴对齐矩形的距离（点在矩形内 ⇒ 0，并给负号表示"在里面"）。"""
    dx = max(r['x0'] - px, 0.0, px - r['x1'])
    dy = max(r['y0'] - py, 0.0, py - r['y1'])
    if dx == 0.0 and dy == 0.0:
        # 在矩形内：到最近面的距离取负 ✓（这样"停点落在封边块里"会算成负余量、必然 FAIL ✓）
        inside = min(px - r['x0'], r['x1'] - px, py - r['y0'], r['y1'] - py)
        return -inside
    return math.hypot(dx, dy)


def _normalize_argv(argv):
    """★ 把 `--stop X,Y` 合写成 `--stop=X,Y` ✓ —— 消掉"值以 `-` 开头被当成选项"那个坑 ✓。

    来历：reviewer 2026-10-09 在容器里用 `--stop -3.5,-2` 得到
    `error: argument --stop: expected one argument` ✗，而**我本机同一条命令能跑** ✗。
    ⇒ **同一份脚本两个行为** ⇒ 与其争论"该用哪种写法"，不如**两种都收** ✓（下一个人不会再踩 ✓）。
    """
    out, i = [], 0
    while i < len(argv):
        if argv[i] == '--stop' and i + 1 < len(argv) and ',' in argv[i + 1]:
            out.append('--stop=' + argv[i + 1])
            i += 2
            continue
        out.append(argv[i])
        i += 1
    return out


def main():
    ap = argparse.ArgumentParser(
        epilog='★ 停点两种写法都收：--stop=-3.0,-2.0 或 --stop -3.0,-2.0 ✓')
    ap.add_argument('--world', default=DEFAULT_SDF)
    ap.add_argument('--stop', action='append', default=[],
                    help='观察停点，可重复：--stop=X,Y（写成 --stop X,Y 也收 ✓）')
    ap.add_argument('--inflation-radius', type=float, default=0.25,
                    help='部署值 inflation_radius（本项 0.25 ✓）；致命带内边界 = 封边面 + 该值 ✓')
    ap.add_argument('--min-margin', type=float, default=0.5,
                    help='★ 要求的余量下限 (m)（reviewer 2026-10-09 指定 0.5 ✓）')
    ap.add_argument('--json', default='')
    a = ap.parse_args(_normalize_argv(sys.argv[1:]))

    if not a.stop:
        print('!! 至少要给一个 --stop=X,Y ⇒ 退出码 1', file=sys.stderr)
        return 1
    world = os.path.abspath(a.world)
    if not os.path.exists(world):
        print('!! 找不到世界文件 %s ⇒ 退出码 1' % world, file=sys.stderr)
        return 1
    try:
        stops = [[float(v) for v in s.split(',')[:2]] for s in a.stop]
    except ValueError:
        print('!! --stop 必须是 X,Y ⇒ 退出码 1', file=sys.stderr)
        return 1

    boxes = parse_raised_boxes(world)
    if not boxes:
        print('!! 世界里没解析到"高出平台面的箱体" ⇒ 解析或世界不对 ⇒ 退出码 1', file=sys.stderr)
        return 1

    out = {'world': world,
           'world_sha256': __import__('hashlib').sha256(open(world, 'rb').read()).hexdigest(),
           'raised_boxes': [{'name': b['name'], 'x': [round(b['x0'], 3), round(b['x1'], 3)],
                             'y': [round(b['y0'], 3), round(b['y1'], 3)], 'top': b['top']} for b in boxes],
           'inflation_radius': a.inflation_radius, 'min_margin_m': a.min_margin, 'stops': []}
    bad = []
    for (sx, sy) in stops:
        best = None
        for b in boxes:
            d = dist_point_to_rect(sx, sy, b)
            margin = d - a.inflation_radius
            rec = {'box': b['name'], 'face_dist_m': round(d, 4), 'margin_m': round(margin, 4)}
            if best is None or margin < best['margin_m']:
                best = rec
        ok = best['margin_m'] >= a.min_margin
        if not ok:
            bad.append([sx, sy])
        out['stops'].append({'x': sx, 'y': sy, 'nearest': best, 'ok': ok})
        print('停点 (%+.2f,%+.2f)：最近封边 %-11s 面距 %.3f m − inflation %.2f = 余量 %+.3f m  %s'
              % (sx, sy, best['box'], best['face_dist_m'], a.inflation_radius, best['margin_m'],
                 '✓ 达线' if ok else '✗ **不足 %.2f m**' % a.min_margin))

    out['ok'] = not bad
    out['note'] = ('✅ 全部停点余量 ≥ %.2f m ✓（致命带内边界 = 封边面 + inflation %.2f ✓）'
                   % (a.min_margin, a.inflation_radius) if not bad else
                   '✗ **有余量不足的停点** %s ⇒ 车一过冲就会进致命带、**规划必失败、出不来** ⇒ 别跑 ✓'
                   % bad)
    print(out['note'])
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1, ensure_ascii=False)
    return 0 if not bad else 2


if __name__ == '__main__':
    sys.exit(main())

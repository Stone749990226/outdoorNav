#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_inflate_unknown.py —— ★ 检验"残留其实来自 `inflate_unknown`"这个候选 ✓（阶段十 ✓）

用法: check_inflate_unknown.py <probe目录> <残留CSV> [--X 0.10] [--out 新json]

★★ 假设（★ reviewer 2026-10-11 提 ✓，★ 假设 ✗ 不是结论 ✓）★★
  ★ 运行时参数里 ★ `inflation_layer.inflate_unknown = **true**` ✗ ⇒
  ★ **膨胀层会在"未知区"旁边自己造出代价** ✗ —— ★ **不需要任何观测** ✓。
  ★ 它能对上那四条**对不上的特征** ✓：★ ① 配不到观测（81% ✓）② 沿路线（★ 前沿就在车走过处 ✓）
  ③ 逐拍零散（★ 前沿每拍长一点 ✓）④ 在路面上（★ 前沿压在路面与未知交界 ✓）。

★★ 检验（★ 纯离线 ✓，✗ 不起仿真 ✗）★★
  ★ 取"两边都不沾"那批格 ✓ ⇒ ★ 算**每一格到"最近的未知格"的距离** ✓（★ 末拍整幅图上做**多源 BFS** ✓）
  ★ **判据（★ 两头 ✓）**：
    ★ a. 若那批格里**大部分 ≤ `inflation_radius`（0.25 m ✓）** ⇒ ★ **就是它** ✓；
    ★ b. ★ **对照** = 另取一批**明确不是残留**的**自由格** ✓ ⇒ ★ 它们的这个距离应当**明显更大** ✓
      （★ 缺 b ⇒ ★ 判据等于没有 ✓ —— 本阶段已经栽过两次"尺子太松"✗）
退出码: 0 = 跑通 ｜ 2 = 读不到图
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import sys


def load_grid(probe_dir):
    """★ 取**最后一拍**的整幅图 ✓（★ 残留是"后半段每拍都占用"⇒ 末态就是它 ✓）"""
    maps = sorted(glob.glob(os.path.join(probe_dir, 'map_*.bin')))
    js = sorted(glob.glob(os.path.join(probe_dir, 'map_*.json')))
    if not maps or not js:
        return None, None
    info = json.load(open(js[-1], encoding='utf-8'))
    blob = open(maps[-1], 'rb').read()
    return info, blob


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('residue_csv')
    ap.add_argument('--X', type=float, default=0.10)
    ap.add_argument('--cols', default='/home/xjy/.claude/jobs/3e3b5c65/tmp/cols.json')
    ap.add_argument('--geo', default='/home/xjy/projects/outdoorNav/gazebo_campus/generated/scene_geometry.json')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    info, blob = load_grid(a.probe_dir)
    if info is None or blob is None:
        print('✗ 读不到图 ⇒ ★ 记成"未取到" ✗')
        return 2
    W, H, res = info['width'], info['height'], info['resolution']
    ox, oy = info['origin']['x'], info['origin']['y']
    print('  ★ 用**末拍**那张图 ✓：%d×%d @%.3f m ✓' % (W, H, res))

    # ★ 未知格 = wire 上是 255（= −1 ✓）⇒ ★ 多源 BFS 求"到最近未知格"的距离（格数 ✓）
    INF = 10 ** 9
    dist = [INF] * (W * H)
    dq = collections.deque()
    for i, b in enumerate(blob):
        if b >= 128:                      # ★ 未知（−1 → 255 ✓）
            dist[i] = 0
            dq.append(i)
    while dq:
        i = dq.popleft()
        x, y = i % W, i // W
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and dist[ny * W + nx] > dist[i] + 1:
                dist[ny * W + nx] = dist[i] + 1
                dq.append(ny * W + nx)
    print('  ★ 已算完"到最近未知格的距离场" ✓（多源 BFS ✓，单位=格 ✓）')

    def d_unknown(wx, wy):
        gx, gy = int((wx - ox) / res), int((wy - oy) / res)
        if not (0 <= gx < W and 0 <= gy < H):
            return None
        return dist[gy * W + gx] * res      # ★ 换算成米 ✓

    # ★ 把残留里"两边都不沾"那批挑出来 ✓（★ 与上一轮同口径 ✓）
    allc = json.load(open(a.cols, encoding='utf-8'))
    cols = [c for c in allc if c['n'] != 'ground']
    rows = list(csv.DictReader(open(a.residue_csv, encoding='utf-8')))

    def is_pair(v):
        return isinstance(v, list) and len(v) == 2 and all(isinstance(t, (int, float)) for t in v)

    def rings_of(g):
        out = []
        def walk(c):
            if is_pair(c):
                return
            if isinstance(c, list) and c and is_pair(c[0]):
                out.append(c)
                return
            for x in c:
                walk(x)
        walk(g)
        return out

    G = json.load(open(a.geo, encoding='utf-8'))
    edges = []
    for geom in (G['free']['coordinates'], G['plaza']['coordinates']):
        for ring in rings_of(geom):
            for i in range(len(ring) - 1):
                edges.append((ring[i][0], ring[i][1], ring[i + 1][0], ring[i + 1][1]))

    def d_seg(px, py, e):
        x0, y0, x1, y1 = e
        dx, dy = x1 - x0, y1 - y0
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
        return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))

    def d_mesh(px, py):
        return min((d_seg(px, py, e) for e in edges), default=1e9)

    def d_col(px, py):
        best = 1e9
        for c in cols:
            d = (math.hypot(max(0, abs(px - c['x']) - c['hx']), max(0, abs(py - c['y']) - c['hy']))
                 if c['k'] == 'box' else max(0.0, math.hypot(px - c['x'], py - c['y']) - c['hx']))
            if d < best:
                best = d
        return best

    unexp = []
    for r in rows:
        px, py = float(r['wx']), float(r['wy'])
        if d_mesh(px, py) > a.X and d_col(px, py) > a.X:
            unexp.append((px, py))

    du = sorted(v for v in (d_unknown(px, py) for px, py in unexp) if v is not None)
    print()
    print('  ★★ a. 那 **%d** 格到**最近未知格**的距离 ✓：' % len(du))
    print('     min %.2f ｜ p25 %.2f ｜ ★ **中位 %.2f** ｜ p75 %.2f ｜ p90 %.2f ｜ max %.2f m'
          % (du[0], du[int(len(du) * .25)], du[len(du) // 2], du[int(len(du) * .75)],
             du[int(len(du) * .90)], du[-1]))
    for X in (0.0, 0.10, 0.25, 0.50, 1.00):
        k = sum(1 for v in du if v <= X + 1e-9)
        print('       ★ ≤ %.2f m（★ 0.25 = `inflation_radius` ✓）：%5d（%5.1f%%）'
              % (X, k, 100.0 * k / max(1, len(du))))

    # ★★ b. 对照：★ 取一批**明确不是残留**的**自由格** ✓
    occ = {int(r['j']) * W + int(r['i']) for r in rows if r.get('i') and r.get('j')}
    # ★★ 2026-10-11 修 ✗：★ 第一版步长取 7 ⇒ ★ 只拿到 **3 个**对照格 ✗（★ 全图 98% 是未知 ✓、
    #   还被"排除残留格"再削一刀 ✓）⇒ ★ **3 个格算不出分布** ✗ ⇒ ★ 那个"不支持"的判决**不可信** ✗。
    #   ⇒ ★ 改成 ★ **逐个自由格**（★ 自由格本来就不多 ✓）并**上限 20000** ✓。
    ctrl = []
    for k in range(W * H):
        if blob[k] != 0:                       # ★★ 只看**自由格**（★ 值恰为 0 ✓）
            continue                           #   ★ 第一版写反了 ✗（`or blob[k] == 0` ⇒ 把自由格全跳过 ✗）
                                               #   ⇒ ★ 对照只剩 123 个 ✗ ⇒ ★ 已修 ✓
        if k in occ:
            continue                           # ★ 排除残留格 ✓
        gx, gy = k % W, k // W
        ctrl.append((ox + (gx + 0.5) * res, oy + (gy + 0.5) * res))
        if len(ctrl) >= 20000:
            break
    dc = sorted(v for v in (d_unknown(px, py) for px, py in ctrl) if v is not None)
    print()
    print('  ★★ b. **对照**：★ 随机自由格 **%d** 个 ✓ 到最近未知格的距离 ✓：' % len(dc))
    print('     min %.2f ｜ p25 %.2f ｜ ★ **中位 %.2f** ｜ p75 %.2f ｜ p90 %.2f ｜ max %.2f m'
          % (dc[0], dc[int(len(dc) * .25)], dc[len(dc) // 2], dc[int(len(dc) * .75)],
             dc[int(len(dc) * .90)], dc[-1]))
    for X in (0.25, 0.50, 1.00):
        k = sum(1 for v in dc if v <= X + 1e-9)
        print('       ★ ≤ %.2f m：%5d（%5.1f%%）' % (X, k, 100.0 * k / max(1, len(dc))))

    ka = sum(1 for v in du if v <= 0.25 + 1e-9) / float(max(1, len(du)))
    kb = sum(1 for v in dc if v <= 0.25 + 1e-9) / float(max(1, len(dc)))
    print()
    print('  ★★ 判读（★ ≤0.25 m 那档 ✓）：★ 残留格 **%.1f%%** ｜ ★ 对照自由格 **%.1f%%**'
          % (100.0 * ka, 100.0 * kb))
    if ka >= 0.7 and ka > kb * 2:
        print('     ⇒ ★★ **支持"就是它"** ✓（★ 残留格紧贴未知区、而对照不紧贴 ✓）')
    elif ka > kb * 2:
        print('     ⇒ ★ **部分支持** ✓（★ 残留明显更贴未知区，但比例不足 70%% ✓）')
    else:
        print('     ⇒ ★ **不支持** ✗（★ 两批贴得差不多 ⇒ ★ 不是"紧邻未知"这个特征 ✓）')
    out = {'a_residue_d_unknown_m': {'n': len(du), 'min': du[0], 'p25': du[int(len(du) * .25)],
                                     'median': du[len(du) // 2], 'p75': du[int(len(du) * .75)],
                                     'p90': du[int(len(du) * .90)], 'max': du[-1],
                                     'frac_le_0.25': ka},
           'b_control_d_unknown_m': {'n': len(dc), 'median': dc[len(dc) // 2],
                                     'frac_le_0.25': kb},
           'inflation_radius_m': 0.25, 'supported': bool(ka > kb * 2)}
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

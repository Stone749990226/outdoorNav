#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lethal_per_meter.py —— ★ 二分要用的**归一化指标** ✓（阶段十 ✓，★ 纯离线 ✓）

用法: lethal_per_meter.py <probe目录> --out <新json> [--at-frac 1.0] [--traj <另一个probe目录>]

★★ 指标被**写死**成这样（★ reviewer + 业主 2026-10-11 定 ✓）★★
    ★★ **致命格数 ÷ 轨迹长度（米）** ✓ ＝ `lethal_per_m` ✓
  ★ 为什么不用"残留总数"✗：★ 总数里 **68.2% 是膨胀晕** ✗（★ 前一阶段实测 ✓），
    ★ 晕随致命格**线性**铺开 ⇒ ★ 跨轮比总数会被**放大** ✗；★ 且各轮**长度不同** ✗ ⇒ 必须归一化 ✓。
  ★ **产物里同时记分子与分母** ✓（★ 报数带口径 ✓ —— 只给比值 ⇒ 复算方只能采信转述 ✗）：
    ★ `n_lethal`（分子 ✓）、`traj_len_m`（分母 ✓）、`lethal_per_m`（比值 ✓）；
    ★ 另记 `n_inflated` 作参考 ✓（★ 但要标明"✗ 不进指标" ✗）。

★ 口径（★ 都写清楚 ✓，✗ 不留默认值不说 ✗）：
  · ★ 取**末拍**的整幅图 ✓（★ 与"残留"口径一致 ✓）
  · ★ 致命 = 值 ≥ 99 ✓（★ nav2 约定 ✓）；膨胀 = 1..98 ✓；未知 = −1 ✓；自由 = 0 ✓
  · ★ 轨迹长度 = ★ 探针 `tick_*.json` 里 `robot_xy_map` 的**相邻位移之和** ✓（★ 米 ✓）
退出码: 0 = 跑通 ｜ 2 = 读不到图
"""
import argparse
import glob
import json
import math
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--out', required=True)
    ap.add_argument('--at-frac', type=float, default=1.0, help='★ 取第几拍的图（1.0 = 末拍 ✓）')
    a = ap.parse_args()

    maps = sorted(glob.glob(os.path.join(a.probe_dir, 'map_*.bin')))
    ticks = sorted(glob.glob(os.path.join(a.probe_dir, 'tick_*.json')))
    if not maps or not ticks:
        print('✗ 读不到图 ⇒ ★ 记成"未取到" ✗')
        return 2
    idx = max(0, min(len(maps) - 1, int(len(maps) * a.at_frac) - 1))
    blob = open(maps[idx], 'rb').read()
    n_lethal = sum(1 for b in blob if 99 <= b <= 100)
    n_infl = sum(1 for b in blob if 1 <= b <= 98)
    n_unknown = sum(1 for b in blob if b >= 128)
    n_free = sum(1 for b in blob if b == 0)

    traj = 0.0
    prev = None
    for f in ticks:
        d = json.load(open(f, encoding='utf-8'))
        r = d.get('robot_xy_map')
        if r and prev:
            traj += math.hypot(r[0] - prev[0], r[1] - prev[1])
        if r:
            prev = r

    per_m = (n_lethal / traj) if traj > 1e-9 else None
    out = {'probe_dir': a.probe_dir, 'map_file': os.path.basename(maps[idx]),
           'grid_cells': len(blob),
           'n_lethal': n_lethal,                 # ★ 分子 ✓
           'traj_len_m': round(traj, 3),         # ★ 分母 ✓
           'lethal_per_m': (round(per_m, 4) if per_m is not None else None),
           'n_inflated': n_infl,                 # ★ 参考（✗ 不进指标 ✗）
           'n_unknown': n_unknown, 'n_free': n_free,
           'metric': '★ lethal_per_m = n_lethal ÷ traj_len_m ✓（★ 写死 ✓，✗ 不用残留总数 ✗）',
           'note': ('★ 分子分母都记了 ✓ ⇒ 复算方能自己核 ✓；'
                    '★ n_inflated 只是参考 ✓（★ 那是膨胀晕 ✗，✗ 不进指标 ✗）')}
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=2)
    print('  ★ 图 = %s（第 %d/%d 张 ✓）' % (out['map_file'], idx + 1, len(maps)))
    print('  ★ 分子 **n_lethal = %d** ✓ ｜ ★ 分母 **traj_len = %.2f m** ✓ ｜ ★★ **lethal_per_m = %s** ✓'
          % (n_lethal, traj, ('%.4f' % per_m) if per_m is not None else '（★ 轨迹为 0 ⇒ 未取到 ✗）'))
    print('  ★ 参考（✗ 不进指标 ✗）：n_inflated %d ｜ n_unknown %d ｜ n_free %d ✓'
          % (n_infl, n_unknown, n_free))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

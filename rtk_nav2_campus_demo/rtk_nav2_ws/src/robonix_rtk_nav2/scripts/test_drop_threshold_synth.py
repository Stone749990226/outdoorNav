#!/usr/bin/env python3
"""② `drop_threshold` 降线前的**两张合成表**（reviewer 2026-10-08 定，不花栈）。

表一（靶子）：南沿几何在**卡死点那一档** `H = 0.1219 m` 附近扫 `H ∈ {0.10,0.11,0.12,0.1219,0.13}`，
   对每个 `H` 报 `T = 0.15` vs `0.10` 的**逐格**（边缘格集合 + 差集）以及**框架级跨度**
   （跨度 <0.5 的就算出了列，节点也不会确认 ⇒ 表里直接标出来，避免高估）。
表二（代价）：**顺坡下行/上行**（视线沿坡）时 `T = 0.15` vs `0.10` 的出列数 ——
   预期仍 0；**若非 0 就是"降线换坡面假阳性"，② 当场停。**

几何与渲染复用 `test_ground_ref_synth.py`（解析场景 + 针孔渲染 + 空洞守卫思路）。
"""
import importlib.util
import sys

import numpy as np

_HERE = '/work/rtk_nav2_ws/src/robonix_rtk_nav2/scripts'
sys.path.insert(0, _HERE)
spec = importlib.util.spec_from_file_location('h', _HERE + '/test_ground_ref_synth.py')
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)

spec2 = importlib.util.spec_from_file_location(
    'dd', '/work/rtk_nav2_ws/src/robonix_rtk_nav2/robonix_rtk_nav2/depth_drop.py')
DD = importlib.util.module_from_spec(spec2)
spec2.loader.exec_module(DD)

INTR = (120.0, 120.0, 160.0, 120.0)
CH, GT = 0.55, 0.06
X_OF_H = lambda hh: 0.8 + (0.15 - hh) / 0.18752      # H(x) 的反函数


def cells_for(x, y, yaw, roll, T):
    R = H.body_to_map(yaw, roll) @ H.R_CAM2BODY
    t = np.array([x, y, float(H.z_surf_vec(np.array(x), np.array(y))) + CH])
    depth = H.render(R, t, INTR)
    if float(np.isfinite(depth).mean()) < 0.20:
        return None, 0.0, '空洞守卫：合成无效'
    e, _, _, hd = DD.edge_columns(depth, INTR, H.Tf(R, t), CH, T, GT, ground_ref='plane')
    ea = np.asarray(e).reshape(-1, 2)
    if len(ea) >= 2:
        axis = np.array([-hd[1], hd[0]])
        span = float(np.ptp(ea @ axis))
    else:
        span = 0.0
    return {tuple(np.round(q, 3)) for q in ea}, span, ''


def main():
    print('=== 表一：南沿靶子（沿坡南行姿态，离坎 1.0 m；坎在 y=0.5）===')
    print('%-9s %-6s | %-28s | %-28s | %s' % ('H(m)', 'x_坎', 'T=0.15 格数/跨度/确认?',
                                              'T=0.10 格数/跨度/确认?', '降线新增格'))
    for hh in (0.100, 0.110, 0.120, 0.1219, 0.130):
        xh = X_OF_H(hh)
        y = 1.5
        s15, sp15, err = cells_for(xh, y, -np.pi / 2, 0.18534, 0.15)
        s10, sp10, err2 = cells_for(xh, y, -np.pi / 2, 0.18534, 0.10)
        if s15 is None or s10 is None:
            print('%.4f  %.3f  %s' % (hh, xh, err or err2))
            continue
        ok15 = 'Y' if (len(s15) >= 12 and sp15 >= 0.5) else 'n'
        ok10 = 'Y' if (len(s10) >= 12 and sp10 >= 0.5) else 'n'
        new = sorted(s10 - s15)
        print('%-9.4f %-6.3f | %3d 格  %.3f m  %s        | %3d 格  %.3f m  %s        | %d %s'
              % (hh, xh, len(s15), sp15, ok15, len(s10), sp10, ok10, len(new),
                 ('前3:' + str(new[:3])) if new else ''))

    print()
    print('=== 表二：代价 —— 视线沿坡（顺坡下行 / 逆坡上行），看坡面自身会不会被标 ===')
    print('%-26s | %-16s | %-16s' % ('姿态', 'T=0.15 格数/跨度', 'T=0.10 格数/跨度'))
    for name, x, y, yaw in (
            ('顺坡下行（x=0.5→东）', 0.5, 1.5, 0.0),
            ('逆坡上行（x=1.2→西）', 1.2, 1.5, np.pi),
            ('坡上朝南（对照：该出南沿）', 0.5, 1.3, -np.pi / 2)):
        s15, sp15, e1 = cells_for(x, y, yaw, 0.18534, 0.15)
        s10, sp10, e2 = cells_for(x, y, yaw, 0.18534, 0.10)
        if s15 is None or s10 is None:
            print('%-26s | %s' % (name, e1 or e2))
            continue
        print('%-26s | %3d 格  %.3f m     | %3d 格  %.3f m'
              % (name, len(s15), sp15, len(s10), sp10))
    print()
    print('注：格数 <12 或跨度 <0.5 ⇒ 节点**不会确认**（框架级门 min_columns=12 / ptp(across)≥0.5）')


if __name__ == '__main__':
    main()

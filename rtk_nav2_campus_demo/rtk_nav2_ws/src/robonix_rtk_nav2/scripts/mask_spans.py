#!/usr/bin/env python3
"""量 mask 在坡道上的**自由带**（reviewer 的口径 b）与新口径 a 的框内计数。

判据：
  b. 坡道上每列（沿 x）连续无致命格的 y 跨度 ≥0.5 m；
  a. 框 x∈[0.15,1.5], y∈[0.65,2.85] 内 0 格 hazard。
用法：mask_spans.py --json out.json --cols 0.3,0.8,1.3,1.58
"""
import argparse, json, time
import numpy as np, rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE)
RAMP = (0.0, 1.65, 0.5, 3.0)      # 坡道足迹
# 审计框 a（"坡面可行走区 0 格"）。**默认值 = 外扩 0.15 时代的口径**（历史轮次可复现）；
# 外扩改成 hazard_margin=0.30 后，干净区口径 = 到真落差线段 ≥0.30 m ⇒ 坡面即
# (0.30, 1.60, 0.80, 2.70)（见 docs/rtk-nav2-ramp.md §7.18），用 `--box-a` 显式传。
BOX_A = (0.15, 1.5, 0.65, 2.85)
BOX_A_CLEAN30 = (0.30, 1.60, 0.80, 2.70)   # margin=0.30 口径


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--cols', default='0.3,0.8,1.3,1.58')
    ap.add_argument('--seconds', type=float, default=20.0)
    # 审计框 a：默认历史 0.15 口径；传 'clean30' 用 margin=0.30 的干净区口径（§7.18）
    ap.add_argument('--box-a', default='', help="x0,x1,y0,y1 或 clean30")
    a = ap.parse_args()
    cols = [float(v) for v in a.cols.split(',')]
    global BOX_A
    if a.box_a == 'clean30':
        BOX_A = BOX_A_CLEAN30
    elif a.box_a:
        BOX_A = tuple(float(v) for v in a.box_a.split(','))
    rclpy.init(); n = Node('mask_spans'); d = {}
    n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map',
                          lambda m: d.__setitem__('m', m), Q)
    t = time.time()
    while time.time() - t < a.seconds and 'm' not in d:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = d.get('m')
    out = {'has_mask': m is not None}
    if m is not None:
        i = m.info; res = i.resolution
        g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
        X = i.origin.position.x + (np.arange(i.width) + 0.5) * res
        Y = i.origin.position.y + (np.arange(i.height) + 0.5) * res
        leth = g >= 100
        def count(x0, x1, y0, y1, box_leth=leth):
            mx = (X >= x0) & (X <= x1); my = (Y >= y0) & (Y <= y1)
            return int(box_leth[np.ix_(my, mx)].sum())
        out['mask_cells_total'] = int(leth.sum())
        out['ramp_footprint_cells'] = count(*RAMP)
        out['audit_box_a_cells'] = count(*BOX_A)
        spans = {}
        for c in cols:
            j = int(np.argmin(np.abs(X - c)))
            col = leth[:, j]
            best = cur = 0
            for k in range(len(Y)):
                if RAMP[2] <= Y[k] <= RAMP[3]:
                    if col[k]:
                        cur = 0
                    else:
                        cur += 1
                        best = max(best, cur)
                else:
                    cur = 0
            spans[str(c)] = round(best * res, 3)
        out['column_free_span_m'] = spans
        out['pass_b_all_ge_0.5'] = all(v >= 0.5 for v in spans.values())
        out['pass_a_zero'] = out['audit_box_a_cells'] == 0
    with open(a.json, 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

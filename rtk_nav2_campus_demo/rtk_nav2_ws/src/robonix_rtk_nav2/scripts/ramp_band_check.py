#!/usr/bin/env python3
"""★ 新世界验收读数 ③（reviewer 2026-10-08 加）：**坡口与坡面不能被"自己的标记"堵死**。

依据（本项目先例）：§7.7 —— 坡道入口被标出 hazard、**把车困在 `x=0.12`** ⇒ 换新世界后
"坡面自身被当落差"仍是**最大风险** ✓。

三条判据：
· **① 坡口（`y=3`）可通行带 ≥ 1.4 m** ✓（扣掉坡两侧沿的外扩）；
· **② 坡面中线（`x` 中心，`y∈[3,4.6]`）全程无致命格** ✓；
· **③ ★ 负向 = 诊断项（`N1`/`N2`），不阻塞** ✓（reviewer 2026-10-09 更正：坡口两端紧邻真台阶 ⇒ **外扩必然从角部渗进** ⇒
  **角部命中是几何必然、不是缺陷** ✓）⇒ 报**原始命中** + **排除两端 `--corner` 后的命中**；只有**角部之外**仍有命中才值得查 ✓。

用法：ramp_band_check.py --json out.json [--mask-seconds 60] [--margin 0.30]
"""
import argparse
import json
import os
import sys
import time

# ★ **`numpy` 可以放顶层**（它是纯数值库、宿主机也有 ✓，且模块级 `free_band_at()` 要用它 ✓）；
#   **`rclpy`/ROS 必须惰性导入**（只在"取数"路径里 import ✓）——
#   这样 `--from-json` 这条**纯离线**路径在**没装 ROS 的机器上也能跑** ✓
#   （与 `plan_vs_wall.py --rederive` / `main_edge_gap_check.py --from-attribution` 同规格 ✓；
#    踩过两次：把 ROS 放顶层 ⇒ 离线重算直接 ModuleNotFoundError ✗）
import numpy as np  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_edges as WE  # noqa: E402


def free_band_at(mx, my, y, x_lo, x_hi, half):
    """在 y 处、x∈[x_lo,x_hi] 内找"含中心点"的最大无格连续带，返回 (lo, hi, 宽)。

    做法：把 x 区间切成 1 cm 采样，落在任一格 half 半径内的采样点算"占用"。
    """
    xs = np.arange(x_lo, x_hi + 1e-9, 0.01)
    if len(mx) == 0:
        return (x_lo, x_hi, x_hi - x_lo)
    occ = np.zeros(len(xs), dtype=bool)
    near = np.abs(my - y) <= half
    if np.any(near):
        X = mx[near]
        for i, xv in enumerate(xs):
            if np.any(np.abs(X - xv) <= half):
                occ[i] = True
    # 找含 x=0 侧中心的连续自由段：取中点
    mid = (x_lo + x_hi) / 2.0
    j = int(np.argmin(np.abs(xs - mid)))
    if occ[j]:
        return (None, None, 0.0)
    lo = j
    while lo - 1 >= 0 and not occ[lo - 1]:
        lo -= 1
    hi = j
    while hi + 1 < len(xs) and not occ[hi + 1]:
        hi += 1
    return (float(xs[lo]), float(xs[hi]), float(xs[hi] - xs[lo]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--from-json', default='',
                    help='★ 离线重算：读一份已存的 ramp_band_*.json（须含 cells_xy）⇒ **同一段判据代码**重算 ✓；'
                         '**不起栈、不订阅** ✓（reviewer 2026-10-09 要求：结论要能被重算，不能只被相信 ✓）')
    ap.add_argument('--mask-seconds', type=float, default=60.0)
    ap.add_argument('--margin', type=float, default=0.30, help='外扩量（= hazard_margin 部署值）')
    ap.add_argument('--band-min', type=float, default=1.4, help='坡口可通行带下限 (m)')
    ap.add_argument('--neg-tol', type=float, default=0.10, help='负向**诊断项**的容差 (m)')
    ap.add_argument('--corner', type=float, default=0.35,
                    help='负向诊断项里 角部渗出 的界定距离 (m)：距任一端点小于此值即记作角部、不计入可疑命中')
    a = ap.parse_args()

    # ★★ 离线重算（reviewer 2026-10-09 要求）：**同一段代码、只把 cells 的来源换掉** ✓
    #   来历：本工具原是"活测"工具 ✗ ⇒ 喂它存好的产物只会报 `UNMEASURED` ✗（`--json` 是**输出**路径 ✗）
    #   ⇒ **结论不能被重算** ✗ ⇒ 加 `--from-json`：读产物里的 `cells_xy` 当作 mx/my ⇒ **下面一行不改** ✓
    #   （不复制判据代码 ⇒ 不存在"两份实现漂移" ✓；产物从本版起都落 `cells_xy` ✓）
    if a.from_json:
        d0 = json.load(open(a.from_json))
        cells = d0.get('cells_xy')
        if not cells:
            print('!! %s 里没有 `cells_xy` ⇒ **无法离线重算**（旧产物没落 cells）⇒ 退出码 1' % a.from_json)
            return 1
        mx = np.array([float(c[0]) for c in cells])
        my = np.array([float(c[1]) for c in cells])
        out = {'world_sha256': WE.WORLD_SHA256, 'has_mask': True, 'margin': a.margin,
               'source': 'offline:%s (cells_xy)' % os.path.basename(a.from_json),
               'rederived': True, 'n_lethal_cells': len(cells)}
        out['mask_bbox'] = ([round(float(mx.min()), 3), round(float(my.min()), 3),
                             round(float(mx.max()), 3), round(float(my.max()), 3)] if len(cells) else None)
        print(json.dumps({'rederived': True, 'n_cells': len(cells)}, ensure_ascii=False))
    else:
        # ★ 以下才需要 ROS（惰性导入 ✓；numpy 已在模块级 ✓）
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        rclpy.init()
        n = Node('ramp_band_check')
        d = {}
        n.create_subscription(OccupancyGrid, '/rtk/drop/mask_map', lambda m: d.__setitem__('m', m), Q)
        t = time.time()
        while time.time() - t < a.mask_seconds and 'm' not in d:
            rclpy.spin_once(n, timeout_sec=0.2)

        out = {'world_sha256': WE.WORLD_SHA256, 'has_mask': 'm' in d, 'margin': a.margin}
        if 'm' not in d:
            # ★ "未发布 / 没等到" ≠ "0 格" ✗
            out['verdict'] = 'UNMEASURED'
            out['note'] = 'mask 未在等待窗口内到达 ⇒ **未测**（不是"无格/通过" ✗）'
            with open(a.json, 'w') as f:
                json.dump(out, f, indent=1, ensure_ascii=False)
            print(json.dumps(out, indent=1, ensure_ascii=False))
            n.destroy_node()
            rclpy.shutdown()
            return

        m = d['m']
        i = m.info
        g = np.asarray(m.data, dtype=np.int16).reshape(i.height, i.width)
        ys, xs = np.nonzero(g >= 100)
        mx = i.origin.position.x + (xs + 0.5) * i.resolution if len(xs) else np.array([])
        my = i.origin.position.y + (ys + 0.5) * i.resolution if len(ys) else np.array([])
        # ★ 落 cells ✓（供离线复算；没有它，本工具的结论就只能被相信 ✗）
        out['cells_xy'] = [[round(float(x), 3), round(float(y), 3)] for x, y in zip(mx, my)]
        out['n_lethal_cells'] = int(len(xs))

    # ① 坡口可通行带（y=3 处，横跨高台北沿那一段 x∈[-4,0]）
    lo, hi, w = free_band_at(mx, my, WE.RAMP_Y0, WE.PLAT_X0, WE.PLAT_X1, a.margin / 2)
    out['mouth_band'] = {'y': WE.RAMP_Y0, 'x_lo': None if lo is None else round(lo, 3),
                         'x_hi': None if hi is None else round(hi, 3), 'width_m': round(w, 3)}
    out['mouth_band_ok'] = bool(w >= a.band_min)
    out['mouth_band_note'] = ('✅ 宽 %.2f m ≥ %.1f m' % (w, a.band_min) if out['mouth_band_ok']
                              else '✗ 宽 %.2f m < %.1f m ⇒ **坡口被自己的标记堵住**' % (w, a.band_min))

    # ② 坡面中线：x 中心、y∈[3,4.6] 全程无致命格
    xc = (WE.RAMP_X0 + WE.RAMP_X1) / 2.0
    hit = []
    for yv in np.arange(WE.RAMP_Y0, WE.RAMP_Y1 + 1e-9, 0.05):
        if len(mx) and np.any((np.abs(mx - xc) <= a.margin / 2) & (np.abs(my - yv) <= a.margin / 2)):
            hit.append(round(float(yv), 3))
    out['centerline_lethal_y'] = hit
    out['centerline_ok'] = (len(hit) == 0)
    out['centerline_note'] = ('✅ 中线 (x=%.2f) 全程无致命格' % xc if out['centerline_ok']
                              else '✗ 中线在 y=%s 处被标 ⇒ 坡面自身被当落差' % hit[:8])

    # ③ ★ 负向：**诊断项、不阻塞** ✓ —— reviewer 2026-10-09 更正：
    #   坡口两端紧邻真台阶（x<-3.25 / x>-0.75 都是 0.30 m 台阶）⇒ **外扩必然从角部渗进 N1/N2 线** ⇒
    #   **角部命中是几何必然、不是缺陷** ✓ ⇒ 报**原始命中** + **排除两端 --corner 后的命中** ✓；
    #   只有**角部之外**仍有命中才值得查 ✓。
    neg = {}
    for name, (p1, p2) in WE.NOT_EDGES:
        cnt, cnt_c, sample = 0, 0, []
        for t_ in np.arange(0, 1.0001, 0.02):
            px = p1[0] + t_ * (p2[0] - p1[0])
            py = p1[1] + t_ * (p2[1] - p1[1])
            if not (len(mx) and np.any(np.hypot(mx - px, my - py) <= a.neg_tol)):
                continue
            cnt += 1
            if min(np.hypot(px - p1[0], py - p1[1]), np.hypot(px - p2[0], py - p2[1])) < a.corner:
                continue                      # 距任一端点 < corner ⇒ 记作"角部渗出"、不计入可疑命中 ✓
            cnt_c += 1
            if len(sample) < 5:
                sample.append([round(px, 3), round(py, 3)])
        neg[name] = {'hit_samples': cnt, 'hit_samples_excl_corner': cnt_c,
                     'corner_m': a.corner, 'examples_excl_corner': sample}
    out['negative_edges'] = neg
    # ★ 诊断项：**不阻塞** ✓（角部渗出是几何必然 ✓）；仍报出来供对照 ✓
    neg_bad = {k: v['hit_samples_excl_corner'] for k, v in neg.items() if v['hit_samples_excl_corner'] > 0}
    out['negative_diagnostic_only'] = True
    out['negative_note'] = (
        '（**诊断项、不阻塞** ✓）两条 0 落差边：原始命中 %s；**排除两端 %.2f m 角部后** %s ⇒ %s'
        % ({k: v['hit_samples'] for k, v in neg.items()}, a.corner,
           {k: v['hit_samples_excl_corner'] for k, v in neg.items()},
           '角部之外的命中为 0 ✓（角部渗出属几何必然 ✓）' if not neg_bad
           else '**角部之外仍有命中** %s ⇒ 值得查（不是角部渗出能解释的 ✗）' % neg_bad))

    # ★ 判据 = 坡口带 ✓ ∧ 中线干净 ✓（**负向只作诊断、不进判据** ✓ —— reviewer 2026-10-09 更正 ✓）
    out['verdict'] = ('PASS' if (out['mouth_band_ok'] and out['centerline_ok']) else 'FAIL')
    with open(a.json, 'w') as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    if not a.from_json:          # ★ 离线路径**没有** n/rclpy ✓（只有 live 路径才初始化过 ✓）
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()

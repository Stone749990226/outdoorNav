#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""选放箱点：**相机光轴正前方约 2 m、开阔无遮挡**，放之前先核"该方位 ~2 m 处无回波"。

为什么不是"按世界文件猜一个固定点"：相机光轴方向**由当帧 TF 决定**（车停在哪、朝哪
都要现算），固定点只在"车每次停在同一个位姿"时才成立。这里**一律用活节点 TF 实算**。

判据（放之前必须全过）：
  ① 该点在**全局 raw 图**里是 free（0），且 5×5 邻域全 free ⇒ 图上也认为那儿没东西；
  ② 该方位的 scan 束（±2 束）在 2.0 m 附近**没有回波**（全 `inf`，或最近的有限值 ≫ 2 m）
     ⇒ "该方位 ~2 m 处无回波" ✓。
先试光轴 0°，不过就按 ±5°、±10°… 依次试，**优先取最靠近光轴的**。

用法（容器内）：
  pickbox.py --dist 2.0 --max-offset 20 --out /work/.../r4_pickbox.json
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=2.0)
    ap.add_argument("--max-offset", type=float, default=20.0)
    ap.add_argument("--step", type=float, default=5.0)
    ap.add_argument("--fixed", default="",
                    help="**不改选点**：只核这个 map 坐标 x,y 是否仍满足三条判据"
                         "（图上 free / 5×5 全 free / 该方位 ~dist 处无回波）——"
                         "复跑轮要'用同一对放/移点'时，用它给出**这些点仍然合格**的证据")
    ap.add_argument("--timeout", type=float, default=40.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    rclpy.init()
    node = Node("pickbox")
    import tf2_ros
    from geometry_msgs.msg import PointStamped
    from tf2_geometry_msgs import do_transform_point
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    node.create_timer(0.05, lambda: None)

    from nav2_msgs.msg import Costmap
    from sensor_msgs.msg import LaserScan
    gq = QoSProfile(depth=1)
    gq.reliability = ReliabilityPolicy.RELIABLE
    gq.durability = DurabilityPolicy.TRANSIENT_LOCAL
    gm = {}
    node.create_subscription(Costmap, "/global_costmap/costmap_raw",
                             lambda m: gm.__setitem__("m", m), gq)
    sq = QoSProfile(depth=5)
    sq.reliability = ReliabilityPolicy.BEST_EFFORT
    sq.durability = DurabilityPolicy.VOLATILE
    sm = {}
    node.create_subscription(LaserScan, "/rtk/scan", lambda m: sm.__setitem__("m", m), sq)

    t0 = time.time()
    while ("m" not in gm or "m" not in sm) and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "m" not in gm or "m" not in sm:
        print("** 缺输入：global_raw=%s scan=%s **" % ("有" if "m" in gm else "无",
                                                      "有" if "m" in sm else "无"))
        return 1

    # 相机光轴（map 帧）：用 **当帧 TF** map←camera_optical_frame，光轴 = 该帧的 +z
    got = {}
    t1 = time.time()
    while "tf" not in got and time.time() - t1 < 20:
        try:
            got["tf"] = buf.lookup_transform("map", "camera_optical_frame", rclpy.time.Time())
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    if "tf" not in got:
        print("** 取不到 map←camera_optical_frame ⇒ 光轴算不出来（相机链路没起）**")
        return 1
    t = got["tf"]
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    cam = np.array([t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z])
    axis = R @ np.array([0.0, 0.0, 1.0])          # 光学帧 +z = 光轴
    yaw = math.degrees(math.atan2(axis[1], axis[0]))
    print("相机位置（map 帧）=(%.3f, %.3f, %.3f)  光轴方向=%.1f°（单位向量 %.3f, %.3f, %.3f）"
          % (cam[0], cam[1], cam[2], yaw, axis[0], axis[1], axis[2]))

    md = gm["m"].metadata
    res, ox, oy = md.resolution, md.origin.position.x, md.origin.position.y
    gw, gh = int(md.size_x), int(md.size_y)
    grid = np.asarray(gm["m"].data).astype(np.int16).reshape((gh, gw))
    s = sm["m"]
    inc = s.angle_increment
    sframe = (s.header.frame_id or "?").lstrip("/")

    def gval(px, py):
        mx, my = int((px - ox) / res), int((py - oy) / res)
        if not (0 <= mx < gw and 0 <= my < gh):
            return None
        return int(grid[my, mx])

    def neighborhood_free(px, py, half=2):
        mx, my = int((px - ox) / res), int((py - oy) / res)
        sub = grid[max(0, my - half):my + half + 1, max(0, mx - half):mx + half + 1]
        return sub.size == (2 * half + 1) ** 2 and bool((sub == 0).all())

    # 把候选点转到 scan 帧算方位（用 scan 自己的帧，方位才准）
    scan_tf = None
    t2 = time.time()
    while scan_tf is None and time.time() - t2 < 20:
        try:
            scan_tf = buf.lookup_transform("map", sframe, rclpy.time.Time())
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    if scan_tf is None:
        print("** 取不到 map←%s **" % sframe)
        return 1
    sq_q = scan_tf.transform.rotation
    qx, qy, qz, qw = sq_q.x, sq_q.y, sq_q.z, sq_q.w
    Rs = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                   [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                   [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    sp = np.array([scan_tf.transform.translation.x, scan_tf.transform.translation.y,
                   scan_tf.transform.translation.z])
    rs = np.asarray(s.ranges, dtype=np.float64)

    def beam_at(px, py):
        d = np.array([px - sp[0], py - sp[1], 0.11 - sp[2]])
        loc = Rs.T @ d
        brg = math.atan2(loc[1], loc[0])
        i = int(round((brg - s.angle_min) / inc))
        if not (0 <= i < len(rs)):
            return None
        return brg, i

    def clear_at(px, py, tol=0.35):
        """该点方位 ±2 束里，有没有落在这点附近的回波。返回 (是否开阔, 证据串)。"""
        r = beam_at(px, py)
        if r is None:
            return False, "方位在图外"
        brg, i = r
        lo, hi = max(0, i - 2), min(len(rs), i + 3)
        win = rs[lo:hi]
        fin = win[np.isfinite(win)]
        dist = float(np.hypot(px - sp[0], py - sp[1]))
        near = fin[np.abs(fin - dist) <= tol] if fin.size else np.array([])
        ev = "idx %d..%d 回波：%s" % (lo, hi - 1,
                                     ", ".join("inf" if not np.isfinite(v) else "%.3f" % v for v in win))
        return (near.size == 0), ev + "（该方位 %.1f°、候选距离 %.3f m；落在 ±%.2f m 内的回波 %d 条）" \
            % (math.degrees(brg), dist, tol, near.size)

    cands = []
    if a.fixed:
        # ---- 固定点模式：不扫偏移，只核**给定的这个点** ----
        fx, fy = [float(v) for v in a.fixed.split(",")]
        print("** --fixed 模式：不改选点，只核 (%.3f, %.3f) 是否仍满足三条判据 **" % (fx, fy))
        gv, nf = gval(fx, fy), neighborhood_free(fx, fy)
        ok_scan, ev = clear_at(fx, fy)
        # 它相对光轴偏了多少（只为记录，不参与判据）
        d = math.degrees(math.atan2(fy - cam[1], fx - cam[0]))
        doff = (d - yaw + 180.0) % 360.0 - 180.0
        cands.append({"fixed": True, "offset_deg": round(doff, 2), "xy": [round(fx, 3), round(fy, 3)],
                      "global_raw": gv, "free5x5": nf, "scan_clear": bool(ok_scan),
                      "scan_evidence": ev})
        print("   (%.3f, %.3f)  相对光轴 %+.2f°  全局raw=%s 5x5全free=%s 该方位无回波=%s"
              % (fx, fy, doff, gv, nf, ok_scan))
        print("      %s" % ev)
        good = [c for c in cands if c["global_raw"] == 0 and c["free5x5"] and c["scan_clear"]]
        pick = good[0] if good else None
        print("\n==> %s" % ("固定点仍满足三条判据 ✓（可用于'与上一轮同一对放/移点'）"
                             if pick else
                             "** 固定点**不再**满足三条判据 ⇒ 不能用它做同点复跑，要如实记 **"))
        out = {"mode": "fixed", "fixed_xy": [fx, fy], "cam_xy": [round(float(cam[0]), 3), round(float(cam[1]), 3)],
               "optical_axis_yaw_deg": round(yaw, 2), "dist": a.dist,
               "global_raw_frame": (gm["m"].header.frame_id or "?"), "scan_frame": sframe,
               "candidates": cands, "picked": pick}
        if a.out:
            json.dump(out, open(a.out, "w"), ensure_ascii=False, indent=1)
        if pick:
            print("BOX_XY=%s" % ",".join("%.3f" % v for v in pick["xy"]))
        node.destroy_node()
        rclpy.shutdown()
        return 0 if pick else 1

    off = 0.0
    while off <= a.max_offset + 1e-9:
        for sgn in ((0,) if off == 0 else (1, -1)):
            ang = math.radians(yaw + sgn * off)
            px = cam[0] + a.dist * math.cos(ang)
            py = cam[1] + a.dist * math.sin(ang)
            gv = gval(px, py)
            nf = neighborhood_free(px, py)
            ok_scan, ev = clear_at(px, py)
            cands.append({"offset_deg": sgn * off, "xy": [round(px, 3), round(py, 3)],
                          "global_raw": gv, "free5x5": nf, "scan_clear": bool(ok_scan),
                          "scan_evidence": ev})
            print("  偏移 %+5.1f°  (%.3f, %.3f)  全局raw=%s 5x5全free=%s 该方位无回波=%s"
                  % (sgn * off, px, py, gv, nf, ok_scan))
            print("      %s" % ev)
        off += a.step

    good = [c for c in cands if c["global_raw"] == 0 and c["free5x5"] and c["scan_clear"]]
    pick = min(good, key=lambda c: abs(c["offset_deg"])) if good else None
    print("\n==> 选定放箱点：%s" % ("(%.3f, %.3f)，偏移 %+.1f°（光轴 %.1f° + %+.1f°）"
                                 % (pick["xy"][0], pick["xy"][1], pick["offset_deg"],
                                    yaw, pick["offset_deg"]) if pick else "** 没有候选点同时满足两条判据 **"))
    out = {"cam_xy": [round(float(cam[0]), 3), round(float(cam[1]), 3)],
           "optical_axis_yaw_deg": round(yaw, 2), "dist": a.dist,
           "global_raw_frame": (gm["m"].header.frame_id or "?"), "scan_frame": sframe,
           "candidates": cands, "picked": pick}
    if a.out:
        json.dump(out, open(a.out, "w"), ensure_ascii=False, indent=1)
    if pick:
        print("BOX_XY=%s" % ",".join("%.3f" % v for v in pick["xy"]))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if pick else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐束记录：把 `/rtk/scan` 的**整条回波数组**按帧原样存下来（三态 A/B 的逐束对比用）。

为什么不用 `follow_probe.py --scan-check`（它是上一轮的写法，也只覆盖 ±4 束）：
  · 那条路径打印的是 **±4 束的最小值**，而"取最小/取极值"**正是被 C1 判错的写法**
    （窗口里可能同时有别的真实障碍，"取最小"会把那个障碍当成"目标没打到"的证据）✗；
  · 箱子 0.4 m 见方、2 m 外张角约 11.4°，在 360 束（1°/束）上约占 11 束
    ⇒ **±4 束的窗口装不下它**，会漏掉边缘那几束。
  ⇒ 这里**存整条数组**，比对时按**同一批 beam 索引**逐束看三态，不做任何取极值。

判据（协调者口径）：**哪几束 `inf` ↔ 有限值双向可逆** ⇒ 那几束才是箱子；
**两态都在某个有限值上** ⇒ 那是别的真实障碍（与本轮无关，但要报出来）。

用法（容器内）：
  beams.py --phase before --expect 4.50,-3.30 --frames 6 \
           --json /work/.../r4_beams_A_before.json --window 25
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
    ap.add_argument("--phase", required=True)
    ap.add_argument("--expect", default="", help="map 帧的点 x,y（算它在该 scan 帧里的方位/索引）")
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--window", type=float, default=25.0, help="打印多少度的窗口（以 expect 方位为中心）")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    rclpy.init()
    node = Node("beams")
    import tf2_ros
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    node.create_timer(0.05, lambda: None)

    from sensor_msgs.msg import LaserScan
    got = []
    sq = QoSProfile(depth=10)
    sq.reliability = ReliabilityPolicy.BEST_EFFORT
    sq.durability = DurabilityPolicy.VOLATILE
    node.create_subscription(LaserScan, "/rtk/scan", lambda m: got.append(m), sq)

    t0 = time.time()
    while len(got) < a.frames and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not got:
        print("** %s：/rtk/scan %.0f s 内没收到帧 ⇒ 激光源没在发 **" % (a.phase, a.timeout))
        if a.json:
            json.dump({"phase": a.phase, "error": "no scan"}, open(a.json, "w"), ensure_ascii=False)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    s0 = got[0]
    n = len(s0.ranges)
    frame_id = (s0.header.frame_id or "?").lstrip("/")
    ang0 = math.degrees(s0.angle_min)
    inc = math.degrees(s0.angle_increment)

    exp = None
    if a.expect:
        x, y = [float(v) for v in a.expect.split(",")]
        from geometry_msgs.msg import PointStamped
        from tf2_geometry_msgs import do_transform_point
        p = None
        t1 = time.time()
        while p is None and time.time() - t1 < 20:
            try:
                ps = PointStamped()
                ps.header.frame_id = "map"
                ps.point.x, ps.point.y, ps.point.z = x, y, 0.11     # 激光扫描面高度
                tr = buf.lookup_transform(frame_id, "map", rclpy.time.Time())
                p = do_transform_point(ps, tr).point
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.1)
        if p is not None:
            brg = math.degrees(math.atan2(p.y, p.x))
            dist = float(math.hypot(p.x, p.y))
            idx = int(round((math.radians(brg) - s0.angle_min) / s0.angle_increment))
            exp = {"map_xy": [x, y], "scan_xy": [round(p.x, 3), round(p.y, 3)],
                   "bearing_deg": round(brg, 2), "dist_m": round(dist, 3), "idx": idx,
                   "tf_frame": frame_id, "tf_stamp": round(s0.header.stamp.sec + s0.header.stamp.nanosec * 1e-9, 3)}
        else:
            exp = {"error": "TF 查不到"}

    def ser(r):
        return ["inf" if not np.isfinite(v) else "%.3f" % v for v in r]

    frames = [{"wall": round(time.time(), 3),
               "sim": round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 3),
               "n": len(m.ranges), "ranges": ser(m.ranges)} for m in got]

    out = {"phase": a.phase,
           "frame_id": frame_id, "n_beams": n,
           "angle_min_deg": round(ang0, 3), "angle_increment_deg": round(inc, 4),
           "range_min": s0.range_min, "range_max": s0.range_max,
           "frames": frames, "expect": exp,
           "note": "ranges 按 beam 索引原样存（inf 记为字符串 'inf'）；不取极值、不取最小"}
    print("[%s] scan 帧=%s 波束=%d angle_min=%.2f° inc=%.4f° range=[%.2f, %.2f] 存了 %d 帧"
          % (a.phase, frame_id, n, ang0, inc, s0.range_min, s0.range_max, len(frames)))
    if exp and "idx" in exp:
        print("  期望点 map(%s) → scan帧(%s) 方位=%+.2f° 距离=%.3f m ⇒ idx≈%d"
              % (a.expect, exp["scan_xy"], exp["bearing_deg"], exp["dist_m"], exp["idx"]))
        lo = max(0, exp["idx"] - int(round(a.window / 2 / inc)))
        hi = min(n - 1, exp["idx"] + int(round(a.window / 2 / inc)))
        for f in frames:
            win = f["ranges"][lo:hi + 1]
            fin = [v for v in win if v != "inf"]
            print("   sim=%.3f idx %d..%d：%s" % (f["sim"], lo, hi, ", ".join(win)))
            print("       窗口内有限值束数=%d/%d（**这里只报计数与列表，不取最小**）"
                  % (len(fin), len(win)))
    if a.json:
        json.dump(out, open(a.json, "w"), ensure_ascii=False, indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

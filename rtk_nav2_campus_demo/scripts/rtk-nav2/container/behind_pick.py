#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""选一个**相机看不到**的放箱点（车尾方向 ~2 m）——用来回答**"激光这个源到底被消费了没有"**。

## 为什么必须有这一项

本轮的核心结论是"**换了入口之后全局图不再冻死**"。但这个结论有一个**致命的替代解释**：
**如果那个新源根本没被 nav2 消费**（话题名写错、类型不对、节点没起、来源被忽略），
图当然"不冻"——**它压根没在更新**。这两件事在"整图哈希是否恒定"上**长得一模一样** ✗。

而常规的 A/B 放箱**分不开这两件事**：箱放在相机光轴正前方 2 m，
**相机（±30°）和激光（360°）都看得见它** ⇒ 有反应也说不清是谁的功劳。

⇒ 本脚本选的是**相机水平视场之外**的方位（车尾 180°±25°），
**那个方位上任何标记都只可能来自激光**（相机在水平方向上只有 60°）。
于是"车尾那格有没有被标"就成了"激光源被消费"的**直接**判据 ✓。

## 判据（选点前必须全过，与 `pickbox.py` 同源）

  ① 该点在**全局 raw 图**里是 free(0)，且 5×5 邻域全 free；
  ② 该方位的 scan 束（±2 束）在该距离附近**没有回波**（否则箱子会叠在既有障碍上）；
  ③ **该方位必须在相机水平视场之外**：`|方位 − 光轴| > 30° + 余量`。
     这一条是**本脚本存在的理由**，不过就换点（先试 180°，再 ±180±15°、±180±25°）。

用法（容器内）：
  behind_pick.py --dist 2.0 --out /work/.../r1_behind_pick.json
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

# 相机水平视场 1.0472 rad = 60°（SDF 与 URDF 写死），半视场 30°。
# 取 45° 作余量：视场边缘的像素畸变大、且"光轴"本身有安装误差，
# 贴着 30° 选点会留下"其实还在视场里"的口实。
CAM_HALF_FOV_DEG = 30.0
CAM_MARGIN_DEG = 15.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=2.0)
    ap.add_argument("--timeout", type=float, default=40.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    rclpy.init()
    node = Node("behind_pick")
    import tf2_ros
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

    def look(parent, child, wait=20.0):
        t = time.time()
        while time.time() - t < wait:
            try:
                return buf.lookup_transform(parent, child, rclpy.time.Time())
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.1)
        return None

    def rot(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    # 机器人位置与朝向（map ← base_footprint），以及相机光轴方向（map ← camera_optical_frame）
    tf_bf = look("map", "base_footprint")
    tf_cam = look("map", "camera_optical_frame")
    if tf_bf is None or tf_cam is None:
        print("** 取不到 map←base_footprint 或 map←camera_optical_frame ⇒ 定位/相机链路没起 **")
        return 1
    rb = tf_bf.transform
    rbt = np.array([rb.translation.x, rb.translation.y, rb.translation.z])
    R_bf = rot(rb.rotation)
    base_x = R_bf @ np.array([1.0, 0.0, 0.0])
    base_yaw = math.degrees(math.atan2(base_x[1], base_x[0]))
    axis = rot(tf_cam.transform.rotation) @ np.array([0.0, 0.0, 1.0])
    cam_yaw = math.degrees(math.atan2(axis[1], axis[0]))
    print("机器人（map 帧）=(%.3f, %.3f)  base_link +x 方位=%.1f°；相机光轴方位=%.1f°"
          % (rbt[0], rbt[1], base_yaw, cam_yaw))

    md = gm["m"].metadata
    res, ox, oy = md.resolution, md.origin.position.x, md.origin.position.y
    gw, gh = int(md.size_x), int(md.size_y)
    grid = np.asarray(gm["m"].data).astype(np.int16).reshape((gh, gw))
    s = sm["m"]
    inc = s.angle_increment
    sframe = (s.header.frame_id or "?").lstrip("/")
    rs = np.asarray(s.ranges, dtype=np.float64)
    tf_s = look("map", sframe)
    if tf_s is None:
        print("** 取不到 map←%s **" % sframe)
        return 1
    sp = np.array([tf_s.transform.translation.x, tf_s.transform.translation.y,
                   tf_s.transform.translation.z])
    Rs = rot(tf_s.transform.rotation)

    def gval(px, py):
        mx, my = int((px - ox) / res), int((py - oy) / res)
        if not (0 <= mx < gw and 0 <= my < gh):
            return None
        return int(grid[my, mx])

    def free5(px, py, half=2):
        mx, my = int((px - ox) / res), int((py - oy) / res)
        sub = grid[max(0, my - half):my + half + 1, max(0, mx - half):mx + half + 1]
        return sub.size == (2 * half + 1) ** 2 and bool((sub == 0).all())

    def scan_clear(px, py, tol=0.35):
        loc = Rs.T @ np.array([px - sp[0], py - sp[1], 0.11 - sp[2]])
        brg = math.atan2(loc[1], loc[0])
        i = int(round((brg - s.angle_min) / inc))
        if not (0 <= i < len(rs)):
            return False, "方位在图外", brg
        lo, hi = max(0, i - 2), min(len(rs), i + 3)
        win = rs[lo:hi]
        fin = win[np.isfinite(win)]
        dist = float(np.hypot(px - sp[0], py - sp[1]))
        near = fin[np.abs(fin - dist) <= tol] if fin.size else np.array([])
        ev = ("idx %d..%d 回波：%s（该方位 %.1f°、候选距离 %.3f m；落在 ±%.2f m 内的回波 %d 条）"
              % (lo, hi - 1, ", ".join("inf" if not np.isfinite(v) else "%.3f" % v for v in win),
                 math.degrees(brg), dist, tol, near.size))
        return (near.size == 0), ev, brg

    cands = []
    # 先试正后方 180°，再按 ±15°、±25° 依次试（**优先取最贴正后方的**）。
    for off in (0.0, 15.0, -15.0, 25.0, -25.0):
        ang = math.radians(base_yaw + 180.0 + off)
        px = rbt[0] + a.dist * math.cos(ang)
        py = rbt[1] + a.dist * math.sin(ang)
        gv = gval(px, py)
        nf = free5(px, py)
        ok_scan, ev, brg = scan_clear(px, py)
        # 方位（map 帧）与相机光轴的夹角 —— 这是"相机看不看得见"的判据
        d = abs((math.degrees(brg) - cam_yaw + 180.0) % 360.0 - 180.0)
        outside = d > (CAM_HALF_FOV_DEG + CAM_MARGIN_DEG)
        cands.append({"offset_deg": off, "xy": [round(px, 3), round(py, 3)],
                      "global_raw": gv, "free5x5": nf, "scan_clear": bool(ok_scan),
                      "bearing_map_deg": round(math.degrees(brg), 2),
                      "angle_from_cam_axis_deg": round(d, 2),
                      "outside_camera_fov": bool(outside), "scan_evidence": ev})
        print("  偏移 %+5.1f°  (%.3f, %.3f)  全局raw=%s 5x5全free=%s 该方位无回波=%s "
              "离光轴=%.1f°（相机半视场 %.0f°+余量 %.0f°）⇒ 相机看得见=%s"
              % (off, px, py, gv, nf, ok_scan, d, CAM_HALF_FOV_DEG, CAM_MARGIN_DEG,
                 "否" if outside else "**是**"))
        print("      %s" % ev)

    good = [c for c in cands if c["global_raw"] == 0 and c["free5x5"]
            and c["scan_clear"] and c["outside_camera_fov"]]
    pick = min(good, key=lambda c: abs(c["offset_deg"])) if good else None
    if pick:
        print("\n==> 选定**车尾**放箱点：(%.3f, %.3f)，相对正后方偏移 %+.1f°；"
              "该方位离相机光轴 %.1f° > %.0f° ⇒ **相机看不见** ⇒ 这格的任何反应只能来自激光"
              % (pick["xy"][0], pick["xy"][1], pick["offset_deg"],
                 pick["angle_from_cam_axis_deg"], CAM_HALF_FOV_DEG + CAM_MARGIN_DEG))
    else:
        print("\n==> **没有候选点同时满足「图上 free / 该方位无回波 / 相机看不见」⇒ 这一项跳过**")
    out = {"robot_xy": [round(float(rbt[0]), 3), round(float(rbt[1]), 3)],
           "base_yaw_deg": round(base_yaw, 2), "cam_axis_yaw_deg": round(cam_yaw, 2),
           "cam_half_fov_deg": CAM_HALF_FOV_DEG, "margin_deg": CAM_MARGIN_DEG,
           "dist": a.dist, "scan_frame": sframe, "candidates": cands, "picked": pick,
           "why": "车尾方位在相机水平视场之外 ⇒ 该处的标记只可能来自激光源"}
    if a.out:
        json.dump(out, open(a.out, "w"), ensure_ascii=False, indent=1)
    if pick:
        print("BOX_XY=%s" % ",".join("%.3f" % v for v in pick["xy"]))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if pick else 1


if __name__ == "__main__":
    sys.exit(main())

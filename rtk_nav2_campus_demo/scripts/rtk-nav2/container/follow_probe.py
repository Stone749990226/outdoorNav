#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跟随性探针：全局 raw 图**跟不跟世界变化**。

背景：清空实验测到 —— 清完全局后图**变了一次**（473→159 lethal），随后**又定住**
（字节级不变、stamp 仍在前进）。那到底是"这一层从此不再跟世界"还是"它只在特定事件后写一次"？
判据只能来自**改动世界**：放一个已知的点，看全局 raw 图那一格**变不变**。

两种模式：
  --pick                在机器人周围 2.0~3.5 m 找**当前为 free（0）**的格，输出候选点（map 坐标）
  --at "x,y;x,y;..."    输出这些点当前的 raw 值（0=free / 254=lethal / 255=unknown）
"""
import argparse
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

MAP = "/global_costmap/costmap_raw"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick", action="store_true")
    ap.add_argument("--at", default="")
    ap.add_argument("--also-local", action="store_true",
                    help="同时读 /local_costmap/costmap_raw（对照组：证明障碍真的被传感器看见）")
    ap.add_argument("--scan-check", default="",
                    help="给出 map 帧的点 'x,y'，报告 /rtk/scan 在该方位上的回波距离（传感器侧直接证据）")
    ap.add_argument("--fov-point", type=float, default=0.0,
                    help="打印'相机视场正中'的点：沿 base_link +x 走 D 米（相机光轴方向），"
                         "以及同距离 +90°（视场外，只有激光看得见）两点在 map 帧的坐标")
    ap.add_argument("--json", default=None)
    ap.add_argument("--timeout", type=float, default=30.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node("follow_probe")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    import tf2_ros
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    from nav2_msgs.msg import Costmap
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    got = {}
    node.create_subscription(Costmap, MAP, lambda m: got.__setitem__("m", m), qos)

    t0 = time.time()
    while "m" not in got and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "m" not in got:
        print("** %s 一帧都没收到 **" % MAP)
        return 1
    m = got["m"]
    md = m.metadata
    res = md.resolution
    ox, oy = md.origin.position.x, md.origin.position.y
    w, h = int(md.size_x), int(md.size_y)
    data = np.asarray(m.data).astype(np.uint8).reshape((h, w))
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    # 机器人位置（map 帧）
    pose = None
    t1 = time.time()
    while pose is None and time.time() - t1 < 20:
        try:
            t = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            pose = (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)

    out = {"map_stamp": st, "origin": [ox, oy], "res": res, "size": [w, h], "robot": pose}
    print("全局 raw 图：%dx%d res=%.3f origin=(%.2f, %.2f) stamp=%.3f" % (w, h, res, ox, oy, st))
    print("机器人（map 帧）：%s" % ("(%.3f, %.3f)" % pose if pose else "** TF 查不到 **"))
    print("全图统计：lethal=%d free=%d unknown=%d" %
          (int((data == 254).sum()), int((data == 0).sum()), int((data == 255).sum())))

    def val(x, y):
        mx, my = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= mx < w and 0 <= my < h):
            return None, (mx, my)
        return int(data[my, mx]), (mx, my)

    if a.pick:
        if pose is None:
            print("** 没有机器人位姿，无法选点 **")
            return 1
        rx, ry = pose
        t1 = time.time()
        while time.time() - t1 < 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        cands = []
        for ang in np.arange(0, 360, 15):
            for rad in (2.0, 2.5, 3.0, 3.5):
                x = rx + rad * np.cos(np.radians(ang))
                y = ry + rad * np.sin(np.radians(ang))
                v, _ = val(x, y)
                if v == 0:
                    # 邻域 5x5 也得全是 free，避免选到障碍边缘
                    mx, my = int((x - ox) / res), int((y - oy) / res)
                    sub = data[max(0, my - 2):my + 3, max(0, mx - 2):mx + 3]
                    if sub.size == 25 and (sub == 0).all():
                        cands.append((round(x, 2), round(y, 2), round(float(rad), 2), int(ang)))
        # 分散一点：按角度去重，最多 8 个
        picked, seen_ang = [], set()
        for c in cands:
            if c[3] in seen_ang:
                continue
            seen_ang.add(c[3])
            picked.append(c)
        print("\n候选点（map 坐标 / 距机器人 / 角度）：")
        for x, y, rad, ang in picked[:8]:
            print("  (%.2f, %.2f)  r=%.2f m  ang=%d°  当前值=%d" % (x, y, rad, ang, val(x, y)[0]))
        out["candidates"] = picked[:8]
    local = {}
    if a.also_local:
        lgot = {}
        node.create_subscription(Costmap, "/local_costmap/costmap_raw",
                                 lambda m: lgot.__setitem__("m", m), qos)
        t2 = time.time()
        while "m" not in lgot and time.time() - t2 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)

    if a.at:
        lm = lgot.get("m") if a.also_local else None
        if lm is not None:
            lmd = lm.metadata
            lres, lox, loy = lmd.resolution, lmd.origin.position.x, lmd.origin.position.y
            lw, lh = int(lmd.size_x), int(lmd.size_y)
            ldata = np.asarray(lm.data).astype(np.uint8).reshape((lh, lw))
            lst = lm.header.stamp.sec + lm.header.stamp.nanosec * 1e-9
            print("\n局部 raw 图（对照组）：%dx%d res=%.3f origin=(%.2f, %.2f) stamp=%.3f；"
                  "lethal=%d free=%d unknown=%d" %
                  (lw, lh, lres, lox, loy, lst, int((ldata == 254).sum()),
                   int((ldata == 0).sum()), int((ldata == 255).sum())))
        print("\n指定格读数：")
        pts = []
        for pair in a.at.split(";"):
            if not pair.strip():
                continue
            x, y = [float(s) for s in pair.split(",")]
            v, cell = val(x, y)
            rec = {"xy": [x, y], "value": v, "cell": list(cell)}
            extra = ""
            if lm is not None:
                lmx, lmy = int((x - lox) / lres), int((y - loy) / lres)
                if 0 <= lmx < lw and 0 <= lmy < lh:
                    lv = int(ldata[lmy, lmx])
                    rec["local_value"] = lv
                    rec["local_cell"] = [lmx, lmy]
                    extra = "  局部图同格=%d %s" % (
                        lv, {0: "(free)", 254: "(lethal)", 255: "(unknown)"}.get(lv, ""))
                else:
                    rec["local_value"] = None
                    extra = "  局部图：** 该点在局部图外 **"
            pts.append(rec)
            print("  (%.2f, %.2f) → cell=%s 值=%s %s%s" %
                  (x, y, cell, v, {0: "(free)", 254: "(lethal)", 255: "(unknown)"}.get(v, ""), extra))
        out["points"] = pts

    if a.scan_check:
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import PointStamped
        from tf2_geometry_msgs import do_transform_point
        sgot = {}
        sq = QoSProfile(depth=1)
        sq.reliability = ReliabilityPolicy.BEST_EFFORT
        sq.durability = DurabilityPolicy.VOLATILE
        node.create_subscription(LaserScan, "/rtk/scan", lambda m: sgot.__setitem__("s", m), sq)
        t3 = time.time()
        while "s" not in sgot and time.time() - t3 < 20:
            rclpy.spin_once(node, timeout_sec=0.1)
        print("\n传感器侧对照：/rtk/scan（**这条不依赖任何代价地图**）")
        if "s" not in sgot:
            print("  ** /rtk/scan 20 s 内没收到帧 ⇒ 激光源也没在发 **")
            out["scan"] = None
        else:
            s = sgot["s"]
            x, y = [float(v) for v in a.scan_check.split(",")]
            p = None
            t4 = time.time()
            while p is None and time.time() - t4 < 20:
                try:
                    ps = PointStamped()
                    ps.header.frame_id = "map"
                    ps.point.x, ps.point.y, ps.point.z = x, y, 0.11
                    tr = buf.lookup_transform(s.header.frame_id, "map", rclpy.time.Time())
                    p = do_transform_point(ps, tr).point
                except Exception:
                    rclpy.spin_once(node, timeout_sec=0.1)
            if p is None:
                print("  ** 点 (%s) 转不到 %s 帧 **" % (a.scan_check, s.header.frame_id))
                out["scan"] = {"error": "tf"}
            else:
                brg = np.degrees(np.arctan2(p.y, p.x))
                dist = float(np.hypot(p.x, p.y))
                idx = int(round((np.radians(brg) - s.angle_min) / s.angle_increment))
                rng = np.asarray(s.ranges, dtype=np.float32)
                lo, hi = max(0, idx - 4), min(len(rng), idx + 5)
                win = rng[lo:hi]
                finite = win[np.isfinite(win)]
                print("  scan 帧=%s  帧号=%d.%09d  波束数=%d  angle_min=%.1f° increment=%.3f°" %
                      (s.header.frame_id, s.header.stamp.sec, s.header.stamp.nanosec,
                       len(rng), np.degrees(s.angle_min), np.degrees(s.angle_increment)))
                print("  目标点 map(%.2f, %.2f) → scan 帧 (%.2f, %.2f)：方位=%+.1f° 距离=%.3f m" %
                      (x, y, p.x, p.y, brg, dist))
                print("  该方位 ±4 束（idx %d..%d）回波：%s" %
                      (lo, hi - 1, ", ".join("%.3f" % v for v in win)))
                if finite.size:
                    print("  ⇒ 该方位最小回波 %.3f m%s" %
                          (finite.min(),
                           "（**≈ 目标距离 ⇒ 激光真看见了**）" if abs(finite.min() - dist) < 0.2
                           else "（与目标距离差 %.3f m ⇒ 这一束没打到目标）" % abs(finite.min() - dist)))
                out["scan"] = {"frame": s.header.frame_id, "bearing_deg": brg, "dist_m": dist,
                               "idx": idx, "window": [float(v) for v in win],
                               "min_in_window": float(finite.min()) if finite.size else None}

    if a.fov_point > 0 and pose is not None:
        # 相机光轴 ≈ base_link +x（相机 static TF 的旋转是 (-0.5,0.5,-0.5,0.5)，光轴指向 +x）。
        # 用 map←base_link 的**旋转**把 base 帧的方向向量转到 map 帧。
        t = None
        t5 = time.time()
        while t is None and time.time() - t5 < 15:
            try:
                t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.1)
        if t is None:
            print("\n** 取不到 map←base_link，算不了视场点 **")
        else:
            q = t.transform.rotation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            base = np.array([t.transform.translation.x, t.transform.translation.y, 0.0])
            d = a.fov_point
            fwd = base + R @ np.array([d, 0.0, 0.0])          # 光轴方向：相机看得见
            side = base + R @ np.array([0.0, d, 0.0])          # +90°：视场外，只有激光看得见
            yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
            print("\n机器人 yaw（map 帧）= %+.1f°" % yaw)
            print("相机光轴 %0.1f m 处（map 帧）：(%.2f, %.2f)  ← 相机+激光都看得见" % (d, fwd[0], fwd[1]))
            print("同距离 +90°（map 帧）   ：(%.2f, %.2f)  ← 视场外，只有激光看得见" % (side[0], side[1]))
            out["fov_point"] = {"yaw_deg": yaw, "fwd": [round(float(fwd[0]), 3), round(float(fwd[1]), 3)],
                                "side": [round(float(side[0]), 3), round(float(side[1]), 3)]}

    if a.json:
        json.dump(out, open(a.json, "w"), ensure_ascii=False, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())

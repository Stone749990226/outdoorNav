#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""选**对照格**：全局 raw 图上"离机器人最近的 lethal 格"，但**排除箱子附近**的那些。

为什么要排除：箱子本身就放在车前 2 m ⇒ 它就是"离机器人最近的 lethal"，
拿它当对照格等于**拿箱子给自己当对照** ✗。对照格要的是"**另一个**真实障碍"。

用法（容器内）：find_ctrl.py --exclude <箱 x>,<箱 y> [--exclude-radius 1.5] [--out json]
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
    ap.add_argument("--exclude", default="", help="箱的 map 坐标 x,y（排除它附近的格）")
    ap.add_argument("--exclude-radius", type=float, default=1.5)
    ap.add_argument("--timeout", type=float, default=40.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    rclpy.init()
    node = Node("find_ctrl")
    import tf2_ros
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    node.create_timer(0.05, lambda: None)

    from nav2_msgs.msg import Costmap
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    got = {}
    node.create_subscription(Costmap, "/global_costmap/costmap_raw",
                             lambda m: got.__setitem__("m", m), qos)

    t0 = time.time()
    while "m" not in got and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "m" not in got:
        print("** 没收到全局 raw 图 **")
        return 1

    pose = None
    t1 = time.time()
    while pose is None and time.time() - t1 < 20:
        try:
            t = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            pose = (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    if pose is None:
        print("** 取不到机器人位姿 **")
        return 1

    m = got["m"]
    md = m.metadata
    res, ox, oy = md.resolution, md.origin.position.x, md.origin.position.y
    w, h = int(md.size_x), int(md.size_y)
    grid = np.asarray(m.data).astype(np.int16).reshape((h, w))
    ys, xs = np.where(grid == 254)
    if xs.size == 0:
        print("** 全局 raw 图上没有 lethal 格 ⇒ 对照格无解 **")
        return 1
    wx = ox + (xs + 0.5) * res
    wy = oy + (ys + 0.5) * res
    d = np.hypot(wx - pose[0], wy - pose[1])
    order = np.argsort(d)
    ex = None
    if a.exclude:
        ex = [float(v) for v in a.exclude.split(",")]
    print("机器人 map(%.3f, %.3f)；全局 raw 图 lethal 格 %d 个" % (pose[0], pose[1], xs.size))
    print("%-4s %-18s %-12s %-12s" % ("#", "格中心(map)", "距机器人(m)", "距箱(m)"))
    shown, pick = 0, None
    for i in order:
        dbox = math.hypot(wx[i] - ex[0], wy[i] - ex[1]) if ex else float("nan")
        ok = (ex is None) or (dbox > a.exclude_radius)
        if shown < 6:
            print("%-4d (%8.3f, %8.3f) %-12.3f %-12.3f %s"
                  % (xs.size - shown, wx[i], wy[i], d[i], dbox, "" if ok else "← 太靠近箱子，排除"))
            shown += 1
        if ok and pick is None:
            pick = {"xy": [round(float(wx[i]), 3), round(float(wy[i]), 3)],
                    "dist_robot_m": round(float(d[i]), 3),
                    "dist_box_m": None if ex is None else round(float(dbox), 3),
                    "cell": [int(xs[i]), int(ys[i])], "global_raw": int(grid[ys[i], xs[i]])}
    if pick is None:
        print("** 排除箱子附近后没有可用的对照格 **")
        return 1
    print("\n选定对照格：(%.3f, %.3f)  距机器人 %.3f m  距箱 %s m  全局raw=%d  cell=(%d,%d)"
          % (pick["xy"][0], pick["xy"][1], pick["dist_robot_m"],
             pick["dist_box_m"], pick["global_raw"], pick["cell"][0], pick["cell"][1]))
    print("CTRL_XY=%.3f,%.3f" % (pick["xy"][0], pick["xy"][1]))
    if a.out:
        json.dump({"robot_xy": [pose[0], pose[1]], "picked": pick, "n_lethal": int(xs.size),
                   "exclude": ex, "exclude_radius": a.exclude_radius},
                  open(a.out, "w"), ensure_ascii=False, indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

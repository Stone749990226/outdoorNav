#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""取值直方图（reviewer 要的"用集合说话"）：判 `costmap_raw` 到底是膨胀前还是膨胀后。

为什么要它：**"159 vs 473" 这种比较可能是两种编码在比**（同名 ≠ 同义）。
`costmap_raw`（nav2_msgs/Costmap）与 `costmap`（nav_msgs/OccupancyGrid）的**取值集合**
必须分别报出来：
  - **膨胀前主网格**（Overwrite 语义）应当只有 `0 / 254 / 255`；
  - 出现**大量非二值**（如 166/193/225）⇒ **该 topic 含膨胀** ⇒ 定位要重做。
配合"改 `inflation_radius` 看 raw 变不变"这条**可逆实验**一起用（改完记得改回来）。

用法（容器内）：map_hist.py [--run-dir DIR] [--seconds 20]
"""
import argparse
import hashlib
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

TOPICS = [
    ("/global_costmap/costmap_raw", "g_raw"),
    ("/global_costmap/costmap", "g_inf"),
    ("/local_costmap/costmap_raw", "l_raw"),
    ("/local_costmap/costmap", "l_inf"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", default="")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    rclpy.init()
    node = Node("map_hist")
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    got = {}
    for topic, tag in TOPICS:
        mt = Costmap if topic.endswith("raw") else OccupancyGrid

        def make_cb(tg):
            def cb(msg):
                if tg not in got:                       # 每张图只取第一帧（同一时刻对比）
                    got[tg] = msg
            return cb
        node.create_subscription(mt, topic, make_cb(tag), qos)

    t0 = time.time()
    while len(got) < 4 and time.time() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(min(a.seconds, 2.0))

    lines = []
    out = {}
    lines.append("取值直方图（每张图取**同一时刻的第一帧**；判据用**取值集合**说话）")
    lines.append("%-8s %-22s %-12s %-10s %s" % ("图", "话题", "类型", "尺寸", "取值集合（值:格数）"))
    lines.append("-" * 100)
    for topic, tag in TOPICS:
        m = got.get(tag)
        if m is None:
            lines.append("%-8s %-22s ** 60 s 内一帧都没收到 **" % (tag, topic))
            out[tag] = None
            continue
        kind = type(m).__name__
        if kind == "Costmap":
            md = m.metadata
            size = "%dx%d" % (md.size_x, md.size_y)
            res = md.resolution
            ox, oy = md.origin.position.x, md.origin.position.y
            d = np.asarray(m.data).astype(np.int16).ravel()
        else:
            size = "%dx%d" % (m.info.width, m.info.height)
            res = m.info.resolution
            ox, oy = m.info.origin.position.x, m.info.origin.position.y
            d = np.asarray(m.data).astype(np.int16).ravel()
        vals, cnts = np.unique(d, return_counts=True)
        hist = {int(v): int(c) for v, c in zip(vals, cnts)}
        top = sorted(hist.items(), key=lambda kv: -kv[1])[:8]
        st = m.header.stamp
        # ★ 两支图的"对齐"要逐项打出来：同一个世界点在两支里**未必是同一个格**
        lines.append("%-8s %-22s %-12s %-9s res=%.3f origin=(%.2f, %.2f) frame=%s" % (
            tag, topic, kind, size, res, ox, oy, m.header.frame_id))
        lines.append("%-8s   取值集合（计数）：%s%s" % (
            "", "  ".join("%d:%d" % (v, c) for v, c in top),
            "  …共 %d 种取值" % len(hist) if len(hist) > 8 else ""))
        lines.append("%-8s   stamp=%d.%09d 全图哈希=%s" % (
            "", st.sec, st.nanosec,
            hashlib.sha256(np.ascontiguousarray(d).tobytes()).hexdigest()[:16]))
        out[tag] = {"topic": topic, "type": kind, "size": size, "frame": m.header.frame_id,
                    "res": res, "origin": [ox, oy],
                    "stamp": st.sec + st.nanosec * 1e-9, "hist": hist,
                    "values": [int(v) for v in vals],
                    "hash": hashlib.sha256(np.ascontiguousarray(d).tobytes()).hexdigest()[:16]}
    lines.append("")
    lines.append("判读要点：raw 若**只有 0/254/255** ⇒ 膨胀前主网格；**出现大量非二值** ⇒ 该 topic 含膨胀。")
    print("\n".join(lines))
    if a.run_dir:
        with open(os.path.join(a.run_dir, "map_hist.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
    if a.json:
        json.dump(out, open(a.json, "w"), ensure_ascii=False, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

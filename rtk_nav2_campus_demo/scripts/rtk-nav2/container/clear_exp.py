#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""清空实验驱动（run-id 级工具，读 v2）：**配置一字不改**，只看"清空之后图变成什么样"。

要回答的问题
------------
冻结（全局图不再更新）发生在**障碍图层的输出**（同轮对照：`costmap_raw` 恒定、
局部图同期在变）。那么这一层"到底还写不写"？清一次就知道：

  (a) 清完全局后**一片空白、之后仍空白**   ⇒ **该层什么都没写**（不再往外吐任何更新）
  (b) 清完**回填成"拐点前那套"**           ⇒ 层还在写，但写的是**旧内容**（输入侧被卡在某一刻）
  (c) 清完**回填成"当前世界"**             ⇒ **这本身是更强的新信息**，按"定位需重做"如实报

判据（全部用**活节点**，不看 daemon 缓存 —— §9.6："缓存里有" ≠ "现在有"）
--------------------------------------------------------------------
- 每张图 = 订阅**真收到一帧**（TRANSIENT_LOCAL + RELIABLE，nav2 的图是 latched 的）；
- 采样点：**清空前 / +1 s / +5 s / +15 s**，每点给**两组哈希**（raw 与 inflated）+ lethal 计数；
- 清空服务：**先 `wait_for_service` 再调**，并且**只认实际返回**（不看 `service list` 里有没有）。

用法（容器内）：
  clear_exp.py --run-dir /work/artifacts/rtk-nav2/<run-id> [--which global|local|both]
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

TOPICS = [
    ("/global_costmap/costmap_raw", "raw", "global"),
    ("/global_costmap/costmap", "inflated", "global"),
    ("/local_costmap/costmap_raw", "raw", "local"),
    ("/local_costmap/costmap", "inflated", "local"),
]

SERVICES = {
    "global": "/global_costmap/clear_entirely_global_costmap",
    "local": "/local_costmap/clear_entirely_local_costmap",
}


def _stats(msg):
    """把 nav2_msgs/Costmap（raw）与 nav_msgs/OccupancyGrid（inflated）统一成一份统计。"""
    kind = type(msg).__name__
    data = np.asarray(msg.data)
    if kind == "Costmap":
        meta = msg.metadata
        w, h = int(meta.size_x), int(meta.size_y)
        d = data.astype(np.uint8)
        lethal = int((d == 254).sum())          # LETHAL_OBSTACLE
        unknown = int((d == 255).sum())         # NO_INFORMATION
    else:
        w, h = int(msg.info.width), int(msg.info.height)
        d = data.astype(np.int16)
        lethal = int((d >= 100).sum())
        unknown = int((d < 0).sum())
    free = int((d == 0).sum())
    return {
        "type": kind,
        "w": w, "h": h, "cells": int(d.size),
        "lethal": lethal, "free": free, "unknown": unknown,
        "max": int(d.max()), "min": int(d.min()),
        "hash": hashlib.sha256(np.ascontiguousarray(d).tobytes()).hexdigest()[:16],
        "stamp": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--which", default="both", choices=["global", "local", "both"])
    ap.add_argument("--baseline-s", type=float, default=5.0)
    ap.add_argument("--first-wait", type=float, default=90.0, help="等四张图各收到一帧的上限")
    ap.add_argument("--service-wait", type=float, default=15.0)
    args = ap.parse_args()

    arrivals = []          # [{wall, topic, role, map, stats}]
    t_start = time.monotonic()

    rclpy.init()
    node = Node("clear_exp")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.history = HistoryPolicy.KEEP_LAST

    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid

    def mk(topic, role, mp, msgtype):
        def cb(msg):
            arrivals.append({"wall": time.monotonic() - t_start, "topic": topic,
                             "role": role, "map": mp, "stats": _stats(msg)})
        return cb

    got = set()
    for topic, role, mp in TOPICS:
        mt = Costmap if role == "raw" else OccupancyGrid
        node.create_subscription(mt, topic, mk(topic, role, mp, mt), qos)

    print("== 等四张图各真收到一帧（上限 %.0f s）==" % args.first_wait, flush=True)
    t0 = time.monotonic()
    while len(got) < 4 and time.monotonic() - t0 < args.first_wait:
        rclpy.spin_once(node, timeout_sec=0.1)
        got = {a["topic"] for a in arrivals}
    for topic, role, mp in TOPICS:
        n = sum(1 for a in arrivals if a["topic"] == topic)
        print("  %-34s %-8s %s" % (topic, role, "收到 %d 帧" % n if n else "** 一帧都没收到 **"),
              flush=True)
    if len(got) < 4:
        print("** 四张图没收齐，实验作废（先过 0/1/2/3 关）**", flush=True)

    print("\n== 基线观察 %.0f s（清空前）==" % args.baseline_s, flush=True)
    tb = time.monotonic()
    while time.monotonic() - tb < args.baseline_s:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 服务客户端：**先 wait_for_service 再调**，只认实际返回
    from nav2_msgs.srv import ClearEntireCostmap
    clients = {}
    for key, srv in SERVICES.items():
        clients[key] = node.create_client(ClearEntireCostmap, srv)

    events = []            # [{t, what, ok, detail}]
    marks = {}             # marker 名 -> wall 时刻（相对 t_start）

    def do_clear(key):
        cli = clients[key]
        srv = SERVICES[key]
        ready = cli.wait_for_service(timeout_sec=args.service_wait)
        mark = time.monotonic() - t_start
        marks["clear_" + key] = mark
        if not ready:
            print("\n[%.2f s] ** %s：wait_for_service(%.0f s) 失败 ⇒ 服务不可调用 **" %
                  (mark, srv, args.service_wait), flush=True)
            events.append({"t": mark, "what": "clear_" + key, "ok": False,
                           "detail": "wait_for_service 超时"})
            return
        req = ClearEntireCostmap.Request()
        fut = cli.call_async(req)
        t1 = time.monotonic()
        while not fut.done() and time.monotonic() - t1 < 20.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        ok = fut.done() and fut.exception() is None
        print("\n[%.2f s] %s：wait_for_service ✓；call %s%s" %
              (mark, srv, "✓ 已返回" if ok else "** 未返回/异常 **",
               "" if ok else " (%s)" % (fut.exception() if fut.done() else "20 s 超时")), flush=True)
        events.append({"t": mark, "what": "clear_" + key, "ok": bool(ok),
                       "detail": "返回" if ok else "未返回"})

    order = ["global", "local"] if args.which == "both" else [args.which]
    for i, key in enumerate(order):
        do_clear(key)
        # 该次清空之后的三个采样点
        for off in (1.0, 5.0, 15.0):
            tgt = marks["clear_" + key] + off
            while time.monotonic() - t_start < tgt:
                rclpy.spin_once(node, timeout_sec=0.05)
            label = "清空%s+%.0fs" % ("全局" if key == "global" else "局部", off)
            marks[label] = time.monotonic() - t_start
        if i == 0 and len(order) > 1:      # 两次清空之间留出干净间隔
            tgt = time.monotonic() - t_start + 3.0
            while time.monotonic() - t_start < tgt:
                rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    # ---- 采样点取值：每个标签取"该时刻之前最后收到的那一帧" ----
    def snapshot(t_mark):
        out = {}
        for topic, role, mp in TOPICS:
            best = None
            for a in arrivals:
                if a["topic"] == topic and a["wall"] <= t_mark:
                    best = a
            out[topic] = best
        return out

    lines = []
    lines.append("清空实验（配置未改）：采样点 = 清空前 / 各次清空 +1 s / +5 s / +15 s")
    lines.append("采样点时刻（相对探针启动，秒）：%s" %
                 ", ".join("%s=%.2f" % (k, v) for k, v in sorted(marks.items())))
    lines.append("")
    header = "%-22s | %-30s | %-8s | %-16s | %8s %8s %8s | %s" % (
        "采样点", "话题", "类别", "内容哈希", "lethal", "free", "unknown", "stamp")
    lines.append(header)
    lines.append("-" * len(header))

    sample_marks = [("清空前", marks.get("clear_global", marks.get("clear_local", 0.0)))]
    for key in order:
        cn = "清空%s" % ("全局" if key == "global" else "局部")
        for off in (1.0, 5.0, 15.0):
            sample_marks.append(("%s+%.0fs" % (cn, off), marks["%s+%.0fs" % (cn, off)]))

    summary = {}
    for label, t_mark in sample_marks:
        snap = snapshot(t_mark)
        row = {}
        for topic, role, mp in TOPICS:
            a = snap[topic]
            if a is None:
                lines.append("%-22s | %-30s | %-8s | ** 该时刻前没收到帧 **" % (label, topic, role))
                row[topic] = None
                continue
            s = a["stats"]
            lines.append("%-22s | %-30s | %-8s | %-16s | %8d %8d %8d | %d.%09d" % (
                label, topic, role, s["hash"], s["lethal"], s["free"], s["unknown"],
                int(s["stamp"]), int(round((s["stamp"] - int(s["stamp"])) * 1e9))))
            row[topic] = s
        summary[label] = row

    # 各话题在整个窗口里收到的帧数（能直接看出"图还发不发"）
    lines.append("")
    lines.append("各话题收到帧数（整段）：")
    for topic, role, mp in TOPICS:
        n = sum(1 for a in arrivals if a["topic"] == topic)
        walls = [a["wall"] for a in arrivals if a["topic"] == topic]
        lines.append("  %-34s %-8s %4d 帧%s" %
                     (topic, role, n,
                      "（首 %.2f s 末 %.2f s）" % (walls[0], walls[-1]) if walls else ""))

    report = "\n".join(lines)
    print("\n" + report, flush=True)
    os.makedirs(args.run_dir, exist_ok=True)
    with open(os.path.join(args.run_dir, "clear_exp.txt"), "w") as f:
        f.write(report + "\n")
    with open(os.path.join(args.run_dir, "clear_exp.json"), "w") as f:
        json.dump({"marks": marks, "events": events, "summary": summary,
                   "arrivals": [{"wall": a["wall"], "topic": a["topic"], "role": a["role"],
                                 "map": a["map"], "hash": a["stats"]["hash"],
                                 "lethal": a["stats"]["lethal"], "stamp": a["stats"]["stamp"]}
                                for a in arrivals]},
                  f, ensure_ascii=False, indent=2)
    print("\n已写入 %s/clear_exp.txt 与 clear_exp.json" % args.run_dir, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单格值序列：**"箱所在那一格的值 vs 时刻"**（整图哈希/lethal 计数会把"该格没变"淹掉）。

为什么用它（reviewer 定的判据形式）：
- 全局图是 `map` 帧、索引固定 ⇒ "箱的世界坐标对应的那一格"逐帧取值，**滚窗伪造不了**；
- 局部图（`base_link` 滚窗）**车不动时帧也固定** ⇒ 同一世界坐标同样对到固定格 ⇒ 两图都能做单格；
- 出现"**箱在该格里、而值一直是 free**"就是"不消费新输入"的直接证据。

用法（容器内）：
  cell_series.py --track "box=2.68,-0.61;ctrl=0.6,0.6" --seconds 15 --label A
                  [--auto-control] [--out out.csv] [--json out.json]
`--auto-control`：自动在**当前全局 raw 图**上找**离机器人最近的 lethal 格**当对照格
（比"按世界文件猜一个"稳：它保证对照格**当前确实有值**，且不受遮挡猜测影响）。
"""

import argparse
import csv
import hashlib
import json
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


def _meta(msg):
    """(frame, res, ox, oy, w, h, data(as int16), stamp) —— 两类消息统一。

    ★ frame 必须返回：局部图是**滚窗**，它的 `metadata.origin` 表达在**它自己的全局帧**里
    （nav2 局部图默认 `odom`），不是 `map`。拿 map 帧的世界坐标直接减 origin ⇒ **会算错格**
    （2026-09-21 实测踩到：箱格被判成"在图外"）。所以按帧做一次 TF。
    """
    kind = type(msg).__name__
    if kind == "Costmap":
        md = msg.metadata
        res, ox, oy = md.resolution, md.origin.position.x, md.origin.position.y
        w, h = int(md.size_x), int(md.size_y)
        data = np.asarray(msg.data).astype(np.int16)
    else:
        inf = msg.info
        res, ox, oy = inf.resolution, inf.origin.position.x, inf.origin.position.y
        w, h = int(inf.width), int(inf.height)
        data = np.asarray(msg.data).astype(np.int16)
    frame = (msg.header.frame_id or "map").lstrip("/")
    return frame, res, ox, oy, w, h, data.reshape((h, w)), \
        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _val(grid, res, ox, oy, w, h, x, y):
    mx, my = int((x - ox) / res), int((y - oy) / res)
    if not (0 <= mx < w and 0 <= my < h):
        return None
    return int(grid[my, mx])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", required=True, help='名=世界坐标，多项用 ; 分隔，如 "box=2.68,-0.61;ctrl=0.6,0.6"')
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--auto-control", action="store_true")
    ap.add_argument("--out", default="")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    cells = []
    for item in a.track.split(";"):
        if not item.strip():
            continue
        name, xy = item.split("=")
        x, y = [float(v) for v in xy.split(",")]
        cells.append({"name": name.strip(), "xy": [x, y]})

    rclpy.init()
    node = Node("cell_series")
    import tf2_ros
    from geometry_msgs.msg import PointStamped
    from tf2_geometry_msgs import do_transform_point
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    def to_frame(x, y, frame):
        """map 帧的世界点 → 该图自己的帧（局部图是 odom）。取不到就返回 None。"""
        if frame == "map":
            return (x, y)
        try:
            ps = PointStamped()
            ps.header.frame_id = "map"
            ps.point.x, ps.point.y, ps.point.z = x, y, 0.0
            tr = buf.lookup_transform(frame, "map", rclpy.time.Time())
            p = do_transform_point(ps, tr).point
            return (p.x, p.y)
        except Exception:
            return None

    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    latest = {}
    rows = []
    for topic, tag in TOPICS:
        mt = Costmap if topic.endswith("raw") else OccupancyGrid

        def make_cb(t, tg):
            def cb(msg):
                latest[tg] = msg
                frame, res, ox, oy, w, h, grid, st = _meta(msg)
                # 每帧把**这张图自己的原点**记下来：局部图 origin 会随车动而漂
                # ⇒ "origin 不动"由数据自证"车真没动"；origin 动了这一轮的格级结论当场作废。
                row = {"wall": round(time.time(), 3), "tag": tg, "frame": frame,
                       "stamp": round(st, 3), "ox": round(ox, 3), "oy": round(oy, 3),
                       # ★ 整图哈希：**拐点自测**要用它（"同一内容持续不变"的起点），
                       # 不能只靠别人那轮的拐点来套。
                       "hash": hashlib.sha256(np.ascontiguousarray(grid).tobytes()).hexdigest()[:16]}
                for c in cells:
                    p = to_frame(c["xy"][0], c["xy"][1], frame)
                    if p is None:
                        row[c["name"]] = None
                        row[c["name"] + "_idx"] = None
                        continue
                    row[c["name"]] = _val(grid, res, ox, oy, w, h, p[0], p[1])
                    # **当帧反算**的格索引（局部图会漂，所以必须每帧算，不能当常量用）
                    row[c["name"] + "_idx"] = "%d,%d" % (int((p[0] - ox) / res),
                                                         int((p[1] - oy) / res))
                rows.append(row)
            return cb
        node.create_subscription(mt, topic, make_cb(topic, tag), qos)

    t0 = time.time()
    while len(latest) < 4 and time.time() - t0 < 40:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("（%.1f s 内收到 %d/4 张图）" % (time.time() - t0, len(latest)))

    # 对照格：当前全局 raw 图上离机器人最近的 lethal 格
    if a.auto_control and "g_raw" in latest:
        pose = None
        t1 = time.time()
        while pose is None and time.time() - t1 < 15:
            try:
                t = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                pose = (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                rclpy.spin_once(node, timeout_sec=0.1)
        if pose is not None:
            _f, res, ox, oy, w, h, grid, _st = _meta(latest["g_raw"])
            ys, xs = np.where(grid == 254)
            if xs.size:
                wx = ox + (xs + 0.5) * res
                wy = oy + (ys + 0.5) * res
                d = np.hypot(wx - pose[0], wy - pose[1])
                i = int(np.argmin(d))
                cells.append({"name": "ctrl_nearest_lethal",
                              "xy": [round(float(wx[i]), 3), round(float(wy[i]), 3)]})
                print("对照格（离机器人最近的 lethal 格）：(%.2f, %.2f)，距机器人 %.2f m"
                      % (cells[-1]["xy"][0], cells[-1]["xy"][1], float(d[i])))
            else:
                print("** 全局 raw 图上当前没有任何 lethal 格 ⇒ 对照格无解 **")
        else:
            print("** 取不到机器人位姿 ⇒ 自动对照格跳过 **")

    print("跟踪格：%s" % "; ".join("%s=(%.2f,%.2f)" % (c["name"], c["xy"][0], c["xy"][1])
                                   for c in cells))
    print("\n[%s] 采样 %.0f s（每帧一行）" % (a.label, a.seconds))
    hdr = "%-8s %-12s %s" % ("图", "stamp", " ".join("%12s" % c["name"] for c in cells))
    print(hdr)
    t2 = time.time()
    while time.time() - t2 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    for r in rows:
        print("%-8s %-12.3f %s" % (r["tag"], r["stamp"],
                                   " ".join("%12s" % r.get(c["name"]) for c in cells)))
    print("\n小结（每个 图×格：帧数 / 取值集合 / 末值）：")
    summary = {}
    for _topic, tag in TOPICS:
        sub = [r for r in rows if r["tag"] == tag]
        if not sub:
            print("  %-8s ** 一帧都没收到 **" % tag)
            continue
        for c in cells:
            vals = [r.get(c["name"]) for r in sub if r.get(c["name"]) is not None]
            if not vals:
                print("  %-8s %-18s 该格所有帧都在图外" % (tag, c["name"]))
                continue
            uniq = sorted(set(vals))
            summary["%s|%s" % (tag, c["name"])] = {
                "frames": len(vals), "uniq": uniq, "last": vals[-1],
                "stamp_first": sub[0]["stamp"], "stamp_last": sub[-1]["stamp"]}
            flag = ""
            if c["name"].startswith("box"):
                flag = "  ← 箱格" + ("【一直 free：该图没消费它】" if uniq == [0] else
                                    "【有变化】" if len(uniq) > 1 else "【恒定非 free】")
            print("  %-8s %-18s %3d 帧  取值=%s  末值=%s%s"
                  % (tag, c["name"], len(vals), uniq, vals[-1], flag))

    if a.out:
        fnames = ["wall", "tag", "frame", "stamp", "ox", "oy", "hash"]
        for c in cells:
            fnames += [c["name"], c["name"] + "_idx"]
        with open(a.out, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=fnames)
            wr.writeheader()
            for r in rows:
                wr.writerow({k: r.get(k) for k in fnames})
    if a.json:
        json.dump({"label": a.label, "cells": cells, "summary": summary, "rows": rows},
                  open(a.json, "w"), ensure_ascii=False, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

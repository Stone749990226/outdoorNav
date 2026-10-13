#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""长连续序列记录（≥400 s 墙钟）：全局图**整图哈希逐帧** + **活采 TF 位姿** + 可选格值。

本轮判据（协调者 2026-09-21 口径）：
  · **恒定段 = 连续 ≥3 个相同哈希样本**（不许用"两个相同"当恒定）；
  · **判据 = "最长恒定段 = 0"**；有则**如实报起止（墙钟 + 仿真钟）与长度**；
  · **采样周期必须 ≤ 发布周期**（发布周期 ~1 s、共享机实测 ~0.71 Hz）。

★ 本脚本为什么**以"逐帧"为权威序列**（而不是"每 0.5 s 取最新一帧"）：
  "0.5 s 抽点、取最新一帧"在**发布周期 > 0.5 s** 时会把**同一帧读进去两三次**
  ⇒ 那是**假恒定段**（冻结的假阳性），正是要防的东西 ✗。
  **逐帧记录（每收到一帧写一行，天然按 header stamp 去重）的采样周期 = 发布周期本身**
  ⇒ 既 ≤ 发布周期，又**不可能重复读同一帧** ✓。
  ⇒ 因此本脚本**两条序列都产出**：
      `frame`  —— 逐帧（**权威**）；
      `tick0.5` —— 0.5 s 抽点（**仅用于报"实际间隔"**；重复帧会被显式标出来，
                   恒定段判定**只认 frame 序列**）。
  ⇒ 并打印**实际相邻间隔的中位数 / 最大值**与**样本数**（读的人可自己复算）。

用法（容器内）：
  longseq.py --seconds 420 --t0 <起栈墙钟 epoch> \
             --out-csv /work/.../r4_longseq.csv --json /work/.../r4_longseq.json \
             [--track "box=4.5,-3.3;ctrl=0.6,0.6"] [--wait 600]
"""

import argparse
import csv
import hashlib
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ★★ 聚合列的**刻度**（2026-09-21 修，reviewer 从 CSV 里发现，别删）★★
#   `lethal254_raw_only` / `unk255_raw_only` **只对 `*_raw` 行有意义**：
#     `*_raw` = nav2_msgs/Costmap      → 0–255，致命 254、未知 255
#     `*_inf` = nav_msgs/OccupancyGrid → 0–100，致命 100、未知 **−1**
#   ⇒ 那两列在 `*_inf` 行里**恒为 0**。**横着比这两列会得出"膨胀后的图没有未知格"** ✗
#   ⇒ 要用**跨族可比**的那一对：`lethal_norm` / `unknown_norm`（按该族自己的编码数，另附
#     `scale` / `lethal_code` / `unknown_code` 三列说明这一行用的是什么刻度）。
#     `free0` 两族都是 0 ⇒ 本来就可比。
TOPICS = [
    ("/global_costmap/costmap_raw", "g_raw"),
    ("/global_costmap/costmap",     "g_inf"),
    ("/local_costmap/costmap_raw",  "l_raw"),
    ("/local_costmap/costmap",      "l_inf"),
]
TICK = 0.5           # 抽点周期（发布周期的一半）
CONST_MIN = 3        # 恒定段判据：连续 ≥3 个相同样本


def _meta(msg):
    """(frame, res, ox, oy, w, h, grid, stamp) —— Costmap 与 OccupancyGrid 统一。"""
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
    st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return frame, res, ox, oy, w, h, data.reshape((h, w)), st


def _val(grid, res, ox, oy, w, h, x, y):
    mx, my = int((x - ox) / res), int((y - oy) / res)
    if not (0 <= mx < w and 0 <= my < h):
        return None
    return int(grid[my, mx])


def _jsonable(o):
    """numpy 标量不是 JSON 可序列化的（`np.bool_` 尤其：它不是 Python `bool` 的子类）
    ⇒ 统一降级。2026-09-21 实测：nav10-r4 的 A 轮 `r4_reproj_A.json` 就是这么丢的
    （`.txt` 那份仍在；数值本身没错，只是落盘那一步炸了）。"""
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.integer):
        return int(o)
    return str(o)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def const_runs(rows, min_len=CONST_MIN):
    """rows 已按 wall 升序；返回**极大**恒定段（连续相同 hash 且长度 ≥ min_len）。

    返回 [{i0,i1,n,wall0,wall1,sim0,sim1,hash,wall_span,sim_span}, ...]
    """
    runs = []
    i, n = 0, len(rows)
    while i < n:
        j = i
        while j + 1 < n and rows[j + 1]["hash"] == rows[i]["hash"]:
            j += 1
        k = j - i + 1
        if k >= min_len:
            runs.append({"i0": i, "i1": j, "n": k,
                         "wall0": rows[i]["wall"], "wall1": rows[j]["wall"],
                         "sim0": rows[i]["sim_hdr"], "sim1": rows[j]["sim_hdr"],
                         "wall_span": round(rows[j]["wall"] - rows[i]["wall"], 3),
                         "sim_span": round(rows[j]["sim_hdr"] - rows[i]["sim_hdr"], 3),
                         "hash": rows[i]["hash"]})
        i = j + 1
    return runs


def all_run_length_hist(rows):
    """**全部**极大恒等段的长度直方图（不设阈值）——只报"最长"会藏住形态。"""
    hist = {}
    i, n = 0, len(rows)
    while i < n:
        j = i
        while j + 1 < n and rows[j + 1]["hash"] == rows[i]["hash"]:
            j += 1
        k = j - i + 1
        hist[k] = hist.get(k, 0) + 1
        i = j + 1
    return {str(k): hist[k] for k in sorted(hist)}


def intervals(rows, key="wall"):
    if len(rows) < 2:
        return None
    d = [rows[i + 1][key] - rows[i][key] for i in range(len(rows) - 1)]
    d = sorted(d)
    return {"n": len(d), "min": round(d[0], 4), "median": round(d[len(d) // 2], 4),
            "max": round(d[-1], 4),
            "mean": round(sum(d) / len(d), 4)}


def pose_summary(rows):
    """位姿小结。**注意 `sim` 可能是 None**（见下），别直接拿去 `%.1f` 格式化。"""
    if len(rows) < 2:
        return None
    x0, y0, th0 = rows[0]["x"], rows[0]["y"], rows[0]["yaw"]
    dmax = max(math.hypot(r["x"] - x0, r["y"] - y0) for r in rows)
    dth = 0.0
    for r in rows:
        d = (r["yaw"] - th0 + math.pi) % (2 * math.pi) - math.pi
        dth = max(dth, abs(d))
    # ★★ 2026-09-21 修（`nav11-r2` 实测踩到，是真 bug，不是环境问题）★★
    #   `tick_pose` 是 0.05 s 的定时器，**在等第一帧全局图之前就已经在跑了**；
    #   而 `/clock` 是**订阅**、可能晚一拍。于是**头几行位姿的 `sim` 是 None**。
    #   原来 `sim0/sim1` 直接取 `rows[0]["sim"]` / `rows[-1]["sim"]`，
    #   再用 `%.1f` 格式化 ⇒ **TypeError: must be real number, not NoneType**
    #   ⇒ 崩在"车动没动"那一段的**打印**里 ⇒ **它后面的 CSV/JSON 落盘整段没跑**！
    #   后果极隐蔽：屏幕上"长连续序列"那一段**已经完整打出来了**（判据看着没问题），
    #   而**权威序列产物一个都没有**。`nav11-r2` 第一次尝试就是这样作废的。
    #   ⇒ 修法：sim 只从**有值**的行里取；`sim 未覆盖`的行数如实记下来。
    sims = [r["sim"] for r in rows if r.get("sim") is not None]
    return {"frames": len(rows),
            "wall0": rows[0]["wall"], "wall1": rows[-1]["wall"],
            "sim0": sims[0] if sims else None,
            "sim1": sims[-1] if sims else None,
            "sim_rows": len(sims), "sim_missing_rows": len(rows) - len(sims),
            "max_disp_m": round(dmax, 4), "max_yaw_deg": round(math.degrees(dth), 3),
            "first_xy": [round(x0, 4), round(y0, 4)]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=420.0)
    ap.add_argument("--t0", type=float, default=0.0, help="起栈墙钟 epoch（用来算 起栈后+N s）")
    ap.add_argument("--wait", type=float, default=600.0, help="等第一帧全局图的上限（s）")
    ap.add_argument("--track", default="", help='额外跟踪格 名=x,y;名2=x,y')
    ap.add_argument("--label", default="")
    ap.add_argument("--out-csv", default="")
    ap.add_argument("--json", default="")
    a = ap.parse_args()

    cells = []
    for item in (a.track or "").split(";"):
        if not item.strip():
            continue
        name, xy = item.split("=")
        x, y = [float(v) for v in xy.split(",")]
        cells.append({"name": name.strip(), "xy": [x, y]})

    rclpy.init()
    node = Node("longseq")
    import tf2_ros
    from geometry_msgs.msg import PointStamped
    from tf2_geometry_msgs import do_transform_point
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    node.create_timer(0.05, lambda: None)          # 让 executor 有机会跑 TF 订阅

    def to_frame(x, y, frame):
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
    from rosgraph_msgs.msg import Clock
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    state = {"sim": None}
    qc = QoSProfile(depth=10)
    qc.reliability = ReliabilityPolicy.BEST_EFFORT
    qc.durability = DurabilityPolicy.VOLATILE
    node.create_subscription(
        Clock, "/clock",
        lambda m: state.__setitem__("sim", m.clock.sec + m.clock.nanosec * 1e-9), qc)

    map_rows = []
    seen = {}

    def make_cb(tag):
        def cb(msg):
            frame, res, ox, oy, w, h, grid, st = _meta(msg)
            key = (tag, st)
            seen[key] = seen.get(key, 0) + 1
            # ★★ 刻度：**两族用的不是同一套编码**（2026-09-21 修，reviewer 从 CSV 里发现）★
            #   `*_raw`（nav2_msgs/Costmap）   ：0–255 代价刻度，致命 = **254**、未知 = **255**
            #   `*_inf`（nav_msgs/OccupancyGrid）：0–100 占据刻度，致命 = **100**、未知 = **−1**
            #   ⇒ 旧列 `lethal254` / `unk255` 在 `*_inf` 行里**恒为 0**（不是"没有致命/未知格"，
            #     是**编码不同**）⇒ 谁横着比这两列，就会得出
            #     "**膨胀后的图没有未知格**"，而它其实有约 58.8 万个（编码是 −1）✗✗
            #   ⇒ 修法：① 旧列名**标明"仅对 raw 有意义"**；② **另给一对归一化列**
            #     （按该族自己的编码数），跨族可比。**`free0` 两族都是 0 ⇒ 本来就可比，不动。**
            is_raw = tag.endswith("_raw")
            lethal_code = 254 if is_raw else 100
            unknown_code = 255 if is_raw else -1
            row = {"wall": round(time.time(), 3), "tag": tag, "frame": frame,
                   "sim_hdr": round(st, 3), "sim_now": state["sim"],
                   "res": res, "ox": round(ox, 3), "oy": round(oy, 3), "w": w, "h": h,
                   "hash": hashlib.sha256(np.ascontiguousarray(grid).tobytes()).hexdigest()[:16],
                   # 归一化列（**跨族可比**，推荐用这两列）
                   "scale": "0-255" if is_raw else "0-100(-1=unknown)",
                   "lethal_code": lethal_code, "unknown_code": unknown_code,
                   "lethal_norm": int((grid == lethal_code).sum()),
                   "unknown_norm": int((grid == unknown_code).sum()),
                   "free0": int((grid == 0).sum()),
                   # 旧列名：值一个没改，只把**适用范围**写进列名
                   "lethal254_raw_only": int((grid == 254).sum()),
                   "unk255_raw_only": int((grid == 255).sum())}
            for c in cells:
                p = to_frame(c["xy"][0], c["xy"][1], frame)
                row[c["name"]] = None if p is None else _val(grid, res, ox, oy, w, h, p[0], p[1])
            map_rows.append(row)
        return cb

    for topic, tag in TOPICS:
        mt = Costmap if topic.endswith("raw") else OccupancyGrid
        node.create_subscription(mt, topic, make_cb(tag), qos)

    # ---- 位姿：**活采**（不走 bag —— 上一轮 bag 提前结束，晚窗没有数）----
    pose_rows = {"map->base_footprint": [], "odom->base_footprint": []}

    def tick_pose():
        for parent, key in (("map", "map->base_footprint"),
                            ("odom", "odom->base_footprint")):
            try:
                t = buf.lookup_transform(parent, "base_footprint", rclpy.time.Time())
            except Exception:
                continue
            pose_rows[key].append({
                "wall": round(time.time(), 3), "sim": state["sim"],
                "stamp": round(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, 3),
                "x": round(t.transform.translation.x, 4),
                "y": round(t.transform.translation.y, 4),
                "yaw": round(_yaw(t.transform.rotation), 6)})

    node.create_timer(0.05, tick_pose)

    # ---- 等第一帧全局图 ----
    t0 = time.time()
    while not any(r["tag"] == "g_raw" for r in map_rows) and time.time() - t0 < a.wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not any(r["tag"] == "g_raw" for r in map_rows):
        print("** 等 %.0f s 没等到 /global_costmap/costmap_raw ⇒ 本轮作废 **" % a.wait)
        return 1
    first = [r for r in map_rows if r["tag"] == "g_raw"][0]
    print("第一帧全局 raw：起栈后 +%.0f s（墙钟）sim=%.3f  拿到的图：%s"
          % (first["wall"] - a.t0, first["sim_hdr"],
             sorted({r["tag"] for r in map_rows})))

    t_end = time.time() + a.seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    # ---------------- 分析 ----------------
    out = {"label": a.label, "t0": a.t0, "seconds": a.seconds,
           "tick_period": TICK, "const_min_samples": CONST_MIN,
           "first_g_raw_wall": first["wall"],
           "first_g_raw_rel": round(first["wall"] - a.t0, 1),
           "cells": cells, "tags": {}, "poses": {}}

    for _topic, tag in TOPICS:
        sub = sorted([r for r in map_rows if r["tag"] == tag], key=lambda r: r["wall"])
        if not sub:
            out["tags"][tag] = {"error": "一帧都没收到"}
            continue
        dup = sorted({k for k, v in seen.items() if k[0] == tag and v > 1})
        # 0.5 s 抽点序列（仅供参考）：每个 tick 取"wall ≤ tick 的最后一帧"
        ticks, wt = [], first["wall"]
        while wt <= sub[-1]["wall"]:
            cand = [r for r in sub if r["wall"] <= wt]
            if cand:
                ticks.append(cand[-1])
            wt += TICK
        tick_uniq = 1 + sum(1 for i in range(1, len(ticks)) if ticks[i]["hash"] != ticks[i - 1]["hash"])
        out["tags"][tag] = {
            "frames": len(sub), "frame_first_wall": sub[0]["wall"], "frame_last_wall": sub[-1]["wall"],
            "frame_first_sim": sub[0]["sim_hdr"], "frame_last_sim": sub[-1]["sim_hdr"],
            "frame_wall_span": round(sub[-1]["wall"] - sub[0]["wall"], 3),
            "sample_interval_s": intervals(sub),
            "dup_stamps": len(dup),
            "tick0p5_samples": len(ticks), "tick0p5_distinct_values": tick_uniq,
            "run_len_hist": all_run_length_hist(sub),
            "const_runs_ge3": const_runs(sub, CONST_MIN),
            "const_runs_ge2": const_runs(sub, 2),
        }

    for key, rows in pose_rows.items():
        rows.sort(key=lambda r: r["wall"])
        out["poses"][key] = pose_summary(rows) or {"error": "样本不足"}
    out["pose_header"] = "位姿为**活采**（tf2 Buffer，20 Hz），不是从 bag 里读"

    print("\n================ 长连续序列（逐帧，权威） ================")
    for tag in ("g_raw", "g_inf", "l_raw", "l_inf"):
        s = out["tags"].get(tag)
        if not s or "error" in s:
            print("  %-6s %s" % (tag, (s or {}).get("error", "?")))
            continue
        iv = s["sample_interval_s"] or {}
        print("  %-6s 帧数=%-5d 墙钟跨度=%.1f s  sim %.1f→%.1f  实际相邻间隔 中位=%.3f s 最大=%.3f s"
              % (tag, s["frames"], s["frame_wall_span"], s["frame_first_sim"],
                 s["frame_last_sim"], iv.get("median", -1), iv.get("max", -1)))
        print("         恒等段长度直方图 %s" % s["run_len_hist"])
        r3 = s["const_runs_ge3"]
        # ★ 必须取 **n 最大**的那一段：const_runs 按时间先后返回，
        #   原来写 r3[0] 打的是**第一个** ≥3 的段 ⇒ 会把"最长"报成一小段
        #   （2026-09-21 nav10-r4 实测踩到：真实最长 234 个样本，却印成 4 个 ✗）。
        r3max = max(r3, key=lambda r: r["n"]) if r3 else None
        print("         **连续 ≥%d 相同样本的恒定段：%d 段；最长 %s**"
              % (CONST_MIN, len(r3),
                 ("%d 个样本 / %.1f s 墙钟（起栈后 +%.0f → +%.0f s；sim %.1f → %.1f）"
                  % (r3max["n"], r3max["wall_span"], r3max["wall0"] - a.t0,
                     r3max["wall1"] - a.t0, r3max["sim0"], r3max["sim1"])) if r3max else "无（=0）"))
        if s["dup_stamps"]:
            print("         ** 有 %d 个 header stamp 重复出现 ⇒ 逐帧序列本身也有重复，需要看 **"
                  % s["dup_stamps"])

    print("\n================ 车动没动（活采 TF） ================")
    for key, ps in out["poses"].items():
        if not ps or "error" in ps:
            print("  %-22s %s" % (key, (ps or {}).get("error", "?")))
            continue
        # sim 可能为 None（见 pose_summary 的说明）⇒ **不打 %.1f，先判空**
        _s0 = "n/a" if ps["sim0"] is None else "%.1f" % ps["sim0"]
        _s1 = "n/a" if ps["sim1"] is None else "%.1f" % ps["sim1"]
        print("  %-22s %5d 帧  墙钟 %.0f→%.0f（起栈后 +%.0f→+%.0f s）sim %s→%s"
              % (key, ps["frames"], ps["wall0"], ps["wall1"],
                 ps["wall0"] - a.t0, ps["wall1"] - a.t0, _s0, _s1))
        if ps.get("sim_missing_rows"):
            print("       （其中 %d 行在收到第一帧 /clock 之前采到 ⇒ sim 为 None，"
                  "已在 sim0/sim1 里跳过；**不影响 wall 与位姿判据**）"
                  % ps["sim_missing_rows"])
        print("       **最大位移=%.4f m  最大转角=%.3f°**  起点 xy=%s"
              % (ps["max_disp_m"], ps["max_yaw_deg"], ps["first_xy"]))

    if a.out_csv:
        fnames = ["wall", "tag", "frame", "sim_hdr", "sim_now", "res", "ox", "oy",
                  "w", "h", "hash",
                  # 归一化列（跨族可比）
                  "scale", "lethal_code", "unknown_code", "lethal_norm", "unknown_norm",
                  "free0",
                  # 旧列名（值未变，仅标注"只对 *_raw 有意义"）
                  "lethal254_raw_only", "unk255_raw_only"]
        for c in cells:
            fnames.append(c["name"])
        with open(a.out_csv, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=fnames)
            wr.writeheader()
            for r in sorted(map_rows, key=lambda r: r["wall"]):
                wr.writerow({k: r.get(k) for k in fnames})
    if a.json:
        out["map_rows"] = map_rows
        out["pose_rows"] = pose_rows
        json.dump(out, open(a.json, "w"), ensure_ascii=False, indent=1,
                  allow_nan=False, default=_jsonable)
    return 0


if __name__ == "__main__":
    sys.exit(main())

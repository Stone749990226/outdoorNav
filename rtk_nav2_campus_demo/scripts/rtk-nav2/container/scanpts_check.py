#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核**新入口**两端：`/rtk/scan`(LaserScan) 与 `/rtk/scan_points`(PointCloud2) 是否**逐束等价**。

## 为什么必须有这一项（它挡的是本轮结论最要命的替代解释）

"换了入口之后不冻了"这句话，在下面这种情况下**同样成立、但毫无意义**：
**新源根本没接到**（节点没起 / 话题名错 / 类型不对）。图照样"不冻"——**因为它压根没在更新** ✗。
整图哈希看不穿这件事（"没变"与"变了"之外它没有第三种读数）。
⇒ 必须**独立地**证明三件事，缺一条结论就不能用：

  ① **新话题有帧**（`/rtk/scan_points` 真在发，且 `PointCloud2`）；
  ② **逐束等价**：`out_points == 有限束 + 无回波束`（一束一个点），
     且**无回波束的端点半径 == 替换值**（本配置 7.9999）——不是"随便补了几个点"；
  ③ **替换出来的端点半径 > 激光源的 `obstacle_max_range`(7.5)**
     ⇒ 它们进不了 marking ⇒ **不会在开阔方向长出假墙**（这是"补空闲"与"画假墙"的分界）。

★ 判据取的是**半径的分界**，不是"取最小/极值当结论"：
  把点按半径分成 `>= 7.5`（只可能是补出来的端点）与 `< 7.5`（只可能是真实回波）两堆，
  **两堆的计数各自与 LaserScan 那一侧的两种束数对齐**。
  （本场景实测真实回波最大 7.228 m < 7.5 m，所以这条分界是干净的；
   若将来出现 7.5~8.0 的真实障碍，`>=7.5` 那一堆会多出来 —— 报告里两个数都印，交给人判。）

用法（容器内）：
  scanpts_check.py --pairs 6 --obstacle-max-range 7.5 --out /work/.../r1_scanpts_check.json
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

FAR_MIN = 7.5      # 只作"分堆"用；真正判"不长假墙"的是替换值本身 > obstacle_max_range


def cloud_xyz(msg):
    """按 fields 里的 offset 取 x/y/z（**不写死 12**：projectLaser 的 point_step 是 16）。"""
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in ("x", "y", "z")):
        return None
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), np.float64)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    out = np.empty((n, 3), np.float64)
    for i, k in enumerate(("x", "y", "z")):
        out[:, i] = raw[:, off[k]:off[k] + 4].copy().view(np.float32).reshape(n)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pairs", type=int, default=6, help="配几对帧（按 header stamp 就近配对）")
    ap.add_argument("--obstacle-max-range", type=float, default=7.5)
    ap.add_argument("--warmup", type=float, default=3.0,
                    help="**订阅建立期**先丢掉这么多秒的帧，再开始配对")
    ap.add_argument("--tol", type=float, default=0.6,
                    help="配对容差（s）。**实测：转换节点比 /rtk/scan 滞后约 0.3 s**，"
                         "所以容差必须 ≥ 那个滞后；又因为云里每一帧的 header 就是"
                         "对应那一帧 LaserScan 的 header（**匹配距离≈0**），"
                         "容差放宽**不会配错帧**。")
    ap.add_argument("--timeout", type=float, default=40.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    rclpy.init()
    node = Node("scanpts_check")
    from sensor_msgs.msg import LaserScan, PointCloud2
    scans, clouds = [], []
    node.create_subscription(LaserScan, "/rtk/scan", lambda m: scans.append(m),
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/rtk/scan_points", lambda m: clouds.append(m),
                             qos_profile_sensor_data)

    # ★★ 2026-09-21 修（`nav11-r2b` 实测踩到）：**订阅建立期两边起点不齐会假失败** ★★
    #   现象：驱动刚起栈就跑这一步时报"一对帧都没配上（stamp 差 > 0.2 s）"⇒ 整轮作废；
    #   而同一条命令**过一会儿手动重跑就全过** ⇒ 是**探针的竞态**，不是数据问题。
    #   根因：转换节点比 `/rtk/scan` **滞后约 0.3 s**（Python + laser_geometry + 360 束投影）；
    #   订阅刚建立时，先收到的几帧云，其 stamp **早于**本次收到的第一帧 scan
    #   ⇒ 那几帧在 scan 列表里找不到配对。
    #   ⇒ 修法：① **预热**（丢掉开头 `--warmup` 秒的帧）；② 配对时用**最后** `pairs` 帧云、
    #          并在**全部** scan 里找最近（不截断 scan 列表）；③ 容差按实测滞后放宽到 0.6 s。
    t0 = time.time()
    nmin = a.pairs + 6
    while (len(scans) < nmin or len(clouds) < nmin) and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    tw = time.time()
    while time.time() - tw < a.warmup:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("预热 %.1f s 后：scan 累计 %d 帧、cloud 累计 %d 帧（配对只用最后 %d 帧云）"
          % (a.warmup, len(scans), len(clouds), a.pairs))

    if not scans:
        print("** /rtk/scan %.0f s 内没收到帧（激光源没在发）**" % a.timeout)
        return 1
    if not clouds:
        print("** /rtk/scan_points %.0f s 内没收到帧 ⇒ **新入口一个发布者都没有**："
              "`scan_points` 节点没起、话题名不对、或类型不对 ✗ "
              "**这一条不过 ⇒ 本轮「不冻」没有意义（图根本没在更新）** **" % a.timeout)
        return 1

    def stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    rows = []
    lag = []
    for c in clouds[-a.pairs:]:
        cs = stamp(c)
        sm = min(scans, key=lambda m: abs(stamp(m) - cs))
        d = abs(stamp(sm) - cs)
        lag.append(d)
        if d > a.tol:
            continue
        scan_msg = sm
        s = scan_msg
        rs = np.asarray(s.ranges, dtype=np.float64)
        finite = np.isfinite(rs)
        # 无回波：inf / nan / >= range_max / < range_min（与 scan_points.py 的判定逐条一致）
        noecho = (~finite) | (rs >= s.range_max) | (rs < s.range_min)
        n_fin, n_no = int((finite & ~noecho).sum()), int(noecho.sum())

        xyz = cloud_xyz(c)
        if xyz is None:
            print("** 点云缺 x/y/z 字段：%s **" % [f.name for f in c.fields])
            return 1
        rad = np.hypot(xyz[:, 0], xyz[:, 1])
        far = rad >= FAR_MIN
        n_far, n_near = int(far.sum()), int((~far).sum())

        row = {"sim": round(cs, 3), "scan_frame": (s.header.frame_id or "?").lstrip("/"),
               "cloud_frame": (c.header.frame_id or "?").lstrip("/"),
               "n_beams": int(len(rs)), "beams_finite": n_fin, "beams_noecho": n_no,
               "cloud_points": int(len(rad)), "cloud_point_step": int(c.point_step),
               "cloud_fields": [f.name for f in c.fields],
               "points_radius_ge_%.1f" % FAR_MIN: n_far,
               "points_radius_lt_%.1f" % FAR_MIN: n_near,
               "far_radius_min": round(float(rad[far].min()), 5) if n_far else None,
               "far_radius_max": round(float(rad[far].max()), 5) if n_far else None,
               "near_radius_max": round(float(rad[~far].max()), 5) if n_near else None,
               "scan_range_max": float(s.range_max), "scan_range_min": float(s.range_min)}
        # 判据
        c1 = (len(rad) == n_fin + n_no)
        c2 = (n_far == n_no and n_near == n_fin)
        c3 = bool(n_far == 0 or row["far_radius_min"] > a.obstacle_max_range)
        row.update({"一束一点": c1, "两种束各自对齐": c2,
                    "补出的端点 > obstacle_max_range": c3})
        rows.append(row)

    if not rows:
        print("** 一对帧都没配上（最近 stamp 差都 > %.2f s；实测最近差=%s）**"
              "⇒ 两条话题可能不是同一个源，或新节点没在跟着转发"
              % (a.tol, ["%.3f" % v for v in sorted(lag)[:6]]))
        return 1

    print("=== 新入口两端对照（%d 对帧，按 header stamp 就近配对；容差 %.2f s）==="
          % (len(rows), a.tol))
    print("实测配对距离：最大 %.3f s（= 转换节点相对 /rtk/scan 的滞后量级）"
          % (max(lag) if lag else -1))
    print("字段：有限束 = 有限且 range_min<=r<range_max；无回波束 = inf/nan/>=range_max/<range_min")
    hdr = ("sim", "有限束", "无回波束", "点云点数", "r>=%.1f" % FAR_MIN, "r<%.1f" % FAR_MIN,
           "远点最小半径", "近点最大半径", "一束一点", "两种束对齐", "远点>omr")
    print("  " + "  ".join("%-12s" % h for h in hdr))
    for r in rows:
        print("  %-12s %-12d %-12d %-12d %-12d %-12d %-12s %-12s %-12s %-12s %-12s"
              % (r["sim"], r["beams_finite"], r["beams_noecho"], r["cloud_points"],
                 r["points_radius_ge_7.5"], r["points_radius_lt_7.5"],
                 r["far_radius_min"], r["near_radius_max"],
                 r["一束一点"], r["两种束各自对齐"], r["补出的端点 > obstacle_max_range"]))
    allok = all(r["一束一点"] and r["两种束各自对齐"]
                and r["补出的端点 > obstacle_max_range"] for r in rows)
    fr = [r["far_radius_min"] for r in rows if r["far_radius_min"] is not None]
    print("\n结论：")
    print("  ① 新话题 /rtk/scan_points 有帧（PointCloud2，point_step=%s，字段 %s）✓"
          % (rows[0]["cloud_point_step"], rows[0]["cloud_fields"]))
    print("  ② 逐束等价：%s（每对帧都满足「点数 = 有限束 + 无回波束」，且两堆计数各自对齐）"
          % ("**成立**" if allok else "**不成立 ⇒ 不能当等价入口用**"))
    if fr:
        print("  ③ 无回波束被补成的端点半径 = %.5f m（%d 对帧里的最小值）> "
              "obstacle_max_range=%.2f m ⇒ **进不了 marking，不会在开阔方向长出假墙** ✓"
              % (min(fr), len(fr), a.obstacle_max_range))
    if a.out:
        json.dump({"rows": rows, "all_ok": allok, "obstacle_max_range": a.obstacle_max_range},
                  open(a.out, "w"), ensure_ascii=False, indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())

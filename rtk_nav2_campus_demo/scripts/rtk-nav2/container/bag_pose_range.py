#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从包里取"车到底动没动"的**真证人**：位姿序列（不是 info.origin —— 那两个 origin 是常量）。

reviewer 反证：局部图 `global_frame: base_link` + `rolling_window: true` ⇒ origin 按
"机器人在 base_link 里的位置(=0,0)"算 ⇒ **恒为常量**；全局图 `rolling_window: false` ⇒
origin 只在 `resizeMap` 时设定 ⇒ **也是常量**。⇒ 两者都不是证人。

判据：窗口内 **最大位移 / 最大转角** < 阈值 ⇒ "车没动"，并且**把这两个数写出来**。
另：`/clock` 的 stamp 在推进 ≠ 车在动（仿真在跑不等于车在动）。

用法：bag_pose_range.py <bag_dir> [--tail 秒]
"""
import argparse
import glob
import math
import os
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--tail", type=float, default=0.0, help="只看最后 N 秒（0=全程）")
    a = ap.parse_args()

    dbs = sorted(glob.glob(os.path.join(a.bag, "*.db3")))
    if not dbs:
        print("** 没有 db3 **")
        return 1

    # 话题 -> type
    types = {}
    con = sqlite3.connect(dbs[0])
    for name, typ in con.execute("SELECT name, type FROM topics"):
        types[name] = typ
    con.close()

    want = {}
    for t in ("/ground_truth/odom", "/odometry/global", "/odometry/local"):
        if t in types:
            want[t] = []
    if not want:
        print("** 包里没有位姿话题；有的：%s **" % sorted(types))
        return 1

    t0 = t1 = None
    for db in dbs:
        con = sqlite3.connect(db)
        for topic in list(want):
            cls = get_message(types[topic])
            for _ts, data in con.execute(
                    "SELECT timestamp, data FROM messages WHERE topic_id = "
                    "(SELECT id FROM topics WHERE name = ?)", (topic,)):
                m = deserialize_message(bytes(data), cls)
                st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                if t0 is None or st < t0:
                    t0 = st
                if t1 is None or st > t1:
                    t1 = st
                p = m.pose.pose
                want[topic].append((st, p.position.x, p.position.y, yaw_of(p.orientation)))
        con.close()

    print("包里位姿话题：%s；时间跨度 sim %.1f → %.1f s" % (sorted(want), t0, t1))
    for topic, rows in want.items():
        rows.sort()
        if a.tail > 0:
            rows = [r for r in rows if r[0] >= t1 - a.tail]
        if len(rows) < 2:
            print("  %-22s 样本不足（%d）" % (topic, len(rows)))
            continue
        x0, y0, th0 = rows[0][1], rows[0][2], rows[0][3]
        dx = max(abs(r[1] - x0) for r in rows)
        dy = max(abs(r[2] - y0) for r in rows)
        # 最大位移（相对首帧）
        dmax = max(math.hypot(r[1] - x0, r[2] - y0) for r in rows)
        # 最大转角（相对首帧，取最短弧）
        dth = 0.0
        for r in rows:
            d = r[3] - th0
            d = (d + math.pi) % (2 * math.pi) - math.pi
            dth = max(dth, abs(d))
        print("  %-22s %5d 帧  sim %.1f→%.1f  |Δx|max=%.3f m |Δy|max=%.3f m "
              "**最大位移=%.3f m  最大转角=%.2f°**" %
              (topic, len(rows), rows[0][0], rows[-1][0], dx, dy, dmax, math.degrees(dth)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

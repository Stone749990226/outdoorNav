#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**本轮自测拐点**：从**本轮自己的包**里算 `/global_costmap/costmap` 的整图哈希序列。

判据（brief §3）：拐点 = **"同一内容持续不变"的起点** —— **不套别人那轮的拐点**。
做法：逐条取该话题的消息、对 `data` 做哈希；(stamp, hash) 成序列后，
**从尾部往前找**：最早的那个"此后一直不变"的下标 ⇒ 它的 stamp 就是拐点。

用法：bag_map_onset.py <bag_dir> [--topic /global_costmap/costmap]
"""
import argparse
import glob
import hashlib
import os
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/global_costmap/costmap")
    ap.add_argument("--show", type=int, default=12, help="打印前/后各多少行")
    a = ap.parse_args()

    dbs = sorted(glob.glob(os.path.join(a.bag, "*.db3")))
    con = sqlite3.connect(dbs[0])
    types = dict(con.execute("SELECT name, type FROM topics"))
    con.close()
    if a.topic not in types:
        print("** 包里没有 %s；有的：%s **" % (a.topic, sorted(types)))
        return 1
    cls = get_message(types[a.topic])

    rows = []
    for db in dbs:
        con = sqlite3.connect(db)
        for _ts, data in con.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = "
                "(SELECT id FROM topics WHERE name = ?)", (a.topic,)):
            m = deserialize_message(bytes(data), cls)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            rows.append((st, hashlib.sha256(bytes(m.data)).hexdigest()[:16]))
        con.close()
    rows.sort()
    if len(rows) < 3:
        print("** %s 只有 %d 条，算不了拐点 **" % (a.topic, len(rows)))
        return 1

    # 从尾部往前找"此后一直不变"的最早下标
    cur = rows[-1][1]
    i = len(rows) - 1
    while i > 0 and rows[i - 1][1] == cur:
        i -= 1
    onset = rows[i][0]
    changes = sum(1 for k in range(1, len(rows)) if rows[k][1] != rows[k - 1][1])
    print("话题 %s：%d 条，stamp %.1f → %.1f s" % (a.topic, len(rows), rows[0][0], rows[-1][0]))
    print("内容变化次数 %d；**末段恒定从 stamp=%.1f s 开始**（此后 %d 条哈希全同）"
          % (changes, onset, len(rows) - i))
    print("⇒ **本轮自测拐点 = %.1f s（仿真钟）**" % onset)
    if changes:
        last_change = max(k for k in range(1, len(rows)) if rows[k][1] != rows[k - 1][1])
        print("   最后一次变化在 stamp=%.1f s（= 拐点前一刻）" % rows[last_change][0])
    print("\n序列（起 %d 行）：" % a.show)
    for st, h in rows[:a.show]:
        print("  sim %8.3f  %s" % (st, h))
    print("  …")
    for st, h in rows[-a.show:]:
        print("  sim %8.3f  %s" % (st, h))
    return 0


if __name__ == "__main__":
    sys.exit(main())

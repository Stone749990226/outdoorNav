#!/usr/bin/env python3
"""查 `map→odom` 这条 TF 还在不在推进（2026-09-22；轻量验收里撞到"全局图不吃新输入"时用）。

为什么用**时间戳差**而不是"值变没变"：值变不变分不清"停了"与"车没动"；
时间戳差（查到的最新 stamp 与当前 sim 时刻之差）直接把"停更"与"在推进"分开 ✓。
用法（容器内）：tf_stamp_probe.py [--seconds 12] [--pairs map:odom,map:base_footprint]
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
# /tf 与 /clock 都是 volatile（**别照抄代价地图那套 LATCHED**：本项目在 QoS 上栽过好几次）
VOL = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE)


class Probe(Node):
    def __init__(self, want):
        super().__init__('tf_stamp_probe')
        self.want = set(want)
        self.seen = {}          # "parent->child" -> (stamp, count)
        self.sim = None
        self.create_subscription(TFMessage, '/tf', self.on_tf, VOL)
        self.create_subscription(Clock, '/clock', self.on_clock, VOL)

    def on_tf(self, msg):
        for t in msg.transforms:
            key = '%s->%s' % (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
            if key in self.want:
                stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                prev = self.seen.get(key)
                self.seen[key] = (stamp, (prev[1] + 1) if prev else 1)

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--ref', default='map:odom')
    ap.add_argument('--pairs', default='map:odom,odom:base_footprint,map:base_footprint')
    a = ap.parse_args()
    want = [p.replace(':', '->') for p in a.pairs.split(',')]

    rclpy.init()
    n = Probe(want)
    t_end = time.time() + a.seconds
    rows = []
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.2)
        rows.append({'wall': round(time.time(), 3), 'sim': n.sim,
                     **{k: (v[0], v[1]) for k, v in n.seen.items()}})
    out = {'produced_by': 'tf_stamp_probe.py',
           'produced_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
           'via': '/tf 与 /clock 订阅（volatile，depth 100）+ 时间戳差',
           'seconds': a.seconds, 'rows': rows[-1:], 'samples': len(rows)}
    print(json.dumps(out, ensure_ascii=False, indent=1))
    if not rows:
        return 1
    last = rows[-1]
    sim = last.get('sim')
    print('=== 时间戳差（查到的最新 stamp − 当前 sim 时刻）===')
    bad = 0
    for p in want:
        key = p.replace(':', '->')
        if key not in last:
            print('  %-24s **一次都没收到** ⇒ 查不到（不等于停更，见 §9.6 的"空 ≠ 不变"）' % key)
            bad += 1
            continue
        stamp, cnt = last[key]
        diff = (sim - stamp) if sim is not None else None
        verdict = ('**在推进 ✓**（差 %.3f s）' % diff) if (diff is not None and abs(diff) < 3.0) \
            else ('**停更 ✗**（stamp 落后当前时刻 %.3f s）' % diff if diff is not None else '(无 /clock)')
        print('  %-24s stamp=%.3f 帧数=%d ⇒ %s' % (key, stamp, cnt, verdict))
        if diff is not None and abs(diff) >= 3.0:
            bad += 1
    n.destroy_node()
    rclpy.shutdown()
    return 0 if bad == 0 else 2


if __name__ == '__main__':
    sys.exit(main())

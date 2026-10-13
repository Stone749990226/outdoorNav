#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""attitude_recorder.py —— ★ 把**车体俯仰**按高频**写进产物** ✓（✗ 不是只进 bag ✗）

用法（容器内）:
  attitude_recorder.py --out <产物目录>/attitude.csv [--hz 50] [--frame <传感器帧>] [--seconds 3600]

★★ 为什么要有它（reviewer 2026-10-12 指派 ✓）★★
  ★ 探针（`residue_probe.py`）里**已经有**姿态 —— ★ 逐拍 `tick_*.json` 的 `scan.tf.M` 是 3x4 ✓、
    取 `acos(|M[2][2]|)` 就是车体俯仰 ✓（★ 与 `arc_pitch_watcher.py` **同一把尺** ✓）。
  ★ 但探针是 **2 s 一拍** ✗ ⇒ ★ 业主说的"**停车时前倾后恢复**"是**亚秒级瞬态** ✗ ⇒ **看不见** ✗。
  ⇒ ★ 本节点**只为补这一件事** ✓：★ 按 **50 Hz**（IMU 的原始速率 ✓）把同一个量记下来 ✓。

★★ 尺子必须与既有的一致 ✓（✗ 别自造一把 ✗）★★
  ★ 取 `map → <传感器帧>` 的变换 ✓，`pitch_deg = degrees(acos(|R[2][2]|))` ✓ ——
    ★ 与探针 `scan.tf.M[2][2]` / `arc_pitch_watcher` **逐字同一把尺** ✓。
  ★ ★ 别拿 `robot_tf` 那一份 ✗：★ 它是**二维定位**给的（第三行恒 `[0,0,1,0]`）⇒ 会恒得 0 ✗ —— 本项目踩过 ✓。

★ 输出 ✓：`attitude.csv`，列 = `t_sim,pitch_deg,roll_deg,ok`（★ 逐行 flush ✓ ⇒ 半截产物也能用 ✓）
★ 收尾 ✓：★ `SIGTERM` 也要**把小结写出去** ✓（★ `arc_pitch_watcher.py` 栽过一次：只正常退出才写 ⇒ 被杀就啥也没有 ✗）
"""
import argparse
import csv
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

DEFAULT_FRAME = 'robot_rtk/base_footprint/near_lidar'


def quat_to_R(x, y, z, w):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


class Recorder(Node):
    def __init__(self, out, frame, hz):
        super().__init__('attitude_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
                                                       rclpy.Parameter.Type.BOOL, True)])
        self.out = out
        self.frame = frame
        self.f = open(out, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_sim', 'pitch_deg', 'roll_deg', 'ok'])
        self.f.flush()
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.n = 0
        self.n_ok = 0
        self.n_fail = 0
        self.max_pitch = 0.0
        self.summary_done = False
        self.create_timer(1.0 / float(hz), self.tick)
        self.get_logger().info('attitude_recorder 起 ✓ out=%s frame=%s hz=%s' % (out, frame, hz))

    def tick(self):
        t = self.get_clock().now()
        try:
            tr = self.buf.lookup_transform('map', self.frame, Time())
            q = tr.transform.rotation
            R = quat_to_R(q.x, q.y, q.z, q.w)
            c = max(-1.0, min(1.0, abs(R[2][2])))
            pitch = math.degrees(math.acos(c))
            roll = math.degrees(math.atan2(R[2][1], R[2][2]))
            ok = 1
            self.n_ok += 1
            self.max_pitch = max(self.max_pitch, pitch)
        except Exception:
            pitch = roll = float('nan')
            ok = 0
            self.n_fail += 1
        self.w.writerow(['%.6f' % (t.nanoseconds * 1e-9), '%.4f' % pitch, '%.4f' % roll, ok])
        self.n += 1
        if self.n % 50 == 0:
            self.f.flush()

    def summary(self, why):
        if self.summary_done:
            return
        self.summary_done = True
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass
        print('★ attitude_recorder 小结（%s）：n=%d ok=%d fail=%d max_pitch=%.3f°  ⇒ %s'
              % (why, self.n, self.n_ok, self.n_fail, self.max_pitch, self.out), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=50.0)
    ap.add_argument('--frame', default=DEFAULT_FRAME)
    ap.add_argument('--seconds', type=float, default=3600.0)
    a = ap.parse_args()

    rclpy.init()
    rec = Recorder(a.out, a.frame, a.hz)

    def stop(signum, frame):
        rec.summary('SIGTERM')
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    deadline = rec.get_clock().now().nanoseconds * 1e-9 + a.seconds
    try:
        while rclpy.ok():
            rclpy.spin_once(rec, timeout_sec=0.1)
            if rec.get_clock().now().nanoseconds * 1e-9 > deadline:
                break
    finally:
        rec.summary('正常退出')
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmdvel_recorder.py —— ★ 把**控制器实际下的速度**记进产物 ✓（reviewer 2026-10-12 指派 ✓）

用法（容器内）: cmdvel_recorder.py --out <产物目录>/cmdvel.csv [--seconds 3600]

★★ 为什么单独写一个、而不是加进 bag ★★
  ★ 实测 ✓：★ 各轮的 bag **只有 4 个话题** ✓（`/clock`、`/rtk/scan`、相机两张 ✓）——
    ★ 而 `start_stack.sh` 的 `ros2 bag record` 列表**远不止这 4 个** ✓（`/cmd_vel`、`/rtk/imu`、
    两张 costmap …… 都写着 ✓）⇒ ★★ **`ros2 bag record` 起得太早** ✗：★ 它**只录"起录那一刻已存在"的话题** ✗
    ⇒ ★ 那一刻 nav2 还没起 ⇒ `/cmd_vel`、costmap 这些**全被跳过** ✗✓（★ 这解释了"列表很长、包里只有 4 个" ✓）
  ⇒ ★ 所以**加一行到 bag 列表没用** ✗ ⇒ ★ 本节点**在起栈后**才起 ✓，故能订到 ✓。

★ 记什么 ✓：★ `t_sim,linear_x,angular_z` ✓（★ 逐条 flush ✓；★ `SIGTERM` 也写小结 ✓）
★ 为什么值得 ✓：★ `gt` 只给"车走了多快" ✓；★ 要查"同一棵树同一航线，录屏那轮 0.493 m/s、
  本机 0.217 ✗"这条 2.3× 差落在哪 ⇒ ★ 必须看**指令侧** ✓（★ 指令相同而结果不同 ⇒ 在物理/时序；
  ★ 指令本身就不同 ⇒ 在规划/控制 ✓）。
"""
import argparse
import csv
import signal
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Rec(Node):
    def __init__(self, out):
        super().__init__('cmdvel_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',
                                                       rclpy.Parameter.Type.BOOL, True)])
        self.f = open(out, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_sim', 'linear_x', 'angular_z'])
        self.f.flush()
        self.n = 0
        self.sum = 0.0
        self.mx = 0.0
        self.done = False
        self.create_subscription(Twist, '/cmd_vel', self.cb, 50)
        self.get_logger().info('cmdvel_recorder 起 ✓ out=%s' % out)

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        v = msg.linear.x
        self.w.writerow(['%.6f' % t, '%.6f' % v, '%.6f' % msg.angular.z])
        self.n += 1
        self.sum += abs(v)
        self.mx = max(self.mx, abs(v))
        if self.n % 50 == 0:
            self.f.flush()

    def summary(self, why):
        if self.done:
            return
        self.done = True
        try:
            self.f.flush(); self.f.close()
        except Exception:
            pass
        mean = self.sum / self.n if self.n else 0.0
        print('★ cmdvel_recorder 小结（%s）：n=%d ｜ |linear_x| 均值 %.4f ｜ max %.4f ｜ 文件 %s'
              % (why, self.n, mean, self.mx, self.f.name), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=3600.0)
    a = ap.parse_args()
    rclpy.init()
    rec = Rec(a.out)

    def stop(signum, frame):
        rec.summary('SIGTERM'); rclpy.shutdown(); sys.exit(0)

    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
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

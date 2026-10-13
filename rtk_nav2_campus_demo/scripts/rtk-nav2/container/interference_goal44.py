#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""interference_goal44.py —— ★ **故意造的干扰源**：在**域 44** 上往 `/goal_pose` 发一个**异目标** ✗

用法（容器内）:
    ROS_DOMAIN_ID=44 python3 interference_goal44.py --x 123.0 --y 456.0 [--hz 1.0] [--seconds 1800]

★★ 为什么要有它（★ 本项目定式"**验隔离要带着干扰验**"✓）★★
  ★ "空场里验过" = **没验** ✗ —— ★ 我们的 `campus9-res1` 跑在**域 45** ✓，
    而"隔离到底成不成立"只有在**域 44 真有人发东西**时才有意义 ✓。
  ★ 本轮原本有个**现成的干扰源**（业主那轮残留的驱动 ✓，还在域 44 上发 `campus9-demo2` 的点 ✓）——
    ★ 但他把它收掉了 ✗ ⇒ ★ **空场** ✗ ⇒ ★ 自己造一个 ✓（★ 这正是那条定式要求的做法 ✓）。

★★ 坐标**故意取域外值** ✗（默认 `(123, 456)` ✓，★ 离场景几百米 ✓）——
  ★ 这样"我们的 nav2 收到过它"这件事**不可能有别的解释** ✓（✗ 不会与航线点混淆 ✓）。

★ 判据（★ 由 `residue_analyze` / 日志核 ✓）：
  ★ 我们那一轮 nav2 的 `goal_navigator` 日志里，★ 收到过的目标**必须全是本轮的 8 个点** ✓，
  ★ 且 ★ **这个异坐标出现次数必须 = 0** ✓ ⇒ ★ 隔离成立 ✓（★ 带着干扰验的 ✓）。
退出码: 0 = 发满 --seconds ｜ 2 = 起不来
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=123.0)
    ap.add_argument('--y', type=float, default=456.0)
    ap.add_argument('--hz', type=float, default=1.0)
    ap.add_argument('--seconds', type=float, default=1800.0)
    ap.add_argument('--topic', default='/goal_pose')
    a = ap.parse_args()

    rclpy.init()
    n = Node('interference_goal44')
    pub = n.create_publisher(PoseStamped, a.topic, 10)

    import os
    dom = os.environ.get('ROS_DOMAIN_ID', '（未设）')
    print('★ 干扰源起：ROS_DOMAIN_ID=%s ｜ topic=%s ｜ 异目标=(%.1f, %.1f) ｜ %.2f Hz ｜ %.0f s'
          % (dom, a.topic, a.x, a.y, a.hz, a.seconds), flush=True)
    if dom != '44':
        print('★★ 警告：本进程的域**不是 44** ✗ ⇒ ★ 它不构成"域 44 的干扰" ✗，'
              '本轮的隔离判据**不成立** ✗（★ 别把它当干扰源 ✓）', flush=True)

    m = PoseStamped()
    m.header.frame_id = 'map'
    m.pose.position.x = a.x
    m.pose.position.y = a.y
    m.pose.orientation.w = 1.0
    t0 = time.time()
    k = 0
    while rclpy.ok() and time.time() - t0 < a.seconds:
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        k += 1
        if k % 30 == 0:
            print('  ★ 已发 %d 条（域 %s ✓）' % (k, dom), flush=True)
        time.sleep(1.0 / max(1e-3, a.hz))
    print('★ 干扰源结束：共发 %d 条' % k, flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

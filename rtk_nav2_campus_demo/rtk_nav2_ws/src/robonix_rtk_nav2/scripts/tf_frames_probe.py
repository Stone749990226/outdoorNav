#!/usr/bin/env python3
"""统计 /tf 与 /tf_static 上实际出现的坐标变换及其频率（来源审计用）。

ROS 2 的 TF 消息不带发布者身份，所以取证的思路是：
  1. 用本脚本列出运行中真实存在的变换与频率；
  2. 再结合 `ros2 node info <节点>` 的 Publishers，确认谁在发 /tf；
  3. 必要时做注入式反证（例如杀掉里程计，看 odom→base 是否仍然存在）。

本演示期望的变换链：map→odom（全局 EKF）、odom→base_footprint（局部 EKF）、
base_footprint→各传感器（robot_state_publisher）。**不应存在**来自真值的变换。

用法：tf_frames_probe.py [采样秒数，默认 10]
"""

import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Node('tf_frames_probe')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

    counts = defaultdict(int)
    statics = set()
    node.create_subscription(TFMessage, '/tf',
                             lambda m: [counts.__setitem__((t.header.frame_id, t.child_frame_id),
                                                           counts[(t.header.frame_id,
                                                                   t.child_frame_id)] + 1)
                                        for t in m.transforms], 100)
    node.create_subscription(TFMessage, '/tf_static',
                             lambda m: [statics.add((t.header.frame_id, t.child_frame_id))
                                        for t in m.transforms], 100)

    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.2)
    elapsed = time.time() - t0

    print('采样 %.1f 秒（use_sim_time=true）' % elapsed)
    print('/tf 上的动态变换（按频率降序）：')
    if counts:
        for (parent, child), n in sorted(counts.items(), key=lambda kv: -kv[1]):
            print('  %-22s -> %-22s %7.2f Hz  (%d 条)' % (parent, child, n / elapsed, n))
    else:
        print('  （没有收到动态变换）')
    print('/tf_static 上的静态变换：')
    if statics:
        for parent, child in sorted(statics):
            print('  %-22s -> %s' % (parent, child))
    else:
        print('  （没有收到静态变换）')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

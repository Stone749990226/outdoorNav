#!/usr/bin/env python3
"""量"轮式里程计劣化确实生效"（判据 6 里最容易被做空的一条）。

第二阶段的教训：`wheel_odom_bridge` 的参数回调返回类型写错，**一设参数节点就崩**，
于是"基准跑"和"劣化跑"其实都是"没有轮式里程计"的同一个配置，
而"行为不变"这个结论什么都证明不了。参数设进去还不够——要看到**行为上真的偏了**。

这里同时收 `/wheel_odom_used`（劣化后的里程计）与 `/ground_truth/odom`（真值），
按时间配对后给两者的位置差。判据是"偏差显著大于 0"（第二阶段实测约 8 m）。

用法：odom_deviation_probe.py [--duration 20] [--json 结果.json]
"""

import argparse
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Probe(Node):

    def __init__(self):
        super().__init__('odom_deviation_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.used = []
        self.gt = []
        self.create_subscription(Odometry, '/wheel_odom_used', self._on_used, 20)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_gt, 20)

    def _on_used(self, msg):
        p = msg.pose.pose.position
        self.used.append((self.get_clock().now().nanoseconds * 1e-9, p.x, p.y))

    def _on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt.append((self.get_clock().now().nanoseconds * 1e-9, p.x, p.y))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--json', default='')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Probe()
    end = time.time() + args.duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    # 同步时间基准：两条消息序列的空翻不同（`/wheel_odom_used` 从 0 开始累积），
    # 所以按"各自的起点"对齐，只比较**相对位移**的差。
    out = {'samples_used': len(node.used), 'samples_gt': len(node.gt)}
    if len(node.used) >= 10 and len(node.gt) >= 10:
        u0 = node.used[0]
        g0 = node.gt[0]
        # 取两条序列的最后一个点，算"相对各自起点走过的位移"之差
        du = (node.used[-1][1] - u0[1], node.used[-1][2] - u0[2])
        dg = (node.gt[-1][1] - g0[1], node.gt[-1][2] - g0[2])
        div = math.hypot(du[0] - dg[0], du[1] - dg[1])
        out.update({
            'used_displacement_m': round(math.hypot(*du), 4),
            'truth_displacement_m': round(math.hypot(*dg), 4),
            'divergence_m': round(div, 4),
            'note': '两者都是"相对各自序列起点走过的位移"，divergence 就是劣化后的里程计'
                    '与真值的偏离量。判据：这个值**显著大于 0** 才算劣化真的生效了'
                    '（第二阶段实测约 8 m）',
        })
    else:
        out['error'] = '样本不足（/wheel_odom_used 或 /ground_truth/odom 没在发？）'
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if 'divergence_m' in out else 1


if __name__ == '__main__':
    sys.exit(main())

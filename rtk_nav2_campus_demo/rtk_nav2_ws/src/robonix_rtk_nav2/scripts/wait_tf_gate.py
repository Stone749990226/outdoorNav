#!/usr/bin/env python3
"""TF 闸门：等到指定的变换真的可查才退出 0。launch 用它决定 nav2 起不起。

**为什么需要它**（第三阶段的启动顺序坑）：`Costmap2DROS::on_activate()` 里有一个
2 Hz 的死等循环（等 global_frame ↔ robot_base_frame 的 TF 能变换），TF 没就绪它就
**一直不返回**；而 nav2 的 `lifecycle_manager` 调 change_state 是**带超时**的，
超时后 `autostart` 直接失败，全局图就永远是死的——现象是"栈起来了、地图话题却一直没有"。

靠"睡几秒"过这一关是脆的（定位链就绪时间随机：EKF 要等 IMU 和里程计、
navsat_transform 要等第一帧 GPS）。所以这里用一个真的去查 TF 的进程当闸门：
launch 里 `OnProcessExit` 挂在它身上，它退出了才起 nav2 那三个节点。

用法：wait_tf_gate.py --target map --source base_footprint [--timeout 180]
退出码：0 = 变换可查；1 = 超时
"""

import argparse
import sys
import time

import rclpy
import tf2_ros
from rclpy.node import Node


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', required=True, help='父帧（如 map）')
    parser.add_argument('--source', required=True, help='子帧（如 base_footprint）')
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('rtk_nav2_tf_gate')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    t0 = time.time()
    ok = False
    while time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf_buffer.lookup_transform(args.target, args.source, rclpy.time.Time())
            ok = True
            break
        except Exception:
            time.sleep(0.3)

    if ok:
        print('TF 闸门通过：%s → %s 可查（%.1f s），放行 nav2'
              % (args.target, args.source, time.time() - t0))
    else:
        # **报错要分类**（reviewer 要求）：闸门超时有两种完全不同的原因，
        # 混成一句话就会像 nav4-2 那次一样——"Gazebo 没起来"被下游伪装成"nav2 卡住"，
        # 查错方向。这里按 TF 里实际有哪些帧分开报。
        frames = ''
        try:
            frames = tf_buffer.all_frames_as_string()
        except Exception:
            pass
        have_source = args.source in frames
        have_target = args.target in frames
        have_odom = 'odom' in frames
        if not have_source and not have_odom:
            why = ('**仿真/机器人上游没就绪**：TF 里连 `%s` / `odom` 都没有'
                   '（Gazebo 世界服务不可达、车没 spawn、或 /clock 没来）'
                   % args.source)
        elif have_source and not have_target:
            why = ('**定位链没就绪**：`%s` 在，但 `%s` 不在'
                   '（absolute_localizer 没发出 map→odom）' % (args.source, args.target))
        else:
            why = '**nav2/TF 自身超时**：帧都在但变换一直查不到'
        print('** TF 闸门超时 **：%.0f s 内 %s → %s 一直查不到。分类：%s'
              % (args.timeout, args.target, args.source, why), file=sys.stderr)
        print('  当前 TF 里的帧：%s'
              % (', '.join(sorted(set(frames.split()))) if frames else '（读不到）'),
              file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())

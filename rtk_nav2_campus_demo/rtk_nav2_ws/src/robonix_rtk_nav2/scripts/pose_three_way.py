#!/usr/bin/env python3
"""★ **静止态三方对照**（reviewer 2026-10-09 指定 ✓，④ 的开门件 ✓）。

**为什么** ✗：`ramp7-bf` 里车**全轮最近到过 0.0522 m**，可**当时真值速度 0.0547 m/s** ⇒ **跑着擦身而过** ✗，
`d≤0.25 m` 的 12 秒里**全程在动** ⇒ **"连续 3 拍停稳"在 0.25 m 内一次都没有** ✗。
**失败形态 = "路过却没停住"** ✓，不是"停在 0.39 m 处没到" ✓。
⇒ 要分清 **nav2 眼里的位姿**与**真值**的关系：

· **常数偏移** ⇒ **帧/外参问题** ✓（`map` 与真值世界系有固定错位 ✓）；
· **随航程增长** ⇒ **里程计漂移** ✓。

**怎么分** ✓：在**多个位置、每次都在静止态**各取一次三方 ——
· **真值** = `ign model -m robot_rtk --pose`（由外层 shell 取 ✓，本脚本取不到 Gazebo ✓）；
· **导航估计** = `/odometry/global`；
· **TF** = `map → base_footprint`（nav2 的 `getRobotPose` 走的就是它 ✓ ⇒ **判据实际用的是这一路** ✗）。
★ **三个都印** ✓ —— 只比两个的话，"估计漂了"和"TF 配错了"分不开 ✓。

用法：`pose_three_way.py --json out.json [--label S1]`（**需要 ROS** ✓；真值由外层补 ✓）
"""
import argparse
import json
import math
import time

import rclpy
import tf2_ros
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

Q = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE,
               reliability=ReliabilityPolicy.RELIABLE)


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--label', default='')
    ap.add_argument('--seconds', type=float, default=20.0)
    a = ap.parse_args()

    rclpy.init()
    n = Node('pose_three_way', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    buf = tf2_ros.Buffer()
    lis = tf2_ros.TransformListener(buf, n)
    od = {}
    n.create_subscription(Odometry, '/odometry/global', lambda m: od.__setitem__('m', m), Q)

    t0 = time.time()
    while time.time() - t0 < a.seconds and ('m' not in od):
        rclpy.spin_once(n, timeout_sec=0.1)

    out = {'label': a.label, 'ts': time.time()}
    if 'm' in od:
        p = od['m'].pose.pose
        out['odom_global'] = {'x': round(p.position.x, 4), 'y': round(p.position.y, 4),
                              'yaw': round(yaw_of(p.orientation), 4)}
    else:
        out['odom_global'] = None

    # ★ TF：等它可查 ✓（刚起栈时 `map→base_footprint` 可能还没接通 ✓）
    tf = None
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        try:
            tr = buf.lookup_transform('map', 'base_footprint', Time())
            tf = {'x': round(tr.transform.translation.x, 4),
                  'y': round(tr.transform.translation.y, 4),
                  'yaw': round(yaw_of(tr.transform.rotation), 4)}
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.1)
    out['tf_map_base_footprint'] = tf

    n.destroy_node()
    rclpy.shutdown()

    json.dump(out, open(a.json, 'w'), indent=1, ensure_ascii=False)
    print('  [%s] odom/global=%s' % (a.label, out['odom_global']))
    print('  [%s] tf map->base_footprint=%s' % (a.label, tf))
    return 0 if (out['odom_global'] and tf) else 4


if __name__ == '__main__':
    raise SystemExit(main())

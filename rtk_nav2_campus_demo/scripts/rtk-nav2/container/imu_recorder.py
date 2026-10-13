#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_recorder.py —— ★ 逐帧记 IMU 的 pitch/roll（sensor_msgs/Imu ✓，四元数→RPY ✓）

★ 为什么：★ reviewer 2026-10-12 指出的机理验算要 **逐帧配姿态** ✓：
  ★ `r_后 · sin(pitch) ≈ 0.100` ✓ ∧ ★ `r_左 · sin(roll) ≈ 0.100` ✓（0.100 = 激光安装高 ✓）
★ 时间基沿用消息头的 stamp（与点云记录器同一套 ✓ ⇒ 可按 `t_sim` 对齐 ✓）。
★ 纯 struct 之外用 math ✓（✗ 不引 numpy ✗）；四元数→RPY 用标准公式（航空序 ZYX ✓）。
"""
import argparse, json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def q2rpy(x, y, z, w):
    sinr = 2.0 * (w * x + y * z); cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x); sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y); cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class Rec(Node):
    def __init__(self, a):
        super().__init__('imu_recorder')
        self.fh = open(a.out, 'w', encoding='utf-8')
        self.fh.write(json.dumps({'_header': 'imu_recorder/v1', 'topic': a.topic,
                                  'run_id': a.run_id, 'mode': a.mode,
                                  'fields': 't_sim,roll_deg,pitch_deg,yaw_deg',
                                  'started_wall': time.strftime('%F %T')}, ensure_ascii=False) + '\n')
        self.fh.flush()
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Imu, a.topic, self.cb, qos)
        self.n = 0
        self.get_logger().info('订 %s ⇒ %s' % (a.topic, a.out))

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.orientation
        r, p, y = q2rpy(q.x, q.y, q.z, q.w)
        self.n += 1
        self.fh.write(json.dumps({'t_sim': round(t, 4), 'roll_deg': round(r, 4),
                                  'pitch_deg': round(p, 4), 'yaw_deg': round(y, 4)},
                                 ensure_ascii=False) + '\n')
        if self.n % 100 == 0:
            self.fh.flush()

    def close(self):
        self.fh.flush(); self.fh.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/rtk/imu')
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=1800.0)
    ap.add_argument('--run-id', default='(未标)')
    ap.add_argument('--mode', default='')
    a = ap.parse_args()
    rclpy.init(); node = Rec(a)
    t_end = time.time() + a.seconds
    try:
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.close(); node.destroy_node(); rclpy.shutdown()
    print('★ 累计 %d 帧 ⇒ %s' % (node.n, a.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())

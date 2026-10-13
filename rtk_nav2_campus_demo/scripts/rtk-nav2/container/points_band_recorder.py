#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""points_band_recorder.py —— ★ 在**容器里**量点云：每帧点数 ＋ 按距离/方位分带 ✓

★★ 为什么不用 bag 录点云 ✗（★ 今天已量过同一个坑 ✓）★★
  ★ `ros2 bag record` **只录"起录那一刻已存在"的话题** ✗ —— ★ 而起录发生在 **nav2 之前** ✓
  ⇒ ★ 点云那时还没起来 ⇒ **加了等于没加** ✗✗（★ 与 `/cmd_vel` 那次**逐字同一个坑** ✓）。
  ★ 改用**起栈之后自己起的小记录器** ✓（★ 与 `cmdvel_recorder.py` 同一族 ✓）：★ 订得到 ✓、★ 不影响别的轮次 ✓。
  ★★ 而且它**只写"每帧的数"** ✗ —— ★ 不写整个云 ✓ ⇒ ★ 产物**小** ✓、★ 且**直接就是判据那一格** ✓✓。

★ 量什么（★ reviewer 2026-10-12 要的那两样 ✓）：
  ★ ① **每帧点数**（总 ✓）
  ★ ② **按"离传感器原点的距离"分带** ✓ —— ★ 尤其 **≈range_max（8.00 m ✓）那一带**
    （★ `emit_far_points` 补的就是这一带 ✓）＋ ★ **按方位分带** ✓（★ **车后** ＝ |方位角| > 90° ✓✓）
★ 判据（A/B ✓）：★ 关掉两个开关 ⇒ ★ **8 m 那一带 ＋ 车后那一段显著下降** ✓✓；★ 几乎不变 ⇒ 另找来源 ✗

用法（★ 在容器里 ✓，起栈**之后**起 ✓）：
  python3 points_band_recorder.py --topic /rtk/scan_points --out <run>/points_scan_bands.jsonl [--seconds 900]
"""
import argparse
import json
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
# ★★ 世界系检验（reviewer 换的判据）：逐帧取 map 到该云 frame 的完整变换
#   理由：传感器帧带姿态（pitch/roll），只补 yaw 不够；且它不依赖任何机理假设。
import tf2_ros


def field_offsets(msg):
    off = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            off[f.name] = f.offset
    return off


def count_bands(msg, dist_edges, az_edges):
    """★ 纯 python 解 PointCloud2 ✓（用 fields 里给的 offset ✓，✗ 不猜布局 ✗）"""
    off = field_offsets(msg)
    if len(off) < 3:
        return None
    step = msg.point_step
    buf = bytes(msg.data)
    n = msg.width * msg.height
    dbands = [0] * (len(dist_edges) + 1)
    abands = [0] * (len(az_edges) + 1)
    finite = 0
    near8 = 0
    rmax = None
    for i in range(n):
        base = i * step
        try:
            x, = struct.unpack_from('<f', buf, base + off['x'])
            y, = struct.unpack_from('<f', buf, base + off['y'])
            z, = struct.unpack_from('<f', buf, base + off['z'])
        except struct.error:
            break
        if x != x or y != y or z != z:            # NaN ⇒ 不算有限点 ✓
            continue
        finite += 1
        d = math.sqrt(x * x + y * y + z * z)
        rmax = d if rmax is None or d > rmax else rmax
        for k in range(len(dist_edges)):
            if d < dist_edges[k]:
                dbands[k] += 1
                break
        else:
            dbands[-1] += 1
        az = math.degrees(math.atan2(y, x))
        for k in range(len(az_edges)):
            if az < az_edges[k]:
                abands[k] += 1
                break
        else:
            abands[-1] += 1
    return {'n_total': n, 'n_finite': finite, 'dist_bands': dbands,
            'az_bands': abands, 'd_max': rmax}


def dump_points(msg, ndigits=3):
    """★ 逐点 (x,y,z) ✓（只用消息自己 fields 的 offset ✓）⇒ 离线套标图规则用 ✓"""
    off = field_offsets(msg)
    if len(off) < 3:
        return []
    step, buf = msg.point_step, bytes(msg.data)
    out = []
    for i in range(msg.width * msg.height):
        base = i * step
        try:
            x, = struct.unpack_from('<f', buf, base + off['x'])
            y, = struct.unpack_from('<f', buf, base + off['y'])
            z, = struct.unpack_from('<f', buf, base + off['z'])
        except struct.error:
            break
        if x != x or y != y or z != z:
            continue
        out.append([round(x, ndigits), round(y, ndigits), round(z, ndigits)])
    return out


class Rec(Node):
    def __init__(self, a):
        super().__init__('points_band_recorder')
        self.a = a
        self.fh = open(a.out, 'w', encoding='utf-8')
        # ★★ 首行写清"这是哪一轮／什么模式" ✓（reviewer 2026-10-12 ✓）★
        #   ★ 为什么：★ 两端的文件长得一样 ✗ ⇒ ★ 首行带 run-id 与 mode ⇒ **不可能被弄混** ✓
        #   （★ 与"产物要能追溯到哪一轮"同族 ✓）
        self.fh.write(json.dumps({
            '_header': 'points_band_recorder/v1', 'topic': a.topic,
            'run_id': a.run_id, 'mode': a.mode,
            'dist_edges': a.dist_edges, 'az_edges': a.az_edges,
            'started_wall': time.strftime('%F %T'), 'note': a.note or ''},
            ensure_ascii=False) + '\n')
        self.fh.flush()
        self.pfh = open(a.dump_points, 'w', encoding='utf-8') if a.dump_points else None
        if self.pfh:
            self.pfh.write(json.dumps({'_header': 'points_xyz/v1', 'topic': a.topic,
                'run_id': a.run_id, 'mode': a.mode, 'dump_every': a.dump_every,
                'frame': 'sensor（该云自己的 frame_id ✓）', 'started_wall': time.strftime('%F %T')},
                ensure_ascii=False) + '\n')
            self.pfh.flush()
        self.tfh = (open(a.out + '.tf.jsonl', 'w', encoding='utf-8')
                    if a.tf_target else None)
        self.tferr = 0
        self.n = 0
        self.t0 = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, a.topic, self.cb, qos)
        self.tfbuf = None
        if a.tf_target:
            self.tfbuf = tf2_ros.Buffer()
            self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
            self.get_logger().info('world-frame check: %s -> %s' % (a.tf_target, a.topic))
        self.get_logger().info('订 %s ⇒ %s' % (a.topic, a.out))

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_sim_now = t
        b = count_bands(msg, self.a.dist_edges, self.a.az_edges)
        if b is None:
            return
        if self.t0 is None:
            self.t0 = t
        self.n += 1
        rec = {'t_sim': round(t, 3), 'topic': self.a.topic, 'frame_id': msg.header.frame_id,
               'n_total': b['n_total'], 'n_finite': b['n_finite'],
               'd_max': (round(b['d_max'], 3) if b['d_max'] is not None else None),
               'dist_bands': b['dist_bands'], 'az_bands': b['az_bands']}
        self.fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
        if self.tfbuf is not None:
            try:
                tr = self.tfbuf.lookup_transform(self.a.tf_target, msg.header.frame_id,
                                                 rclpy.time.Time.from_msg(msg.header.stamp),
                                                 rclpy.duration.Duration(seconds=0.2))
                q = tr.transform.rotation
                t = tr.transform.translation
                self.tfh.write(json.dumps({'t_sim': round(t_sim_now, 3),
                                           'frame_id': msg.header.frame_id,
                                           'target': self.a.tf_target,
                                           't': [t.x, t.y, t.z],
                                           'q': [q.x, q.y, q.z, q.w]}) + '\n')
                self.tfh.flush()
            except Exception as e:
                self.tferr += 1
                if self.tferr <= 3:
                    self.get_logger().warn('tf miss: %s' % e)
        if self.pfh is not None and (self.n % self.a.dump_every == 0):
            pts = dump_points(msg)
            if pts:
                # ★★ 2026-10-12 修：这里原来写的是 `round(t, 3)` ✗ —— 而上面 TF 那段
                #   把 `t` **覆盖**成了 translation（Vector3）✓ ⇒ 开了 `--tf-target` 时
                #   **每帧都抛 `TypeError: type Vector3 doesn't define __round__ method`** ✗
                #   ⇒ 记录器**静默死掉**（`docker exec -d` 起的，外面看不见 ✗）。
                #   实测代价：final-align 那轮的 scan 记录器 **13:55:40 起、14:03:34 就死了** ✗
                #   （车 13:58:50 才开跑 ⇒ 只录到起步后约 5 分钟 ✗）。
                #   ★ 改用 `t_sim_now`（第 147 行赋值、不会被覆盖 ✓）。★ 下面那行同一个坑 ✓。
                self.pfh.write(json.dumps({'t_sim': round(t_sim_now, 3), 'frame_id': msg.header.frame_id,
                                           'n': len(pts), 'xyz': pts}, ensure_ascii=False) + '\n')
                self.pfh.flush()
        if self.n % 20 == 0:
            self.fh.flush()
            self.get_logger().info('已记 %d 帧（t=%.1f s）' % (self.n, t_sim_now - self.t0))

    def close(self):
        self.fh.flush()
        self.fh.close()
        if self.pfh is not None:
            self.pfh.flush()
            self.pfh.close()
        if self.tfh is not None:
            self.tfh.flush()
            self.tfh.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/rtk/scan_points')
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=900.0)
    ap.add_argument('--run-id', default='(未标)', help='★ 哪一轮 ⇒ 写进首行 ✓')
    ap.add_argument('--mode', default='', help='★ 例 render_fix=off|emit_far_points=false|scan_emit_far_points=false ✓')
    ap.add_argument('--note', default='')
    # ★★ 逐点坐标（reviewer 2026-10-12 要 ✓）：只为**离线套标图规则**用 ✓
    #   ★ 全量太大 ✗ ⇒ ★ 抽帧（默认每 5 帧取一整帧 ✓）⇒ ★ 空间覆盖够（车走得慢 ✓）、体积可控 ✓
    ap.add_argument('--dump-points', default='', help='逐点坐标输出路径（空=不写 ✓）')
    ap.add_argument('--dump-every', type=int, default=5, help='每 N 帧取一整帧 ✓')
    ap.add_argument('--tf-target', default='', help='给了就逐帧写 target 到该云 frame 的变换')
    ap.add_argument('--dist-edges', default='0.5,1,2,4,7.9,8.01')
    ap.add_argument('--az-edges', default='-135,-90,-45,0,45,90,135')
    a = ap.parse_args()
    a.dist_edges = [float(v) for v in a.dist_edges.split(',')]
    a.az_edges = [float(v) for v in a.az_edges.split(',')]

    rclpy.init()
    node = Rec(a)
    t_end = time.time() + a.seconds
    try:
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    print('★ 累计 %d 帧 ⇒ %s' % (node.n, a.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())

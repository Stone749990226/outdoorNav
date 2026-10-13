#!/usr/bin/env python3
"""局部图"原地转一圈会不会拉出闭环"的取证采样器（2026-09-22）。

为什么单独一个工具：AuYang 定的判据是"**原地转一圈 ⇒ 局部图上不再出现那个圈**"，
而那个圈**是个形状**——"占据格总数"只能当辅证（转一圈本来就会看到新方向上的真障碍）。
所以这里同时落三样东西，让"看得见画面的人"能直接判，也能让机器复核：

  ① **每样本一条 JSON**：两张图的占据/空闲计数、**局部图自身原点的世界坐标**、
     **机器人在 odom 与真值里的位置与朝向**（同一时刻）——**为了能回答"改过 ≠ 生效"**：
     `global_frame: base_link` 时局部图原点是**结构性常量 (0,0)**，换到 `odom` 后
     它必须**跟着车走**（`origin ≈ pos − 图宽/2`）。这一条是"改动真的生效了"的直接证据 ✗ 不靠推断。
  ② **首/末帧 npz**（与 `nav2_probe.py localmap --snapshot` 同格式，
     可直接喂 `diff_snapshots.py`）⇒ 能逐格比"转之前 / 转之后"。
  ③ **末帧 PNG**（纯 stdlib zlib 手写的灰度 PNG，容器里不需要 PIL）⇒ 给人看形状。

★ 判据怎么读（**只看这两条，别只看计数**）：
  · **圈在不在**：PNG 里占据格是否连成一圈**环绕车体中心**、且半径处**没有真实障碍**；
  · **位置对不对**：占据格是否落在**这个世界里真实障碍**的方位上（对照真值位姿看）。
  计数只用来判"有没有大量新增"，**不能单独判圈**（转一圈看到新方向上的真障碍也会增加）。

用法（容器内）：
  ring_probe.py --run-dir <R> --phase <名字> --seconds 12 [--rate-hz 2] [--spin-rate 0.0]
    `--spin-rate <rad/s>` > 0 ⇒ 本阶段**自己发 `/cmd_vel`** 让车原地转（linear 恒 0），
      阶段结束时再发几次**零速**把车停住。**为什么放在这里而不是 shell 里**：
      `ros2 topic pub` 的 yaml 花括号要穿过两层 `bash -lc` 引号，本工程已经在这上面翻过车；
      写在 Python 里还顺带能**自己核**"到底转了没有"（真值朝向跨度写进 JSON）。
      **注意**：`--spin-rate` 只在**没有目标在跑**的时候用（nav2 控制器也在发 `/cmd_vel`，
      两边一起发就是"两个写入者"，转速会变成谁后发谁算）。
退出码：0 正常；1 没收到局部图（**如实失败，不当通过**）。
"""

import argparse
import json
import math
import os
import struct
import sys
import time
import zlib

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

LOCAL_TOPIC = '/local_costmap/costmap'
GLOBAL_TOPIC = '/global_costmap/costmap'
GT_TOPIC = '/ground_truth/odom'

# 与 nav2_probe.py 同一份：nav2 的代价地图发布者是 transient_local
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
# **同时**订一份 volatile 的：若某天发布端换成 volatile，前者会一条都收不到，
# 而后者能收 ⇒ **不许"收不到"被读成"图没在发"**（本项目的 QoS 老账）。
VOLATILE = QoSProfile(depth=1,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST,
                      durability=QoSDurabilityPolicy.VOLATILE)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Sampler(Node):
    def __init__(self):
        super().__init__('ring_probe')
        self.latest = {}
        self.seen = {}
        self.counts = {}
        for topic, key, qos in ((LOCAL_TOPIC, 'local', LATCHED), (GLOBAL_TOPIC, 'global', LATCHED),
                                (LOCAL_TOPIC, 'local_vol', VOLATILE),
                                (GLOBAL_TOPIC, 'global_vol', VOLATILE)):
            self.create_subscription(OccupancyGrid, topic,
                                     lambda m, k=key: self._on_map(m, k), qos)
        for topic, key, qos in ((GT_TOPIC, 'gt', VOLATILE),):
            self.create_subscription(Odometry, topic,
                                     lambda m, k=key: self._on_odom(m, k), qos)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)

    def _on_map(self, msg, key):
        self.latest[key] = msg
        self.seen[key] = self.seen.get(key, 0) + 1

    def _on_odom(self, msg, key):
        self.latest[key] = msg
        self.seen[key] = self.seen.get(key, 0) + 1

    def tf_pose(self, target, source):
        """返回 (x, y, yaw) 或 None；查不到就 None（**不许用 0 冒充**）。"""
        try:
            t = self.tf.lookup_transform(target, source, Time())
        except Exception:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, yaw_of(t.transform.rotation))


def grid_array(msg):
    return np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


def counts_of(data):
    """三态计数。**注意 `track_unknown_space: false` 的图里 -1 会很少** ——
    所以这里的 free 里可能混着"其实是未知但被当成空闲"的格（本项目已记录过）。"""
    return {'free': int(np.count_nonzero(data == 0)),
            'occupied': int(np.count_nonzero(data > 0)),
            'lethal': int(np.count_nonzero(data >= 99)),
            'unknown': int(np.count_nonzero(data < 0))}


def write_png(path, data):
    """纯 stdlib 写灰度 PNG（8bit、无滤波、无隔行）。**容器里没有 PIL 也能出图。**"""
    h, w = data.shape
    img = np.empty((h, w), dtype=np.uint8)
    img[data < 0] = 128                    # 未知 = 中灰
    occ = data >= 99
    img[occ] = 0                           # 致命 = 黑
    mid = (data > 0) & (data < 99)         # 膨胀区 = 深灰
    img[mid] = 60
    img[data == 0] = 255                   # 空闲 = 白
    raw = b''.join(b'\x00' + img[r].tobytes() for r in range(h))

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload
                + struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 9))
           + chunk(b'IEND', b''))
    with open(path, 'wb') as fh:
        fh.write(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--phase', required=True)
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--rate-hz', type=float, default=2.0)
    ap.add_argument('--spin-rate', type=float, default=0.0,
                    help='>0 ⇒ 本阶段自己发 /cmd_vel 原地转（rad/s）；0 ⇒ 只采样')
    a = ap.parse_args()

    rclpy.init()
    node = Sampler()
    cmd_pub = None
    if a.spin_rate > 0:
        cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    t_end = time.time() + a.seconds
    period = 1.0 / a.rate_hz
    n_cmd = 0
    samples = []
    first_npz = last_npz = None
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=min(period, 0.4))
        if cmd_pub is not None:
            # 每轮发两条：单发一条很容易被底盘的控制周期漏掉（本工程实测过"发了没动"）
            for _ in range(2):
                tw = Twist()
                tw.angular.z = float(a.spin_rate)
                cmd_pub.publish(tw)
                n_cmd += 1
            time.sleep(0.05)
        if 'local' not in node.latest:
            continue
        now = time.time()
        lm = node.latest['local']
        row = {'wall': round(now, 3),
               't_rel': None,
               'sim': round(lm.header.stamp.sec + lm.header.stamp.nanosec * 1e-9, 3),
               'local_frame': lm.header.frame_id,
               'local': counts_of(grid_array(lm)),
               'local_origin': [round(lm.info.origin.position.x, 4),
                                round(lm.info.origin.position.y, 4)],
               'local_size': [lm.info.width, lm.info.height],
               'local_res': round(lm.info.resolution, 4),
               'seen': dict(node.seen)}
        gm = node.latest.get('global')
        if gm is not None:
            row['global_frame'] = gm.header.frame_id
            row['global'] = counts_of(grid_array(gm))
        p_odom = node.tf_pose('odom', 'base_footprint')
        row['odom_pose'] = ([round(v, 4) for v in p_odom] if p_odom else None)
        g = node.latest.get('gt')
        if g is not None:
            p = g.pose.pose.position
            row['gt_pose'] = [round(p.x, 4), round(p.y, 4), round(yaw_of(g.pose.pose.orientation), 4)]
        samples.append(row)

        if first_npz is None:
            first_npz = os.path.join(a.run_dir, 'ring_%s_first.npz' % a.phase)
            np.savez_compressed(first_npz, data=grid_array(lm),
                                resolution=lm.info.resolution,
                                origin_x=lm.info.origin.position.x,
                                origin_y=lm.info.origin.position.y,
                                frame=lm.header.frame_id)
        # 末帧每轮都覆盖 ⇒ 循环结束时它就是最后一帧
        last_npz = os.path.join(a.run_dir, 'ring_%s_last.npz' % a.phase)
        np.savez_compressed(last_npz, data=grid_array(lm),
                            resolution=lm.info.resolution,
                            origin_x=lm.info.origin.position.x,
                            origin_y=lm.info.origin.position.y,
                            frame=lm.header.frame_id)
        write_png(os.path.join(a.run_dir, 'ring_%s_last.png' % a.phase), grid_array(lm))

    if not samples:
        print('** 一帧局部图都没收到 ⇒ 如实失败（不许当通过）**', file=sys.stderr)
        print('   已订 topic：%s（transient_local 与 volatile 各一份）' % LOCAL_TOPIC, file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if cmd_pub is not None:
        # 阶段结束**必须把车停住**：不停的话"转完之后"的采样期车还在转，
        # 那段读数就不叫"静止基线"了（判据 8 的读数必须能说清车当时在不在动）
        for _ in range(10):
            cmd_pub.publish(Twist())
            n_cmd += 1
            time.sleep(0.05)

    if samples:
        t0 = samples[0]['wall']
        for r in samples:
            r['t_rel'] = round(r['wall'] - t0, 3)

    def span(key, sub):
        vals = [r[key] for r in samples if r.get(key) is not None]
        if not vals:
            return None
        return [sub(x) for x in vals]

    # ★ 朝向**必须解缠**：转一圈的 yaw 差在 ±π 处会回绕，
    #   直接 `末−首` 会得到"转了 0.1 rad"这种**看起来没转**的数（本项目的老账：角度差要解缠）
    def unwrap_total(vals):
        if not vals or len(vals) < 2:
            return None
        tot = 0.0
        for p, q in zip(vals, vals[1:]):
            d = q - p
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            tot += d
        return round(tot, 3)

    gt_yaws = [r['gt_pose'][2] for r in samples if r.get('gt_pose')]
    odom_yaws = [r['odom_pose'][2] for r in samples if r.get('odom_pose')]

    out = {
        'produced_by': 'ring_probe.py',
        'produced_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'via': 'rclpy 订阅 %s / %s（transient_local 与 volatile 各订一份）+ tf2 查 odom→base_footprint'
               ' + /ground_truth/odom' % (LOCAL_TOPIC, GLOBAL_TOPIC),
        'phase': a.phase,
        'run_dir': a.run_dir,
        'seconds_requested': a.seconds,
        'samples_n': len(samples),
        'local_frame_seen': sorted({r['local_frame'] for r in samples}),
        'local_origin_span': {'x': span('local_origin', lambda v: v[0]),
                              'y': span('local_origin', lambda v: v[1])},
        'local_occupied': span('local', lambda v: v['occupied']),
        'global_occupied': span('global', lambda v: v['occupied']),
        'odom_pose_span': {'x': span('odom_pose', lambda v: v[0]),
                           'y': span('odom_pose', lambda v: v[1]),
                           'yaw': span('odom_pose', lambda v: v[2])},
        'gt_yaw_span': span('gt_pose', lambda v: v[2]),
        'spin_cmd_rate_rad_s': a.spin_rate,
        'spin_cmd_messages': n_cmd,
        'spin_cmd_total_rad': round(a.spin_rate * a.seconds, 3),
        'gt_yaw_unwrapped_total_rad': unwrap_total(gt_yaws),
        'odom_yaw_unwrapped_total_rad': unwrap_total(odom_yaws),
        'samples': samples,
        'note': '判据读法见本文件 docstring：**圈要看 PNG 的形状**，计数只判"有没有大量新增"；'
                'local_origin 是否跟着车走 = "改到 odom 真的生效了"；'
                '**朝向一律看解缠后的总转角**（yaw 在 ±π 回绕，直接相减会把整圈读成没转）。',
    }
    js = os.path.join(a.run_dir, 'ring_%s.json' % a.phase)
    with open(js, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, ensure_ascii=False, indent=1)

    def rng(name, vals):
        if not vals:
            return '%s=（无）' % name
        return '%s=%.3f→%.3f（Δ%.3f）' % (name, vals[0], vals[-1], vals[-1] - vals[0])

    print('=== ring_probe 阶段 %s：%d 个样本 / %.0f s ===' % (a.phase, len(samples), a.seconds))
    print('  局部图 frame：%s' % '、'.join(out['local_frame_seen']))
    print('  局部图原点 %s' % rng('x', out['local_origin_span']['x']))
    print('              %s' % rng('y', out['local_origin_span']['y']))
    print('  局部图占据格：%s → %s' % (out['local_occupied'][0], out['local_occupied'][-1]))
    if out['global_occupied']:
        print('  全局图占据格：%s → %s' % (out['global_occupied'][0], out['global_occupied'][-1]))
    print('  odom 位姿 x %s' % rng('', out['odom_pose_span']['x']))
    print('  odom 位姿 y %s' % rng('', out['odom_pose_span']['y']))
    print('  odom 朝向 %s' % rng('', out['odom_pose_span']['yaw']))
    print('  **真值朝向 %s（判"真转了一圈没有"用这个）**' % rng('', out['gt_yaw_span']))
    print('  **解缠后的总转角：真值 %s rad ；odom %s rad**（本阶段发的转速指令 %s rad/s × %.0f s = %s rad，'
          '共 %d 条 /cmd_vel）'
          % (out['gt_yaw_unwrapped_total_rad'], out['odom_yaw_unwrapped_total_rad'],
             a.spin_rate, a.seconds, out['spin_cmd_total_rad'], n_cmd))
    print('  产物：%s ；%s ；%s'
          % (os.path.basename(js), os.path.basename(first_npz or '(无)'),
             'ring_%s_last.png' % a.phase))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

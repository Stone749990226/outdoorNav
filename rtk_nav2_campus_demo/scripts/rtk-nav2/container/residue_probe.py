#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""residue_probe.py —— ★ 全局图"残留障碍点"取证探针（阶段十第 1 步 ✓）

用法（**容器内、栈运行中**）:
    source /opt/ros/humble/setup.bash; source <ws>/install/setup.bash
    python3 residue_probe.py --out <run>/probe --seconds 600 [--interval 2.0]

★ 为什么要**一个节点同时采三样** ✗（而不是三次分别采 ✓）：
  ★ 结论要判"**残留格 ↔ 哪个传感器看到的**" ✓ ⇒ ★ 那三样必须是**同一时刻**的 ✓；
  ★ 分三次采 ⇒ ★ "这条点云"与"那张图"**差多少秒说不清** ✗（本项目在"读数带时刻"上栽过 ✓）。

每 tick 落 5 个文件（★ 全部带 `seq` 与**双时刻** ✓）：
  tick_<seq>.json  · 墙钟 ＋ **仿真时刻** ＋ 机器人位姿(map) ＋ **用到的 TF 矩阵与其 stamp** ＋ 点数
  map_<seq>.bin    · 全局图**原始 data**（int8，行主序 ✓）＋ map_<seq>.json 给栅格 info
  scan_<seq>.xyz   · `/rtk/scan_points` **已变换到 map 帧**（★ 变换矩阵见 tick json ✓）
  cam_<seq>.xyz    · `/rtk/camera/points` 同上

★ 采到的点云**原样变换**（✗ 不取极值、✗ 不抽稀取代表 ✓）；★ 只做**上限截断**，
  并把 `n_total` 与 `n_kept` **都写进 tick** ✓（★ 截断必须可见 ✗，不许静默 ✓）。
★ 落盘一律 **临时文件 + os.replace** ✓（✗ 不原地写 ✗ —— 分析侧可能正在读 ✓）。

退出码: 0 = 跑满 --seconds ｜ 2 = 一个图都没收到（★ 记成"未取到"，✗ 不记成"没残留" ✗）
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def q_to_R(x, y, z, w):
    """四元数 → 3×3 旋转（✗ 不用 numpy ✓：容器里未必有 ✓）。"""
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


class Probe(Node):
    def __init__(self, args):
        super().__init__('residue_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.a = args
        self.seq = 0
        self.map = None
        self.map_stamp = None
        self.map_recv = 0
        self.clouds = {}
        self.pose = None

        q_map = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, args.map_topic, self._map, q_map)
        q_cloud = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        for name, topic in (('scan', args.scan_topic), ('cam', args.cam_topic)):
            self.create_subscription(
                PointCloud2, topic, lambda m, n=name: self._cloud(n, m), q_cloud)

        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buf, self)

        os.makedirs(args.out, exist_ok=True)

    def _map(self, m):
        self.map = m
        self.map_stamp = m.header.stamp
        self.map_recv += 1

    def _cloud(self, name, m):
        self.clouds[name] = m

    def _lookup(self, target, source):
        """★ 活节点 TF 实查 ✓；★ 查不到就返回 None（✗ 不许用静态值兜底 ✗）。"""
        try:
            tf = self.buf.lookup_transform(target, source, Time())
        except Exception as e:                                  # noqa: BLE001
            return None, 'tf2 查不到 %s←%s：%s' % (target, source, str(e)[:80])
        t = tf.transform.translation
        q = tf.transform.rotation
        R = q_to_R(q.x, q.y, q.z, q.w)
        M = [[R[r][0], R[r][1], R[r][2], (t.x, t.y, t.z)[r]] for r in range(3)]
        stamp = '%d.%09d' % (tf.header.stamp.sec, tf.header.stamp.nanosec)
        return M, stamp

    @staticmethod
    def _apply(M, x, y, z):
        return (M[0][0] * x + M[0][1] * y + M[0][2] * z + M[0][3],
                M[1][0] * x + M[1][1] * y + M[1][2] * z + M[1][3],
                M[2][0] * x + M[2][1] * y + M[2][2] * z + M[2][3])

    def _write(self, path, text, mode='w'):
        tmp = path + '.tmp'
        with open(tmp, mode, encoding=None if 'b' in mode else 'utf-8') as fh:
            fh.write(text)
        os.replace(tmp, path)

    def tick(self):
        self.seq += 1
        seq = self.seq
        t = self.get_clock().now()
        rec = {'seq': seq, 'wall_epoch': time.time(),
               'sim_t': t.nanoseconds / 1e9, 'map_recv': self.map_recv}
        if self.map is None:
            rec['map'] = None
            self._write(os.path.join(self.a.out, 'tick_%04d.json' % seq),
                        json.dumps(rec, ensure_ascii=False, indent=2))
            return False

        inf = self.map.info
        stamp = self.map_stamp
        rec['map'] = {'stamp': '%d.%09d' % (stamp.sec, stamp.nanosec),
                      'resolution': inf.resolution, 'width': inf.width,
                      'height': inf.height,
                      'origin': {'x': inf.origin.position.x, 'y': inf.origin.position.y},
                      'cells_total': inf.width * inf.height}
        self._write(os.path.join(self.a.out, 'map_%04d.bin' % seq),
                    bytes((v & 0xFF) for v in self.map.data), 'wb')
        self._write(os.path.join(self.a.out, 'map_%04d.json' % seq),
                    json.dumps(rec['map'], ensure_ascii=False, indent=2))

        # ★ 机器人位姿（map 帧 ✓）—— ★ 记的是**查到的 TF 的 stamp** ✓
        M, info = self._lookup('map', self.a.base_frame)
        rec['robot_tf'] = ({'M': M, 'stamp_or_err': info} if M else {'err': info})
        if M:
            rec['robot_xy_map'] = [round(M[0][3], 4), round(M[1][3], 4)]

        for name in ('scan', 'cam'):
            msg = self.clouds.get(name)
            d = {'frame_id': None, 'stamp': None, 'n_total': 0, 'n_kept': 0,
                 'tf': None, 'file': None}
            if msg is not None:
                d['frame_id'] = msg.header.frame_id
                d['stamp'] = '%d.%09d' % (msg.header.stamp.sec, msg.header.stamp.nanosec)
                pts = []
                for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                    pts.append((float(p[0]), float(p[1]), float(p[2])))
                d['n_total'] = len(pts)
                stride = max(1, int(math.ceil(len(pts) / float(self.a.max_points))))
                kept = pts[::stride]
                d['n_kept'] = len(kept)
                d['stride'] = stride
                M2, info2 = self._lookup('map', msg.header.frame_id)
                if M2 is None:
                    d['tf'] = {'err': info2}
                else:
                    d['tf'] = {'M': M2, 'stamp_or_err': info2}
                    lines = []
                    for (x, y, z) in kept:
                        X, Y, Z = self._apply(M2, x, y, z)
                        lines.append('%.4f %.4f %.4f' % (X, Y, Z))
                    f = os.path.join(self.a.out, '%s_%04d.xyz' % (name, seq))
                    self._write(f, '\n'.join(lines) + ('\n' if lines else ''))
                    d['file'] = os.path.basename(f)
            rec[name] = d

        self._write(os.path.join(self.a.out, 'tick_%04d.json' % seq),
                    json.dumps(rec, ensure_ascii=False, indent=2))
        print('[%s] tick %d ｜ sim_t=%.1f ｜ 占用格见 map ｜ scan=%s cam=%s'
              % (time.strftime('%H:%M:%S'), seq, rec['sim_t'],
                 rec['scan']['n_kept'], rec['cam']['n_kept']), flush=True)
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=600.0)
    ap.add_argument('--interval', type=float, default=2.0)
    ap.add_argument('--map-topic', default='/global_costmap/costmap')
    ap.add_argument('--scan-topic', default='/rtk/scan_points')
    ap.add_argument('--cam-topic', default='/rtk/camera/points')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--max-points', type=int, default=60000)
    a = ap.parse_args()

    rclpy.init()
    n = Probe(a)
    t0 = time.time()
    last = 0.0
    ok_any = False
    try:
        while rclpy.ok() and time.time() - t0 < a.seconds:
            rclpy.spin_once(n, timeout_sec=0.1)
            if time.time() - last >= a.interval:
                last = time.time()
                ok_any = n.tick() or ok_any
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()
    print('★ 结束：共 %d tick，收到图 %d 次' % (n.seq, n.map_recv), flush=True)
    return 0 if (ok_any and n.map_recv > 0) else 2


if __name__ == '__main__':
    sys.exit(main())

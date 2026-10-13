#!/usr/bin/env python3
"""`rtk_depth_points` 的**离线自测**：不启 Gazebo、不依赖演示栈。

做法：本脚本假装是一台深度相机（发一帧 32FC1 深度图 + 对应的 CameraInfo），
让被测节点把点云发到 /test/points，然后**逐像素**核对：

  · 有效像素 → `(x_n·Z, y_n·Z, Z)`（沿光轴的深度，与 point_cloud_xyz 同约定）；
  · 无效像素（NaN / inf / 0 / 超远裁剪）→ **落在远裁剪面上**：`(x_n·far, y_n·far, far)`，
    这就是"无回波 ⇒ 有端点"的全部含义；
  · `emit_far_points:=false` 时 → 无效像素必须是 **NaN**（= 改前行为，A/B 的对照臂）。

为什么要单独有它：判据 ①（无回波⇒空闲）在栈上量的是**地图**的格子；
而"远点有没有生成、生成在哪"是**节点**的行为，可以在没有栈的情况下先钉死。
节点这一层错了（比如远点落到 6 m 以内），地图那一层怎么量都是错的。

用法（容器内）：
  ros2 run robonix_rtk_nav2 rtk_depth_points --ros-args -p use_sim_time:=false \
      -p input_topic:=/test/depth -p info_topic:=/test/info \
      -p output_topic:=/test/points -p emit_far_points:=true &
  python3 depth_points_selftest.py --expect-far-points true
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

W, H = 6, 4                 # 故意用非方阵：能同时暴露 u/v 写反、cx/cy 写反
FAR = 8.0
FX = FY = 277.13
CX, CY = W / 2.0, H / 2.0
# 像素图案：左半列有效（深 2.0 m），右半列四种无效各占一列
DEPTH = np.array([
    [2.0, 2.0, 2.0, np.nan, np.inf, 0.0],
    [1.5, 1.5, 1.5, np.nan, np.inf, 0.0],
    [0.5, 0.5, 0.5, np.nan, np.inf, 9.0],       # 9.0 > far → 也算无回波
    [3.0, 3.0, 3.0, np.nan, np.inf, 0.0],
], dtype=np.float32)


class Tester(Node):

    def __init__(self):
        super().__init__('depth_points_selftest')
        self.cloud = None
        self.pub_img = self.create_publisher(Image, '/test/depth', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, '/test/info', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/test/points', self._on_cloud,
                                 qos_profile_sensor_data)

    def _on_cloud(self, msg):
        self.cloud = msg

    def send(self):
        info = CameraInfo()
        info.width, info.height = W, H
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        img = Image()
        img.header.frame_id = 'test_optical_frame'
        img.height, img.width = H, W
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = W * 4
        img.data = DEPTH.tobytes()
        self.pub_info.publish(info)
        self.pub_img.publish(img)
        self.get_logger().info('已发一帧 %dx%d 深度图（左半有效 / 右半四种无效）' % (W, H))


def to_array(msg):
    off = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = [np.ascontiguousarray(buf[:, off[n]:off[n] + 4]).view(np.float32).reshape(-1)
            for n in ('x', 'y', 'z')]
    return np.stack(cols, axis=1).astype(np.float64)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--expect-far-points', default='true',
                    help='被测节点当前的 emit_far_points 取值（true|false）')
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--json', default=None)
    args = ap.parse_args(argv)
    expect_far = args.expect_far_points.lower() in ('1', 'true', 'yes')

    rclpy.init()
    node = Tester()
    t0 = time.time()
    node.send()
    while node.cloud is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.cloud is None:
        print('** 没收到 /test/points（节点没起？话题没对上？）**', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    msg = node.cloud
    pts = to_array(msg)
    fails = []
    result = {'cloud': {'width': msg.width, 'height': msg.height,
                        'frame_id': msg.header.frame_id, 'point_step': msg.point_step},
              'expect_far_points': expect_far, 'checks': [], 'failures': fails}

    if msg.width * msg.height != W * H:
        fails.append('点数 %d != %d' % (msg.width * msg.height, W * H))
    if msg.header.frame_id != 'test_optical_frame':
        fails.append('frame_id 没沿用深度图：%r' % msg.header.frame_id)

    valid_mask = np.isfinite(DEPTH) & (DEPTH > 0.0) & (DEPTH <= FAR)
    idx = 0
    for v in range(H):
        for u in range(W):
            x, y, z = pts[idx]
            idx += 1
            xn, yn = (u - CX) / FX, (v - CY) / FY
            if valid_mask[v, u]:
                ez = float(DEPTH[v, u])
                ex, ey = xn * ez, yn * ez
                tag = '有效像素(d=%.2f)' % ez
            elif expect_far:
                ez = FAR
                ex, ey = xn * FAR, yn * FAR
                tag = '无回波像素(应补远点)'
            else:
                if not (math.isnan(x) and math.isnan(y) and math.isnan(z)):
                    fails.append('(%d,%d) 无回波像素在 emit_far_points=false 下不是 NaN：%s'
                                 % (u, v, [x, y, z]))
                    continue
                result['checks'].append({'u': u, 'v': v, 'tag': '无回波像素(应为 NaN)',
                                         'point': [None, None, None], 'ok': True})
                continue
            ok = (abs(x - ex) < 1e-4) and (abs(y - ey) < 1e-4) and (abs(z - ez) < 1e-4)
            if not ok:
                fails.append('(%d,%d) %s：得到 (%.4f, %.4f, %.4f)，应为 (%.4f, %.4f, %.4f)'
                             % (u, v, tag, x, y, z, ex, ey, ez))
            result['checks'].append({'u': u, 'v': v, 'tag': tag,
                                     'point': [round(x, 4), round(y, 4), round(z, 4)],
                                     'expect': [round(ex, 4), round(ey, 4), round(ez, 4)],
                                     'ok': bool(ok)})

    far_pts = sum(1 for c in result['checks']
                  if '远点' in c['tag'] or 'NaN' in c['tag'])
    result['far_or_nan_pixels'] = far_pts
    result['ok'] = not fails
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(result, fh, ensure_ascii=False, indent=2)
    print('自测结果：%s' % ('通过（%d 个像素逐点核对）' % len(result['checks']) if result['ok']
                          else '** 不通过：%d 项 **' % len(fails)))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())

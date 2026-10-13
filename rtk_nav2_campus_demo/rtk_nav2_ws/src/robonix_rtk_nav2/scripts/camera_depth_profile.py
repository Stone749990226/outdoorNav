#!/usr/bin/env python3
"""**深度图逐行画像**：看清"没有打到东西"的像素返回什么（幽灵障碍排查用）。

**⚠️ 修正（2026-09-21）**：下面这段背景里的"场地是一块边长 10.70 m 的方板""越过板边之后"
是**当时的假设**；**方板板边不是源头**（ablation 已反证），且现在板半边长是 **11.35 m**。
本工具本身仍然有用：它量的是"没有打到东西的像素返回什么深度"——
这正是上飘像素的机制所在，只是**那条轮廓线不是板边**。

背景（2026-09-21）：去掉四周围墙后，代价地图上仍出现**成片幽灵格**
（`camera_ground_probe.py` 量到：出现在车头方向 1.5~2.7 m、离地 0.12~1.2 m、
离任何真实障碍都很远的位置）。场地是一块边长 10.70 m 的方板，**出了板就没有几何**，
而相机是从 ±2.6 m 的路线环往板外看的——所以必须先看清"越过板边之后"那些像素
返回的是什么深度（`0`？`inf`？`NaN`？还是被钳到裁剪远平面 `far=8.0`），
因为不同的返回值经 `depth_image_proc/point_cloud_xyz` 反投影后落点完全不同。

用法：
  camera_depth_profile.py [--json 结果.json] [--row-step 10]
"""

import argparse
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = '/rtk/camera/depth_image'


class Probe(Node):
    def __init__(self):
        super().__init__('camera_depth_profile')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.msg = None
        self.create_subscription(Image, TOPIC, self._cb, qos_profile_sensor_data)

    def _cb(self, m):
        if self.msg is None:
            self.msg = m


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', default=None)
    ap.add_argument('--row-step', type=int, default=10)
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Probe()
    t0 = time.time()
    while node.msg is None and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.msg is None:
        print('** %.0f 秒内没收到 %s **' % (args.timeout, TOPIC), file=sys.stderr)
        return 1

    m = node.msg
    if m.encoding == '32FC1':
        img = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
    elif m.encoding == '16UC1':
        img = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
    else:
        print('** 没见过的编码 %s **' % m.encoding, file=sys.stderr)
        return 1

    total = img.size
    nan = int(np.isnan(img).sum())
    inf = int(np.isinf(img).sum())
    zero = int((img == 0).sum())
    fin = np.isfinite(img) & (img != 0)
    vals = img[fin]
    rows = []
    for r in range(0, m.height, args.row_step):
        row = img[r]
        f = np.isfinite(row) & (row != 0)
        rows.append({
            'row': r,
            'n_finite': int(f.sum()),
            'n_nan': int(np.isnan(row).sum()),
            'n_inf': int(np.isinf(row).sum()),
            'n_zero': int((row == 0).sum()),
            'min': round(float(row[f].min()), 3) if f.any() else None,
            'median': round(float(np.median(row[f])), 3) if f.any() else None,
            'max': round(float(row[f].max()), 3) if f.any() else None,
        })

    result = {
        'topic': TOPIC,
        'encoding': m.encoding,
        'size': [m.width, m.height],
        'counts': {'total': total, 'nan': nan, 'inf': inf, 'zero': zero,
                   'finite_nonzero': int(fin.sum())},
        'finite_nonzero_stats': {
            'min': round(float(vals.min()), 3), 'median': round(float(np.median(vals)), 3),
            'max': round(float(vals.max()), 3)} if vals.size else None,
        'over_6m': int((vals > 6.0).sum()), 'over_8m': int((vals > 8.0).sum()),
        'rows': rows,
        'note': '每一行的"有限值"就是真正打到几何的像素；越靠上的行越接近地平线，'
                '越过方板边之后就打不到东西了——这里看的就是那些像素返回什么。',
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
        print('已写入 %s' % args.json, file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

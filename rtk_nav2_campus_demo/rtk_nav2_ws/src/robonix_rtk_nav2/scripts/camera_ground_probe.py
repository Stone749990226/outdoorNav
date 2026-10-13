#!/usr/bin/env python3
"""**幽灵障碍根因探针**：逐帧量相机点云的"离地高度 vs 距离"，并直接揪出幽灵点。

背景（2026-09-21）：去掉四周围墙后，代价地图上**原来那圈外墙的位置仍有致命格**
（north 73 / west 52 / south 4，共 129 个），而世界侧已确认没有墙。要回答的判别问题是
"是不是地面点被抬到了 `min_obstacle_height`(=0.12) 之上"。

两把尺子：

1. **按地面距离分段看 z 的分位数**（`bands`）。地面那一族用"每段 z 的下 40% 的中位数"
   （`floor_z_median`）代表，再做直线拟合 `z = a + b·d`：
     - `b ≈ 0` → 不是几何问题（`a` 就是"地面整体抬高"的常数偏差）；
     - `b > 0` → 地面在远处被抬起，这是"俯仰误差 / 主点 cy 偏差"的特征
       （两者在点云上不可区分：都表现为 y 方向的角度偏差；要区分得去读仿真的真实位姿）。
   注意 `frac_in_obstacle_band` 会把**墙上、柱上**的点一起算进去，所以它高不代表有幽灵——
   **唯一的幽灵判据是第 2 把尺子**。

2. **幽灵点计数**（`map_frame`）。把点云变到 **map 帧**（nav2 障碍层用的就是这条链），
   取 `0.12 <= z <= 1.2`（会被标成致命格）且 `离最近真实障碍表面 > --far_m`（默认 0.35 m，
   与 `obstacle_map_check.py` 同口径）的点——**这些点就是把幽灵墙写进代价地图的那批点**。
   每个幽灵点还带上它在**深度图里的像素坐标 (u,v)** 与**深度值 Z**，
   这样能直接对上 `camera_depth_profile.py` 的逐行画像（是"没打到东西"的行，还是真几何）。

**实现上的两个坑**（都踩过）：
  - 每帧必须**收到就立刻算**：TF 缓冲只保 ~10 s，攒够一批再回头逐帧查旧时间戳，
    每次都等到超时（120 帧 × 30 s），看起来像"卡死"。
  - `nearest_surface` 要对 ~6 万个点逐点算，纯 Python 太慢（每帧 0.6 s 起）；
    这里改成 numpy 向量化（与 `map_geometry.py` 的几何逐项一致）。

用法：
  camera_ground_probe.py [--samples 40] [--interval 0.3] [--json 结果.json]
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_geometry as G  # noqa: E402

TOPIC = '/rtk/camera/points'
GROUND_FRAME = 'base_footprint'      # 贴地，变换后的 z 就是"离地高度"
MAP_FRAME = 'map'                    # nav2 障碍层判 z 用的帧
# 与 nav2_params.yaml 的 min/max_obstacle_height / obstacle_max_range 同值，
# 这样"会不会被标成障碍"的判断口径与代价地图一致。
Z_LO, Z_HI, R_MAX = 0.12, 1.2, 6.0
BANDS = [(0.4, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 6.0)]


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_to_np(tf):
    tr = tf.transform.translation
    return quat_to_R(tf.transform.rotation), np.array([tr.x, tr.y, tr.z])


def cloud_to_array(msg):
    """PointCloud2 -> (N,3) float64，xyz 字节偏移按实际 fields 取（不假设步长）。"""
    off = {f.name: f.offset for f in msg.fields}
    need = [off.get('x'), off.get('y'), off.get('z')]
    if any(v is None for v in need):
        raise RuntimeError('点云缺少 x/y/z 字段: %s' % [f.name for f in msg.fields])
    # 注意：`np.frombuffer` 出来的数组可能不是 C 连续，`buf[:, v:v+4]` 又是切片视图，
    # 直接 `.view(np.float32)` 会报 "To change to a dtype of a different size, the array
    # must be C-contiguous"；而少了 `.reshape(-1)` 又会得到 (M,1)
    # （`view` 把最后一轴的 4 个 uint8 并成 1 个 float32），后面 `np.stack` 就成 3 维、
    # 布尔索引整片错位。
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    cols = [np.ascontiguousarray(buf[:, v:v + 4]).view(np.float32).reshape(-1).astype(np.float64)
            for v in need]
    return np.stack(cols, axis=1)


def nearest_surface_np(x, y):
    """`map_geometry.nearest_surface` 的 numpy 向量化版（几何逐项一致，别再各写一份）。"""
    d = np.maximum(np.abs(x) - G.BLOCK_HALF, 0.0)
    e = np.maximum(np.abs(y) - G.BLOCK_HALF, 0.0)
    best = np.hypot(d, e)

    t = G.PARTITION_THICK / 2
    d = np.maximum(np.abs(x) - G.PARTITION_HALF, 0.0)
    e = np.maximum(np.abs(y - G.PARTITION_OFFSET) - t, 0.0)
    best = np.minimum(best, np.hypot(d, e))
    d = np.maximum(np.abs(x + G.PARTITION_OFFSET) - t, 0.0)
    e = np.maximum(np.abs(y) - G.PARTITION_HALF, 0.0)
    best = np.minimum(best, np.hypot(d, e))

    h = G.PILLAR_SIDE / 2
    for px, py in G.PILLARS:
        d = np.maximum(np.abs(x - px) - h, 0.0)
        e = np.maximum(np.abs(y - py) - h, 0.0)
        best = np.minimum(best, np.hypot(d, e))

    for seg in G.SEGMENTS:
        xmin, xmax, ymin, ymax = G.segment_rect(seg)
        d = np.maximum(np.maximum(xmin - x, x - xmax), 0.0)
        e = np.maximum(np.maximum(ymin - y, y - ymax), 0.0)
        best = np.minimum(best, np.hypot(d, e))
    return best


class Probe(Node):
    def __init__(self):
        super().__init__('camera_ground_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.clouds = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 点云发布端（depth_image_proc）用 BEST_EFFORT，订阅端必须一致
        self.create_subscription(PointCloud2, TOPIC, self.clouds.append, qos_profile_sensor_data)

    def lookup(self, target, source, stamp, timeout=3.0):
        """优先按点云自己的时间戳取 TF，取不到退回最新（处理是即时的，差一帧 ~1.6 cm）。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for t in (rclpy.time.Time(seconds=stamp, nanoseconds=0), rclpy.time.Time()):
                try:
                    return self.tf_buffer.lookup_transform(target, source, t)
                except Exception:
                    pass
            rclpy.spin_once(self, timeout_sec=0.05)
        return None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--samples', type=int, default=40, help='分析多少帧点云')
    ap.add_argument('--interval', type=float, default=0.3,
                    help='相邻两帧的最小仿真时间间隔（秒）；太密只是同一姿势的重复')
    ap.add_argument('--json', default=None)
    ap.add_argument('--jsonl', default=None,
                    help='逐帧追加落盘（每行一个样本），一次跑几十秒时用，中途断了也不丢')
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--far_m', type=float, default=0.35,
                    help='离最近真实障碍表面超过这个距离的点算"幽灵候选"（默认 0.35 m）')
    args = ap.parse_args(argv)

    rclpy.init()
    node = Probe()
    band_acc = {b: [] for b in BANDS}
    samples = []
    fwd = []
    src = None
    done = 0
    last_t = -1.0
    t0 = time.time()
    while done < args.samples and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        while node.clouds:
            msg = node.clouds.pop(0)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if src is None:
                src = msg.header.frame_id
            if stamp - last_t < args.interval:
                continue
            last_t = stamp
            tg = node.lookup(GROUND_FRAME, src, stamp)
            tm = node.lookup(MAP_FRAME, src, stamp)
            if tg is None or tm is None:
                continue
            try:
                pts_src = cloud_to_array(msg)
            except Exception as exc:                   # pragma: no cover
                print('跳过一帧：%s' % exc, file=sys.stderr)
                continue
            pid = np.nonzero(np.isfinite(pts_src).all(axis=1))[0]
            pts_src = pts_src[pid]
            if not len(pts_src):
                continue
            R, T = transform_to_np(tg)
            Rm, Tm = transform_to_np(tm)
            fwd.append(R @ np.array([0.0, 0.0, 1.0]))

            pts = pts_src @ R.T + T
            r = np.hypot(pts[:, 0], pts[:, 1])
            zz = pts[:, 2]
            for lo, hi in BANDS:
                sel = (r >= lo) & (r < hi) & (r <= R_MAX)
                if sel.sum():
                    band_acc[(lo, hi)].append(zz[sel])

            p_map = pts_src @ Rm.T + Tm
            tb = node.lookup(MAP_FRAME, GROUND_FRAME, stamp)
            if tb is None:
                continue
            rb = tb.transform.translation
            rq = tb.transform.rotation
            rYaw = math.degrees(math.atan2(2 * (rq.w * rq.z + rq.x * rq.y),
                                           1 - 2 * (rq.y * rq.y + rq.z * rq.z)))
            rr = np.hypot(p_map[:, 0] - rb.x, p_map[:, 1] - rb.y)
            ang = np.degrees(np.arctan2(p_map[:, 1] - rb.y, p_map[:, 0] - rb.x))
            rel = (ang - rYaw + 180.0) % 360.0 - 180.0     # 相对车头方位角
            cand = np.nonzero((p_map[:, 2] >= Z_LO) & (p_map[:, 2] <= Z_HI) & (rr <= R_MAX))[0]
            near = nearest_surface_np(p_map[cand, 0], p_map[cand, 1])
            ph = cand[near > args.far_m]
            # 幽灵点在**深度图**里的像素坐标与深度值（点云是有序的，行列能反推）
            px = []
            if len(ph) and msg.height > 1:
                row = pid[ph] // msg.width
                col = pid[ph] % msg.width
                px = [[int(col[i]), int(row[i]), round(float(pts_src[ph[i], 2]), 3)]
                      for i in range(min(10, len(ph)))]
            samples.append({
                'stamp': round(stamp, 2),
                'robot_xy_yaw': [round(rb.x, 3), round(rb.y, 3), round(rYaw, 1)],
                'n_band': int(len(cand)),
                'n_phantom': int(len(ph)),
                'phantom_r_p50': round(float(np.median(rr[ph])), 3) if len(ph) else None,
                'phantom_z_p50': round(float(np.median(p_map[ph, 2])), 3) if len(ph) else None,
                'phantom_z_max': round(float(p_map[ph, 2].max()), 3) if len(ph) else None,
                'phantom_az_p50': round(float(np.median(rel[ph])), 1) if len(ph) else None,
                'phantom_dist_bins': {
                    '%s-%s' % (lo, hi): int(((near > args.far_m) & (near >= lo) & (near < hi)).sum())
                    for lo, hi in ((0.35, 0.6), (0.6, 1.0), (1.0, 1.6), (1.6, 3.0))},
                'phantom_max_range': round(float(rr[ph].max()), 3) if len(ph) else None,
                # 幽灵点自身的样本（map 帧 xyz + 到车的地面距离 + 相对车头方位角 + 离最近真实表面距离）
                'phantom_sample': [[round(float(p_map[i, 0]), 3), round(float(p_map[i, 1]), 3),
                                    round(float(p_map[i, 2]), 3), round(float(rr[i]), 3),
                                    round(float(rel[i]), 1), round(float(nn), 3)]
                                   for i, nn in list(zip(ph, near[near > args.far_m]))[:8]],
                'phantom_pixels_uvZ': px,
            })
            # 逐帧立刻落盘（一次调用要跑几十秒，中途被打断不能丢数据）
            if args.jsonl:
                with open(args.jsonl, 'a', encoding='utf-8') as handle:
                    handle.write(json.dumps(samples[-1], ensure_ascii=False) + '\n')
            done += 1
        if done and not node.clouds:
            pass
    if not done:
        print('** 没能分析到任何一帧（%s 没数据或 TF 不齐）**' % TOPIC, file=sys.stderr)
        return 1

    bands = []
    fit_pts = []
    for lo, hi in BANDS:
        acc = band_acc[(lo, hi)]
        entry = {'band_m': [lo, hi], 'n': int(sum(len(a) for a in acc))}
        if entry['n'] >= 20:
            zs = np.concatenate(acc)
            q = np.percentile(zs, [1, 5, 10, 25, 50, 75, 90, 95, 99])
            floor = zs[zs <= np.percentile(zs, 40)]
            entry.update({
                'z_p01': round(float(q[0]), 4), 'z_p05': round(float(q[1]), 4),
                'z_p10': round(float(q[2]), 4), 'z_p25': round(float(q[3]), 4),
                'z_median': round(float(q[4]), 4), 'z_p75': round(float(q[5]), 4),
                'z_p90': round(float(q[6]), 4), 'z_p95': round(float(q[7]), 4),
                'z_p99': round(float(q[8]), 4),
                'frac_in_obstacle_band': round(float(((zs >= Z_LO) & (zs <= Z_HI)).mean()), 4),
                'floor_z_median': round(float(np.median(floor)), 4),
            })
            fit_pts.append((0.5 * (lo + min(hi, R_MAX)), float(np.median(floor))))
        bands.append(entry)

    fit = None
    if len(fit_pts) >= 2:
        d = np.array([p[0] for p in fit_pts])
        v = np.array([p[1] for p in fit_pts])
        b, a = np.polyfit(d, v, 1)
        fit = {'a_m': round(float(a), 4), 'b_rad': round(float(b), 5),
               'b_deg': round(math.degrees(math.atan(float(b))), 3),
               'cross_m': round(float((Z_LO - a) / b), 3) if b > 1e-9 else None,
               'max_resid_m': round(float(np.abs(v - (a + b * d)).max()), 4),
               'note': 'z(地面) = a + b*d；b 就是"地面被抬起的斜率"，等价于俯仰/主点误差'}

    fm = np.mean(fwd, axis=0) if fwd else None
    result = {
        'source_topic': TOPIC,
        'cloud_frame': src,
        'n_clouds_used': done,
        'z_lo': Z_LO, 'z_hi': Z_HI, 'range_max': R_MAX, 'far_m': args.far_m,
        'tf_optical_axis_in_base': [round(float(v), 5) for v in fm] if fm is not None else None,
        'tf_optical_axis_pitch_deg': round(math.degrees(math.asin(
            max(-1.0, min(1.0, float(fm[2]))))), 3) if fm is not None else None,
        'bands': bands,
        'floor_fit': fit,
        'phantom_total': int(sum(s['n_phantom'] for s in samples)),
        'phantom_samples_with_hits': int(sum(1 for s in samples if s['n_phantom'])),
        'samples': samples,
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

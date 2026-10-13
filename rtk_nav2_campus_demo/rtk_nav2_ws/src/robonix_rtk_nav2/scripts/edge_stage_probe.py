#!/usr/bin/env python3
"""实验 P/Q 的**原始行**：坡南沿那些列到底被哪一道判据拒了。

★ 2026-10-08 教训（本文件的由来）：上一版探针把 `camera_height` 默认写成 0.19，
而部署值是 0.55，于是算出的 `floor=0.35` 被误读成"检测器失明"，整条结论作废。
本版**开机自动读 `/depth_drop`（部署节点名）的实际参数**并逐行打印；读不到就大声告警、
且每行都带 `params_source`，让"工具参数 ≠ 部署参数"这类错**不可能再隐身**。

判据顺序与 `depth_drop.edge_columns` 逐字一致（see depth_drop.py）：
  每列：upper≥2 且 lower≥2 → visible(0.10<sep<3.0) → 遮挡 → 边缘前 0.40 m 水平面 |斜率| ≤ 0.05
        → 下坡面 |斜率| ≤ 0.045 → 成列
  **遮挡门的谓词方向（2026-10-08 更正，之前写反了）**：间隙（最后一个上层样点 → 最近下层样点）
  内**实测点数 ≥3 ⇒ 表面连续 ⇒ 不是悬崖 ⇒ 拒**；点数 <3（近似无回波）才保留为候选下降沿。
  即"测得越多越说明是连续面"，样例 `between=22` 被拒是自洽的。

**列的选择不用图像列号猜**，且**取两条规则的并集**（单一规则会把整类帧滤掉）：
`end`（候选边缘点=最后一个上层样点）落在 `--band x0,x1,y0,y1`（默认 0.1,1.6,0.40,0.75，坡上视角）
**或** `lower` 样点落在 `--band-lo`（默认 0.1,1.6,0.0,0.60，坎之外的平台 = reviewer 口径）。
每列记 `sel` 标明是哪条规则选中的。
每列输出一行记录：`up_n / low_n / sep / between / pre_slope / low_slope / reject`，
`reject` = 第一道把它拒掉的判据（None = 通过）。
"""
import argparse
import json
import sys
import time

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, '/work/rtk_nav2_ws/src/robonix_rtk_nav2')
from robonix_rtk_nav2.depth_drop import rotation  # noqa: E402

PARAM_NAMES = ['camera_height', 'drop_threshold', 'ground_tolerance', 'resolution',
               'min_columns', 'hazard_margin', 'floor_consistency_tol']


def frames_from_depth(depth, intr, transform, cam_h, drop_th, gtol, stride=2):
    """复刻 edge_columns 的中间量，并留下逐列的门信息。返回 (agg, cols)。"""
    fx, fy, cx, cy = intr
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0.05) & (z < 7.95)
    xyz = np.stack(((u-cx)*z/fx, (v-cy)*z/fy, z), axis=-1)
    q = transform.transform.rotation
    offset = transform.transform.translation
    t = np.array([offset.x, offset.y, offset.z])
    world = xyz @ rotation(q).T + t
    floor = t[2] - cam_h
    upper = valid & (np.abs(world[..., 2] - floor) <= gtol)
    lower = valid & (world[..., 2] <= floor - drop_th)
    heading = rotation(q)[:, 2].copy()
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    forward = (world - t) @ heading
    upper &= (forward > 0.55) & (forward < 5.5)
    lower &= (forward > 0.55) & (forward < 7.5)
    agg = {'valid_px': int(valid.sum()), 'upper_px': int(upper.sum()),
           'lower_px': int(lower.sum()), 'floor': round(float(floor), 4),
           'cand': 0}
    cols = []
    for col in range(z.shape[1]):
        idx_up = np.where(upper[:, col])[0]
        idx_lo = np.where(lower[:, col])[0]
        rec = {'col': int(col), 'up_n': int(len(idx_up)), 'low_n': int(len(idx_lo)),
               'sep': None, 'between': None, 'pre_slope': None, 'low_slope': None,
               'reject': None, 'low_xyz': None, 'end_xyz': None}
        if len(idx_lo) == 0:
            continue
        # 列的选择：用重建出的三维点，不用图像列号（reviewer 2026-10-08 的要求）。
        # **两条规则取并集**，因为 reviewer 给的口径（按 `lower` 落点选）与"坡上朝南看"的
        # 几何**天然错开**：站坡上面朝南时，`lower` 样本是坎**之外**的平台（y<0.45），
        # 而候选边缘点（`end` = 最后一个上层样点）才落在唇上（y≈0.5~0.7）。
        # 所以：`end` 在 y∈[0.40,0.75]、x∈[0.1,1.6]（坡上视角）**或**
        #       `lower` 在 y∈[0.00,0.60]、x∈[0.1,1.6]（reviewer 口径，含高台远视角）
        # 都收，并记 `sel` 标明来源，避免再用单一规则把整类帧滤掉（上一版就是这么丢的）。
        near_lo = idx_lo[np.argmin(forward[idx_lo, col])]
        p = world[near_lo, col]
        rec['low_xyz'] = [round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)]
        if len(idx_up) >= 2:
            e = world[idx_up[np.argmax(forward[idx_up, col])], col]
            rec['end_xyz'] = [round(float(e[0]), 3), round(float(e[1]), 3), round(float(e[2]), 3)]
        end_in = (rec['end_xyz'] is not None
                  and BAND[0] <= rec['end_xyz'][0] <= BAND[1]
                  and BAND[2] <= rec['end_xyz'][1] <= BAND[3])
        low_in = (BAND_LO[0] <= rec['low_xyz'][0] <= BAND_LO[1]
                  and BAND_LO[2] <= rec['low_xyz'][1] <= BAND_LO[3])
        if not (end_in or low_in):
            continue
        rec['sel'] = 'both' if (end_in and low_in) else ('end' if end_in else 'low')
        if len(idx_up) < 2 or len(idx_lo) < 2:
            rec['reject'] = 'few_samples'
            cols.append(rec)
            continue
        end_idx = idx_up[np.argmax(forward[idx_up, col])]
        end = world[end_idx, col]
        rec['end_xyz'] = [round(float(end[0]), 3), round(float(end[1]), 3),
                          round(float(end[2]), 3)]
        lower_range = forward[idx_lo, col]
        separation = lower_range - forward[end_idx, col]
        visible = (separation > 0.10) & (separation < 3.0)
        if not np.any(visible):
            rec['reject'] = 'visible'
            rec['sep'] = round(float(separation.min()), 3)
            cols.append(rec)
            continue
        rec['sep'] = round(float(separation[visible].min()), 3)
        lo_c = max(0, col - 5); hi_c = min(z.shape[1], col + 6)
        nearest = lower_range[visible].min()
        r_from = forward[end_idx, col] + 0.02
        r_to = nearest - 0.02
        between = 0
        for cc in range(lo_c, hi_c):
            if r_to > r_from:
                between += int(np.count_nonzero(
                    valid[:, cc] & (forward[:, cc] > r_from) & (forward[:, cc] < r_to)))
        rec['between'] = int(between)
        if between >= 3:
            rec['reject'] = 'occlusion'
            cols.append(rec)
            continue
        r_edge = forward[end_idx, col]
        rr, zz = [], []
        for cc in range(lo_c, hi_c):
            m = valid[:, cc] & (forward[:, cc] > r_edge - 0.40) & (forward[:, cc] <= r_edge + 0.02)
            if np.any(m):
                rr.extend(forward[m, cc].tolist()); zz.extend(world[m, cc, 2].tolist())
        if len(rr) >= 6:
            ps = float(np.polyfit(np.asarray(rr), np.asarray(zz), 1)[0])
            rec['pre_slope'] = round(ps, 4)
            if abs(ps) > 0.05:
                rec['reject'] = 'pre_slope'
                cols.append(rec)
                continue
        lr, lz = [], []
        for cc in range(lo_c, hi_c):
            ridx = np.where(lower[:, cc])[0]
            if len(ridx):
                lr.extend(forward[ridx, cc].tolist()); lz.extend(world[ridx, cc, 2].tolist())
        if len(lr) >= 6:
            ls = float(np.polyfit(np.asarray(lr), np.asarray(lz), 1)[0])
            rec['low_slope'] = round(ls, 4)
            if abs(ls) > 0.045:
                rec['reject'] = 'low_slope'
                cols.append(rec)
                continue
        rec['reject'] = None       # 通过全部判据 ⇒ 成列
        agg['cand'] += 1
        cols.append(rec)
    return agg, cols


BAND = (0.1, 1.6, 0.40, 0.75)      # 候选边缘点(end) 的框：坡上朝南看的唇点
BAND_LO = (0.1, 1.6, 0.00, 0.60)   # lower 样点的框：reviewer 口径（坎之外的平台）


class Probe(Node):
    def __init__(self, out_path, cli, every):
        super().__init__('edge_stage_probe')
        self.out = open(out_path, 'w', encoding='utf-8')
        self.every = every
        self.n = 0
        self.attempts = 0
        self.p = dict(cli)
        self.params_source = 'cli'
        self.intr = None
        self.pending = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, '/rtk/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/rtk/camera/depth_image', self.on_depth,
                                 qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def fetch_params(self):
        """开机读部署节点的实际判据参数（工具参数 ≠ 部署参数是上一版的致命错）。"""
        cli = self.create_client(GetParameters, '/depth_drop/get_parameters')
        if not cli.wait_for_service(timeout_sec=8.0):
            self.get_logger().error(
                '读不到 /depth_drop/get_parameters ⇒ 用命令行值 %s；'
                '**这些值可能与节点实际不同，本文件所有 floor 相关结论不可用**' % self.p)
            return
        req = GetParameters.Request()
        req.names = PARAM_NAMES
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        try:
            vals = {n: v for n, v in zip(PARAM_NAMES, fut.result().values)}
        except Exception as e:
            self.get_logger().error('参数读回失败 %s；沿用命令行值' % e)
            return
        self.p['camera_height'] = vals['camera_height'].double_value
        self.p['drop_threshold'] = vals['drop_threshold'].double_value
        self.p['ground_tolerance'] = vals['ground_tolerance'].double_value
        self.p['min_columns'] = vals['min_columns'].integer_value
        self.p['hazard_margin'] = vals['hazard_margin'].double_value
        self.params_source = 'node'
        self.get_logger().info('已读部署节点参数：%s' % self.p)

    def on_info(self, m):
        if m.k[0] > 0 and m.k[4] > 0:
            self.intr = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_depth(self, m):
        if m.encoding == '32FC1':
            self.pending = m

    def tick(self):
        m = self.pending
        if m is None or self.intr is None:
            return
        stamp = Time.from_msg(m.header.stamp)
        # ★ 有界重试（与节点同口径，2026-10-08 reviewer 批准）：运动时图像时间戳与 TF
        #   常常差 2–20 ms ⇒ 上一版直接丢帧，结果**只捞到停车姿态的列**（49916 列全来自
        #   `cam_x=-1.9`），答不上"坡上朝南看"的问题。这里同一帧最多重试 5 拍（≈0.25 s），
        #   仍失败就退回"最新 TF"并标 `tf_stale=True`（读数降级但不丢帧）。
        stale = False
        try:
            tf_map = self.tf.lookup_transform('map', m.header.frame_id, stamp)
        except Exception:
            self.attempts += 1
            if self.attempts <= 5:
                return
            try:
                tf_map = self.tf.lookup_transform('map', m.header.frame_id, Time())
                stale = True
            except Exception:
                self.attempts = 0
                return
        self.attempts = 0
        self.pending = None
        self.n += 1
        if self.n % self.every:
            return
        data = np.frombuffer(m.data, dtype=np.float32).reshape(
            m.height, m.step // 4)[:, :m.width]
        agg, cols = frames_from_depth(data, self.intr, tf_map,
                                      self.p['camera_height'], self.p['drop_threshold'],
                                      self.p['ground_tolerance'])
        tr = tf_map.transform.translation
        row = {'wall': round(time.time(), 3), 'params_source': self.params_source,
               'tf_stale': bool(stale),
               'cam_h': self.p['camera_height'], 'drop_th': self.p['drop_threshold'],
               'gtol': self.p['ground_tolerance'], 'min_columns': self.p['min_columns'],
               'cam_x': round(tr.x, 3), 'cam_y': round(tr.y, 3), 'cam_z': round(tr.z, 4),
               'band_cols': len(cols), 'cols': cols}
        row.update(agg)
        self.out.write(json.dumps(row) + '\n')
        self.out.flush()
        self.get_logger().info(
            'frame up=%d low=%d band=%d cand=%d floor=%.3f cam=(%.2f,%.2f) src=%s stale=%s' %
            (agg['upper_px'], agg['lower_px'], len(cols), agg['cand'],
             agg['floor'], tr.x, tr.y, self.params_source, stale))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True)
    p.add_argument('--camera-height', type=float, default=0.55)
    p.add_argument('--drop-threshold', type=float, default=0.15)
    p.add_argument('--ground-tolerance', type=float, default=0.06)
    p.add_argument('--band', default='0.1,1.6,0.40,0.75')
    p.add_argument('--band-lo', default='0.1,1.6,0.0,0.60')
    p.add_argument('--every', type=int, default=5)
    p.add_argument('--seconds', type=float, default=900)
    a = p.parse_args()
    global BAND, BAND_LO
    BAND = tuple(float(v) for v in a.band.split(','))
    if a.band_lo:
        BAND_LO = tuple(float(v) for v in a.band_lo.split(','))
    rclpy.init()
    n = Probe(a.out, {'camera_height': a.camera_height,
                      'drop_threshold': a.drop_threshold,
                      'ground_tolerance': a.ground_tolerance}, a.every)
    n.fetch_params()
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.out.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

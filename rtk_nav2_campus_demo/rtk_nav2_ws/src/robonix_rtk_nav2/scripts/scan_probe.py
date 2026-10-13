#!/usr/bin/env python3
"""低矮障碍的**传感器侧旁证 + 激光自击自检**（在跑着的栈上量，判据 ② 的归因部分）。

判据 ② 要的不只是"紧盒 lethal > 0"，还要**归因**：那个读数是**激光**打进去的，
还是**相机本来就看得到**（0.30 m 档按 brief 是"相机自己也该看到"的对照组）。
所以本工具对同一个世界坐标 (x, y) 同时量两件事：

  · **激光侧**：`/rtk/scan` 在"机器人 → 目标"方位上的回波距离
    （取该方位 ±3° 窗口内**最近**的回波），以及整帧的最短回波。
    **自击自检**就在这一项里：机器人自身结构（轮/底盘/安装座）会在**固定方位**
    给出很近的固定回波；把"最短的 8 条回波 + 它们的相对方位"打出来，
    与"该方位上世界几何有没有障碍"对照，就能判断有没有自击
    （`--selfcheck` 只做这一项，不要求给目标点）。
  · **相机侧**：`/rtk/camera/points` 在目标 ±box 内的点数，变到 **map 帧**后按 z 分类：
    `0.30 <= z <= 1.2` 的那部分**才是相机源真正会交给 nav2 的**（源阈值 0.30/1.2，
    ObservationBuffer 按 z **丢弃**带外的点——离线探针实测过）；
    z 的最大值直接说明"这个高度的障碍是不是落在相机的可用高度带里"。

**地图侧不在这里**：紧盒 lethal 用 `nav4_map_box.py`（同一个世界坐标、同一个紧盒口径）。

判据 ② 每一档的完整证据 = nav4_map_box.py 的读数（地图侧）+ 本工具的读数（传感器侧）。

用法（容器内）：
  scan_probe.py --traj /work/artifacts/rtk-nav2/<run>/trajectory.csv \
      --x 2.6 --y 0.0 --json .../lowobs_h015_d1_scan.json
  scan_probe.py --selfcheck --traj ... --json .../scan_selfcheck.json
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_ground_probe as C  # noqa: E402  （复用点云解析与 TF 取用，别再各写一份）
import noecho_probe as N  # noqa: E402  （复用"最后一行真值位姿"的口径）

SCAN_TOPIC = '/rtk/scan'
CLOUD_TOPIC = '/rtk/camera/points'
MAP_FRAME = 'map'
# 相机源在 nav2 里的高度带（相机源自己的 min/max_obstacle_height）
CAM_LO, CAM_HI = 0.30, 1.2
# 判"这一档有没有回波"的距离容差（m）：回波距离与几何距离差在这个范围内就算命中
HIT_TOL = 0.35


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Probe(Node):
    def __init__(self):
        super().__init__('scan_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.scan = None
        self.cloud = None
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._on_cloud, qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_scan(self, msg):
        self.scan = msg

    def _on_cloud(self, msg):
        self.cloud = msg

    def spin_until(self, what, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if what == 'scan' and self.scan is not None:
                return True
            if what == 'cloud' and self.cloud is not None:
                return True
        return False


def scan_report(msg, yaw0, target=None):
    """一帧激光的读数：量程/回波分布/目标方位回波/最短回波（自击自检）。"""
    n = len(msg.ranges)
    finite = [(i, float(r)) for i, r in enumerate(msg.ranges) if math.isfinite(r)]
    inf_n = n - len(finite)
    inc = msg.angle_increment
    az = lambda i: math.degrees(msg.angle_min + i * inc)
    out = {
        'beams': n,
        'angle_min_deg': round(math.degrees(msg.angle_min), 2),
        'angle_max_deg': round(math.degrees(msg.angle_max), 2),
        'range_min': msg.range_min, 'range_max': msg.range_max,
        'finite_beams': len(finite), 'inf_beams': inf_n,
        'finite_ratio': round(len(finite) / max(1, n), 4),
        'min_range_m': round(min((r for _, r in finite), default=float('nan')), 4),
        'max_finite_range_m': round(max((r for _, r in finite), default=float('nan')), 4),
        'frame_id': msg.header.frame_id,
    }
    # 回波距离直方图（自击会在最左一档堆出异常）
    bands = [(0.0, 0.2), (0.2, 0.3), (0.3, 0.5), (0.5, 1.0), (1.0, 2.0),
             (2.0, 4.0), (4.0, 6.0), (6.0, 8.001)]
    out['hist'] = {'%.1f-%.1f' % b: sum(1 for _, r in finite if b[0] <= r < b[1]) for b in bands}
    # 最短的 8 条回波 + 相对机器人朝向的方位（自击自检的证据面）
    shortest = sorted(finite, key=lambda t: t[1])[:8]
    out['shortest_echoes'] = [{'range_m': round(r, 4),
                               'rel_deg': round(wrap180(az(i) - yaw0), 2)} for i, r in shortest]
    # 目标方位上的回波
    if target is not None:
        tx, ty = target
        rel = wrap180(math.degrees(math.atan2(ty, tx)))
        out['target'] = {'x': tx, 'y': ty, 'rel_deg_from_robot': round(rel, 2),
                         'geom_distance_m': round(math.hypot(tx, ty), 3)}
        cand = [(i, r) for i, r in finite if abs(wrap180(az(i) - rel)) <= 3.0]
        if cand:
            i, r = min(cand, key=lambda t: t[1])
            out['target'].update({
                'nearest_echo_m': round(r, 4),
                'nearest_echo_rel_deg': round(wrap180(az(i) - rel), 2),
                'hit': abs(r - math.hypot(tx, ty)) <= HIT_TOL,
                'delta_m': round(r - math.hypot(tx, ty), 4),
            })
        else:
            out['target'].update({'nearest_echo_m': None, 'hit': False,
                                  'note': '该方位 ±3° 内没有任何回波（全 inf）'})
    return out


def camera_report(node, target, box, box_z=None):
    """相机点云在目标 **3D 盒**内的点数与 map 帧 z 分布（相机源能不能看见的关键）。

    **`box_z` 不能省**（第一轮踩过）：只按 x/y 取 ±box 会把**障碍后面**的东西一起算进来
    （0.30 m 那档就报出 2267 个 z>=0.30 的点，实际是远处墙），于是"相机看不看得见这个矮物"
    这个判断被污染。现在按三维盒取：|x-tx|<=box 且 |y-ty|<=box 且 |z-z_center|<=box_z，
    `z_center` 取障碍半高（矮障碍的中心）。`box_z` 不传时退回只看 x/y，并在结果里标注。
    """
    msg = node.cloud
    if msg is None:
        return {'error': '没收到点云'}
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    # **必须等 TF 缓冲填好再查**（第一轮就是这样批量失败的）：
    # 探针一收到第一帧点云就查 `map <- 相机帧`，而 tf2 缓冲里那时还没有那一帧，
    # 抛 `LookupException: ".../depth_camera" ... does not exist`，
    # 于是整份 JSON 都没产出——看起来像"相机侧读数缺失"，其实只是查得太早。
    # **两段要分开看**（这是本工具上一轮批量失败的原因）：
    #   · `camera → base_link` 这一段是**静态**的（launch 里的 static_transform_publisher）；
    #   · `map → odom` 那一段是**动态**的，由我们自己的 `absolute_localizer` 发布——
    #     **只有定位链走完 no_fix→settling→ok 之后才存在**，所以**起栈初期查不到它是正常的**，
    #     不是缺陷、也不是"要等更久"。
    # 做法：**先等 `map → base_footprint` 可查**（与 wait_tf_gate.py / wait_ready.py 同一个闸门判据），
    # 就绪后再查 `map ← <相机帧>`，只留**短重试**（2 s）应对时间戳抖动。
    t0 = time.time()
    while time.time() - t0 < 25.0:
        try:
            node.tf_buffer.lookup_transform(MAP_FRAME, 'base_footprint', rclpy.time.Time())
            break
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    tfm = None
    t1 = time.time()
    while tfm is None and time.time() - t1 < 2.0:
        try:
            tfm = node.tf_buffer.lookup_transform(
                MAP_FRAME, msg.header.frame_id,
                rclpy.time.Time(seconds=int(stamp), nanoseconds=int((stamp % 1) * 1e9)))
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    if tfm is None:
        return {'error': 'map→base_footprint 就绪后 2 秒内仍取不到 map <- %s 的 TF'
                         % msg.header.frame_id,
                'frame_id': msg.header.frame_id,
                'note': '先确认定位链已到 ok（map→base_footprint 可查）再看这一条'}
    R, t = C.transform_to_np(tfm)
    pts = C.cloud_to_array(msg)
    pm = pts @ R.T + t
    tx, ty = target
    sel = (np.abs(pm[:, 0] - tx) <= box) & (np.abs(pm[:, 1] - ty) <= box)
    if box_z is not None:
        sel = sel & (np.abs(pm[:, 2] - box_z) <= box)
    sub = pm[sel]
    if sub.shape[0] == 0:
        return {'points_in_box': 0, 'box_z': box_z,
                'note': '目标 3D 盒（水平 ±%.2f m%s）内相机一个点都没有'
                        % (box, '' if box_z is None else '、z 中心 %.2f m ±%.2f' % (box_z, box))}
    z = sub[:, 2]
    in_cam = int(((z >= CAM_LO) & (z <= CAM_HI)).sum())
    return {
        'frame_id': msg.header.frame_id,
        'box_z_center': box_z,
        'points_in_box': int(sub.shape[0]),
        'z_min': round(float(z.min()), 4), 'z_max': round(float(z.max()), 4),
        'z_median': round(float(np.median(z)), 4),
        'points_z_ge_0.30': in_cam,
        'points_z_in_0.02_1.2': int(((z >= 0.02) & (z <= 1.2)).sum()),
        'camera_source_would_mark': in_cam,
        'note': ('相机源阈值 0.30/1.2：只有 z>=0.30 的点会交给 nav2。'
                 '本档若 `camera_source_would_mark`=0，说明"进图的致命格"只能来自激光。'),
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj', required=True, help='trajectory.csv（最后一行真值位姿）')
    ap.add_argument('--x', type=float, default=None)
    ap.add_argument('--y', type=float, default=None)
    ap.add_argument('--box', type=float, default=0.35, help='相机侧盒子的半边长（m）')
    ap.add_argument('--box-z', type=float, default=None,
                    help='相机侧**竖直角度的盒子中心**（m）：给矮障碍的半高（如 0.15 m 箱给 0.075）。'
                         '不给就退回只看水平 x/y——**那会把障碍后面的墙算进来**，结论会被污染')
    ap.add_argument('--selfcheck', action='store_true',
                    help='只做激光自击自检（不需要 --x/--y）')
    ap.add_argument('--json', default=None)
    ap.add_argument('--timeout', type=float, default=60.0)
    args = ap.parse_args(argv)

    x0, y0, yaw0, nrows = N.last_truth_pose(args.traj)
    print('# 机器人真值位姿 (%.3f, %.3f, yaw=%.2f°)，trajectory 行数 %d'
          % (x0, y0, yaw0, nrows))

    rclpy.init()
    node = Probe()
    if not node.spin_until('scan', args.timeout):
        print('** %.0f 秒内没收到 %s **' % (args.timeout, SCAN_TOPIC), file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    out = {'traj': os.path.abspath(args.traj),
           'pose': {'gt_x': x0, 'gt_y': y0, 'gt_yaw_deg': round(yaw0, 2)},
           'scan': scan_report(node.scan, yaw0)}

    if not args.selfcheck:
        if args.x is None or args.y is None:
            print('** 除 --selfcheck 外必须给 --x --y **', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 2
        # 目标点在机器人坐标系下的相对方位（scan 帧与 base_footprint 同向：安装关节无旋转）
        th = math.radians(yaw0)
        dx = args.x - x0
        dy = args.y - y0
        tx = dx * math.cos(th) + dy * math.sin(th)
        ty = -dx * math.sin(th) + dy * math.cos(th)
        out['scan'] = scan_report(node.scan, yaw0, (tx, ty))
        if node.spin_until('cloud', 15.0):
            rclpy.spin_once(node, timeout_sec=0.2)   # 多攒一帧，日志更稳
            out['camera'] = camera_report(node, (args.x, args.y), args.box, args.box_z)
        else:
            out['camera'] = {'error': '15 秒内没收到 %s' % CLOUD_TOPIC}

    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

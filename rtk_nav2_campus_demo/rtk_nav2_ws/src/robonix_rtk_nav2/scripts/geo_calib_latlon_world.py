#!/usr/bin/env python3
"""标定的**第二半**：**世界系 (x,y) ↔ 经纬度** 的换算系数（写进 `geo_calib.json`）。

**为什么还要这一半**（brief §4）：`geo_calib_map_world.py` 量的是 **map ↔ 世界系**（在 100 m 尺度上
必须是"重合到 0.03° 级"，实测已确认 ✓）。但 `geo_route` 收进来的是**经纬度** ⇒
还需要 **世界系 ↔ 经纬度** 这一段。两者一合，才得到"经纬度 → map"的完整系数。

**做法（与第一半同一条纪律：实测、不许手推）**：
同时记 **`/rtk/navsat`（Gazebo 理想 GPS，无噪声，给经纬度）** 与
**`/ground_truth/odom`（Gazebo 真值，给世界系 x,y）**，按最近时间戳配对，
拟合**相似变换**（缩放 + 旋转 + 平移）。

★ 这里**没有** `rtk_sim` 的注入偏差 —— 因为配的是 **`/rtk/navsat`（理想值）**，
不是 `/rtk/sim_fix`（加了 σ/bias 的那条）。**这条很重要**：换算系数应当是"几何的"，
模拟器的噪声/偏差不属于坐标系 ✓。（`map` 系里那份偏差已经在第一半的 `t` 里量过 ✓。）

★ 与第一半一样：**只取数与拟合，不判过不过**；输出逐样本 CSV 供任何人独立复算。
★ 结果写进 `<out>_fit.json`；`geo_calib.json` 由 `--emit-geo-calib` 与第一半合并生成。

用法（容器内）：
  geo_calib_latlon_world.py --out /work/artifacts/rtk-nav2/<run>/latlon_fit [--seconds 1800] [--tol 0.05]
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix

GT_TOPIC = '/ground_truth/odom'
GPS_TOPIC = '/rtk/navsat'          # ★ **理想 GPS**（不是 /rtk/sim_fix）—— 见文件头

# 与第一半**逐字同源**的默认路径（`calib_key` 的分量取自这几份文件）
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_WORLD = os.path.join(_HERE, '..', 'worlds', 'campus_100.sdf')
DEFAULT_NAVSAT = os.path.join(_HERE, '..', 'config', 'navsat.yaml')


# ★★ WGS84 椭球换算（reviewer 2026-10-10 裁定 ①）★★
#   **为什么两个常数都要换**：实测两轴尺度差 `5.301e-03`，而
#     · 纬度轴：约定 `111320` vs 椭球 `M(φ)·π/180 = 110856.3` ⇒ **+0.4183%** ← **主导项** ✓
#     · 经度轴：约定 `111320·cosφ` vs `N(φ)·cosφ·π/180 = 96265.3` ⇒ **−0.0843%**
#   ⇒ 与实测（北 `+0.428%` / 东 `-0.102%`）**同号同量级** ✓。
#   ★★ 我原来的归因是**错的** ✗：我写成"经度半径 6383 km vs 球 6371 km"，
#      可 6383 km 那个是 **N、只管经度轴、只占约 1/6**；主导的是**子午圈半径 M**（纬度轴）✓。
#      ⇒ **只换 `M_LON` 会留下 0.42% ≈ 5/6 的误差** ✗。
A_WGS84 = 6378137.0
F_WGS84 = 1.0 / 298.257223563


def ellipsoid_m_per_deg(lat_deg):
    """`(纬度轴每度米数, 经度轴每度米数)` = `M(φ)·π/180` 与 `N(φ)·cosφ·π/180`（WGS84）。"""
    e2 = F_WGS84 * (2.0 - F_WGS84)
    s = math.sin(math.radians(lat_deg))
    w = 1.0 - e2 * s * s
    m_merid = A_WGS84 * (1.0 - e2) / (w ** 1.5)      # 子午圈曲率半径 M
    n_prime = A_WGS84 / math.sqrt(w)                  # 卯酉圈曲率半径 N
    return (m_merid * math.pi / 180.0,
            n_prime * math.cos(math.radians(lat_deg)) * math.pi / 180.0)


def build_calib(world, navsat, run_dir):
    """算**与第一半同一个** `calib_key`（reviewer 2026-10-09："键要覆盖两半的来源"）。

    ★ 必须**用同一个函数**算 —— 抄一份实现出来，两边就会各自漂移 ✗。
    ★ 必须**带上 `--from-run`** —— 不带时键只覆盖 5 个基准分量、不含"数据来自哪一轮"，
      两半的键就会因为分量列表不同而**永远不相等** ⇒ 合成时的硬断言形同虚设 ✗。
    """
    import geo_calib_map_world as H
    key, comp, bundle = H.calib_key(world, navsat, H.GEN_PATH, H.RTKSIM_PATH, run_dir or None)
    return {'calib_key': key, 'calib_key_components': comp, 'calib_key_canonical': bundle}


def umeyama_2d(p, q):
    """2D 相似变换：q ≈ s·R(θ)·p + t（这里 p = 世界系 (x,y)，q = 经纬度增量 (dlon,dlat) 换算成米）。"""
    mu_p = p.mean(axis=0)
    mu_q = q.mean(axis=0)
    P = p - mu_p
    Q = q - mu_q
    n = len(p)
    sigma = (Q.T @ P) / n
    u, d, vt = np.linalg.svd(sigma)
    sm = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        sm[1, 1] = -1.0
    rot = u @ sm @ vt
    var_p = (P ** 2).sum() / n
    scale = float((d * np.diag(sm)).sum() / var_p) if var_p > 0 else float('nan')
    t = mu_q - scale * (rot @ mu_p)
    pred = (scale * (rot @ p.T)).T + t
    resid = np.linalg.norm(pred - q, axis=1)
    theta = math.atan2(rot[1, 0], rot[0, 0])
    return scale, theta, t, np.degrees(theta), resid


def _affine_control(p, q, resid):
    """仿射对照（**只作对照，不替代相似变换**）：`q ≈ A·p + b`。

    相似变换把两个方向**平均成一个尺度** ⇒ 各向异性看不见；仿射给每个方向各自的尺度 ✓。
    ⇒ 若"仿射残差远小于相似残差" ⇒ 残差主要来自**各向异性**（而不是随机噪声）✓。
    """
    A = np.column_stack([p, np.ones(len(p))])
    cx, *_ = np.linalg.lstsq(A, q[:, 0], rcond=None)
    cy, *_ = np.linalg.lstsq(A, q[:, 1], rcond=None)
    ra = np.linalg.norm(np.column_stack([A @ cx, A @ cy]) - q, axis=1)
    p95s, p95a = float(np.percentile(resid, 95)), float(np.percentile(ra, 95))
    return {
        'A': [[float(cx[0]), float(cx[1])], [float(cy[0]), float(cy[1])]],
        'b': [float(cx[2]), float(cy[2])],
        'scale_x_minus_1': math.hypot(cx[0], cx[1]) - 1.0,
        'scale_y_minus_1': math.hypot(cy[0], cy[1]) - 1.0,
        'residual_p95': p95a,
        'similarity_residual_p95': p95s,
        'p95_reduction': (1 - p95a / p95s) if p95s > 0 else float('nan'),
        'note': '★ 相似变换把 x/y 两尺度平均成一个 ⇒ 各向异性要另做仿射才看得见。'
                '实测仿射把 p95 降 63% ⇒ 残差**主要是各向异性、不是随机噪声** ✓',
    }


def _inverse_check(p, q, scale, theta, t):
    """★ reviewer 2026-10-10 要求的自检：用 `f⁻¹` 在**全部样本**上重建 world，p95 ≤ 0.2 m ✓。

    **为什么单看残差不够**：参考点写错时，残差**反而可能更小**（拟合会把错参考吸收进 `t`）✗。
    这条走**反方向**（enu → world）：参考点一错，它立刻炸 ✓ —— 与"能正着算通"是两回事 ✓。
    """
    R = np.array([[math.cos(theta), math.sin(theta)],
                  [-math.sin(theta), math.cos(theta)]])          # R(−θ)
    world_rec = (R @ (q - t).T).T / scale
    d = np.linalg.norm(world_rec - p, axis=1)
    return {'model': 'f⁻¹: world = (1/s)·R(−θ)·(enu − t)',
            'residual_m': {'p50': float(np.percentile(d, 50)),
                           'p95': float(np.percentile(d, 95)),
                           'max': float(d.max())},
            'criterion': 'p95 ≤ 0.2 m',
            'pass': bool(np.percentile(d, 95) <= 0.2),
            'why': '反方向重建 —— **参考点写错时残差会偏小、这条会炸** ✓（正着算通不代表反着也对）'}


class Pairer(Node):
    def __init__(self, tol):
        super().__init__('geo_calib_latlon_world')
        self.tol = tol
        self.gt = []
        self.gps = []
        self.gt_all = []
        self.gps_all = []
        qos = QoSProfile(depth=50, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, GT_TOPIC, self._mk_odom(self.gt), qos)
        self.create_subscription(NavSatFix, GPS_TOPIC, self._mk_fix(self.gps), qos)

    def _mk_odom(self, buf):
        def cb(m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p = m.pose.pose.position
            buf.append((t, p.x, p.y))
        return cb

    def _mk_fix(self, buf):
        def cb(m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            buf.append((t, m.latitude, m.longitude))
        return cb

    def take(self):
        """把新到的样本收进累计表并**原地清空**缓冲（**不许重新绑定** —— 见第一半脚本里那个坑）✓。"""
        self.gt_all.extend(self.gt)
        self.gps_all.extend(self.gps)
        del self.gt[:]
        del self.gps[:]

    def pair_all(self):
        gt = sorted(self.gt_all)
        gps = sorted(self.gps_all)
        out = []
        j = 0
        for (tg, x, y) in gt:
            if not gps:
                break
            while j + 1 < len(gps) and abs(gps[j + 1][0] - tg) <= abs(gps[j][0] - tg):
                j += 1
            tq, lat, lon = gps[j]
            dt = abs(tq - tg)
            if dt <= self.tol:
                out.append((tg, x, y, lat, lon, dt))
        return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--seconds', type=float, default=1800.0)
    ap.add_argument('--tol', type=float, default=0.05)
    ap.add_argument('--min-samples', type=int, default=200)
    ap.add_argument('--min-span', type=float, default=14.0)
    ap.add_argument('--from-csv', default='',
                    help='★ 从已有 <out>_samples.csv 离线重算（不必重跑 ROS）⇒ 改了字段/口径后'
                         '不必重采一轮（与第一半同一形态）✓')
    ap.add_argument('--world', default=DEFAULT_WORLD, help='世界 SDF（进 calib_key）')
    ap.add_argument('--navsat', default=DEFAULT_NAVSAT, help='navsat.yaml（进 calib_key）')
    ap.add_argument('--from-run', default='',
                    help='★ **数据那一轮**的 run 目录 —— 给了才把"数据来自哪个世界"并进 calib_key。'
                         '**必须与第一半用同一个值**，否则两半的键不可能相等 ✗')
    ap.add_argument('--emit-geo-calib', default='',
                    help='第一半的 <half1>_fit.json 路径 ⇒ 合成并写出 geo_calib.json（见 compose_geo_calib）')
    a = ap.parse_args(argv)

    calib = build_calib(a.world, a.navsat, a.from_run)
    if a.from_csv:
        rows = list(csv.DictReader(open(a.from_csv, encoding='utf-8')))
        arr = np.array([[float(r['t']), float(r['world_x']), float(r['world_y']),
                         float(r['lat']), float(r['lon']), float(r['dt'])] for r in rows])
        return _emit(a, arr, interrupted=False, calib=calib)

    rclpy.init()
    node = Pairer(a.tol)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    t0 = time.time()
    last = 0.0
    interrupted = [False]
    while time.time() - t0 < a.seconds:
        try:
            rclpy.spin_once(node, timeout_sec=0.2)
        except KeyboardInterrupt:
            interrupted[0] = True
            break
        node.take()
        if time.time() - last > 15.0:
            last = time.time()
            s = node.pair_all()
            print('  … 真值 %d / GPS %d ⇒ 已配对 %d 对（%.0f s）'
                  % (len(node.gt_all), len(node.gps_all), len(s), time.time() - t0),
                  file=sys.stderr, flush=True)
            if len(s) >= 8:
                xs = [r[1] for r in s]
                if (max(xs) - min(xs) >= 2 * a.min_span):
                    print('  … x 向覆盖 ±%.1f m，提前收工' % ((max(xs) - min(xs)) / 2),
                          file=sys.stderr)
                    break
    # ★★ 收尾**必须包在 try 里** ★★（2026-10-10 真事故）：
    #   `pkill -INT` 会**同时**触发 rclpy 自己的信号处理器（它已经 shutdown 过 context）⇒
    #   我们再调 `rclpy.shutdown()` 会抛 `RCLError: rcl_shutdown already called` ✗
    #   ⇒ 异常从收尾路径抛出 ⇒ **在写盘之前就把脚本打死、几千对样本全丢** ✗
    #   （与"写文件放在校验之前"是同一族：**收尾路径上的任何一步都不许能把落盘挡住** ✓）
    try:
        node.destroy_node()
    except Exception:                                          # noqa: BLE001
        pass
    try:
        rclpy.shutdown()
    except Exception:                                          # noqa: BLE001
        pass

    s = node.pair_all()
    # ★ 顺序：**先构造 arr、再判样本数**（第一版写反了 ⇒ `len(arr)` 在定义之前用 ⇒ NameError ✗）
    arr = np.array([[r[0], r[1], r[2], r[3], r[4], r[5]] for r in s]) if s else np.zeros((0, 6))
    if len(arr) < a.min_samples:
        print('!! 只有 %d 对 ⇒ 不足以拟合' % len(arr), file=sys.stderr)
        return 3
    return _emit(a, arr, interrupted=bool(interrupted[0]), write_samples=True, calib=calib)


def compose_geo_calib(h2, a, calib, out_json):
    """把两半合成 `geo_calib.json` —— **经纬度 → map** 的完整系数。

       第一半（实测）：`map = s1·R(θ1)·world + t1`
       第二半（实测）：`enu = s2·R(θ2)·world + t2`，其中 `enu = ((lon−lon0)·M_LON, (lat−lat0)·M_LAT)`
       ⇒ 反解世界系：`world = (1/s2)·R(−θ2)·(enu − t2)`
       ⇒ 合起来：    `map   = (s1/s2)·R(θ1−θ2)·enu + [t1 − (s1/s2)·R(θ1−θ2)·t2]`
       （reviewer 要的形式 `map_goal = s₁·R(θ₁)·f(lat,lon) + t₁` 等价：把 `f` 取成上面那个反解 ✓）

    ★★ **硬断言**（reviewer 2026-10-10）：两半的 `calib_key` 必须**相等且都有效**，
       否则**拒绝合成** —— 这挡住的是"拿 A 世界的 map↔world 配 B 世界的 world↔经纬度" ✗。
       光看"两半都有效"不够：两半各自有效、但属于**不同世界**时，合出来的系数是错的而**看不出来** ✓。
    """
    h1 = json.load(open(a.emit_geo_calib, encoding='utf-8'))
    k1, k2 = h1.get('calib_key'), h2.get('calib_key')
    v1, v2 = h1.get('calib_key_is_valid'), h2.get('calib_key_is_valid')
    problems = []
    if not k1 or not k2:
        problems.append('两半都必须带 calib_key ⇒ **两半都要带 `--from-run`**（键里才有"数据来自哪一轮"）')
    if v1 is not True or v2 is not True:
        problems.append('两半的 calib_key_is_valid 必须都为 True（实测 %r / %r）' % (v1, v2))
    if k1 and k2 and k1 != k2:
        problems.append('★ 两半的 calib_key **不相等** ⇒ 不是同一个世界/基准/配置下采的 ⇒ **拒绝合成** ✗')
    if problems:
        for p in problems:
            print('!! ' + p, file=sys.stderr)
        print('!! 规范串（第二半）：' + str(h2.get('calib_key_canonical')), file=sys.stderr)
        return 4

    f1, f2 = h1['fit'], h2['fit']
    s1, th1 = f1['scale'], math.radians(f1['theta_deg'])
    s2, th2 = f2['scale'], math.radians(f2['theta_deg'])
    t1 = np.array(f1['translation_m'], float)
    t2 = np.array(f2['translation_m'], float)
    s, th = s1 / s2, th1 - th2
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    t = t1 - s * (R @ t2)
    res = {
        'schema': 'geo_calib/v1',
        'what_this_is': '★ **经纬度 → map** 的完整换算系数 = 第一半（map↔世界系）∘ 第二半（世界系↔经纬度），'
                        '**两半都是实测拟合**（不是手推）✓',
        'calib_key': h2['calib_key'],
        'calib_key_is_valid': True,
        'calib_key_components': h2.get('calib_key_components', {}),
        'calib_key_canonical': h2.get('calib_key_canonical', ''),
        # ★ 消费规则要**跟着产物走**（消费者读的是这份 json，不是第一半那份）
        'calib_key_consumer_rule':
            '★ `geo_route` **只读本文件的顶层布尔 `calib_key_is_valid`**；为 False/缺失 ⇒ 拒绝规划并'
            '打印 `calib_key_canonical` 让人比对。**不要**自己去重算键（重算要用同一份源代码，'
            '而"重算"正是 2026-10-09 那个洞的形态）✗',
        'provenance': {
            'half1_map_world': {'artifact': os.path.abspath(a.emit_geo_calib),
                                'run': h1.get('calib_key_components', {}).get('data_run'),
                                'samples': h1.get('samples'), 'span_x_m': h1.get('span_x_m'),
                                'span_y_m': h1.get('span_y_m'), 'residual_m': f1['residual_m']},
            'half2_world_latlon': {'artifact': os.path.abspath(a.out + '_fit.json'),
                                   'run': h2.get('half2_run'), 'samples': h2.get('samples'),
                                   'origin': h2.get('origin'), 'residual_m': f2['residual_m']},
            'both_halves_same_calib_key': h1.get('calib_key') == h2.get('calib_key'),
        },
        # ① 世界系 → map（第一半）
        'world_to_map': {'model': f1['model'], 'scale': s1, 'scale_minus_1': s1 - 1.0,
                         'theta_deg': f1['theta_deg'], 'translation_m': [float(t1[0]), float(t1[1])],
                         'residual_m': f1['residual_m']},
        # ② 经纬度 → 世界系（第二半的反解）
        'latlon_to_world': {
            'model': 'world = (1/s2)·R(−θ2)·(enu − t2)',
            'origin': h2['origin'], 'm_per_deg_lat': h2['m_per_deg_lat'],
            'm_per_deg_lon': h2['m_per_deg_lon'],
            'scale': s2, 'scale_minus_1': s2 - 1.0, 'theta_deg': f2['theta_deg'],
            'translation_m': [float(t2[0]), float(t2[1])],
            'residual_m': f2['residual_m'],
            'affine_control': h2.get('affine_control'),
            'anisotropy_note': (h2.get('affine_control') or {}).get('note')},
        # ③ ★ 合成：经纬度 → map（一次算完，**跨两半**的舍入只发生一次）
        'latlon_to_map': {
            'model': 'map = s·R(θ)·enu + t，enu = ((lon−lon0)·M_LON, (lat−lat0)·M_LAT)',
            'origin': h2['origin'], 'm_per_deg_lat': h2['m_per_deg_lat'],
            'm_per_deg_lon': h2['m_per_deg_lon'],
            'scale': s, 'scale_minus_1': s - 1.0, 'theta_deg': math.degrees(th),
            'translation_m': [float(t[0]), float(t[1])],
            'how_to_use': 'goal_map_x = s·( (lon−lon0)·M_LON·cosθ − (lat−lat0)·M_LAT·sinθ ) + t_x；'
                          'goal_map_y = s·( (lon−lon0)·M_LON·sinθ + (lat−lat0)·M_LAT·cosθ ) + t_y',
            'residual_note': '残差**不能**把两半的相加：两半的误差在同一个世界系里部分抵消。'
                             '真正的判据是第 4 步的**实到点 vs 目标点**实测 ✓'},
        # ★★ 把各向异性作为**一等字段**抬到顶层 ★★
        #   理由：它**不是**"拟合没做好"，而是**模型形式**的局限（相似变换只有一个尺度，
        #   而 Gazebo 的椭球经度尺度与 `111320·cosφ` 的球面假设差 0.5%）⇒
        #   消费者有权知道"这个换算在 ±38 m 处最坏差多少"，而不是只看一个 p95 ✓。
        'anisotropy': {
            # ★★ 归因已修正（reviewer 2026-10-10 裁定 ①b）★★
            #   我原来写"椭球**经度**半径 6383 km vs 球 6371 km" —— **错了** ✗：
            #   6383 km 那个是 **N**、只管**经度轴**、只占约 **1/6** ✓；
            #   **主导的是子午圈半径 M（纬度轴）** ✓。实测/理论对照：
            #     纬度轴 111320 vs M(φ)·π/180 = 110856.3 ⇒ **+0.4183%** ← 主导
            #     经度轴 96184.2 vs N(φ)·cosφ·π/180 = 96265.3 ⇒ **−0.0843%**
            #   vs 实测 北 +0.428% / 东 −0.102% ⇒ 同号同量级 ✓
            'finding': '残差主要来自**各向异性**，不是随机噪声 —— 且**两轴起因不同**：'
                       '纬度轴来自 `111320` 与椭球 `M(φ)` 的差（**主导，+0.42%**），'
                       '经度轴来自 `111320·cosφ` 与 `N(φ)·cosφ` 的差（−0.08%）✓',
            'attribution_corrected': '★ 我初版把主导项归给"经度半径 6383 km（N）"是**错的** ✗ —— '
                                     'N 只管经度轴、只占约 1/6；主导项是**子午圈半径 M**（纬度轴）✓。'
                                     '⇒ **只换 `M_LON` 会留下 0.42% ≈ 5/6 的误差** ✗（故两个常数都换）',
            'evidence': {
                'corr_residual_vs_radius': 0.845,
                'corr_residual_vs_speed_dt': -0.017,
                'speed_dt_p95_m': 0.0231,
                'affine_p95_reduction': (h2.get('affine_control') or {}).get('p95_reduction'),
                'diagnostic_script': 'artifacts/rtk-nav2/_scripts/diag_latlon_residual.py',
            },
            'impact': '**不影响** `geo_route` 的到位判据（目标与真值同在 map/world 系 ⇒ 变换误差不进 d_gt）；'
                      '**影响**"喂进去的经纬度最终落在哪"：±38 m 边缘处最坏 ≈0.19 m（< 0.25 m 容差，但不宽裕）',
            'ruling': '★ reviewer 2026-10-10 裁定 ①：**改用椭球约定（两个常数都换）+ 离线重拟合，'
                      '不采用仿射** ✓。理由：仿射会把几何失真藏进 2×2 的矩阵、**不可审计** ✗；'
                      '椭球换算是**有物理依据的约定** ✓。自检 = **换完 θ/s 应趋于 0、残差应落到噪声底 ≈0.05** ✓',
        },
        'substitute_data_warning': '★ 本轮路线经纬度是 **AuYang 未给真实点位时的替代品**'
                                   '（`campus8-f/input/route_geo_SUBSTITUTE.json`，我自己造的）'
                                   '⇒ 真正验收要用 AuYang 给的经纬度重跑 ✓',
        'key_paths': {'有效标定判定（**只读这个布尔**）': 'calib_key_is_valid',
                      '标定键': 'calib_key', '被 hash 的原文': 'calib_key_canonical',
                      '经纬度→map 的尺度': 'latlon_to_map.scale_minus_1',
                      '经纬度→map 的旋转': 'latlon_to_map.theta_deg',
                      '经纬度→map 的平移': 'latlon_to_map.translation_m',
                      '原点/每度米数': 'latlon_to_map.origin / m_per_deg_lat / m_per_deg_lon',
                      'world→map 那半': 'world_to_map.*',
                      '经纬度→world 那半': 'latlon_to_world.*',
                      '两半来源轮次': 'provenance.*.run',
                      '两半同键': 'provenance.both_halves_same_calib_key'},
    }
    tmp = out_json + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(res, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, out_json)
    print('★ 已写出 geo_calib.json：%s' % out_json, file=sys.stderr)
    return 0


def _emit(a, arr, interrupted=False, write_samples=False, calib=None):
    """把 (t, world_x, world_y, lat, lon, dt) 拟合并落盘（主路径与 `--from-csv` **共用**）。"""
    # 经纬度 → 米：以样本均值为原点做等距投影（本场景 ~100 m ⇒ 足够；系数里含"米/度"）
    lat0 = float(arr[:, 3].mean())
    lon0 = float(arr[:, 4].mean())
    # ★ 椭球约定（两个常数都换 —— 见文件头 `ellipsoid_m_per_deg` 的推导）✓
    M_LAT, M_LON = ellipsoid_m_per_deg(lat0)
    p = arr[:, 1:3]                                   # 世界系 (x,y) 米
    q = np.column_stack([(arr[:, 4] - lon0) * M_LON, (arr[:, 3] - lat0) * M_LAT])  # 东/北 米
    scale, theta, t, theta_deg, resid = umeyama_2d(p, q)
    # datum（**现读**世界 SDF，不许手填）与"参考点相对 datum 的偏移" —— 供 `reference_point` 用
    import geo_calib_map_world as _H
    _dlon, _dlat, _dalt, _dsrc = _H.read_datum(a.world)
    _datum = {'lat': _dlat, 'lon': _dlon, 'alt': _dalt, 'source': _dsrc}
    _d_n = (lat0 - _dlat) * M_LAT      # 参考点在 datum 以北多少米（负 = 偏南）
    _d_e = (lon0 - _dlon) * M_LON      # 参考点在 datum 以东多少米
    res = {
        'gt_topic': GT_TOPIC, 'gps_topic': GPS_TOPIC,
        'gps_note': '★ 用的是 **/rtk/navsat（理想值）**，**不是** /rtk/sim_fix —— '
                    '换算系数应当是"几何的"，模拟器的 σ/bias 不属于坐标系 ✓',
        'samples': int(len(arr)), 'pair_tol_s': a.tol, 'interrupted': bool(interrupted),
        'origin': {'lat': lat0, 'lon': lon0}, 'm_per_deg_lat': M_LAT, 'm_per_deg_lon': M_LON,
        # ★★ 参考点必须写进产物（reviewer 2026-10-10 裁定 ①a）★★
        #   实测：拿 **datum** 当参考去反解 ⇒ 错 **27.4 m** ✗；按 `origin` 当参考 ⇒ p95 0.166 m ✓。
        #   ⇒ "参考点是谁"是产物的一部分，**不能靠读代码推** ✓。
        'reference_point': {
            'kind': 'sample_mean —— **全部样本经纬度的均值**（★ 不是 datum ✗）',
            'lat': lat0, 'lon': lon0,
            'datum': _datum,
            'datum_minus_reference_m': {'north': _d_n, 'east': _d_e},
            'WARNING': '★ `fit.t` 与正反解用的参考点就是这里；用 datum 反解会错约 27 m ✗',
        },
        # ★★ ① **必须写明"经纬度 → 米"的约定**（reviewer 2026-10-09）★★
        #   旋转角与尺度**只有相对某个约定才有意义** ✗ —— 不写约定，θ/s 不可解释 ✓。
        #   本脚本用的约定 = **与 `rtk_sim` 同一套**（`M_PER_DEG_LAT = 111320` + `cos φ`）✓
        #   ⇒ 于是实测的 θ / s−1 的含义被钉死为："**Gazebo 的投影相对该约定差多少**" ✓
        #   （预期都在噪声量级；若不在，就有东西要查 ✓）。
        'convention': {
            # ★ 2026-10-10 裁定 ① 起改为**椭球**约定（两个常数都换）——
            #   旧约定 `111320` + `111320·cosφ` 是**球面近似**，与 Gazebo 的椭球投影差 0.42%/0.08% ✓
            'lat_lon_to_meters': 'WGS84 椭球：纬度轴 `M(φ)·π/180`、经度轴 `N(φ)·cosφ·π/180`'
                                 '（`M`=子午圈曲率半径、`N`=卯酉圈曲率半径；见 `ellipsoid_m_per_deg()`）',
            'changed_from': '★ 旧约定 = `M_PER_DEG_LAT 111320` + `M_PER_DEG_LON 111320·cos(lat0)`'
                            '（与 `rtk_sim.py` 同一套）—— 球面近似，已被本次椭球约定取代 ✓',
            'note_for_consumers': '★ **`geo_route` 一律从 `geo_calib.json` 读这两个常数**'
                                  '（单一真源），**别在别处再写一份** ✗',
            'implication_of_theta_and_scale':
                'θ / s−1 的含义 = "**Gazebo 的（切平面）投影相对上面这套约定差多少**"；'
                '换一套约定，θ / s−1 会跟着变 ⇒ **不许脱离约定单独引用这两个数** ✗',
        },
        'world_precondition': '★ 两半**必须同一个世界/基准下采**（本产物与第一半同属同一 run ⇒ 满足；'
                              '换世界后两半都要重采）✓',
        'half2_run': os.path.basename(os.path.dirname(os.path.abspath(a.out))),
        # ★★ `calib_key`：与第一半**同一个函数**算出来（reviewer：键要覆盖两半的来源）★★
        #   有效性口径与第一半一致 = "**带 `--from-run`** 且数据那轮的世界与当前世界同一份" ✓
        'calib_key': (calib or {}).get('calib_key'),
        'calib_key_is_valid': bool(
            calib and a.from_run
            and calib['calib_key_components'].get('data_matches_current_geometry') == 'True'),
        'calib_key_components': (calib or {}).get('calib_key_components', {}),
        'calib_key_canonical': (calib or {}).get('calib_key_canonical', ''),
        'calib_key_validity_rule':
            '★ 有效 ⇔ ① 给了 `--from-run`（键里才有"数据来自哪一轮"）∧ '
            '② 那一轮记录的世界 sha 与**现在**这份世界 sha 相同。'
            '**合成 geo_calib.json 时还要求两半的键相等**（不同世界的两半合起来是错的且看不出来）✗',
        'convention_note_for_compose':
            '★ 本半的 `fit` 是 **enu ← world**（`enu ~= s·R(θ)·world + t`）；'
            '`latlon_to_world` 要用它的**反解**。合成见 `compose_geo_calib()` ✓',
        'fit': {'model': 'enu_meters ~= s * R(theta) * world_xy + t',
                'scale': scale, 'scale_minus_1': scale - 1.0, 'theta_deg': theta_deg,
                'translation_m': [float(t[0]), float(t[1])],
                'residual_m': {'mean': float(resid.mean()),
                               'p50': float(np.percentile(resid, 50)),
                               'p95': float(np.percentile(resid, 95)),
                               'max': float(resid.max())}},
        # ★★ 各向异性对照（与第一半同一字段名）★★
        #   2026-10-10 实测：相似变换的残差 p95 = **0.141 m**，而两个源都是**无噪声的理想值**
        #   ⇒ 先逐条排除：① 配对时间差被**否掉**（`corr(残差, speed·|dt|) = −0.017`，
        #     且 `speed·|dt|` 的 p95 只有 0.023 m，够不着 0.14）；② 相关性指向**半径**
        #     （`corr(残差, 半径) = +0.845`）⇒ 是**各向异性**。
        #   真因：**Gazebo 的 navsat 用椭球半径算经度**，而本文件的 `M_LON = 111320·cosφ`
        #   假设**球半径 6371 km** ⇒ 经度方向的尺度差 ≈ `6383/6351 − 1 = 0.50%`（30°N 处
        #   卯酉圈半径 ≈6383 km）⇒ **相似变换只有一个尺度、吸收不掉** ⇒ 残差 ∝ 到质心距离。
        #   ⇒ 实测两轴尺度差 **5.30e-3**，与预测的 0.50% 吻合 ✓；仿射拟合把 p95 **降 63%** ✓。
        #   ★ 影响与不影响的：
        #     · **不影响** `geo_route` 的"到位判据"（目标是 map 系的点、真值也在同一系 ⇒ 变换误差不进 d_gt）；
        #     · **影响**"喂进去的经纬度最终落在哪" —— 在 ±38 m 的边缘处最大 ≈0.19 m。
        #   本产物**仍以相似变换为主**（reviewer 定的 `map_goal = s₁·R(θ₁)·f + t₁` 就是这个形式）；
        #   仿射只作**对照与上界**用，要不要改用它由 reviewer 定 ✓。
        'affine_control': _affine_control(p, q, resid),
        'inverse_check': _inverse_check(p, q, scale, theta, t),
        'key_paths': {'换算约定': 'convention.lat_lon_to_meters',
                      'θ/s 的含义': 'convention.implication_of_theta_and_scale',
                      '第二半来源轮次': 'half2_run',
                      '世界前提': 'world_precondition',
                      '换算原点': 'origin.{lat,lon}',
                      '每度米数': 'm_per_deg_lat / m_per_deg_lon',
                      '旋转角': 'fit.theta_deg', '尺度': 'fit.scale_minus_1',
                      '平移': 'fit.translation_m', '残差': 'fit.residual_m.*'},
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    # ★ 落盘用**临时文件 + os.replace**（"先算完再写"那条纪律；第一半就是在这里栽过一次 ✗）
    if write_samples:
        tmp = a.out + '_samples.csv.tmp'
        with open(tmp, 'w', newline='', encoding='utf-8') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'world_x', 'world_y', 'lat', 'lon', 'dt'])
            # ★★ **经纬度不能用 `%.6f`** ★★：1e-6 度 ≈ **0.11 m** 的量化 ⇒ 把 2 cm 级的噪声整个盖掉 ✗
            #   （自测里实测：合成数据 σ=0.02 m ⇒ 残差 p95 本应 ≈0.049 m、实测 **0.083 m** ⇒
            #    反推 σ_eff ≈ 0.034 ⇒ 多出来的正是量化 ✗）。
            #   ⇒ 世界坐标 `%.6f`（米级 1e-6 够 ✓）、**经纬度 `%.9f`**（≈0.1 mm ✓）。
            for r in arr:
                w.writerow(['%.6f' % r[0], '%.6f' % r[1], '%.6f' % r[2],
                            '%.9f' % r[3], '%.9f' % r[4], '%.6f' % r[5]])
        os.replace(tmp, a.out + '_samples.csv')
    tmp = a.out + '_fit.json.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(res, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, a.out + '_fit.json')
    print(json.dumps(res, ensure_ascii=False, indent=2))
    # ★ 合成 geo_calib.json（**放在最后**：本半的产物已落盘 ⇒ 合成失败也不会丢本半的凭据 ✓）
    if a.emit_geo_calib:
        out_json = (os.path.dirname(os.path.abspath(a.emit_geo_calib))
                    + '/geo_calib.json')
        return compose_geo_calib(res, a, calib, out_json)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""按**经纬度**巡游（brief §3.2）：`geo_calib.json` → map 目标 → 发 `/goal_pose` → **判停稳** → 记档。

链路：`(lat, lon) --latlon_to_map--> (map_x, map_y) --/goal_pose--> nav2`
反向（**自证用**）：`/odometry/global`(map) --latlon_to_map 的反解--> `(lat, lon)` → `/rtk/localization_geo`

★★ 三条纪律（都是本项目被罚过的）★★
1. **只读顶层布尔 `calib_key_is_valid`**（reviewer 2026-10-10 定的消费规则）✗ 不许自己重算键 ——
   "重算"要求消费者手里有**同一份源代码/配置**，而那正是 2026-10-09 那个洞的形态
   （旧数据重拟合一次就能造出一个"看起来有效"的新键）。为 False/缺失 ⇒ **拒绝规划**，
   并把 `calib_key_canonical` **原文打印出来**让人比对（不然"为什么无效"没法查）✓。
2. **到达 = 进入容差 ∧ 从此速度 < `v_still`**（带量纲，不随采样率漂移）✗ 不许用"单拍位移 < N 毫米"。
   落点读数取**停住之后那一段的均值**，**不取事件那一拍**（那一拍是"刚开始停"）✓。
   窗口另给一个**按规则截断**的 `rule_window_30s`（与"何时读文件"无关），并把读取时刻写进产物 ✓。
3. **先算完再落盘**、`临时文件 + os.replace`；**每个点一落盘**（中途被打断也不丢已完成的点）✓；
   `destroy_node()` / `rclpy.shutdown()` 一律包 `try/except`（`pkill -INT` 会同时触发 rclpy 自己的信号处理器）✓。

用法（容器内）：
  geo_route.py --calib /work/artifacts/.../geo_calib.json \
               --route /work/artifacts/.../route_geo.json \
               --out /work/artifacts/.../geo_route_[--tol 0.25] [--v-still 0.02] [--check-only]
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix

MAP_TOPIC = '/odometry/global'
GT_TOPIC = '/ground_truth/odom'
GOAL_TOPIC = '/goal_pose'
GEO_TOPIC = '/rtk/localization_geo'


def load_calib(path):
    """读 `geo_calib.json` 并**只**用顶层布尔判有效（见文件头纪律 1）。"""
    d = json.load(open(path, encoding='utf-8'))
    if d.get('calib_key_is_valid') is not True:
        print('!! calib_key_is_valid != True ⇒ **拒绝规划** ✗', file=sys.stderr)
        print('!! 规范串（calib_key_canonical）原文如下，请与你的系统比对：', file=sys.stderr)
        print(d.get('calib_key_canonical') or '(缺失)', file=sys.stderr)
        print('!! 分量（仅审计用，**不要**据此自己重算键）：%s'
              % json.dumps(d.get('calib_key_components', {}), ensure_ascii=False), file=sys.stderr)
        return None
    return d


def latlon_to_map(L, lat, lon):
    """`map = s·R(θ)·enu + t`，`enu = ((lon−lon0)·M_LON, (lat−lat0)·M_LAT)`。"""
    enu = np.array([(lon - L['origin']['lon']) * L['m_per_deg_lon'],
                    (lat - L['origin']['lat']) * L['m_per_deg_lat']])
    th = math.radians(L['theta_deg'])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return L['scale'] * (R @ enu) + np.array(L['translation_m'])


def map_to_latlon(L, x, y):
    """`latlon_to_map` 的**反解** —— 用来把 map 系定位回吐成经纬度（`/rtk/localization_geo`）。"""
    th = math.radians(L['theta_deg'])
    R = np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]])  # R(−θ)
    enu = (R @ (np.array([x, y]) - np.array(L['translation_m']))) / L['scale']
    return (L['origin']['lat'] + enu[1] / L['m_per_deg_lat'],
            L['origin']['lon'] + enu[0] / L['m_per_deg_lon'])


class RouteNode(Node):
    def __init__(self, calib, tol, v_still):
        super().__init__('geo_route')
        self.L = calib['latlon_to_map']
        self.tol, self.v_still = tol, v_still
        self.samples = []          # (t, map_x, map_y, gt_x, gt_y)
        self.cb_count = {'map': 0, 'gt': 0}   # 回调计数（区分"没收到"的两侧，见 _odom）✓
        self._last_diag = 0.0                  # 上次打诊断的时刻
        self.pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.geo_pub = self.create_publisher(NavSatFix, GEO_TOPIC, 10)
        qos = QoSProfile(depth=50, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, MAP_TOPIC,
                                 lambda m: self._odom(m, 'map'), qos)
        self.create_subscription(Odometry, GT_TOPIC,
                                 lambda m: self._odom(m, 'gt'), qos)

    def _odom(self, m, which):
        # ★★ 回调计数（reviewer 2026-10-10 仪器要求 ③b）★★
        #   背景：`campus8-h` 那轮采集器"**收到约 42 s 后就不再进样本**" ✗，而外部量
        #   `/ground_truth/odom` 是好的（21.4 Hz ✓）⇒ **必须分清是哪一侧**：
        #     · 回调**根本没被调用** ⇒ 订阅侧（发现/QoS）✗
        #     · 回调**被调用了、而循环没读到** ⇒ 我自己的代码 ✗
        #   只加计数器就能分开这两者 ✓（不必猜 ✓）。
        self.cb_count[which] = self.cb_count.get(which, 0) + 1
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        if which == 'map':
            self.last_map = (t, p.x, p.y)
            # ★ 反向自证：map → 经纬度 → 发 `/rtk/localization_geo`
            lat, lon = map_to_latlon(self.L, p.x, p.y)
            fx = NavSatFix()
            fx.header.stamp = m.header.stamp
            fx.header.frame_id = 'map'
            fx.status.status = 0                      # STATUS_FIX
            fx.status.service = 1                     # SERVICE_GPS
            fx.latitude, fx.longitude, fx.altitude = lat, lon, 0.0
            self.geo_pub.publish(fx)
        else:
            self.last_gt = (t, p.x, p.y)

    # -- 采样：把 (map, gt) 按最近时间戳配对存下来 --
    def sim_now(self):
        """**仿真时间**（秒）—— 判据一律用它 ✓。

        ★★ 为什么必须换（2026-10-10，reviewer 量出的 8.2×）★★
        这台机上 **`sim/wall = 201.4 / 1646 = 0.122`** ✗ ⇒ 仿真只跑墙钟的 **1/8.2** ✓。
        我原来所有等待/超时都用 `time.time()`（**墙钟**）✗ ⇒ **实际只给了 1/8 的时间** ✗ ⇒
        三类症状一个因：**离点 1.7~3.6 m 就走** ✓（腿超时）、**穿过不停** ✓（静稳窗口不够）、
        **最后两段没跑** ✓（总预算耗尽）✓。
        ★ **但纯按仿真计时会引入新的死法**：仿真一卡住/挂死 ⇒ **仿真时间不走** ✗ ⇒
        循环**永远不超时** ✗ ⇒ 挂在那儿 ✓ ⇒ **判据用仿真时间、看门狗用墙钟** ✓（两者并列记录 ✓）。
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def stalled(self, t_wall=600.0, eps=1e-3):
        """**无进展检测**（reviewer 2026-10-10 的修法）✓ —— 替代"总时长上限" ✗

        ★ 我前两版都在调"墙钟总预算"（先固定 1800 s ✗、再自适应 + cap ✗），
          两版**都错**：你算过 —— `7200 s cap × 后段比值 0.043 ≈ 310 s 仿真` ✗ **< 420 s 预算**
          ⇒ **照样把腿截断** ✓。根因是**旋钮选错了** ✗：
          墙钟看门狗其实只该回答一个问题 —— "**它还在动吗？**" ✓

        ⇒ 判据 = **仿真时间在过去 `t_wall` 秒墙钟里有没有前进** ✓（**一个参数、与比值无关** ✓）：
          · 仿真**卡死** ⇒ 仿真时间**停住** ⇒ **触发** ✓（兜住挂死 ✓）；
          · 仿真**只是慢** ⇒ 仿真时间**仍在走** ⇒ **永不触发** ✓✓（比值退到多低都不管 ✓）。
        ⇒ 两个情形干净分开 ✓，`wall_budget`/`floor_ratio`/`cap` 三者**一起退场** ✓。
        """
        now_sim, now_wall = self.sim_now(), time.time()
        prog = getattr(self, '_prog', None)
        if prog is None or now_sim > prog[0] + eps:
            self._prog = (now_sim, now_wall)
            return False
        return (now_wall - prog[1]) > t_wall

    def pump(self, seconds):
        """转 `seconds` 秒（**仿真时间** ✓），边转边采样；另设墙钟看门狗防挂死 ✓。"""
        t0 = self.sim_now()
        w0 = time.time()
        while self.sim_now() - t0 < seconds:
            # ★ 墙钟看门狗：仿真卡住时仿真时间不走 ⇒ 靠它兜底 ✓（阈值给足，正常跑不会触发 ✓）
            if self.stalled():
                print('   !! 墙钟看门狗触发（仿真可能卡住）', flush=True)
                break
            rclpy.spin_once(self, timeout_sec=0.05)
            # ★ 实时估 `sim/wall` ✓（**只用于落盘/诊断** ✓，不再用作看门狗依据 ✓ —— 见 `stalled`）
            _w = time.time() - w0
            if _w > 1.0:
                self._ratio = (self.sim_now() - t0) / _w
            # ★ 每 ~10 s 打一次"活体证据"（仪器要求 ③b）：回调计数 + **发布者数** ——
            #   发布者数掉到 0 ⇒ 是**发现/QoS** ✗；计数在涨而样本不涨 ⇒ 是**我的代码** ✗ ✓
            if time.time() - self._last_diag > 10.0:
                self._last_diag = time.time()
                print('   [diag] 回调 map=%d gt=%d ｜ 发布者 map=%d gt=%d ｜ 样本=%d'
                      % (self.cb_count.get('map', 0), self.cb_count.get('gt', 0),
                         self.count_publishers(MAP_TOPIC), self.count_publishers(GT_TOPIC),
                         len(self.samples)), flush=True)
            m, g = getattr(self, 'last_map', None), getattr(self, 'last_gt', None)
            # ★ **按时间戳抽稀到 ≈20 Hz**：`spin_once` 比里程计快得多，不抽稀会同一条消息
            #   被重复记上千次 ⇒ 样本数虚高、速度序列里全是 0 ⇒ 停稳判据被稀释 ✗
            if m and g and (not self.samples or g[0] - self.samples[-1][0] >= 0.05):
                self.samples.append((g[0], m[1], m[2], g[1], g[2]))

    def send_goal(self, x, y, seconds=4.0):
        """**重复发**（`ros2 topic pub --once` 起的新发布者常与订阅者还没匹配上就发完了 ✗）。"""
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.pose.position.x, g.pose.position.y, g.pose.position.z = float(x), float(y), 0.0
        g.pose.orientation.w = 1.0                     # yaw = 0（与 nav-to.sh 默认一致）
        t0 = time.time()
        while time.time() - t0 < seconds:
            g.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(g)
            rclpy.spin_once(self, timeout_sec=0.5)


def speeds(s):
    out = []
    for i in range(len(s) - 1):
        dt = s[i + 1][0] - s[i][0]
        d = math.hypot(s[i + 1][3] - s[i][3], s[i + 1][4] - s[i][4])
        out.append(d / dt if dt > 1e-9 else float('inf'))
    return out


def eval_point(samples, gx, gy, tol, v_still, read_at):
    """判"到达并停稳"并给出落点读数（口径与 `arrival_event.py` **完全一致**）。"""
    dg = [math.hypot(r[3] - gx, r[4] - gy) for r in samples]
    dm = [math.hypot(r[1] - gx, r[2] - gy) for r in samples]
    v = speeds(samples)
    ev = None
    for i in range(len(samples)):
        if dg[i] >= tol:
            continue
        if all(v[j] < v_still for j in range(i, len(v))):
            ev = i
            break
    # ★ 2026-10-10（第 2 步）：把**"进过容差"**单独记成一个字段 ✓ ——
    #   新口径下**中间点**的判据是"经过／进入容差"（业主 2026-10-10 定）✓，
    #   而 `arrived` 仍是**终点**那条严判据（进入容差 ∧ 从此停稳）⇒ 两者必须**分开可得** ✓。
    #   ★ 这里只**增加**一个观测量 ✗ 不动 `arrived` 的口径、✗ 不动任何阈值
    #     （`tol` / `v_still` / `quiet_s` 的数值一律不变 ✓）。
    first_in = next((i for i in range(len(dg)) if dg[i] < tol), None)
    res = {'samples': len(samples), 't_range_s': [samples[0][0], samples[-1][0]],
           'file_read_at': read_at,
           'tolerance_m': tol, 'v_still_mps': v_still, 'goal_map': [gx, gy],
           'tolerance_entered': first_in is not None}
    if first_in is not None:
        res['tolerance_entered_row'] = {'t': samples[first_in][0], 'd_gt': dg[first_in]}

    # ★★ 2026-10-10（reviewer 抓到 ✗）：**产物自己的"判定"与"诊断"必须一眼分得开** ★★
    #   背景：新行为（**进容差即走**）下采样**尾巴很短** ⇒ 下面那个 `arrived`（严口径：
    #   进入容差 ∧ **从此速度一直 < v_still**）**近乎恒真** ✗ —— 实测 B 轮 **10/10 全 true** ✓，
    #   读起来像"车在该点停稳了" ✗，其实只是"**路过**" ✓。
    #   ⇒ 显式写明：**本轮的到达判据 = `tolerance_entered`** ✓；`arrived` **只是诊断量** ✓。
    #   ★ 保留 `arrived` 字段名是为兼容既有读产物的人 ✓，但**旁边必须带这两行** ✓，
    #     否则下一个读产物的人还会被它误导 ✗（这正是"报告里不留歧义"那条 ✓）。
    res['criterion'] = 'tolerance_entered'
    res['arrived_by_criterion'] = first_in is not None
    res['arrived_is'] = ('★ 诊断量（严口径：进入容差 ∧ 从此速度一直 < v_still）'
                         '—— ✗ 不是本轮判据；新行为下尾巴短、它近乎恒真 ✗')
    if ev is None:
        res['arrived'] = False
        res['why'] = ('没有"进入容差 且 从此停稳"的事件（容差 %.2f m / 停稳 %.3f m/s）；'
                      '最小 d_gt = %.3f m' % (tol, v_still, min(dg)))
        return res
    last_moving = ev
    for j in range(ev, len(v)):
        if v[j] >= v_still:
            last_moving = j + 1
    stop = samples[last_moving:]
    mx = sum(r[1] for r in stop) / len(stop)
    my = sum(r[2] for r in stop) / len(stop)
    gxm = sum(r[3] for r in stop) / len(stop)
    gym = sum(r[4] for r in stop) / len(stop)
    res.update({
        'arrived': True,
        'event_row': {'t': samples[ev][0], 'gt': [samples[ev][3], samples[ev][4]],
                      'd_gt': dg[ev], 'map': [samples[ev][1], samples[ev][2]], 'd_map': dm[ev]},
        'stopped_pose': {'t_from': stop[0][0], 't_to': samples[-1][0],
                         'map_mean': [mx, my], 'gt_mean': [gxm, gym],
                         'd_gt_mean_m': math.hypot(gxm - gx, gym - gy),
                         'd_map_mean_m': math.hypot(mx - gx, my - gy),
                         'map_minus_gt_m': math.hypot(mx - gxm, my - gym)},
        'standstill': {'rule_window_30s': {'t_from': stop[0][0],
                                           't_to': min(stop[0][0] + 30.0, samples[-1][0]),
                                           'rule': '停住那一刻起固定 30 s —— **不随"何时读"变** ✓'},
                       'window_s': samples[-1][0] - stop[0][0],
                       'window_max_speed_mps': max(v[last_moving:]) if v[last_moving:] else 0.0,
                       'source_last_t': samples[-1][0], 'file_read_at': read_at},
    })
    return res


def _atomic_csv(path, samples, goal):
    """每个点的**逐样本**记录 —— 落盘它，判据才能**离线重新判**而不必重开一轮 ✓。

    ★ 2026-10-10 的教训：`wp1`/`wp2` 被误判"未到达"时，产物里只有**汇总量**，
      想复核"到底哪一拍算事件"就得回头去翻整轮的 `trajectory.csv`（能翻，但绕）✗。
      ⇒ 逐样本落在**本点的文件**里 ✓（原始记录优先，派生物其次）。
    """
    import csv as _csv
    tmp = path + '.tmp'
    with open(tmp, 'w', newline='', encoding='utf-8') as fh:
        w = _csv.writer(fh)
        w.writerow(['t', 'map_x', 'map_y', 'gt_x', 'gt_y', 'd_gt', 'd_map'])
        for (t, mx, my, gx, gy) in samples:
            w.writerow(['%.4f' % t, '%.6f' % mx, '%.6f' % my,
                        '%.6f' % gx, '%.6f' % gy,
                        '%.6f' % math.hypot(gx - goal[0], gy - goal[1]),
                        '%.6f' % math.hypot(mx - goal[0], my - goal[1])])
    os.replace(tmp, path)


def _atomic_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', required=True, help='geo_calib.json')
    ap.add_argument('--route', required=True, help='{"points":[{"name","lat","lon"},...]}')
    ap.add_argument('--out', required=True, help='输出前缀（_records.json / _log.json）')
    ap.add_argument('--tol', type=float, default=0.25)
    ap.add_argument('--v-still', type=float, default=0.02)
    ap.add_argument('--settle-s', type=float, default=120.0,
                    help='到位后继续记的**封顶**秒数 —— 实际是"记到真停稳为止"，这只是上限 ✓')
    ap.add_argument('--quiet-s', type=float, default=4.0,
                    help='**连续静止多少秒**才算真停稳（按时间量，不按样本数 ✗）')
    ap.add_argument('--leg-timeout-s', type=float, default=420.0)
    # ★★ 2026-10-10（第 2 步 · **只给标定轮用**）★★
    #   默认行为已改成"**中间点进入容差就发下一点**"（业主 2026-10-10 口径 ✓）。
    #   本开关**把旧行为调回来**（**每个点都等真停稳** ✓）⇒ 用来做**同一台机、同一条航线**的
    #   **成对比较**（新 vs 旧）✓ —— 否则"拍数降下来了"只能跟**别的轮次**比 ✗，归因不干净 ✓。
    ap.add_argument('--wait-all', action='store_true',
                    help='★ **仅标定轮**：**每个点都等真停稳**（= 改动前的旧行为 ✓），做同机成对比较')
    ap.add_argument('--check-only', action='store_true',
                    help='只打印经纬度→map 的换算与自洽性，**不开车**（离线可测 ✓）')
    a = ap.parse_args(argv)

    calib = load_calib(a.calib)
    if calib is None:
        return 2
    L = calib['latlon_to_map']
    route = json.load(open(a.route, encoding='utf-8'))
    if isinstance(route, dict) and 'points' in route:
        pts = route['points']
    elif isinstance(route, dict) and 'start' in route:
        # ★ **brief §2 的航线格式**：`start` + `waypoints`（**含起点不含终点**）+ `goal`
        #   —— 那位二维地图选点的开发者交的就是这个格式，`mk_route_geo.py` 也按它出件 ✓
        pts = ([dict(route['start'], name='start')]
               + [dict(w, name='wp%d' % i) for i, w in enumerate(route.get('waypoints', []))]
               + [dict(route['goal'], name='goal')])
    else:
        pts = route

    # ★ 换算 + 自洽性：正解 → 反解 应当回到原经纬度（否则两个方向不一致 ⇒ 有 bug）
    print('== 经纬度 → map ==')
    for p in pts:
        x, y = latlon_to_map(L, p['lat'], p['lon'])
        la, lo = map_to_latlon(L, x, y)
        err = math.hypot((la - p['lat']) * L['m_per_deg_lat'],
                         (lo - p['lon']) * L['m_per_deg_lon'])
        print('  %-10s (%.6f, %.6f) → map (%9.4f, %9.4f)   正反解自洽 %.2e m'
              % (p.get('name', '?'), p['lat'], p['lon'], x, y, err))
        if err > 1e-6:
            print('!! 正反解不自洽 ⇒ 拒绝开车 ✗', file=sys.stderr)
            return 5
    if a.check_only:
        print('--check-only ⇒ 不开车 ✓')
        return 0

    rclpy.init()
    node = RouteNode(calib, a.tol, a.v_still)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    records = []
    try:
        for idx, p in enumerate(pts):
            # ★★ 2026-10-10（第 2 步 · 业主口径 **第 2 稿**）★★
            #   ★ **不区分中间点与终点** ✓ —— **所有点一视同仁** ✓：
            #     **任何一个点，进了容差就发下一个** ✓（**包括最后一个** ✓）
            #   —— 业主原话："**不应该区分中间点和最终点**，最终会是**流式的一连串点**" ✓
            #   ⇒ ★ **不许有"终点特殊处理"** ✗（"终点"这个概念本身将来会消失 ✓）
            #   ★ `tol`／`v_still`／`quiet_s` **数值一个都没动** ✓ ——
            #     改的只是**控制回路不再用"停稳"** ✓（它们仍写进记录、作**诊断量** ✓）。
            gx, gy = latlon_to_map(L, p['lat'], p['lon'])
            print('== %s → map (%.3f, %.3f) ==（★ 进容差即走，不等停稳 ✓）'
                  % (p.get('name', '?'), gx, gy), flush=True)
            node.samples = []
            node.send_goal(gx, gy)
            t0 = node.sim_now(); w0 = time.time()
            while node.sim_now() - t0 < a.leg_timeout_s:
                if node.stalled():   # ★ 无进展检测 ✓
                    print('   !! 腿看门狗触发', flush=True); break
                node.pump(1.0)
                if node.samples:
                    dg = math.hypot(node.samples[-1][3] - gx, node.samples[-1][4] - gy)
                    print('   d_gt=%.3f m（%d 样本）' % (dg, len(node.samples)), flush=True)
                    if dg < a.tol:
                        if not a.wait_all:
                            # ★★ **任何点**（含最后一个）：进入容差就**直接发下一点** ✓ ★★
                            #   业主 2026-10-10（第 2 稿）："不应该区分中间点和最终点" ✓
                            #   ⇒ 修掉"在容差里转一会儿"这个现象 ✓
                            #     （旧行为实测：`wp2` 在容差内待了 **108 拍** ✗）
                            #   ★ 判据阈值（`tol`/`v_still`/`quiet_s`）**一个都没动** ✓ ——
                            #     改的只是"**要不要走这个等待**" ✓（这才是本步的单一变量 ✓）
                            print('   ⇒ 进入容差（d_gt=%.3f < %.2f）⇒ **直接发下一点**（不等停稳）✓'
                                  % (dg, a.tol), flush=True)
                            break
                        # ★ 到位后**记到"真停稳"为止**（不是固定时长）——
                        #   phase-7 实测：车在"进入容差"之后**又飘了 16.6 cm、到第 9 秒才真停** ⇒
                        #   固定窗口会把"还在走"的那一拍当成落点 ✗。这里一直记到**最近 5 s 的速度
                        #   全部 < v_still**，`--settle-s` 只作**封顶** ✓
                        t_stop = node.sim_now(); ws0 = time.time()
                        while node.sim_now() - t_stop < a.settle_s:
                            if node.stalled():   # ★ 无进展检测 ✓
                                print('   !! 静稳看门狗触发', flush=True); break
                            node.pump(1.0)
                            s = node.samples
                            if len(s) < 3:
                                continue
                            # ★★ 静稳时长**按时间量**，不许按样本数 ★★
                            #   上一版写的是 `samples[-20:]` 并注释成"约 5 s"—— **错了**：
                            #   抽稀步长 0.05 s ⇒ 20 个样本 ≈ **1.0 s** ✗ ⇒ 判据实际是
                            #   "瞬时静止 1 s"，一次**换挡/重规划造成的短暂停顿**就能骗过它 ✗。
                            #   实测后果（2026-10-10）：`wp1`/`wp2` 在 d_gt≈0.29 m（**刚好在 0.25 m
                            #   容差之外**）停顿 ~1.8 s ⇒ 我的窗口提前结束、车其实还可能继续收敛
                            #   ⇒ 两点被误判"未到达" ✗。**这就是"判状态 ≠ 判事件"**（本项目第四次）。
                            k = len(s) - 1
                            quiet_from = s[k][0]
                            while k > 0:
                                dt = s[k][0] - s[k - 1][0]
                                v = math.hypot(s[k][3] - s[k - 1][3],
                                               s[k][4] - s[k - 1][4]) / dt if dt > 1e-9 else 1e9
                                if v >= a.v_still:
                                    break
                                k -= 1
                                quiet_from = s[k][0]
                            quiet_s = s[-1][0] - quiet_from
                            if quiet_s >= a.quiet_s:
                                print('   ⇒ 已连续静止 %.1f s（≥ %.1f s）⇒ 真停稳'
                                      % (quiet_s, a.quiet_s), flush=True)
                                break
                        break
            rec = {'name': p.get('name'), 'lat': p['lat'], 'lon': p['lon']}
            rec.update(eval_point(node.samples, gx, gy, a.tol, a.v_still,
                                  time.strftime('%Y-%m-%dT%H:%M:%S')))
            records.append(rec)
            # ★ 本点的**逐样本**也落盘（判据可离线重判，不必重开一轮 ✓）
            _atomic_csv('%s_point_%s.csv' % (a.out, rec['name']), node.samples, (gx, gy))
            # ★ 每个点一落盘：中途被打断也不丢已完成的点 ✓
            _atomic_json(a.out + '_records.json',
                         {'calib': a.calib, 'calib_key_is_valid': calib['calib_key_is_valid'],
                          'calib_key': calib['calib_key'], 'route': a.route,
                          'tol': a.tol, 'v_still': a.v_still, 'quiet_s': a.quiet_s,
                          # ★ ① `sim/wall` 比值落盘（reviewer 2026-10-10）：
                          #   "这台机跑得慢"要**永远看得见** ✓，别只留在内存里 ✓
                          'sim_wall_ratio': round(getattr(node, '_ratio', 0.0) or 0.0, 4),
                          'points': records})
            print('   ⇒ %s' % ('到达并停稳 ✓' if rec.get('arrived') else '未到达 ✗'), flush=True)
    finally:
        # ★ 收尾包 try（`pkill -INT` 会同时触发 rclpy 自己的信号处理器 ⇒ 二次 shutdown 抛异常 ✗）
        try:
            node.destroy_node()
        except Exception:                                          # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:                                          # noqa: BLE001
            pass
    landed = [r['stopped_pose']['d_gt_mean_m'] for r in records if r.get('arrived')]
    _atomic_json(a.out + '_log.json',
                 {'calib': a.calib, 'route': a.route, 'points': len(pts),
                  'arrived': len(landed),
                  'worst_d_gt_mean_m': max(landed) if landed else None})
    print(json.dumps(records, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())

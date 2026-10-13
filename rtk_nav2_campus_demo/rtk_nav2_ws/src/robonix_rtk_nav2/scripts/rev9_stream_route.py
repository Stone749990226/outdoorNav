#!/usr/bin/env python3
"""rev9_stream_route.py —— ★ **运行中接点**的巡游节点（阶段九第 4 步 ✓）

用法（容器内）:
  rev9_stream_route.py --calib <geo_calib.json> --route <route_geo.json> --out <前缀>
                       [--tol 0.4] [--poll-s 1.0] [--done-after <秒>]

★★ 与 `geo_route.py` 的关系（★ 并存 ✓，✗ 不动那个已验证的批处理 ✓）★★
  · `geo_route.py` = **批处理**：读**一条固定的**航线、走完就退 ✓（第 3 步验收用的就是它 ✓）。
  · 本节点 = **流式**：★ **栈不重启** ✓，★ 航线文件**在运行中被追加** ✓ ⇒ 车**依次接着走** ✓。

★ 到达判据、标定读法、`calib_key_is_valid` 的消费规则**与 `geo_route.py` 一致** ✓
  （**进入容差即发下一点** ✓；只用顶层布尔判标定有效 ✓）。
★ 距离按 `/ground_truth/odom`（**真值** ✓）算 —— 与 map 系路点同源（map ≡ world，实测 |Δ| ~0.08 m ✓）。
★ **每个点一落盘**（`临时文件 + os.replace` ✓）⇒ 中途被打断也不丢已完成的点 ✓。
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

MAP_TOPIC = '/odometry/global'
GT_TOPIC = '/ground_truth/odom'
GOAL_TOPIC = '/goal_pose'


def load_calib(path):
    d = json.load(open(path, encoding='utf-8'))
    if d.get('calib_key_is_valid') is not True:
        print('!! calib_key_is_valid != True ⇒ **拒绝规划** ✗', file=sys.stderr)
        print('!! 规范串：%s' % (d.get('calib_key_canonical') or '(缺失)'), file=sys.stderr)
        return None
    return d


def latlon_to_map(L, lat, lon):
    """★ 与 `geo_route.py` **同一套公式** ✓（✗ 不另写一份 ✗）。"""
    enu = np.array([(lon - L['origin']['lon']) * L['m_per_deg_lon'],
                    (lat - L['origin']['lat']) * L['m_per_deg_lat']])
    th = math.radians(L['theta_deg'])
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return L['scale'] * (R @ enu) + np.array(L['translation_m'])


def read_route(path):
    """★ 读航线 ✓；★ **读不到/半截就返回 None** ✓（✗ 不当"空航线" ✗ —— 追加方写成原子替换 ✓）。"""
    try:
        with open(path, encoding='utf-8') as fh:
            d = json.load(fh)
    except Exception:
        return None
    pts = d.get('points') or []
    try:
        return [(p.get('name') or ('wp%d' % (i + 1)), float(p['lat']), float(p['lon']))
                for i, p in enumerate(pts)]
    except Exception:
        return None


def write_records(prefix, records, samples, last):
    tmp = '%s_point_%s.csv.tmp' % (prefix, last)
    with open(tmp, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'map_x', 'map_y', 'gt_x', 'gt_y', 'd_gt'])
        for s in samples:
            w.writerow(s)
    os.replace(tmp, '%s_point_%s.csv' % (prefix, last))
    tmp2 = prefix + '_records.json.tmp'
    with open(tmp2, 'w', encoding='utf-8') as fh:
        json.dump({'criterion': 'tolerance_entered', 'n_records': len(records),
                   'records': records}, fh, ensure_ascii=False, indent=2)
    os.replace(tmp2, prefix + '_records.json')


class StreamRoute(Node):
    def __init__(self, calib, tol):
        super().__init__('rev9_stream_route')
        # ★★ 必须与 `geo_route.py` 一样显式设 `use_sim_time` ✗★★
        #   （`geo_route.py:353` 就是 `set_parameters([Parameter('use_sim_time', True)])` ✓）
        #   ✗ 不设 ⇒ `get_clock().now()` 返回**墙上时间** ⇒ 消息头的时间戳是 ~1.79e9 s ✗
        #     （仿真时间只有几百秒 ✓）—— 与整条链里所有用 sim time 的节点对不上 ✗。
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.L = calib['latlon_to_map']
        self.tol = tol
        self.goals = []          # [(name, map_x, map_y)]
        self.idx = -1            # 当前目标下标；-1 = 还没发过
        self.gt = None
        self.map = None
        self.entered = False     # 当前目标是否已进入容差
        self.t_resend = 0.0      # 上次**重发**该目标的时刻 ✓（★ 必须在 __init__ 里给 ✓）
        self.samples = []
        self.records = []
        self.out_prefix = ''
        qos = QoSProfile(depth=50, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)
        self.create_subscription(Odometry, GT_TOPIC, self._gt, qos)
        self.create_subscription(Odometry, MAP_TOPIC, self._map, qos)

    def _gt(self, m):
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _map(self, m):
        self.map = (m.pose.pose.position.x, m.pose.pose.position.y)

    def add_points(self, pts):
        """★ 只追加**新的**点 ✓（按名字判重 ⇒ 重复喂同一份文件不重发 ✓）。"""
        have = {g[0] for g in self.goals}
        added = []
        for (nm, lat, lon) in pts:
            if nm in have:
                continue
            x, y = latlon_to_map(self.L, lat, lon)
            self.goals.append((nm, float(x), float(y)))
            added.append(nm)
        return added

    def publish_goal(self, i):
        """★ 只负责**发**（重发也走它 ✓）。"""
        nm, x, y = self.goals[i]
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.w = 1.0
        self.pub.publish(m)

    def send(self, i):
        self.idx = i
        self.entered = False
        self.samples = []
        self.t_resend = time.time()
        self.publish_goal(i)
        nm, x, y = self.goals[i]
        print('[%s] → 发目标 %d/%d  %s  map=(%.3f, %.3f)'
              % (time.strftime('%H:%M:%S'), i + 1, len(self.goals), nm, x, y), flush=True)

    def d_gt(self):
        if self.gt is None or self.idx < 0:
            return None
        _, x, y = self.goals[self.idx]
        return math.hypot(self.gt[0] - x, self.gt[1] - y)

    def tick(self):
        d = self.d_gt()
        if d is None or self.idx < 0:
            return
        self.samples.append((self.get_clock().now().nanoseconds / 1e9,
                             self.map[0] if self.map else float('nan'),
                             self.map[1] if self.map else float('nan'),
                             self.gt[0], self.gt[1], d))
        # ★★ 周期**重发**当前目标 ✗★★（★ 这一条是被实测逼出来的 ✓）
        #   背景（`goal_navigator.py` 的注释里早写着 ✓）：ros 2 的**发现竞态** ⇒
        #   "发布者刚起来就发的那一条"很容易在**还没跟订阅者匹配上**时丢掉 ✓
        #   ⇒ 只发一次 ⇒ 目标**根本没到** ✗（实测：`goal_navigator` 的日志里**没有**
        #     "收到目标 …，开始规划" 那行 ✓，车原地不动 4 分钟 ✗）。
        #   ★ 重发是**安全的** ✓：`goal_navigator` 对"与在办目标重合（<0.05 m）"的目标
        #     **幂等忽略** ✓（不重置、不取消在走的路径 ✓）。
        now = time.time()
        if not self.entered and now - self.t_resend >= 1.0:
            self.t_resend = now
            self.publish_goal(self.idx)
        if not self.entered and d < self.tol:
            self.entered = True
            nm, x, y = self.goals[self.idx]
            print('[%s] ✓ %s 进入容差：d_gt=%.3f < %.2f ⇒ 发下一点'
                  % (time.strftime('%H:%M:%S'), nm, d, self.tol), flush=True)
            self.records.append({'name': nm, 'goal_map': [x, y],
                                 'min_d_gt': min(s[5] for s in self.samples),
                                 'n_samples': len(self.samples),
                                 't_enter_wall': time.time()})
            write_records(self.out_prefix, self.records, self.samples, nm)
            self.samples = []

    def advance(self):
        """★ 统一的推进点 ✓ —— 每次 tick 都调 ✓（这样"车已到队尾时又来了新点"也能接着走 ✓）。"""
        if not self.goals:
            return
        if self.idx < 0:
            self.send(0)
            return
        if self.entered and self.idx + 1 < len(self.goals):
            self.send(self.idx + 1)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib', required=True)
    ap.add_argument('--route', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--tol', type=float, default=0.4)
    ap.add_argument('--poll-s', type=float, default=1.0)
    ap.add_argument('--done-after', type=float, default=30.0,
                    help='★ 全部点走完后再等这么久没有新点就退出 ✓')
    ap.add_argument('--leg-timeout', type=float, default=420.0,
                    help='★ 单个点最长等多久 ✓（超时 ⇒ **带原因退出** ✗，✗ 不许悄悄跳过 ✓）')
    a = ap.parse_args(argv)

    calib = load_calib(a.calib)
    if calib is None:
        return 3
    rclpy.init()
    node = StreamRoute(calib, a.tol)
    node.out_prefix = a.out

    pts = read_route(a.route)
    if not pts:
        print('✗ 航线文件读不出点：%s' % a.route)
        return 2
    node.add_points(pts)
    print('★ 初始航线 %d 个点 ✓（★ 之后**追加**到同一文件即可，栈不重启 ✓）' % len(node.goals), flush=True)

    n_seen = len(pts)
    last_poll = 0.0
    t_leg0 = time.time()
    rc = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
            node.advance()
            now = time.time()
            # ★ 单点超时 ⇒ **带原因退出** ✓（✗ 不许悄悄跳过 ✗ —— 跳过会把"没到"记成"到了" ✗）
            if node.idx >= 0:
                cur = node.records[-1]['name'] if node.records else None
                t_ref = node.records[-1]['t_enter_wall'] if node.records else 0.0
                base = max(t_leg0, t_ref)
                if not node.entered and now - base > a.leg_timeout:
                    d = node.d_gt()
                    print('✗ 单点超时：目标 %s 已等 %.0f s 仍未进入容差（当前 d_gt=%.3f m，tol=%.2f）'
                          % (node.goals[node.idx][0], now - base, d if d is not None else float('nan'),
                             a.tol), flush=True)
                    rc = 4
                    break
            if now - last_poll >= a.poll_s:
                last_poll = now
                pts2 = read_route(a.route)
                if pts2 is not None and len(pts2) > n_seen:
                    added = node.add_points(pts2)
                    n_seen = len(pts2)
                    if added:
                        t_leg0 = now
                        print('[%s] ★ 运行中接到 %d 个新点（文件里共 %d，队列共 %d）⇒ 追加队尾 ✓'
                              % (time.strftime('%H:%M:%S'), len(added), len(pts2), len(node.goals)),
                              flush=True)
            if (node.records and len(node.records) == len(node.goals)
                    and now - node.records[-1]['t_enter_wall'] > a.done_after):
                print('★ 全部 %d 个点走完、且 %.0f s 内没有新点 ⇒ 正常退出 ✓'
                      % (len(node.goals), a.done_after), flush=True)
                break
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    print('★ 结束：走完 %d 个点 ｜ 产物前缀 %s ｜ rc=%d' % (len(node.records), a.out, rc), flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())

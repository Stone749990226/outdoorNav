#!/usr/bin/env python3
"""nav2 两张图的取数与核对（第三阶段验收的取数工具，一个文件多个子命令）。

为什么把取数做成一个脚本而不是散在各处：判据 1/2/3/5/7/8 要的都是"同一时刻的地图快照 +
元数据 + 逐格结论"，散着写必然出现"这次统计口径和上次不一样"。这里所有子命令共用
同一套订阅与统计函数。

子命令：
  stats     打印两张图的元数据与 free/occupied/unknown 计数（判据 1/3 的基本量）
  coverage  按时间采样全局图，落 CSV：free/occupied/unknown 随时间（判据 3 的覆盖增长曲线）
  geometry  ① 占据格到真实表面的距离分布（判据 1）② 逐格核对：真实障碍表面上的格子
            是不是占据 ③ **近处障碍回归**：路线 2 m 内的障碍表面不许被"擦成大片空闲"
            （判据 1 第二半）。
            **2026-09-21 改名**：输出里的 `surface_samples_per_wall` 已改成
            `surface_samples_per_obstacle`。原来叫 per_wall 是因为当初场地上除了
            几根立柱/隔墙，主要就是那圈外墙；四周围墙删掉之后，这里统计的
            **只剩中央方块 / 两道隔墙 / 三根立柱 / 4 段独立墙**，叫 wall 名不副实
            （值本身早就不含外墙了——`surface_samples()` 是按 `map_geometry.SEGMENTS`
            采的）。老产物 JSON 里仍是旧键名，`summarize_evidence.py` 两个键都读。
  perf      发布频率（两张图 / 点云 / 深度图）+ 各进程 CPU 占用（判据 7）
  localmap  局部图的帧、尺寸、旋转前后静止障碍的位置（判据 8 用，配合 spin_probe）

用法示例：
  nav2_probe.py stats --json out/stats.json
  nav2_probe.py coverage --csv out/coverage.csv --duration 120
  nav2_probe.py geometry --json out/geometry.json --dump-snapshot out/global_snapshot.npz
  nav2_probe.py perf --json out/perf.json --duration 60
"""

import argparse
import json
import os
import statistics
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_geometry import (distance_to_route, nearest_surface,  # noqa: E402
                          surface_samples)

GLOBAL_TOPIC = '/global_costmap/costmap'
LOCAL_TOPIC = '/local_costmap/costmap'
# nav2 的 Costmap2DPublisher 用 transient_local 发布：后加入的订阅者能立刻拿到最后一帧。
# 订阅端**必须**用同样的 durability，否则要等到下一次周期性发布才有数据（验证过：
# 用 VOLATILE 订阅要等最多一个 publish 周期，判据里"取一帧快照"会偶发超时）。
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def grid_array(msg):
    return np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


def counts(data):
    """三分类 + **致命格**。

    **`occupied` 与 `lethal` 的区别很关键，别混用**：nav2 发布出来的全局图带着膨胀梯度
    （`trinary_costmap: true` 只把 未知/空闲/致命 三类保干净，其余按 cost/2.55 缩放）。
    实测同一张图上同时存在 64/75/87/99/100 这些值：
      - `lethal`（== 100，对应内部 LETHAL_OBSTACLE）：**障碍表面**真正被打到的格子；
      - `occupied`（>= 50）：还把膨胀带（cost>=50）算进去了，比真实障碍大一圈。

    "占据格到真实表面的距离"这种几何核对**必须用 lethal**——用 occupied 会把
    膨胀带也算进去（实测：用 occupied 得到中位数 0.10 m、最大 0.35 m，
    看起来像"障碍位置偏了 10 cm"，其实是把 0.15~0.29 m 处的膨胀格当成了障碍）。
    """
    return {
        'free': int((data == 0).sum()),
        'lethal': int((data >= 100).sum()),
        'occupied': int((data >= 50).sum()),
        'inflated_only': int(((data > 0) & (data < 50)).sum()),
        'unknown': int((data < 0).sum()),
        'cells': int(data.size),
    }


def meta(msg):
    return {
        'topic_frame': msg.header.frame_id,
        'resolution': msg.info.resolution,
        'width': msg.info.width,
        'height': msg.info.height,
        'origin_x': msg.info.origin.position.x,
        'origin_y': msg.info.origin.position.y,
        'extent_x_m': [msg.info.origin.position.x,
                       msg.info.origin.position.x + msg.info.width * msg.info.resolution],
        'extent_y_m': [msg.info.origin.position.y,
                       msg.info.origin.position.y + msg.info.height * msg.info.resolution],
    }


class Collector(Node):
    """收几张图的最后一帧；rate 类子命令还用它的时间戳序列算频率。"""

    def __init__(self):
        super().__init__('nav2_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.latest = {}
        self.stamps = {}
        sub_specs = [
            (OccupancyGrid, GLOBAL_TOPIC, 'global', LATCHED),
            (OccupancyGrid, LOCAL_TOPIC, 'local', LATCHED),
            (PointCloud2, '/rtk/camera/points', 'points', qos_profile_sensor_data),
            (Image, '/rtk/camera/depth_image', 'depth', qos_profile_sensor_data),
        ]
        for msg_type, topic, key, qos in sub_specs:
            self.create_subscription(
                msg_type, topic, self._mk(key), qos)
        self.t0 = time.time()

    def _mk(self, key):
        def cb(msg):
            self.latest[key] = msg
            self.stamps.setdefault(key, []).append(time.time() - self.t0)
            self.stamps[key] = self.stamps[key][-400:]
        return cb

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, keys, timeout=30.0):
        """等到要求的几张图都到过至少一帧。"""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(k in self.latest for k in keys):
                return True
        return False


def rate_of(stamps):
    if len(stamps) < 3:
        return None
    span = stamps[-1] - stamps[0]
    if span <= 0:
        return None
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    return {
        'samples': len(stamps),
        'mean_hz': round((len(stamps) - 1) / span, 3),
        'max_gap_s': round(max(gaps), 4),
        'mean_gap_s': round(statistics.fmean(gaps), 4),
    }


# ------------------------------------------------------------------ 子命令
def cmd_stats(args):
    node = Collector()
    if not node.wait_for(['global', 'local'], timeout=args.timeout):
        print('** 没收到两张图 **：%s' % sorted(node.latest), file=sys.stderr)
        node.destroy_node()
        return 1
    out = {'global': {'meta': meta(node.latest['global']),
                      'counts': counts(grid_array(node.latest['global']))},
           'local': {'meta': meta(node.latest['local']),
                     'counts': counts(grid_array(node.latest['local']))}}
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    node.destroy_node()
    return 0


def cmd_coverage(args):
    node = Collector()
    if not node.wait_for(['global'], timeout=args.timeout):
        print('** 没收到全局图 **', file=sys.stderr)
        node.destroy_node()
        return 1
    t0 = time.time()
    rows = []
    had = None
    end = t0 + args.duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        msg = node.latest.get('global')
        if msg is None:
            continue
        had = msg
        c = counts(grid_array(msg))
        rows.append({'t_s': round(time.time() - t0, 3),
                     'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                     **c})
    # 同一时刻重复的帧会很多（订阅端比发布快），只留相邻不同的
    dedup = []
    for r in rows:
        if not dedup or (dedup[-1]['free'], dedup[-1]['occupied'], dedup[-1]['unknown']) \
                != (r['free'], r['occupied'], r['unknown']):
            dedup.append(r)
    with open(args.csv, 'w', encoding='utf-8') as handle:
        handle.write('t_s,stamp_s,free,occupied,inflated_only,unknown,cells\n')
        for r in dedup:
            handle.write('%s,%s,%d,%d,%d,%d,%d\n' % (
                r['t_s'], r['stamp_s'], r['free'], r['occupied'],
                r['inflated_only'], r['unknown'], r['cells']))
    first = dedup[0] if dedup else None
    last = dedup[-1] if had is not None and dedup else None
    summary = {'samples': len(dedup), 'duration_s': args.duration,
               'first': first, 'last': last,
               'meta': meta(had) if had is not None else None,
               'note': '逐帧统计的是**发布出来的全图**（always_send_full_costmap: true）；'
                       'free 由"射线清空"与"地面点"共同产生，occupied 是障碍带内的命中点'}
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, ensure_ascii=False, indent=2)
    node.destroy_node()
    return 0


def _lookup(data, m, x, y):
    col = int((x - m['origin_x']) / m['resolution'])
    row = int((y - m['origin_y']) / m['resolution'])
    if row < 0 or col < 0 or row >= data.shape[0] or col >= data.shape[1]:
        return None
    return int(data[row, col])


def cmd_geometry(args):
    node = Collector()
    if not node.wait_for(['global'], timeout=args.timeout):
        print('** 没收到全局图 **', file=sys.stderr)
        node.destroy_node()
        return 1
    msg = node.latest['global']
    data = grid_array(msg)
    m = meta(msg)
    if args.dump_snapshot:
        np.savez_compressed(args.dump_snapshot, data=data,
                            resolution=m['resolution'], origin_x=m['origin_x'],
                            origin_y=m['origin_y'], frame=m['topic_frame'])

    # **用 lethal（==100）而不是 >=50**：>=50 会把膨胀带算进来（见 counts() 的说明）
    occupied = np.argwhere(data >= 100)
    cells = [(m['origin_x'] + (c + 0.5) * m['resolution'],
              m['origin_y'] + (r + 0.5) * m['resolution']) for r, c in occupied]
    dists = [nearest_surface(x, y) for x, y in cells]
    tol = args.tolerance_cells * m['resolution']
    within = sum(1 for d in dists if d <= tol)
    ordered = sorted(dists) if dists else [0.0]

    # ---- 逐格核对：真实障碍表面上的格子是什么状态 ----
    samples = surface_samples(step=m['resolution'])
    per_obstacle = {}
    for x, y, name in samples:
        v = _lookup(data, m, x, y)
        if v is None:
            continue
        bucket = per_obstacle.setdefault(name, {'occupied': 0, 'free': 0, 'unknown': 0,
                                            'inflated': 0, 'route_near': {'occupied': 0,
                                                                          'free': 0,
                                                                          'unknown': 0,
                                                                          'inflated': 0,
                                                                          'n': 0}})
        if v >= 50:
            bucket['occupied'] += 1
            key = 'occupied'
        elif v < 0:
            bucket['unknown'] += 1
            key = 'unknown'
        elif v > 0:
            bucket['inflated'] += 1
            key = 'inflated'
        else:
            bucket['free'] += 1
            key = 'free'
        # 近墙回归只统计**路线 2 m 以内**的墙：远处的墙本来就没被看过，未知是正确状态
        if distance_to_route(x, y) <= args.route_near_m:
            bucket['route_near'][key] += 1
            bucket['route_near']['n'] += 1

    occ_ge50 = np.argwhere(data >= 50)
    cells_ge50 = [(m['origin_x'] + (c + 0.5) * m['resolution'],
                   m['origin_y'] + (r + 0.5) * m['resolution']) for r, c in occ_ge50]
    dists_ge50 = [nearest_surface(x, y) for x, y in cells_ge50]
    geometry = {
        'meta': m,
        'counts': counts(data),
        'occupied_cells': len(cells),
        'lethal_cells': len(cells),
        'occupied_ge50_cells': len(cells_ge50),
        'distance_ge50_m': {
            'note': '把膨胀带（代价 >= 50）也算成"占据"时的同一统计，**仅供对照**；'
                    '它天然比真实障碍大一圈，不代表障碍位置偏了',
            'mean': round(statistics.fmean(dists_ge50), 4) if dists_ge50 else None,
            'median': round(statistics.median(dists_ge50), 4) if dists_ge50 else None,
            'max': round(max(dists_ge50), 4) if dists_ge50 else None,
        },
        'distance_to_nearest_real_surface_m': {
            'tolerance_m': round(tol, 3),
            'within_tolerance': within,
            'within_ratio': round(within / len(cells), 4) if cells else None,
            'mean': round(statistics.fmean(dists), 4) if dists else None,
            'median': round(statistics.median(dists), 4) if dists else None,
            'p95': round(ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))], 4),
            'max': round(ordered[-1], 4),
        },
        # **键名 2026-09-21 由 surface_samples_per_wall 改为 surface_samples_per_obstacle**：
        # 值一直是"每个真实障碍表面上的采样格"，外墙删掉后叫 wall 名不副实。
        # summarize_evidence.py 两个键名都读，老产物不受影响。
        'surface_samples_per_obstacle': per_obstacle,
        'route_near_m': args.route_near_m,
        'note': 'surface_samples 在**真实障碍表面**上按 0.05 m 采样'
                '（采样清单来自 map_geometry：中央方块/两道隔墙/三根立柱/4 段独立墙，'
                '**四周围墙已删除、不在其中**）；'
                'route_near 只统计路线 %.1f m 以内的采样点——'
                '"远处的障碍是未知"是正确状态，不该算进回归' % args.route_near_m,
    }
    print(json.dumps(geometry, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(geometry, handle, ensure_ascii=False, indent=2)
    node.destroy_node()
    return 0


def _proc_cpu():
    """容器内所有进程的 CPU 占用（从 /proc 读两遍算差分）。返回 {pid: (comm, pct)}。"""
    def snap():
        out = {}
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            try:
                with open('/proc/%s/stat' % pid) as handle:
                    fields = handle.read().split()
                with open('/proc/%s/cmdline' % pid, 'rb') as handle:
                    cmd = handle.read().replace(b'\x00', b' ').decode('utf-8', 'replace').strip()
                out[pid] = (fields[1], int(fields[13]) + int(fields[14]), cmd)
            except Exception:
                continue
        return out

    a = snap()
    time.sleep(2.0)
    b = snap()
    total = 0
    rows = []
    for pid, (comm, ticks, cmd) in b.items():
        if pid not in a:
            continue
        delta = ticks - a[pid][1]
        pct = 100.0 * delta / (2.0 * os.sysconf('SC_CLK_TCK'))
        total += pct
        if pct >= 0.5:
            rows.append({'pid': int(pid), 'comm': comm, 'cpu_pct': round(pct, 1),
                         'cmd': cmd[:160]})
    rows.sort(key=lambda r: -r['cpu_pct'])
    return round(total, 1), rows


# CPU 必须**按角色拆开**再报（reviewer 要求）：容器里总 CPU 里最大的一块是 Gazebo 的
# 渲染/物理，把 500% 多笼统报成"nav2 的开销"是误导。分类按 cmdline 里的稳定特征串。
CPU_CATEGORIES = (
    ('gz-gui（Gazebo 渲染/GUI）', ('ruby', 'ign gazebo -r', 'gz sim -r')),
    ('gz-server（Gazebo 物理）', ('ign gazebo', 'gz sim')),
    ('nav2（planner/lifecycle/costmap 宿主）',
     ('planner_server', 'lifecycle_manager', 'local_costmap_host', 'nav2_')),
    ('自写节点（定位/危险距离/桥接等）',
     ('absolute_localizer', 'robonix_rtk_nav2', 'rtk_sim', 'wheel_odom_bridge',
      'trajectory_logger', 'depth_display', 'point_cloud_xyz', 'parameter_bridge')),
    ('探针/工具', ('nav2_probe', 'rev_probe', 'local_map_diag', 'spin_probe',
                   'map_roundtrip', 'pgm_convention', 'plan_path', 'osrf/',
                   'ros2 bag', 'ros2 run', 'ros2 topic', 'ros2 param', 'ros2 node')),
)


def cpu_by_category(procs):
    """把容器 CPU 按角色归类。**先匹配最具体的**：`ign gazebo -r` 是 GUI、
    `ruby` 启动的是 GUI，纯 `ign gazebo` 才是 server——顺序错了会把 GUI 算进 server。"""
    out = {name: 0.0 for name, _ in CPU_CATEGORIES}
    out['其他'] = 0.0
    for p in procs:
        cmd = p.get('cmd') or ''
        placed = False
        for name, keys in CPU_CATEGORIES:
            if any(k in cmd for k in keys):
                out[name] += p['cpu_pct']
                placed = True
                break
        if not placed:
            out['其他'] += p['cpu_pct']
    return {k: round(v, 1) for k, v in sorted(out.items(), key=lambda kv: -kv[1])}


def cmd_perf(args):
    node = Collector()
    node.wait_for(['global', 'local', 'points', 'depth'], timeout=args.timeout)
    node.spin_for(args.duration)
    cpu_total, procs = _proc_cpu()
    by_cat = cpu_by_category(procs)
    try:
        with open('/proc/loadavg') as handle:
            loadavg = handle.read().split()[:3]
    except Exception:
        loadavg = None
    out = {
        'duration_s': args.duration,
        'rates': {k: rate_of(v) for k, v in node.stamps.items()},
        'container_cpu_total_pct': cpu_total,
        # 按角色拆开——**报性能时用这一份**，不要笼统报总量
        'container_cpu_by_category': by_cat,
        'container_cpu_top': procs[:15],
        'host_loadavg_1_5_15': loadavg,
        'note': 'container_cpu_by_category 是按角色拆开的（Gazebo GUI / gz server / nav2 / '
                '自写节点 / 探针），**报性能时用这一份**；container_cpu_top 是原始前 15 条。'
                '容器 CPU 是容器内**全部**进程之和（含 Gazebo 渲染）；'
                '宿主 loadavg 是整机的，本机是共享机（还有其他用户/容器在跑），'
                '所以性能结论要连同 loadavg 一起看，不能只看单次数字',
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    node.destroy_node()
    return 0


def cmd_localmap(args):
    """局部图的静态度量：帧、尺寸、原点，以及（可选）某个世界点在栅格里的位置。

    判据 8（旋转帧）用它取"旋转前后同一个静止障碍落在哪一格"。
    """
    node = Collector()
    if not node.wait_for(['local'], timeout=args.timeout):
        print('** 没收到局部图 **', file=sys.stderr)
        node.destroy_node()
        return 1
    msg = node.latest['local']
    data = grid_array(msg)
    m = meta(msg)
    out = {'meta': m, 'counts': counts(data)}
    if args.snapshot:
        np.savez_compressed(args.snapshot, data=data, resolution=m['resolution'],
                            origin_x=m['origin_x'], origin_y=m['origin_y'],
                            frame=m['topic_frame'])
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    node.destroy_node()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('stats')
    p.add_argument('--json', default='')
    p.add_argument('--timeout', type=float, default=30.0)
    p.set_defaults(func=cmd_stats)

    p = sub.add_parser('coverage')
    p.add_argument('--csv', required=True)
    p.add_argument('--json', default='')
    p.add_argument('--duration', type=float, default=120.0)
    p.add_argument('--timeout', type=float, default=30.0)
    p.set_defaults(func=cmd_coverage)

    p = sub.add_parser('geometry')
    p.add_argument('--json', default='')
    p.add_argument('--dump-snapshot', default='')
    p.add_argument('--tolerance-cells', type=float, default=3.0)
    p.add_argument('--route-near-m', type=float, default=2.0)
    p.add_argument('--timeout', type=float, default=30.0)
    p.set_defaults(func=cmd_geometry)

    p = sub.add_parser('perf')
    p.add_argument('--json', default='')
    p.add_argument('--duration', type=float, default=60.0)
    p.add_argument('--timeout', type=float, default=30.0)
    p.set_defaults(func=cmd_perf)

    p = sub.add_parser('localmap')
    p.add_argument('--json', default='')
    p.add_argument('--snapshot', default='')
    p.add_argument('--timeout', type=float, default=30.0)
    p.set_defaults(func=cmd_localmap)

    args = parser.parse_args(argv)
    rclpy.init()
    try:
        return args.func(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""第四步（目标点导航）验收的**旁路记录器**：把整个演示期间"导航闭环到底干了什么"逐秒记下来。

为什么需要它（而不是事后看 `demo_launch.log`）：
  1. **`ros2 launch` 的 stdout 是块缓冲的**——把 launch 的输出 `nohup ... > demo_launch.log`
     之后，最后 ~8 KB 的行会**在缓冲区里躺着**，看起来像"节点不打印了/卡住了"。
     nav4-fix3 那轮我就是被这个骗过一次（文件 5 分钟没长，进程却还在烧 CPU）。
     所以这里自己订 `/rosout`，拿到的是**带到达时间的结构化日志**，不经过 launch 的缓冲。
  2. 判据 3/5 要的是"**第一次**规划出来的那条路径"（给 A 时它应当大部分落在未知里、
     给 B 时大部分落在已知空闲里）。事后补采拿不到历史，必须在下发目标**之前**就订上
     `/rtk/plan_path`（goal_navigator 每接受一条路径就发一次，抑制掉的不发）。
  3. 判据 1 要的是"**起栈后第一帧**全局图"。`/global_costmap/costmap` 是
     transient_local 发布：**起栈之后**再订阅拿到的是"最后一帧"，不是第一帧。
     本记录器必须在 `start-demo.sh` **之前**启动，才能钉住第一帧（产物里的
     `first_global_map.json` 带 `is_first_frame: true`）。

产物（都在 --out-dir 下，逐行 flush，进程被强杀也不丢已写内容）：
  first_global_map.json   起栈后收到的第一帧全局图：三分类计数 + npz 快照 + 到达时刻
  global_map_timeline.csv 全局图随时间（wall 时间 + 图上 stamp + 三分类计数）
  plan_paths.jsonl        每一次被接受的规划结果：点数/长度/落在未知·空闲·占据的比例/穿墙核对/完整点列
  rosout.jsonl            goal_navigator 与 controller_server 的日志（带 wall 时间，绕开 launch 缓冲）
  nav_states.jsonl        /rtk/goal_navigator_state 的状态迁移
  follow_status.jsonl     /follow_path 的 action 状态（EXECUTING=2 / CANCELED=5 / SUCCEEDED=4 ...）
  rates.csv               每秒消息计数（cmd_vel / 相机点云 / 全局图）——"车在不在被控制"看这三列
  summary.json            退出时写的汇总

用法（**容器内**，且必须在起栈前启动；文件放在 scripts/rtk-nav2/container/ 下有个硬理由：
  `container/stop_stack.sh` 会 `pkill -f robonix_rtk_nav2`，而按包路径
  `/work/rtk_nav2_ws/src/robonix_rtk_nav2/scripts/...` 启动的记录器命令行里就含这个串，
  会被起栈前的清场顺手杀掉——这类"自己的测量工具被自己的清场脚本干掉"的坑本轮已经踩过多次）：
  python3 /work/scripts/rtk-nav2/container/nav4_recorder.py --out-dir /work/artifacts/rtk-nav2/<run>/nav4rec
"""

import argparse
import json
import os
import signal
import sys
import time

import numpy as np

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

# 穿墙核对用第三阶段的同一套场地几何（**复用**，保证口径与判据 7 一致）
GEOM_DIR = os.environ.get('NAV4_GEOM_DIR',
                          '/work/rtk_nav2_ws/src/robonix_rtk_nav2/scripts')
sys.path.insert(0, GEOM_DIR)
from map_geometry import segment_hits_obstacle  # noqa: E402

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

# 全局图很大（800x800），每帧都 np 解一遍也能接受；但 timeline 只按"计数有变化"记，
# 免得写出几十万行没法看的 CSV（覆盖曲线要的是变化，不是采样密度）。
STATUS_NAME = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
               4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def grid_counts(data):
    return {'free': int((data == 0).sum()),
            'lethal': int((data >= 100).sum()),
            'occupied': int((data >= 50).sum()),
            'unknown': int((data < 0).sum()),
            'cells': int(data.size)}


class Recorder(Node):

    def __init__(self, out_dir):
        super().__init__('nav4_recorder')
        self.out = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.stop = False
        self.t0 = time.time()

        # ---- 文件句柄（全部行缓冲，逐行落盘）----
        self.f_timeline = open(os.path.join(out_dir, 'global_map_timeline.csv'), 'w',
                               buffering=1, encoding='utf-8')
        self.f_timeline.write('t_wall,stamp_s,free,occupied,lethal,unknown,cells\n')
        self.f_paths = open(os.path.join(out_dir, 'plan_paths.jsonl'), 'w',
                            buffering=1, encoding='utf-8')
        self.f_rosout = open(os.path.join(out_dir, 'rosout.jsonl'), 'w',
                             buffering=1, encoding='utf-8')
        self.f_states = open(os.path.join(out_dir, 'nav_states.jsonl'), 'w',
                             buffering=1, encoding='utf-8')
        self.f_status = open(os.path.join(out_dir, 'follow_status.jsonl'), 'w',
                             buffering=1, encoding='utf-8')
        self.f_rates = open(os.path.join(out_dir, 'rates.csv'), 'w',
                            buffering=1, encoding='utf-8')
        self.f_rates.write('t_wall,cmd_vel,points,global_map,paths\n')

        # ---- 状态 ----
        self.grid = None            # 最近一帧全局图的 numpy 视图
        self.meta = None
        self.map_stamp = None
        self.map_frames = 0
        self.first_map_written = False
        self.last_counts = None
        self.paths = []
        self.path_count = 0
        self.counts = {'cmd_vel': 0, 'points': 0, 'global_map': 0, 'paths': 0}
        self.rate_rows = 0

        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.on_map, LATCHED)
        self.create_subscription(Path, '/rtk/plan_path', self.on_path, 10)
        self.create_subscription(String, '/rtk/goal_navigator_state',
                                 self.on_state, LATCHED)
        self.create_subscription(Log, '/rosout', self.on_rosout, 1000)
        self.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                                 self.on_status, qos_profile_action_status_default)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 50)
        # **点云必须用 sensor_data（BEST_EFFORT）**：相机桥接是 BEST_EFFORT 发布，
        # 用默认的 RELIABLE 订阅会拿到 "offering incompatible QoS ... RELIABILITY" 警告、
        # **一条都收不到**（实测：rates.csv 里 points 列 40 秒全 0，看起来像"相机死了"）。
        self.create_subscription(PointCloud2, '/rtk/camera/points', self.on_points,
                                 qos_profile_sensor_data)

        self.create_timer(1.0, self.on_tick)
        self.get_logger().info('nav4_recorder 就绪 → %s' % out_dir)

    # ------------------------------------------------------------ 回调
    def _write(self, fh, obj):
        """写一条 JSON 记录。**任何异常都吃掉**。

        为什么必须吃：在 rclpy 里，订阅回调抛出的异常会被 executor 重新抛出、
        **直接终止 spin**（实测：`uuid` 是 array('B') 没转 int，记录器起来 0.6 s 就退出，
        产物全是空的）。旁路记录器绝不能因为一条记录毁掉整轮验收的证据。
        """
        try:
            fh.write(json.dumps(obj, ensure_ascii=False) + '\n')
        except Exception as exc:
            sys.stderr.write('nav4_recorder 写记录失败：%s\n' % exc)

    def on_map(self, msg: OccupancyGrid):
        self.counts['global_map'] += 1
        self.map_frames += 1
        self.map_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        self.grid = data
        self.meta = {'resolution': msg.info.resolution,
                     'origin_x': msg.info.origin.position.x,
                     'origin_y': msg.info.origin.position.y,
                     'width': msg.info.width, 'height': msg.info.height,
                     'frame': msg.header.frame_id}
        c = grid_counts(data)
        if not self.first_map_written:
            # **判据 1 的证物**：起栈后第一帧全局图。必须在起栈前就订上才拿得到这一帧。
            self.first_map_written = True
            np.savez_compressed(os.path.join(self.out, 'global_first_frame.npz'),
                                data=data, **{k: self.meta[k] for k in
                                              ('resolution', 'origin_x', 'origin_y')})
            with open(os.path.join(self.out, 'first_global_map.json'), 'w',
                      encoding='utf-8') as fh:
                json.dump({'is_first_frame': True,
                           't_wall': round(time.time() - self.t0, 3),
                           'received_wall_time': time.time(),
                           'map_stamp_s': self.map_stamp,
                           'counts': c, 'meta': self.meta,
                           'all_unknown': c['unknown'] == c['cells'],
                           'note': '起栈后收到的第一帧 /global_costmap/costmap（transient_local 的'
                                   '第一帧，记录器在 start-demo.sh 之前启动）。'
                                   'all_unknown=true 即"初始无地图"。'}, fh,
                          ensure_ascii=False, indent=2)
        if c != self.last_counts:
            self.last_counts = c
            self.f_timeline.write('%d,%s,%d,%d,%d,%d,%d\n' % (
                self.map_frames, self.map_stamp if self.map_stamp is not None else '',
                c['free'], c['occupied'], c['lethal'], c['unknown'], c['cells']))

    def classify(self, x, y):
        if self.grid is None:
            return 'no_map'
        m = self.meta
        col = int((x - m['origin_x']) / m['resolution'])
        row = int((y - m['origin_y']) / m['resolution'])
        if row < 0 or col < 0 or row >= m['height'] or col >= m['width']:
            return 'outside'
        v = int(self.grid[row, col])
        if v < 0:
            return 'unknown'
        if v >= 50:
            return 'occupied'
        return 'free'

    def on_path(self, msg: Path):
        self.counts['paths'] += 1
        self.path_count += 1
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        length = sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
                     for a, b in zip(pts, pts[1:])) if len(pts) > 1 else 0.0
        classes = [self.classify(x, y) for x, y in pts]
        cell_class = {c: classes.count(c) for c in set(classes)}
        crossings = []
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            hit = segment_hits_obstacle(a, b, margin=0.05)
            if hit:
                crossings.append({'segment': i, 'obstacle': hit,
                                  'from': [round(a[0], 3), round(a[1], 3)],
                                  'to': [round(b[0], 3), round(b[1], 3)]})
        rec = {'seq': self.path_count,
               't_wall': round(time.time() - self.t0, 3),
               't_wall_abs': time.time(),
               'n_poses': len(pts),
               'length_m': round(length, 4),
               'unknown': cell_class.get('unknown', 0),
               'free': cell_class.get('free', 0),
               'occupied': cell_class.get('occupied', 0),
               'outside': cell_class.get('outside', 0),
               'no_map': cell_class.get('no_map', 0),
               'unknown_frac': round(cell_class.get('unknown', 0) / len(pts), 4) if pts else None,
               'free_frac': round(cell_class.get('free', 0) / len(pts), 4) if pts else None,
               'crossing_count': len(crossings),
               'crossings': crossings[:10],
               'map_stamp_s': self.map_stamp,
               'path_stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               'first_pt': [round(pts[0][0], 3), round(pts[0][1], 3)] if pts else None,
               'last_pt': [round(pts[-1][0], 3), round(pts[-1][1], 3)] if pts else None,
               'pts': [[round(x, 4), round(y, 4)] for x, y in pts]}
        self.paths.append(rec)
        try:
            self.f_paths.write(json.dumps(rec, ensure_ascii=False) + '\n')
        except Exception as exc:
            sys.stderr.write('nav4_recorder 写路径记录失败：%s\n' % exc)

    def on_state(self, msg: String):
        self._write(self.f_states, {'t_wall': round(time.time() - self.t0, 3),
                                    't_wall_abs': time.time(), 'state': msg.data})

    def on_rosout(self, msg: Log):
        if msg.name.split('.')[-1] not in ('goal_navigator', 'controller_server',
                                           'planner_server'):
            return
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self._write(self.f_rosout,
                    {'t_wall': round(time.time() - self.t0, 3), 't_wall_abs': time.time(),
                     'stamp_s': stamp, 'level': int(msg.level), 'node': str(msg.name),
                     'msg': str(msg.msg)})

    def on_status(self, msg: GoalStatusArray):
        rows = []
        for s in msg.status_list:
            # **uuid 是 array('B')**：直接塞进 json.dumps 会抛
            # "Object of type uint8 is not JSON serializable"——这个异常在回调里抛出
            # 会把整个 executor 带走（实测：记录器 0.6 s 就退出，什么也没记下来）。
            rows.append({'uuid': [int(b) for b in list(s.goal_info.goal_id.uuid)[:4]],
                         'status': int(s.status),
                         'name': STATUS_NAME.get(int(s.status), str(int(s.status)))})
        if rows:
            self._write(self.f_status, {'t_wall': round(time.time() - self.t0, 3),
                                        't_wall_abs': time.time(), 'goals': rows})

    def on_cmd_vel(self, msg: Twist):
        self.counts['cmd_vel'] += 1

    def on_points(self, msg: PointCloud2):
        self.counts['points'] += 1

    # ------------------------------------------------------------ 秒表
    def on_tick(self):
        self.rate_rows += 1
        self.f_rates.write('%d,%d,%d,%d,%d\n' % (self.rate_rows,
                                                 self.counts['cmd_vel'],
                                                 self.counts['points'],
                                                 self.counts['global_map'],
                                                 self.counts['paths']))
        self.counts = {'cmd_vel': 0, 'points': 0, 'global_map': 0, 'paths': 0}
        if self.stop:
            raise KeyboardInterrupt

    def write_summary(self):
        first = self.paths[0] if self.paths else None
        compact = [{k: v for k, v in p.items() if k != 'pts'} for p in self.paths]
        out = {
            'elapsed_s': round(time.time() - self.t0, 1),
            'map_frames': self.map_frames,
            'paths_published': len(self.paths),
            'first_path': ({k: v for k, v in first.items() if k != 'pts'}
                           if first else None),
            'paths': compact,
            'timeline_rows': self.rate_rows,
            'note': 'plan_paths.jsonl 里每条都带完整点列；本 summary 去掉点列便于阅读。'
                    '约定：goal_navigator 只在"接受并重发"时打 /rtk/plan_path，'
                    '被抖动抑制/最小间隔压住的不会出现。',
        }
        with open(os.path.join(self.out, 'summary.json'), 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=3600.0,
                        help='最长运行时间（秒），到时自己收尾写 summary')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Recorder(args.out_dir)

    def handler(signum, frame):
        node.stop = True

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)

    deadline = time.time() + args.duration
    try:
        while rclpy.ok() and not node.stop and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.write_summary()
        except Exception as exc:                       # 收尾失败不影响主流程
            print('写 summary 失败：%s' % exc, file=sys.stderr)
        for fh in (node.f_timeline, node.f_paths, node.f_rosout, node.f_states,
                   node.f_status, node.f_rates):
            try:
                fh.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

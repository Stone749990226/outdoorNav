#!/usr/bin/env python3
"""路线跟随控制器：读**绝对位姿** + **代价地图**，出 cmd_vel。

位姿来源：TF map → base_footprint（由 EKF 融合 RTK/IMU/低权重里程计产生）。
安全判据：/rtk/costmap_danger —— nav2 局部代价地图给出的"前方扇区内最近障碍距离"。
（第三阶段起该距离由 nav2 的 `local_costmap` 膨胀层产生，不再是自写地图；
 第二阶段的近场 360 度激光通路已按 AuYang 的决定移除。）
两者之外不看任何东西：不订阅真值，不订阅里程计，不订阅相机原图。

退出码约定（与验收口径一一对应）：
    0  正常跑完整条路线
    3  相机/代价地图失效（断流、长时间没有相机信息）——**不得凭 RTK 继续行驶**
    5  代价地图前方危险区内出现障碍，停车
    6  超过任务总时限
    7  定位失效（TF 缺失或长时间不更新）
    130 被 Ctrl-C 中断
"""

import json
import math
import os
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Float32, String

from robonix_rtk_nav2.clock_guard import ClockGuard
from robonix_rtk_nav2.route_logic import (compute_velocity, cross_track_error,
                                         progress_along_route, wrap_angle)

EXIT_OK = 0
EXIT_ABORT_CAMERA = 3
EXIT_ABORT_OBSTACLE = 5
EXIT_ABORT_TIMEOUT = 6
EXIT_ABORT_POSE = 7


class RouteFollower(Node):

    def __init__(self):
        super().__init__('route_follower')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('danger_topic', '/rtk/costmap_danger')
        self.declare_parameter('costmap_status_topic', '/rtk/costmap_status')
        self.declare_parameter('localization_topic', '/odometry/global')
        # 近场保护（360° 激光）：补相机 0.44 m 以内的盲区
        # 路线：map 坐标下的折线（东, 北），单位米。默认是场地里的一圈矩形，
        # 数值来自场景几何（见 worlds/rtk_courtyard.sdf 的头注释）。
        self.declare_parameter('route', [4.0, -4.0, 4.0, 4.0, -4.0, 4.0, -4.0, -4.0])
        self.declare_parameter('loop_count', 1)
        self.declare_parameter('max_linear_speed', 0.16)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('linear_gain', 0.6)
        self.declare_parameter('angular_gain', 1.4)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_stop_distance', 0.45)
        self.declare_parameter('obstacle_slow_distance', 0.90)
        self.declare_parameter('pose_timeout', 0.7)
        self.declare_parameter('danger_timeout', 1.5)
        self.declare_parameter('startup_grace', 30.0)
        self.declare_parameter('total_timeout', 420.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('report_path', '')

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        flat = list(self.get_parameter('route').value)
        if len(flat) % 2 != 0 or len(flat) < 4:
            raise ValueError('route 必须是 >=4 的偶数个数值')
        self.route = [(float(flat[i]), float(flat[i + 1]))
                      for i in range(0, len(flat), 2)]

        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.obstacle_stop = float(self.get_parameter('obstacle_stop_distance').value)
        self.obstacle_slow = float(self.get_parameter('obstacle_slow_distance').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        self.danger_timeout = float(self.get_parameter('danger_timeout').value)
        self.startup_grace = float(self.get_parameter('startup_grace').value)
        self.total_timeout = float(self.get_parameter('total_timeout').value)
        self.loop_count = int(self.get_parameter('loop_count').value)
        self.report_path = str(self.get_parameter('report_path').value)

        # 时钟守卫：时钟未就绪时不记时间戳（见 clock_guard.py 的说明——
        # 不这么做就会出现"起步即以 costmap_stale（20.84 s）中止"的假故障）
        self.clock_guard = ClockGuard(jump_threshold=2.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.position = None
        self.heading = None
        self.last_pose_time = None
        self.pose_frames = 0
        self.danger = None
        self.last_danger_time = None
        self.costmap_status = None
        self.last_status_time = None
        self.localization_frames = 0
        self.last_localization_time = None

        self.start_time = None
        self.mission_start_sim = None
        self.mission_end_sim = None
        self.target_index = 0
        self.lap = 0
        self.done = False
        self.abort_reason = None
        self.exit_code = EXIT_OK
        self.waypoint_log = []
        self.travelled = 0.0
        self._last_pose = None
        self._travel_anchor = None
        self.cross_track_log = []
        self.slow_events = 0
        self.stop_events = 0
        self._in_danger = False
        self._slow_active = False
        self._report_written = False
        self._last_telemetry = 0.0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        path_qos = QoSProfile(depth=1,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              history=QoSHistoryPolicy.KEEP_LAST,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/rtk/route', path_qos)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        for x, y in self.route:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            self.path_msg.poses.append(ps)
        self.path_pub.publish(self.path_msg)

        self.create_subscription(Float32, self.get_parameter('danger_topic').value,
                                 self.on_danger, 10)
        # 代价地图状态是 **latched**（TRANSIENT_LOCAL）发布的：订阅端必须用同样的
        # durability，否则后启动的节点收不到那条"早就发过"的状态，会一直以为没收到
        # （实测：路线跟随因此报 costmap_status_never_received 而中止）。
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, self.get_parameter('costmap_status_topic').value,
                                 self.on_costmap_status, latched)
        self.create_subscription(Odometry,
                                 self.get_parameter('localization_topic').value,
                                 self.on_localization, 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            '路线跟随启动：%d 个路线点（map 坐标），最高线速度 %.2f m/s，'
            '前方危险阈值 停车 %.2f m / 减速 %.2f m，总时限 %.0f s'
            % (len(self.route), self.max_linear, self.obstacle_stop,
               self.obstacle_slow, self.total_timeout))

    # ------------------------------------------------------------ 回调
    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def sensor_stamp(self, now):
        """时钟就绪时返回可用的时间戳；未就绪返回 None（回调据此不记时间）。"""
        return self.clock_guard.stamp(now)

    def on_danger(self, msg: Float32):
        value = float(msg.data)
        stamp = self.sensor_stamp(self.now_seconds())
        if stamp is None:
            return
        self.danger = value if math.isfinite(value) else None
        self.last_danger_time = stamp
        # 只统计**进入**危险区的次数（边沿计数）。danger 是 10 Hz 发的，
        # 逐条累加会得到一个几百的假数字，看起来像"停了几百次"。
        now_danger = self.danger is not None and self.danger < self.obstacle_stop
        if now_danger and not self._in_danger:
            self.stop_events += 1
        self._in_danger = now_danger

    def on_costmap_status(self, msg: String):
        stamp = self.sensor_stamp(self.now_seconds())
        if stamp is None:
            return
        self.costmap_status = msg.data
        self.last_status_time = stamp

    def on_localization(self, msg):
        self.last_localization_time = self.now_seconds()
        self.localization_frames += 1

    def read_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                 rclpy.time.Time())
        except Exception:
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        self.position = (t.x, t.y)
        self.heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = self.sensor_stamp(self.now_seconds())
        if stamp is None:
            return True          # 位姿可用，但时钟未就绪，不据此判定"新鲜"
        self.last_pose_time = stamp
        self.pose_frames += 1
        # 里程统计：**不能对带噪声的逐帧位姿直接求和**——RTK 噪声 σ≈2 cm，
        # 20 Hz 下静止不动也会累加出"走过几十米"（实测报 40.66 m，真值只有 15.26 m）。
        #
        # 做法：维护一个"锚点"，只有相对锚点的位移超过 0.05 m 才计入里程并把锚点前移。
        # 这样噪声（σ≈2 cm）不会推动锚点，而真实行进每 0.05 m 记一次，
        # 量化误差约 0.05 m / 全圈 28 m ≈ 0.2%。**不要用"相邻两帧位移阈值"**——
        # 20 Hz 下正常行驶每帧只走 0.008 m，会被阈值全部过滤掉（实测里程变成 0.03 m）。
        if self._travel_anchor is None:
            self._travel_anchor = self.position
        step = math.hypot(self.position[0] - self._travel_anchor[0],
                          self.position[1] - self._travel_anchor[1])
        if step > 0.05:
            self.travelled += step
            self._travel_anchor = self.position
        self._last_pose = self.position
        return True

    # ------------------------------------------------------------ 控制
    def abort(self, reason, code):
        self.get_logger().error('安全中止：%s' % reason)
        self.abort_reason = reason
        self.exit_code = code
        self.finish()

    def publish_stop_once(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.done:
            return
        now = self.now_seconds()

        # 时钟跳变（含"从 0 跳到真实时间"的首次就绪）：把新鲜度基准与计时起点全部清空重来，
        # 否则会拿跳变前记下的 0.0 当时间戳，算出一个几十秒的假"过期"。
        if self.clock_guard.update(now):
            self.get_logger().warn(
                '检测到仿真时钟跳变（第 %d 次），重置新鲜度基准与计时起点'
                % self.clock_guard.jumps)
            self.last_pose_time = None
            self.last_danger_time = None
            self.last_status_time = None
            self.start_time = None

        if self.start_time is None:
            self.start_time = now
            self.mission_start_sim = now
        elapsed = now - self.start_time

        pose_ok = self.read_pose()

        # ---- 定位新鲜度 ----
        if not pose_ok or self.last_pose_time is None:
            if elapsed > self.startup_grace:
                self.abort('localization_never_received（启动后 %.1f s 内没有 map→%s）'
                           % (elapsed, self.base_frame), EXIT_ABORT_POSE)
            else:
                self.publish_stop_once()
            return
        if (now - self.last_pose_time) > self.pose_timeout:
            self.abort('localization_stale（曾经收到过，已 %.2f s 没有新的 map→%s）'
                       % (now - self.last_pose_time, self.base_frame), EXIT_ABORT_POSE)
            return

        # ---- 相机 / 代价地图 ----
        if self.last_status_time is None:
            if elapsed > self.startup_grace:
                self.abort('costmap_status_never_received（启动后 %.1f s 内没有代价地图状态）'
                           % elapsed, EXIT_ABORT_CAMERA)
            else:
                self.publish_stop_once()
            return
        # 存活性用**状态与危险距离两条里更新的那条**来判断：两者都由 costmap 发布，
        # 任一条还在来就说明那个节点活着。实测出现过"状态话题偶发停了 10 s、
        # 而危险距离一直在发"的情况，只看状态会误判成相机失效（实测退出码 3）。
        last_seen = max(self.last_status_time or 0.0, self.last_danger_time or 0.0)
        if (now - last_seen) > self.danger_timeout:
            self.abort('costmap_stale（代价地图的状态与危险距离都已 %.2f s 没有更新）'
                       % (now - last_seen), EXIT_ABORT_CAMERA)
            return
        # 第三阶段：`costmap_stale` 是"nav2 局部代价地图不再更新"（点云还在但图停了），
        # 与 `stale`（点云断流）、`no_camera_info`（点云从未到达）都归到 3 号出口
        # ——文档里 3 号的含义本来就是"相机/代价地图失效"，不另开退出码。
        if self.costmap_status in ('stale', 'no_camera_info', 'costmap_stale'):
            self.abort('camera_lost（代价地图状态 %s：感知链路失效，不凭 RTK 继续走）'
                       % self.costmap_status, EXIT_ABORT_CAMERA)
            return
        if self.costmap_status == 'no_pose':
            self.abort('costmap_no_pose（代价地图拿不到绝对位姿）', EXIT_ABORT_POSE)
            return
        if self.costmap_status != 'ok':
            self.publish_stop_once()
            return

        # ---- 前方危险（nav2 局部代价地图）----
        # 第三阶段去掉了近场 360 度激光（AuYang 定，见 docs/rtk-nav2-brief.md §1），
        # 所以这里只剩"前方扇区"这一条停车通路。
        # **边界（必须记住）**：相机水平视场 60 度，视场之外（侧后方）的障碍要等机器人
        # 转过去才会进图；v1 不考虑侧向/后方贴障碍通过的剐蹭（用户已确认的口径）。
        if self.danger is not None and self.danger < self.obstacle_stop:
            self.abort('obstacle（代价地图前方 %.2f m < 停车阈值 %.2f m）'
                       % (self.danger, self.obstacle_stop), EXIT_ABORT_OBSTACLE)
            return

        if elapsed > self.total_timeout:
            self.abort('total_timeout（%.0f s）' % elapsed, EXIT_ABORT_TIMEOUT)
            return

        # ---- 路线跟随 ----
        target = self.route[self.target_index]
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        distance = math.hypot(dx, dy)
        angle_error = wrap_angle(math.atan2(dy, dx) - self.heading)

        # 横向跟踪误差记录在**当前航段**上（离线评估用真值再算一遍，见 trajectory_report）
        self.cross_track_log.append(
            (round(elapsed, 3), round(cross_track_error(self.position, self.route,
                                                        self.target_index), 4)))

        if distance <= self.goal_tolerance:
            self.waypoint_log.append({'index': self.target_index, 'lap': self.lap,
                                      'x': target[0], 'y': target[1],
                                      't': round(elapsed, 3)})
            self.get_logger().info('到达路线点 %d (%.2f, %.2f)，用时 %.1f s'
                                   % (self.target_index, target[0], target[1], elapsed))
            self.target_index += 1
            if self.target_index >= len(self.route):
                self.target_index = 0
                self.lap += 1
                if self.lap >= self.loop_count:
                    self.get_logger().info('路线跑完 %d 圈，正常结束' % self.lap)
                    self.exit_code = EXIT_OK
                    self.finish()
                    return
            return

        linear, angular = compute_velocity(angle_error, distance, self.goal_tolerance,
                                           self.max_linear, self.max_angular,
                                           self.angular_gain, self.linear_gain)

        # 减速区：代价地图前方有东西但还没到停车阈值，按距离线性降速
        if self.danger is not None and self.danger < self.obstacle_slow:
            ratio = max(0.0, (self.danger - self.obstacle_stop) /
                        max(1e-6, (self.obstacle_slow - self.obstacle_stop)))
            linear *= ratio
            if not self._slow_active:
                self.slow_events += 1      # 同样是边沿计数，不是逐条累加
            self._slow_active = True
        else:
            self._slow_active = False

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

        # 1 Hz 遥测：调参/排障时看控制器到底在按什么决策
        if (now - self._last_telemetry) >= 1.0:
            self._last_telemetry = now
            self.get_logger().info(
                'TELEM t=%.1f target=%d pos=(%.2f, %.2f) yaw=%.1f° dist=%.2f '
                'ang_err=%.1f° v=%.3f w=%.3f danger=%s'
                % (elapsed, self.target_index, self.position[0], self.position[1],
                   math.degrees(self.heading), distance, math.degrees(angle_error),
                   linear, angular,
                   'None' if self.danger is None else '%.2f' % self.danger))

    def finish(self):
        if self.done:
            return
        self.mission_end_sim = self.now_seconds()
        self.publish_stop_once()
        # 再发几次零速度，确保底盘收到（单次发布可能被丢）
        for _ in range(3):
            self.publish_stop_once()
        self.done = True
        self.timer.cancel()
        self.get_logger().info('已发送零速度指令')
        self.write_report()

    def write_report(self):
        if self._report_written:
            return
        self._report_written = True
        report = {
            'exit_code': self.exit_code,
            'abort_reason': self.abort_reason,
            'route': [list(p) for p in self.route],
            'reached': self.waypoint_log,
            'loop_count': self.loop_count,
            # 车载里程**只能是估计**：RTK 单点噪声 σ≈2 cm，任何"逐帧/锚点"累计都会虚涨
            # （实测：逐帧求和 41.4 m、锚点法 24.6 m，而真值只有 15.3 m）。
            # 因此这个字段改名并明确标注，**官方里程请用离线报告里的 path_length_truth_m**。
            'travelled_estimate_m': round(self.travelled, 3),
            'travelled_method': '车载估计（锚点法，仅参考）：逐帧求和会被 RTK 噪声虚涨、'
                                '锚点法也会被噪声推着走，实测都明显大于真值。'
                                '**正式里程用 trajectory_report.json 的 path_length_truth_m**（真值口径）',
            'mission_start_sim_s': self.mission_start_sim,
            'mission_end_sim_s': self.mission_end_sim,
            'pose_frames': self.pose_frames,
            'localization_frames': self.localization_frames,
            'last_danger_m': self.danger,
            'stop_events': self.stop_events,
            'slow_events': self.slow_events,
            'limits': {
                'max_linear_speed': self.max_linear,
                'max_angular_speed': self.max_angular,
                'goal_tolerance': self.goal_tolerance,
                'obstacle_stop_distance': self.obstacle_stop,
                'obstacle_slow_distance': self.obstacle_slow,
                'pose_timeout': self.pose_timeout,
                'danger_timeout': self.danger_timeout,
                'total_timeout': self.total_timeout,
            },
        }
        if self.cross_track_log:
            report['cross_track_estimate'] = {
                'samples': len(self.cross_track_log),
                'note': '控制器自己按估计位姿算的横向偏差；验收请用 trajectory_report 按真值算的那份',
                'max_abs_m': max(abs(v) for _, v in self.cross_track_log),
            }
        if self.report_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.report_path)), exist_ok=True)
            with open(self.report_path, 'w', encoding='utf-8') as handle:
                json.dump(report, handle, ensure_ascii=False, indent=2)
        # 报告也打到标准输出，方便在日志里直接看
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


def main(argv=None):
    rclpy.init(args=argv)
    node = RouteFollower()
    exit_code = EXIT_OK
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = node.exit_code
    except KeyboardInterrupt:
        node.get_logger().warn('被中断')
        node.exit_code = 130
        exit_code = 130
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

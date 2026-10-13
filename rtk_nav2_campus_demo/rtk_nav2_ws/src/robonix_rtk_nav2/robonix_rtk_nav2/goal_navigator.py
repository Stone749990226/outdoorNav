#!/usr/bin/env python3
"""导航入口（薄层）：规划 → 跟随 → **实时重规划**（第四步 brief §2 裁定 3）。

**不用 `bt_navigator`/行为树**——那会带来不可控的后退-旋转-清图 recovery，对"看得见的演示"
是风险不是收益。这里把循环写明白，时机可控可解释：

    目标（`/goal_pose` 或命令行）→ ComputePathToPose(use_start:=false) → FollowPath
       ↑                                                                  │
       └── 三种触发重规划：① 1 Hz 周期 ② 全局图有更新 ③ FollowPath 失败/被取消

**抖动抑制**：新路径与"正在走的那条"差异小于阈值时**不重发**。阈值 `jitter_lateral_m` 0.45 /
`jitter_length_m` 1.20（brief §2 原本建议 0.25/0.30，**实测不能用**：仅仅"地图在长"就会带来
0.31~0.45 m 的长度差，那个阈值会被地图生长本身触发，几乎每张新图都取消重发一次 → 车走不动）。
`FollowPath` 是 action、一次只能有一个 goal；**换路径不要自己取消**，直接把新 goal 发出去，
nav2 会自己抢占旧的（理由与实测数据见 `send_follow`）。抖动门之外再叠一道
`min_resend_interval_s` 时间下限，避免"每秒重发"让车在原地来回修正。

**到达**：以 `FollowPath` 成功返回为准（goal_checker 的 `xy_goal_tolerance` = 0.25 m）。
到达/重规划次数、以及每次触发的原因都打日志——现场解说要用。
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def path_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def _dist_to_segment(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / den))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def max_lateral_diff(new_pts, old_pts):
    """新路径相对旧路径的**最大横向差**（单向，够用且便宜）。

    只从新路径往旧路径投影：要判断的是"新路径有没有明显偏离在走的那条"。
    """
    if not new_pts or len(old_pts) < 2:
        return float('inf')
    worst = 0.0
    for p in new_pts:
        d = min(_dist_to_segment(p, a, b) for a, b in zip(old_pts, old_pts[1:]))
        worst = max(worst, d)
    return worst


class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')

        self.declare_parameter('replan_period_s', 1.0)
        # 抖动抑制阈值（brief §2 建议值）
        # ★ 2026-10-10（reviewer 裁定）：**代码默认值改成"实测可用"的那一组** ✓。
        #   原来这里写 0.25 / 0.30，而 launch 覆盖成 0.45 / 1.20 —— 可 docstring 自己写着
        #   0.25/0.30"**实测不能用**" ✗ ⇒ **脱离 launch 单独跑这个节点就会用上不可用的阈值** ✗。
        #   默认值必须是"能用的"，覆盖才是可选的 ✓（"注释不是凭据"的又一落点）。
        self.declare_parameter('jitter_lateral_m', 0.45)
        self.declare_parameter('jitter_length_m', 1.20)
        # ★★ 窄修（reviewer 2026-10-10 裁定 ②）：**同一目标**的 SUCCEEDED 结果不得丢弃 ★★
        #   token 的用途是"丢弃**过期目标**的结果"，**不是**"丢弃**同一目标**的结果" ✗ ——
        #   实测：`Reached the goal` 出现 4 次、每次因 token 过期被丢 ⇒ `arrived` 永不置位 ⇒
        #   图一更新就重发 ⇒ 车被反复下命令、**离开刚到的点** ✗（wp0 最小 0.0234 m、最终 4.08 m）。
        #   **默认关** ✓ —— 保住阶段四~七的行为（它们的结论继续可复跑 ✓）；
        #   **只在需要"多航点顺序停靠"时开**（`geo-route.sh` 用 `ros2 param set` 打开 ✓）。
        #   ★ 用 `get_parameter` **每次现读** ⇒ 运行中 `param set` 立刻生效 ✓（不必重起栈）。
        self.declare_parameter('accept_superseded_same_goal', False)
        # **最小重发间隔**：触发②是"全局图 stamp 更新"（约 1 Hz），在地图正在长的那段时间里
        # 路径几乎每拍都会超过变化阈值 → 没有这条时间下限就会**每秒重发一次**，
        # 车在原地来回修正、"走一步顿一下"，现场看起来像车坏了，而不是"实时重规划"。
        # 所以变化阈值之外再加一条**时间下限**：两次重发至少隔这么久。
        # **实测若发现车顿，先调这条，别去调控制器参数**（那是另一回事，会让问题更难查）。
        # **这道门要真正生效，前提是"换路径时不自己取消"**：自己取消会造出"没有活跃路径"的窗口，
        # 而空闲时那条"跳过抑制直接重发"会绕过这道门——实测去 A 那段 62 次重发、这道门只压住 1 次。
        self.declare_parameter('min_resend_interval_s', 2.0)
        self.declare_parameter('path_timeout_s', 10.0)
        self.declare_parameter('global_map_topic', '/global_costmap/costmap')

        self.period = float(self.get_parameter('replan_period_s').value)
        self.jitter_lat = float(self.get_parameter('jitter_lateral_m').value)
        self.jitter_len = float(self.get_parameter('jitter_length_m').value)
        self.min_resend = float(self.get_parameter('min_resend_interval_s').value)
        self.last_resend_time = 0.0
        self.throttled_by_interval = 0

        self.goal = None                 # (x, y)
        self.current_path = None         # nav_msgs/Path（正在走的那条）
        self.current_pts = []
        self.arrived = False
        self.replan_count = 0
        self.plan_count = 0
        self.skipped_by_jitter = 0
        self.arrivals = []

        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.follow_handle = None
        self.follow_in_flight = False
        # **"已派发"与"已收到结果"的计数器**，用来把 nav2 的**抢占**跟**真失败**分开。
        # 为什么要计数器、而且为什么不能靠 `follow_in_flight` 这个标志：
        # 我们每次换路径都会**直接发新 goal**，nav2 会把旧的那条以 `status=6`（ABORTED）结束。
        # 实测一次会话里 `status=6` **46 次**，而控制器真正自己 abort 的只有 **3 次**——
        # 也就是说绝大多数 6 是**抢占**，不是失败。可 `follow_in_flight` 在"上一条结果已经回来"
        # 之后就是 False，拿它当"我替换过"的凭据会**漏判**（实测两种情形交替出现：
        # 一会儿被正确忽略、一会儿又被当成失败），于是"抢占→当失败→再重发→又抢占"的自持循环
        # 每 ~1.4 s 一轮，`min_resend_interval_s` 那道门永远轮不到（只压住 1~2 次）。
        # **不变式**：每个被接受（accepted）的 goal 恰好产生一个 result。
        # 所以"收到的 result 数 < 已派发的 goal 数"⇒ 这一条属于**更早的、已被我们替换掉的** goal，
        # 与当前意图无关 → 忽略（`status==4` 成功仍照常判到达）。
        # 这与回调到达的先后次序无关，因此不存在竞态。
        self.follow_goals_sent = 0
        self.follow_results = 0
        # Token each accepted FollowPath goal so a late result from a goal
        # superseded by Nav2 cannot clear the state of the current goal.
        self.follow_request_token = 0
        self.active_follow_token = None
        # token → 该次请求对应的**目标点**（窄修用：判断"过期 token 的结果"是不是**同一个目标**）✓
        self.token_goal = {}
        self.last_global_stamp = None
        self.seen_map_stamp = None       # 已经据此触发过的那个全局图时间戳
        self.pending_since = None        # 非 None 表示"到点就去规划"（首次 / 失败后重来）

        # ★ 必须用 LATCHED（= depth1 + RELIABLE + TRANSIENT_LOCAL），不能用默认的 volatile：
        #   RViz 的 PlanPath 显示（`config/demo.rviz`）请求的是 Transient Local ⇒ **durability 不兼容时
        #   订阅端一条都收不到**（发布方只打一条 `incompatible QoS … Last incompatible policy: DURABILITY` 警告，
        #   很容易被当成"发了但没显示"）⇒ 现场那条路径线不会出现。
        #   用 LATCHED 同时兼容两类订阅者（要 transient_local 的 RViz ✓、volatile 的普通订阅者 ✓），
        #   且**晚加入的显示能立刻看到最后一条路径**（RViz 起晚了也不白屏）✓。
        self.path_pub = self.create_publisher(Path, '/rtk/plan_path', LATCHED)
        self.state_pub = self.create_publisher(String, '/rtk/goal_navigator_state', LATCHED)

        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 1)
        self.create_subscription(OccupancyGrid,
                                 self.get_parameter('global_map_topic').value,
                                 self.on_global_map, LATCHED)

        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            '导航入口就绪：等 /goal_pose（RViz 2D Goal Pose）或 nav-to.sh；'
            '重规划周期 %.1f s，抖动抑制 横向<%.2f m 且 长度差<%.2f m'
            % (self.period, self.jitter_lat, self.jitter_len))

    # ------------------------------------------------------------ 输入
    def on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        # **同一个目标重复到达要当没发生**（幂等）。两个现实来源：
        # ① `nav-to.sh` 为了绕开 ROS 2 发现竞态会连发几次（`ros2 topic pub --once`
        #    的那条消息经常在"发布者还没跟订阅者匹配上"时就丢了，实测就是这样）；
        # ② RViz 里手抖点两下同一个点。
        # 不幂等的后果很隐蔽：每次 `on_goal` 都会清空计数并把在走的 `FollowPath` 取消掉，
        # 于是控制器**一次都没跑满**——现象是"/cmd_vel 只有角速度、车不动、
        # controller_server 报 Failed to make progress"，看起来像控制器或底盘的问题，
        # 实际是我自己在 2 秒内把目标取消了 4 次（这一条是实测追出来的）。
        if (self.goal is not None and not self.arrived
                and math.hypot(p.x - self.goal[0], p.y - self.goal[1]) < 0.05):
            self.get_logger().info(
                '重复目标 (%.2f, %.2f) 与在办目标重合，忽略（不重置、不取消在走的路径）'
                % (p.x, p.y))
            return
        self.goal = (p.x, p.y)
        self.arrived = False
        self.current_path = None
        self.current_pts = []
        self.replan_count = 0
        self.skipped_by_jitter = 0
        self.throttled_by_interval = 0
        self.last_resend_time = 0.0
        self.pending_since = time.time()
        self.get_logger().info('收到目标 (%.2f, %.2f)，开始规划' % self.goal)
        self.set_state('heading_to_goal(%.2f,%.2f)' % self.goal)

    def on_global_map(self, msg: OccupancyGrid):
        # 用 header.stamp 判"全局图有更新"——比逐格比对便宜，也足够触发重规划
        self.last_global_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def set_state(self, text):
        m = String()
        m.data = text
        self.state_pub.publish(m)

    # ------------------------------------------------------------ 主循环
    def tick(self):
        """三种触发都在这一个时钟里判（brief §2 裁定 3）。

        **顺序即优先级，这个顺序是实测改出来的**（第四步判据 4/6 失败的直接根因）：
        "把暂存的路径发给 controller"必须排在"全局图有更新"**前面**。
        原来的顺序把"全局图更新"放前面并且直接 `return`，而全局图几乎每拍都有新时间戳，
        于是它每次都抢在前面 → 重规划时**暂存的那条路径永远发不出去**
        （当时的设计是"取消旧 goal → 把新路径暂存起来 → 下一拍再发"；**现在换路径已改成
        直接发新 goal、由 nav2 抢占旧的**，见 `send_follow` 的说明）。
        → 车没有活跃的 `FollowPath` → controller 没有路径就不发 `/cmd_vel` → 车停住。
        现象很有迷惑性：规划在正常返回（128 条全部 SUCCEEDED）、图在正常长，
        但 `/cmd_vel` 1191 s 里只有 44 s 有输出（3.7%），看起来像控制器或底盘的问题。
        """
        if self.goal is None or self.arrived:
            return
        now = time.time()

        # ① 先把"欠着的那次规划"做掉（首次规划 / `FollowPath` 失败后重来）。
        #    换路径不需要走这条路——换路径是**直接发新 goal**、由 nav2 抢占旧的（见 send_follow）。
        if self.pending_since is not None and now >= self.pending_since:
            self.pending_since = None
            self.plan_and_follow('周期/首次')
            return

        # ② 全局图有更新 —— 边建边改的主要来源。**在走也触发**（这正是"实时重规划"），
        #    抖动抑制与最小重发间隔负责不让它变成乱重发。
        if (self.last_global_stamp is not None
                and self.last_global_stamp != self.seen_map_stamp):
            self.seen_map_stamp = self.last_global_stamp
            self.plan_and_follow('全局图更新')
            return

    def plan_and_follow(self, reason):
        if not self.path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('等不到 compute_path_to_pose 动作服务')
            return
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.orientation.w = 1.0
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = float(self.goal[0])
        goal_msg.goal.pose.position.y = float(self.goal[1])
        goal_msg.goal.pose.orientation.w = 1.0
        # **use_start:=false** → 规划器自己取当前位姿作起点（brief §2 裁定 3 指定）
        goal_msg.use_start = False

        fut = self.path_client.send_goal_async(goal_msg)
        fut.add_done_callback(lambda f: self.on_plan_done(f, reason))

    def on_plan_done(self, fut, reason):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('规划目标被拒绝（%s）' % reason)
            return
        res = handle.get_result_async()
        res.add_done_callback(lambda f: self.on_path(f, reason))

    def on_path(self, fut, reason):
        try:
            path = fut.result().result.path
        except Exception as exc:
            self.get_logger().warn('取路径失败：%s' % exc)
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if len(pts) < 2:
            self.get_logger().warn('规划器返回空路径（%s）' % reason)
            return

        self.plan_count += 1
        # **抖动抑制与最小间隔只在"确实有 `FollowPath` 在走"时才允许生效**（实测的坑）：
        # 若此刻没有活跃的 FollowPath（刚被取消、或上一次规划被丢弃），再抑制就等于
        # **让车一直没目标**——controller 没有路径就不发 `/cmd_vel`，车停住；
        # 而抑制那两行日志分别是 DEBUG / INFO，前者看不见，很容易当成"控制器不动"。
        # 判据 4/6 失败的另一半原因就在这里。
        if self.current_pts and self.follow_in_flight:
            lat = max_lateral_diff(pts, self.current_pts)
            dlen = abs(path_length(pts) - path_length(self.current_pts))
            if lat < self.jitter_lat and dlen < self.jitter_len:
                self.skipped_by_jitter += 1
                # 抖动抑制不是"没干活"，但对演示没意义，压到 debug
                self.get_logger().debug(
                    '重规划被抖动抑制跳过：横向差 %.3f m < %.2f，长度差 %.3f m < %.2f'
                    % (lat, self.jitter_lat, dlen, self.jitter_len))
                return
            # **时间下限**：变化够大，但离上次重发太近 → 先不重发（继续走当前这条）。
            # 这一条是为了避免"每秒取消重发一次"造成的顿挫（见参数处的说明）。
            since = time.time() - self.last_resend_time
            if since < self.min_resend:
                self.throttled_by_interval += 1
                self.get_logger().info(
                    '重规划被**最小重发间隔**压住：距上次 %.2f s < %.1f s'
                    '（本次变化 横向 %.3f m / 长度差 %.3f m，已记账不重发）'
                    % (since, self.min_resend, lat, dlen))
                return
            self.get_logger().info(
                '重规划 #%d（%s）@%.2f：横向差 %.3f m、长度差 %.3f m —— 已重发'
                % (self.replan_count + 1, reason, time.time(), lat, dlen))
        elif self.current_pts:
            # 有旧路径但**没有 FollowPath 在走**：不能抑制，否则车就停在这儿了
            self.get_logger().info(
                '重规划 #%d（%s）@%.2f：当前没有活跃的 FollowPath，跳过抑制直接重发'
                % (self.replan_count + 1, reason, time.time()))
        else:
            self.get_logger().info('首次规划（%s）@%.2f：%d 点 / %.2f m'
                                   % (reason, time.time(), len(pts), path_length(pts)))

        self.replan_count += 1
        self.last_resend_time = time.time()
        self.current_path = path
        self.current_pts = pts
        self.path_pub.publish(path)
        self.send_follow(path, reason)

    def send_follow(self, path, reason):
        # **换路径不要自己取消**（这条是 reviewer 从实测数据里定的，我原来那版是错的）。
        # 我原来先显式 `cancel_goal_async()`、再在取消回调里发新 goal，本意是"别丢拍"，
        # 结果造出一个**自持循环**：取消 → 下一拍"没有活跃 `FollowPath`" →
        # 而"空闲时跳过抑制"那条（我为了治另一处楔死加的）就**立刻再重发** → 又取消……
        # 于是 `min_resend_interval_s` 这道门**永远轮不到**。实测去 A 那段：
        # **62 次重发 / 95 次规划，间隔门只压住 1 次，重发间隔约 1.5 s**。
        # **正确做法是根本不取消**：直接把新 goal 发出去，nav2 会自己抢占旧的
        # —— 我们自己日志里那条 `status=6`（ABORTED 而不是 CANCELED）就是"nav2 自动抢占"的证据。
        # 不取消就没有"没有活跃路径"的窗口，间隔门与抖动门才真正生效，运动也不再是断续的。
        self._do_send_follow(path, reason)

    def _do_send_follow(self, path, reason):
        if not self.follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('等不到 follow_path 动作服务')
            return
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        self.follow_request_token += 1
        token = self.follow_request_token
        # ★ 记下"这次请求对应哪个目标点" —— 窄修要拿它判"是不是同一个目标" ✓
        self.token_goal[token] = tuple(self.goal) if self.goal else None
        if len(self.token_goal) > 200:                 # 只留最近 200 个，防字典无限长 ✓
            for k in sorted(self.token_goal)[:-200]:
                self.token_goal.pop(k, None)
        fut = self.follow_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_follow_started(f, token))

    def on_follow_started(self, fut, token):
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('FollowPath 目标被拒绝')
            return
        # 只有**被接受**的 goal 才会产生 result，所以在这里计数才能对上（见 __init__ 的说明）
        self.follow_goals_sent += 1
        if self.active_follow_token is None or token > self.active_follow_token:
            self.follow_handle = handle
            self.follow_in_flight = True
            self.active_follow_token = token
            self.pending_since = None
        res = handle.get_result_async()
        res.add_done_callback(lambda f: self.on_follow_result(f, token))

    def on_follow_result(self, fut, token):
        # A result callback for a superseded goal can arrive after the newer
        # goal was accepted. Only the active token may clear controller state.
        is_active = token == self.active_follow_token
        if is_active:
            self.follow_in_flight = False
            self.follow_handle = None
        try:
            status = fut.result().status
            result = fut.result().result
        except Exception as exc:
            self.get_logger().warn('FollowPath 结果异常：%s' % exc)
            return
        # **这条 `+= 1` 必须无条件执行，不能只写在"非成功"分支里**（2026-09-21 修）。
        # 不变式是"每个被接受的 goal 恰好产生一个 result"，上面 `follow_goals_sent`
        # 在 goal **被接受**时加一；如果这里只在失败时加一，那么**成功一条**
        # 就会让 `follow_results` 永久落后 1 → 不等式 `follow_results < follow_goals_sent`
        # 被无端满足 → **成功一段之后的第一次真失败会被当成"被替换掉的旧 goal"吞掉、
        # 不触发重规划**。它平时看不出来，是因为另一条旁路（"空闲即重发"）在救它——
        # **两个 bug 互相掩盖**是最难查的一类，所以计数必须在唯一的分叉之前加。
        self.follow_results += 1
        # A superseded goal is not evidence about the controller's current
        # path. Its result is still counted for diagnostics, then ignored.
        # ★★ 窄修：**同一目标**的 SUCCEEDED 结果要认下来（reviewer 2026-10-10 裁定 ②）★★
        #   原来这里**无条件**丢弃非当前 token 的结果 ✗ ⇒ 实测 `Reached the goal` 4 次全被丢、
        #   `arrived` 永不置位 ⇒ 图一更新就重发 ⇒ **车离开刚到的点** ✗。
        #   ⇒ 若 ① 功能已开 ② status=SUCCEEDED ③ 该 token 记的目标与**当前目标**一致（≤1 mm）
        #      ⇒ 认下来、置 `arrived` ✓（"过期目标"的结果照旧丢弃 ✓，安全性没丢）。
        if not is_active:
            superseded_same_goal = False
            try:
                if (bool(self.get_parameter('accept_superseded_same_goal').value)
                        and status == 4 and self.goal is not None):
                    g = self.token_goal.get(token)
                    superseded_same_goal = bool(
                        g is not None
                        and math.hypot(g[0] - self.goal[0], g[1] - self.goal[1]) <= 0.001)
            except Exception:                                  # noqa: BLE001
                superseded_same_goal = False
            if superseded_same_goal:
                self.arrived = True
                self.arrivals.append(self.goal)
                self.get_logger().info(
                    '**到达** 目标 (%.2f, %.2f)：结果来自被换掉的 token=%d，但**目标相同** ⇒ 认下'
                    % (self.goal[0], self.goal[1], token))
                self.set_state('arrived(%.2f,%.2f)' % self.goal)
                self.current_path = None
                self.current_pts = []
                return
            self.get_logger().info(
                '忽略非当前 FollowPath 结果（token=%d，当前=%s，status=%s）'
                % (token, self.active_follow_token, status))
            return
        # 4 = SUCCEEDED（rclpy.action 的 GoalStatus）
        if status == 4:
            self.arrived = True
            self.arrivals.append(self.goal)
            self.get_logger().info(
                '**到达** 目标 (%.2f, %.2f)：本段规划 %d 次、重发 %d 次、'
                '抖动抑制跳过 %d 次、最小间隔压住 %d 次'
                % (self.goal[0], self.goal[1], self.plan_count,
                   self.replan_count, self.skipped_by_jitter,
                   self.throttled_by_interval))
            self.set_state('arrived(%.2f,%.2f)' % self.goal)
            self.current_path = None
            self.current_pts = []
        else:
            # **被我们主动换掉的那条 goal，不管它以什么码结束都不算失败。**
            # 实测它返回的是 6（ABORTED）而不是 5（CANCELED）：新 goal 紧跟着发出去，
            # nav2 的 action server 会因为"来了新 goal"把旧的 abort 掉。
            # 但 **status==4（成功）要继续按到达处理**，别把真到达吞掉。
            # （计数已在上面无条件加过；能走到这里必然 status != 4。）
            if self.follow_results < self.follow_goals_sent:
                self.get_logger().info(
                    '这条 FollowPath 属于已被我们替换掉的旧 goal（status=%s，'
                    '已派发 %d / 已收到 %d）—— 不计为失败、不触发重规划'
                    % (status, self.follow_goals_sent, self.follow_results))
                return
            # 失败/被取消 → 触发重规划（brief §2 的第三种触发）
            self.get_logger().warn('FollowPath 未成功（status=%s → %s），触发重规划'
                                   % (status, getattr(result, 'error_msg', '') or result))
            self.pending_since = time.time()
            self.set_state('replan_after_follow_status_%s' % status)


def main(argv=None):
    rclpy.init(args=argv)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

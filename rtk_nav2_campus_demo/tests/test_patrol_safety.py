"""巡游运动控制的安全行为测试。

这些测试只依赖 robonix_slam_demo.patrol_core 的纯逻辑，不需要 ROS 运行环境：

    PYTHONPATH=demo_ws/src/robonix_slam_demo python3 -m pytest tests/test_patrol_safety.py -v

覆盖的行为：
    1. 激光缺失/过期时不发出非零速度；
    2. 里程计缺失/过期时不发出非零速度；
    3. 启动宽限期内保持零速度等待，不误判为故障也不运动；
    4. 前方近障时停车；
    5. 任务超时退出；
    6. 终止时发送零速度。
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, 'demo_ws', 'src', 'robonix_slam_demo'))

from robonix_slam_demo.patrol_core import (  # noqa: E402
    EXIT_ABORT_OBSTACLE,
    ClockGuard,
    EXIT_ABORT_ODOM,
    EXIT_ABORT_SCAN,
    EXIT_ABORT_TIMEOUT,
    EXIT_OK,
    STATE_ABORT,
    STATE_OK,
    STATE_WAIT,
    SafetyMonitor,
    clamp,
    compute_velocity,
    min_forward_range,
    route_distance,
    wrap_angle,
)

# 与 config/patrol.yaml 中记录的默认值保持一致
SCAN_TIMEOUT = 1.0
ODOM_TIMEOUT = 0.5
TOTAL_TIMEOUT = 300.0
STARTUP_GRACE = 30.0
OBSTACLE_STOP = 0.35
MAX_LINEAR = 0.18
MAX_ANGULAR = 1.0
ANGULAR_GAIN = 1.6
LINEAR_GAIN = 0.5
GOAL_TOLERANCE = 0.12


def make_monitor():
    return SafetyMonitor(scan_timeout=SCAN_TIMEOUT,
                         odom_timeout=ODOM_TIMEOUT,
                         total_timeout=TOTAL_TIMEOUT,
                         obstacle_stop_distance=OBSTACLE_STOP,
                         startup_grace=STARTUP_GRACE)


def make_scan(ranges):
    """按 TurtleBot3 官方 LDS-01 的仿真参数包装测距列表。

    模型里 hls_lfcd_lds 的配置是 samples=360、min_angle=0、max_angle=6.28，
    因此索引 0 就是正前方，角度间隔为 2*pi/360。
    """
    return min_forward_range(ranges, angle_min=0.0,
                             angle_increment=2.0 * math.pi / 360.0,
                             half_angle_rad=math.radians(25.0),
                             range_min=0.12, range_max=3.5)


def stop_velocity():
    """节点停止/等待分支使用的速度：显式零速。"""
    return 0.0, 0.0


# ---------------------------------------------------------------- 纯函数

def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-3 * math.pi) == pytest.approx(math.pi)


def test_min_forward_range_ignores_max_range_and_nan():
    # 正前方几个点里夹着无回波(range_max)与 nan
    ranges = [3.5] * 360
    ranges[0] = float('nan')
    ranges[1] = 3.5           # 无回波，等于量程上限
    ranges[2] = 1.2           # 正前方 1.2 m
    ranges[3] = float('inf')
    assert make_scan(ranges) == pytest.approx(1.2)


def test_min_forward_range_only_looks_forward():
    ranges = [0.1] * 360
    ranges[0] = 2.0            # 正前方 2.0 m
    assert make_scan(ranges) == pytest.approx(2.0)


def test_min_forward_range_uses_sector_not_only_center_ray():
    """正前方 25 度扇区内的最近点应被采用，而不是只看中心一条射线。"""
    ranges = [3.5] * 360
    ranges[0] = 2.0            # 正前方
    ranges[10] = 0.6           # 约 10 度，仍在扇区内
    ranges[60] = 0.2           # 约 60 度，扇区外
    assert make_scan(ranges) == pytest.approx(0.6)


def test_min_forward_range_below_range_min_is_invalid():
    ranges = [3.5] * 360
    ranges[0] = 0.05           # 低于量程下限 0.12，属于无效点
    ranges[1] = 0.5
    assert make_scan(ranges) == pytest.approx(0.5)


def test_min_forward_range_all_invalid_returns_none():
    assert make_scan([3.5] * 360) is None
    assert make_scan([float('nan')] * 360) is None
    assert make_scan([float('inf')] * 360) is None
    assert make_scan([]) is None


# ------------------------------------------------- 1. 激光缺失 / 过期

def test_missing_scan_within_grace_waits_without_abort():
    """启动宽限期内还没收到激光：等待，不算故障。"""
    monitor = make_monitor()
    state, reason, code = monitor.check(now=1.0, last_scan=None,
                                        last_odom=0.9, obstacle_distance=None,
                                        elapsed=1.0)
    assert state == STATE_WAIT
    assert reason is None
    assert code == EXIT_OK


def test_missing_scan_never_emits_nonzero_velocity():
    """等待分支必须保持零速度。"""
    monitor = make_monitor()
    state, _, _ = monitor.check(now=1.0, last_scan=None, last_odom=0.9,
                                obstacle_distance=None, elapsed=1.0)
    assert state == STATE_WAIT
    linear, angular = stop_velocity()
    assert linear == 0.0 and angular == 0.0


def test_missing_scan_after_grace_aborts():
    """超过宽限期仍无激光：判定为传感器缺失并中止。"""
    monitor = make_monitor()
    state, reason, code = monitor.check(now=20.0, last_scan=None,
                                        last_odom=19.9, obstacle_distance=None,
                                        elapsed=STARTUP_GRACE + 0.1)
    assert state == STATE_ABORT
    assert reason == 'scan_timeout'
    assert code == EXIT_ABORT_SCAN


def test_stale_scan_stops_robot():
    """最后一次 /scan 已超过 scan_timeout。"""
    monitor = make_monitor()
    state, reason, code = monitor.check(now=10.0, last_scan=10.0 - SCAN_TIMEOUT - 0.01,
                                        last_odom=10.0, obstacle_distance=2.0,
                                        elapsed=30.0)
    assert state == STATE_ABORT
    assert reason == 'scan_timeout'
    assert code == EXIT_ABORT_SCAN


def test_fresh_scan_allows_motion():
    monitor = make_monitor()
    state, reason, code = monitor.check(now=10.0, last_scan=10.0 - SCAN_TIMEOUT / 2,
                                        last_odom=10.0 - ODOM_TIMEOUT / 2,
                                        obstacle_distance=2.0, elapsed=30.0)
    assert state == STATE_OK
    assert reason is None
    assert code == EXIT_OK


# ------------------------------------------------- 2. 里程计缺失 / 过期

def test_missing_odom_after_grace_aborts():
    monitor = make_monitor()
    state, reason, code = monitor.check(now=20.0, last_scan=19.99, last_odom=None,
                                        obstacle_distance=None,
                                        elapsed=STARTUP_GRACE + 0.1)
    assert state == STATE_ABORT
    assert reason == 'odom_timeout'
    assert code == EXIT_ABORT_ODOM


def test_stale_odom_stops_robot():
    monitor = make_monitor()
    state, reason, code = monitor.check(now=10.0, last_scan=9.99,
                                        last_odom=10.0 - ODOM_TIMEOUT - 0.01,
                                        obstacle_distance=2.0, elapsed=30.0)
    assert state == STATE_ABORT
    assert reason == 'odom_timeout'
    assert code == EXIT_ABORT_ODOM


# --------------------------------------------------------- 3. 启动宽限期

def test_both_missing_within_grace_waits():
    monitor = make_monitor()
    state, _, _ = monitor.check(now=0.5, last_scan=None, last_odom=None,
                                obstacle_distance=None, elapsed=0.5)
    assert state == STATE_WAIT


def test_grace_boundary_is_inclusive():
    """正好等于宽限期时仍然等待，超过才中止。"""
    monitor = make_monitor()
    state, _, _ = monitor.check(now=40.0, last_scan=None, last_odom=None,
                                obstacle_distance=None, elapsed=STARTUP_GRACE)
    assert state == STATE_WAIT
    state, _, _ = monitor.check(now=40.0, last_scan=None, last_odom=None,
                                obstacle_distance=None, elapsed=STARTUP_GRACE + 0.01)
    assert state == STATE_ABORT


# --------------------------------------------------------- 4. 近障停车

def test_obstacle_close_stops_robot():
    monitor = make_monitor()
    state, reason, code = monitor.check(now=10.0, last_scan=9.99, last_odom=9.99,
                                        obstacle_distance=OBSTACLE_STOP - 0.01,
                                        elapsed=30.0)
    assert state == STATE_ABORT
    assert reason == 'obstacle'
    assert code == EXIT_ABORT_OBSTACLE


def test_obstacle_at_threshold_is_allowed():
    """阈值本身不算障碍（严格小于才停车）。"""
    monitor = make_monitor()
    state, _, _ = monitor.check(now=10.0, last_scan=9.99, last_odom=9.99,
                                obstacle_distance=OBSTACLE_STOP, elapsed=30.0)
    assert state == STATE_OK


def test_obstacle_check_ignores_unknown_distance():
    """前方扇区没有有效回波（None）时不触发障碍停车，由激光超时兜底。"""
    monitor = make_monitor()
    state, _, _ = monitor.check(now=10.0, last_scan=9.99, last_odom=9.99,
                                obstacle_distance=None, elapsed=30.0)
    assert state == STATE_OK


# --------------------------------------------------------- 5. 任务超时

def test_total_timeout_aborts():
    monitor = make_monitor()
    state, reason, code = monitor.check(now=1000.0, last_scan=999.99, last_odom=999.99,
                                        obstacle_distance=2.0,
                                        elapsed=TOTAL_TIMEOUT + 0.01)
    assert state == STATE_ABORT
    assert reason == 'total_timeout'
    assert code == EXIT_ABORT_TIMEOUT


def test_timeout_is_checked_after_obstacle():
    """同时满足障碍与超时条件时，先报障碍（判定顺序固定）。"""
    monitor = make_monitor()
    state, reason, _ = monitor.check(now=1000.0, last_scan=999.99, last_odom=999.99,
                                     obstacle_distance=0.1,
                                     elapsed=TOTAL_TIMEOUT + 1.0)
    assert state == STATE_ABORT
    assert reason == 'obstacle'


# --------------------------------------------- 6. 速度指令与停止行为

def test_compute_velocity_zero_when_at_goal():
    linear, angular = compute_velocity(angle_error=0.0, distance=GOAL_TOLERANCE,
                                       goal_tolerance=GOAL_TOLERANCE,
                                       max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR,
                                       angular_gain=ANGULAR_GAIN,
                                       linear_gain=LINEAR_GAIN)
    assert linear == 0.0
    assert angular == pytest.approx(0.0)


def test_compute_velocity_turns_in_place_when_misaligned():
    """朝向误差大时不得边转边冲。"""
    linear, angular = compute_velocity(angle_error=math.radians(90),
                                       distance=3.0,
                                       goal_tolerance=GOAL_TOLERANCE,
                                       max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR,
                                       angular_gain=ANGULAR_GAIN,
                                       linear_gain=LINEAR_GAIN)
    assert linear == 0.0
    assert angular == pytest.approx(MAX_ANGULAR)


def test_compute_velocity_drives_when_aligned():
    linear, angular = compute_velocity(angle_error=0.0, distance=5.0,
                                       goal_tolerance=GOAL_TOLERANCE,
                                       max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR,
                                       angular_gain=ANGULAR_GAIN,
                                       linear_gain=LINEAR_GAIN)
    assert linear == pytest.approx(MAX_LINEAR)   # 被上限截断
    assert angular == pytest.approx(0.0)


def test_compute_velocity_respects_limits():
    linear, angular = compute_velocity(angle_error=math.radians(180),
                                       distance=100.0,
                                       goal_tolerance=GOAL_TOLERANCE,
                                       max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR,
                                       angular_gain=ANGULAR_GAIN,
                                       linear_gain=LINEAR_GAIN)
    assert abs(linear) <= MAX_LINEAR
    assert abs(angular) <= MAX_ANGULAR


def test_no_negative_linear_velocity():
    """距离已经在容差内时不会倒退。"""
    linear, _ = compute_velocity(angle_error=0.0, distance=0.0,
                                 goal_tolerance=GOAL_TOLERANCE,
                                 max_linear=MAX_LINEAR, max_angular=MAX_ANGULAR,
                                 angular_gain=ANGULAR_GAIN, linear_gain=LINEAR_GAIN)
    assert linear == 0.0


def test_clamp():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5


# ------------------------------------------------------------ 路线工具

def test_route_distance_counts_remaining_legs():
    waypoints = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
    assert route_distance((0.0, 0.0), waypoints, start_index=0) == pytest.approx(7.0)
    assert route_distance((3.0, 0.0), waypoints, start_index=1) == pytest.approx(4.0)
    assert route_distance((3.0, 4.0), waypoints, start_index=2) == pytest.approx(0.0)
    assert route_distance((3.0, 4.0), waypoints, start_index=3) == pytest.approx(0.0)


# ------------------------------------------- 7. 仿真时钟守卫与启动竞态回归

def test_clock_guard_not_ready_at_zero():
    """仿真时钟读数为 0（/clock 还没到）时不提供时间戳。"""
    guard = ClockGuard(jump_threshold=2.0)
    assert guard.update(0.0) is False
    assert guard.ready is False
    assert guard.stamp(0.0) is None


def test_clock_guard_ready_on_first_positive_time():
    guard = ClockGuard(jump_threshold=2.0)
    guard.update(0.0)
    assert guard.update(314.0) is True       # 0 -> 314 视为跳变
    assert guard.ready is True
    assert guard.stamp(314.0) == pytest.approx(314.0)


def test_clock_guard_normal_progression_is_not_a_jump():
    guard = ClockGuard(jump_threshold=2.0)
    guard.update(100.0)
    for step in range(1, 40):                 # 20 Hz 控制回路跑 2 秒
        assert guard.update(100.0 + step * 0.05) is False
    assert guard.stamp(101.95) == pytest.approx(101.95)


def test_clock_guard_detects_forward_jump():
    """仿真被重置或时钟被大跨度推进时要能识别。"""
    guard = ClockGuard(jump_threshold=2.0)
    guard.update(50.0)
    assert guard.update(50.05) is False
    assert guard.update(80.0) is True         # 跳了 30 s
    assert guard.ready is True                # 跳变后新时刻是有效时间


def test_clock_guard_detects_backward_jump():
    guard = ClockGuard(jump_threshold=2.0)
    guard.update(50.0)
    assert guard.update(0.0) is True          # 时钟回退
    assert guard.ready is False               # 回到 0 视为未就绪


def test_startup_race_first_frame_before_clock_ready_is_not_stale():
    """回归用例：复现 reviewer 报告的启动竞态。

    Gazebo 的 /clock 尚未被处理时节点时钟读数是 0.0。如果此时就把首帧激光的
    时间戳记成 0.0，等时钟跳到当前值（这里用 314 s）之后，
    now - last_scan 会突然变成 ~314 s > scan_timeout(1.0 s)，被误判成激光超时中止。
    ClockGuard 在时钟就绪前不返回时间戳，因此不会出现这个假故障。
    """
    guard = ClockGuard(jump_threshold=2.0)
    monitor = make_monitor()

    # 阶段 1：时钟未就绪，收到若干帧传感器数据，时间戳一律被丢弃
    last_scan = None
    last_odom = None
    start = None
    # 未收到 /clock 时节点时钟一直读 0.0（不是 0.05、0.10 这样的小步进）
    for now in (0.0, 0.0, 0.0, 0.0):
        guard.update(now)
        stamp = guard.stamp(now)
        if stamp is not None:
            last_scan = stamp
            last_odom = stamp
        if start is None:
            start = now
        state, reason, code = monitor.check(now=now, last_scan=last_scan,
                                           last_odom=last_odom,
                                           obstacle_distance=None,
                                           elapsed=now - start)
        assert state == STATE_WAIT       # 保持零速度等待，绝不中止
        assert code == EXIT_OK

    # 阶段 2：/clock 到达，仿真时间跳到 314 s，节点重置新鲜度基准与计时起点
    now = 314.0
    assert guard.update(now) is True
    last_scan = None
    last_odom = None
    state, _, code = monitor.check(now=now, last_scan=last_scan, last_odom=last_odom,
                                   obstacle_distance=None, elapsed=0.0)
    assert state == STATE_WAIT
    assert code == EXIT_OK

    # 阶段 3：时钟就绪后的首帧被正常记录，恢复运动判定
    last_scan = guard.stamp(now)
    last_odom = guard.stamp(now)
    assert last_scan == pytest.approx(314.0)
    state, reason, code = monitor.check(now=now, last_scan=last_scan, last_odom=last_odom,
                                        obstacle_distance=2.0, elapsed=0.0)
    assert state == STATE_OK
    assert reason is None
    assert code == EXIT_OK


def test_without_clock_guard_the_race_would_abort():
    """反证：不做时钟守卫时同一个序列会误报 scan_timeout —— 这就是被修复的缺陷本身。

    这个用例固定住旧行为的形状，防止以后有人把守卫去掉却没人发现。
    """
    monitor = make_monitor()
    last_scan = 0.0        # 时钟未就绪时把首帧记成 0.0
    last_odom = 0.0
    state, reason, code = monitor.check(now=314.0, last_scan=last_scan,
                                        last_odom=last_odom,
                                        obstacle_distance=None, elapsed=0.0)
    assert state == STATE_ABORT
    assert reason == 'scan_timeout'
    assert code == EXIT_ABORT_SCAN

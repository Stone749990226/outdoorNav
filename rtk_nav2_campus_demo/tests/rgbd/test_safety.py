"""RGB-D 巡游安全逻辑的纯逻辑测试（不需要 ROS 运行环境）。

运行：
    PYTHONPATH=rgbd_ws/src/robonix_rgbd_demo python3 -m pytest tests/rgbd/test_safety.py -v

覆盖本轮新增的失败模式：
  1. 图像/深度从未到达（启动宽限期内保持静止、超过则中止）
  2. 图像或深度过期
  3. 有效深度覆盖不足（深度不可用 ≠ 没有障碍）
  4. 近障判定（阈值比较）
  5. 控制律：朝向未对准时不前进、到点停、速度受限
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, os.pardir,
                                'rgbd_ws', 'src', 'robonix_rgbd_demo'))

from robonix_rgbd_demo.depth_logic import (  # noqa: E402
    STATE_LOW_COVERAGE, STATE_OBSTACLE, STATE_OK, STATE_STALE, STATE_WAIT,
    ImageSourceCheck, compute_velocity, depth_stats, sector_mask, wrap_angle)
from robonix_rgbd_demo.patrol_guard import (  # noqa: E402
    ClockGuard, OdometryGuard)

# 与 config/depth_obstacles.yaml 保持一致
IMAGE_TIMEOUT = 1.0
DEPTH_TIMEOUT = 1.0
MIN_VALID_RATIO = 0.02
STARTUP_GRACE = 40.0
OBSTACLE_STOP = 0.45
HFOV = math.radians(60.0)
SECTOR_HALF = math.radians(22.0)


def make_check():
    return ImageSourceCheck(image_timeout=IMAGE_TIMEOUT,
                            depth_timeout=DEPTH_TIMEOUT,
                            min_valid_ratio=MIN_VALID_RATIO,
                            startup_grace=STARTUP_GRACE)


def make_depth(fill, width=320, height=240):
    return np.full((height, width), fill, dtype=np.float32)


# ------------------------------------------------------------ 扇区与统计

def test_sector_mask_shape_and_forward_only():
    mask = sector_mask(320, 240, HFOV, SECTOR_HALF, 0.30, 0.75)
    assert mask.shape == (240, 320)
    # 中心列必定在扇区内
    assert mask[150, 160]
    # 最左和最右列在 60° 视场下是 ±30°，超过 ±22°，不在扇区内
    assert not mask[150, 0]
    assert not mask[150, 319]


def test_depth_stats_ignores_invalid_values():
    depth = make_depth(3.0)
    mask = sector_mask(320, 240, HFOV, SECTOR_HALF, 0.30, 0.75)
    # 在扇区中心放一个 0.8 m 的近点，同时塞入 inf / nan / 0
    depth[150, 160] = 0.8
    depth[151, 160] = float('inf')
    depth[152, 160] = float('nan')
    depth[153, 160] = 0.0
    stats = depth_stats(depth, mask, 0.12, 6.0)
    assert stats['min_distance'] == pytest.approx(0.8)
    assert stats['valid'] > 0
    assert 0.0 < stats['valid_ratio'] <= 1.0


def test_depth_stats_all_invalid_gives_none_and_zero_ratio():
    depth = make_depth(float('inf'))
    mask = sector_mask(320, 240, HFOV, SECTOR_HALF, 0.30, 0.75)
    stats = depth_stats(depth, mask, 0.12, 6.0)
    assert stats['min_distance'] is None
    assert stats['valid_ratio'] == 0.0


def test_depth_stats_range_clipping():
    depth = make_depth(3.0)
    mask = sector_mask(320, 240, HFOV, SECTOR_HALF, 0.30, 0.75)
    depth[150, 160] = 0.05      # 低于量程下限 -> 无效
    depth[151, 160] = 9.0       # 超过量程上限 -> 无效
    stats = depth_stats(depth, mask, 0.12, 6.0)
    assert stats['min_distance'] == pytest.approx(3.0)


# ------------------------------------------------ 1/2. 数据缺失与过期

def test_no_data_within_grace_waits():
    check = make_check()
    state, reason = check.check(now=1.0, last_image=None, last_depth=None,
                                valid_ratio=0.0, elapsed=1.0)
    assert state == STATE_WAIT
    assert reason is None


def test_no_data_after_grace_is_stale():
    check = make_check()
    state, reason = check.check(now=100.0, last_image=None, last_depth=None,
                                valid_ratio=0.0, elapsed=STARTUP_GRACE + 0.1)
    assert state == STATE_STALE
    assert reason == 'image_or_depth_never_received'


def test_stale_image_is_stale():
    check = make_check()
    state, reason = check.check(now=100.0, last_image=100.0 - IMAGE_TIMEOUT - 0.01,
                                last_depth=99.99, valid_ratio=0.5, elapsed=50.0)
    assert state == STATE_STALE
    assert reason == 'image_stale'


def test_stale_depth_is_stale():
    check = make_check()
    state, reason = check.check(now=100.0, last_image=99.99,
                                last_depth=100.0 - DEPTH_TIMEOUT - 0.01,
                                valid_ratio=0.5, elapsed=50.0)
    assert state == STATE_STALE
    assert reason == 'depth_stale'


def test_fresh_data_is_ok():
    check = make_check()
    state, reason = check.check(now=100.0, last_image=99.9, last_depth=99.9,
                                valid_ratio=0.5, elapsed=50.0)
    assert state == STATE_OK
    assert reason is None


# --------------------------------------------------- 3. 有效覆盖不足

def test_low_coverage_is_not_treated_as_clear():
    """深度几乎全是无效值时，必须判为不可用，而不是“前方无障碍”。"""
    check = make_check()
    state, reason = check.check(now=100.0, last_image=99.99, last_depth=99.99,
                                valid_ratio=MIN_VALID_RATIO / 2.0, elapsed=50.0)
    assert state == STATE_LOW_COVERAGE
    assert reason == 'depth_coverage_low'


def test_coverage_boundary_is_inclusive():
    check = make_check()
    state, _ = check.check(now=100.0, last_image=99.99, last_depth=99.99,
                           valid_ratio=MIN_VALID_RATIO, elapsed=50.0)
    assert state == STATE_OK


# --------------------------------------------------------- 4. 近障判定

def test_obstacle_close_triggers_stop_condition():
    """节点用 min_distance < obstacle_stop_distance 判定；这里验证比较边界。"""
    assert 0.44 < OBSTACLE_STOP
    assert not (OBSTACLE_STOP < OBSTACLE_STOP)


def test_min_distance_from_real_depth_frame():
    depth = make_depth(4.0)
    mask = sector_mask(320, 240, HFOV, SECTOR_HALF, 0.30, 0.75)
    depth[150, 165] = 0.42      # 略小于 0.45 的阈值
    stats = depth_stats(depth, mask, 0.12, 6.0)
    assert stats['min_distance'] < OBSTACLE_STOP


# ------------------------------------------------------------ 5. 控制律

def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_velocity_turns_in_place_when_misaligned():
    linear, angular = compute_velocity(math.radians(80), 3.0, 0.20,
                                       0.11, 0.5, 1.2, 0.5)
    assert linear == 0.0
    assert angular == pytest.approx(-0.5) or angular == pytest.approx(0.5)


def test_velocity_zero_at_goal():
    linear, _ = compute_velocity(0.0, 0.20, 0.20, 0.11, 0.5, 1.2, 0.5)
    assert linear == 0.0


def test_velocity_respects_limits():
    linear, angular = compute_velocity(math.radians(180), 100.0, 0.20,
                                       0.11, 0.5, 1.2, 0.5)
    assert abs(linear) <= 0.11
    assert abs(angular) <= 0.5


# ---------------------------------------------------------------- 时钟守卫（R1 回归）
#
# 背景：节点刚启动、Gazebo 的 /clock 还没被处理时，节点时钟读数是 0.0。
# 如果此时把首帧数据的时间戳记成 0.0，等 /clock 到达、时间跳到 268.68 s，
# `now - last` 就变成 268.68 s，被误判成"数据过期"而启动即停车。
# 实测约 20% 的启动会踩到。下面三条把机制钉死。

def test_clock_guard_refuses_stamp_before_clock_ready():
    guard = ClockGuard(jump_threshold=2.0)
    guard.update(0.0)
    assert guard.stamp(0.0) is None, '时钟未就绪时不能给出时间戳'


def test_clock_guard_detects_first_ready_jump():
    guard = ClockGuard(jump_threshold=2.0)
    guard.update(0.0)
    assert guard.update(268.68) is True, '0 → 真实时间必须被判为跳变'
    assert guard.stamp(268.68) == 268.68
    assert guard.jumps == 1


def test_startup_race_would_report_bogus_stale_without_guard():
    """反证：不做守卫（照旧把 0.0 记成时间戳）时算出来的"过期时长"就是仿真时间总量。"""
    bogus_last_stamp = 0.0          # 旧代码在时钟未就绪时记下的值
    now = 268.68                    # /clock 到达后的当前时间
    depth_status_timeout = 1.5
    assert (now - bogus_last_stamp) > depth_status_timeout, \
        '这正是"启动即报 depth_status_stale（268.68 s）"的来源'


def test_clock_guard_resets_on_backward_jump():
    guard = ClockGuard(jump_threshold=2.0)
    guard.update(100.0)
    assert guard.update(3.0) is True, '时钟回退也算跳变'
    assert guard.jumps == 1


# ---------------------------------------------------------------- 里程计守卫（R2 回归）
#
# 背景：reviewer 的 rev-rgbd-4 里，视觉里程计 t=70 s 前与真值误差只有 0.1 m，
# t=76 s 一步之内从 (5.40, 0.05) 跳到 (-8.20, -2.31)（约 14 m）。机器人最大速度
# 只有 0.11 m/s，这个位移物理上不可能，但控制器照它继续开，直到撞墙才被拦下。

def test_odometry_guard_catches_measured_jump():
    guard = OdometryGuard(max_plausible_speed=1.0)
    ok, _, _ = guard.check((5.40, 0.05), 100.0)
    assert ok, '第一个样本没有可比基准，必须放行'
    ok, reason, speed = guard.check((-8.20, -2.31), 100.1)
    assert not ok, '一步 14 m 必须被判为不可信'
    assert speed > 100.0
    assert '物理上限' in reason
    assert guard.jumps == 1


def test_odometry_guard_allows_normal_motion():
    guard = OdometryGuard(max_plausible_speed=1.0)
    t = 0.0
    x = 0.0
    for _ in range(50):
        t += 0.1
        x += 0.011          # 0.11 m/s，与巡游最高速度一致
        ok, reason, _ = guard.check((x, 0.0), t)
        assert ok, '正常巡游不该触发里程计守卫：%s' % reason
    assert guard.jumps == 0


def test_odometry_guard_tolerates_sampling_gap():
    """控制回路偶尔隔很久才读一次 TF：位移也被同样大的 dt 除掉，速度仍然正常。"""
    guard = OdometryGuard(max_plausible_speed=1.0)
    guard.check((0.0, 0.0), 10.0)
    ok, reason, speed = guard.check((0.55, 0.0), 15.0)   # 5 秒走 0.55 m = 0.11 m/s
    assert ok, reason
    assert abs(speed - 0.11) < 1e-6


def test_odometry_guard_reset_drops_baseline():
    guard = OdometryGuard()
    guard.check((0.0, 0.0), 1.0)
    guard.reset()
    ok, _, _ = guard.check((100.0, 0.0), 1.1)
    assert ok, '时钟跳变后重置基准，不该把跨跳变的位移算成一步'

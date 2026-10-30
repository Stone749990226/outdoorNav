"""Outdoor restrictions audit: offline fixtures only, no physical commands."""

import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import time

import pytest

from outdoornav.config import load_robot_config
from outdoornav.chassis_status import chassis_blockers
from outdoornav.tasks import ProgressBudget
from test_rtk_imu import state, fix, observation


def test_fixed_float_switch_does_not_restart_stabilization():
    s = state()
    for i in range(71):
        t = i / 10
        changed = {'fixed': False, 'float': True, 'gga_quality': 5} if i % 2 else {}
        s.update_fix(fix(0, 0, 0, 0, i, **changed), t)
        s.update_imu(observation(0, 0, t), t)
    assert s.ready(7)
    assert s.stable_since == 0


def test_nonconsecutive_position_outliers_do_not_accumulate():
    s = state()
    s.update_fix(fix(0, 0, 0, 0, 1), 0)
    for i in range(5):
        s.update_fix(fix(5, 0, 0, 0, 2+i*2), .1+i)
        s.update_fix(fix(0, 0, 0, 0, 3+i*2), .2+i)
    assert s.fault is None
    assert s.jump_streak == 0


def test_imu_recovery_does_not_add_previous_turn_to_absolute_heading():
    s = state()
    s.update_fix(fix(0, 0, 0, 0, 1), 0)
    s.update_imu(observation(.4, 0, 0), 0)
    s.yaw = .8  # Existing local reference has already turned.
    s.update_fix(fix(0, 0, 0, 0, 2), 1.)
    s.update_imu(observation(1.2, 0, 1.), 1.)  # Dropout clears initialization.
    s.update_imu(observation(1.2, 0, 1.1), 1.1)
    assert s.sample(1.1)['map_pose']['yaw'] == pytest.approx(1.2, abs=.001)


def test_planner_waits_for_time_budget_and_resets_on_recovery():
    budget = ProgressBudget(planner_patience_s=60)
    for now in (0, 1, 2, 10, 59):
        assert budget.planned(False, now)
    assert not budget.planned(False, 60)
    assert budget.planned(True, 61)
    assert budget.planned(False, 70)
    assert budget.planned(False, 129)
    assert not budget.planned(False, 130)


def test_controller_wait_budget_is_independent_of_successful_replans():
    budget = ProgressBudget(controller_patience_s=60)
    budget.moved(0, 0, 0)
    for now in (1, 5, 20, 40, 59):
        budget.planned(True, now)
        assert budget.controller_ended(now)
    assert not budget.controller_ended(60)
    budget.moved(1, 0, 61)
    assert budget.controller_ended(62)


def test_course_threshold_uses_signal_quality_not_max_speed_change():
    s = state()
    s.config['course_min_speed_mps'] = .12
    s.update_fix(fix(0, 0, .2, 0, 1, speed_accuracy_mps=.05), 0)
    assert s.course_yaw(0) == pytest.approx(0)
    s.update_fix(fix(0, 0, .2, 0, 2, speed_accuracy_mps=.09), .1)
    assert s.course_yaw(.1) is None  # Poor signal must not supply a noisy heading.


def test_chassis_not_ready_is_visible_before_accepting_navigation():
    assert chassis_blockers(17, 0x3002, .1, 1, .1) == []
    assert 'not_rl_control' in chassis_blockers(4, 0x3002, .1, 1, .1)
    assert 'navigation_usage_mode_unverified' in chassis_blockers(17, 0x3002, .1, None, .1)
    assert 'motion_feedback_stale' in chassis_blockers(17, 0x3002, 2, 1, .1)
    # Usage mode is polled once per second; a three-second-old value is unusable.
    assert 'navigation_usage_mode_unverified' in chassis_blockers(17, 0x3002, .1, 1, 4.)


def test_field_profile_keeps_obstacle_layer_and_exposes_wait_and_stop_limits():
    cfg = load_robot_config('deploy/lynx/robot.fieldtest.json')
    assert cfg['obstacle_layer_enabled'] is True
    assert cfg['footprint_radius_m'] == .6
    assert cfg['scan_range_min_m'] == .5
    assert cfg['inflation_padding_m'] == .1
    assert cfg['allow_float_during_fixed_task'] is True
    assert cfg['stopped_linear_mps'] == .03
    assert cfg['planner_patience_s'] == cfg['controller_patience_s'] == 60.
    assert cfg['require_chassis_ready'] and cfg['auto_activate']
    assert not load_robot_config('deploy/lynx/robot.mock.json')['require_chassis_ready']


def test_demo_heading_warnings_are_not_false_blocking_reasons():
    from outdoornav.diagnostics import readiness_reasons
    from outdoornav.field_policy import field_warnings
    cfg = load_robot_config('deploy/lynx/robot.fieldtest.json')
    heading = {'source': 'wit_imu', 'ready': True, 'angles_fresh': True, 'gyro_fresh': True,
               'nine_axis_verified': True, 'mounting_confirmed': True, 'quality_confirmed': False}
    assert readiness_reasons({'heading': True}, {'heading': heading}) == []
    assert any(w['code'] == 'heading_accuracy_unverified' for w in field_warnings(cfg, heading))
    assert not any(w['code'] == 'lidar_blind_zone' for w in field_warnings(cfg, heading))
    cfg['scan_range_min_m'] = 3.
    assert any(w['code'] == 'lidar_blind_zone' for w in field_warnings(cfg, heading))


def test_field_float_downgrade_is_allowed_but_loss_still_blocks():
    pytest.importorskip('rclpy')
    from outdoornav.config import DEFAULTS
    from outdoornav.tasks import TaskStore
    from outdoornav_ros.lynx_navigation import LynxNavigation
    node = object.__new__(LynxNavigation)
    now = time.monotonic()
    node.config = {**DEFAULTS, **{k: True for k in ('calibrated', 'antenna_offset_confirmed',
        'footprint_confirmed', 'cloud_frame_confirmed', 'motion_enabled', 'odometry_confirmed')}}
    node.config.update(localization_mode='external_odom', heading_source='configured', allow_float_during_fixed_task=True)
    node.resolve_topic_name = lambda t: t
    node.geo = {'ready': True, 'grade': 'float', 'extrinsics': {'ready': True}}
    node.geo_at = node.velocity_at = node.guard_at = now
    node.guard = {'output_topic': '/cmd_vel', 'conflict': False}
    node.diagnostic = False
    node.stats = {k: now for k in ('lidar', 'global_map', 'local_map')}
    node.tasks = TaskStore()
    node.owner = node.tasks.submit({'x': 0, 'y': 0})['task_id']
    node.tasks.update(node.owner, localization_grade='fixed')
    node.service_active = node.startup_stopped = True
    node.stop_failure = None
    node.lifecycle_clients = {}
    node.path_client = node.follow_client = SimpleNamespace(server_is_ready=lambda: True)
    node.pose = lambda: {'x': 0, 'y': 0, 'yaw': 0, 'age_s': 0}
    node.bundle = {'scene_version': 's', 'route_version': 'r'}
    assert node.status()['ready']
    node.geo['grade'] = None
    node.geo['ready'] = False
    assert not node.status()['ready']


def test_demo_named_place_does_not_force_final_heading_but_explicit_yaw_is_kept():
    from outdoornav.bundle import load_bundle
    from outdoornav.places import Places
    from outdoornav.outdoor import OutdoorService
    bundle = load_bundle()
    cfg = load_robot_config('deploy/lynx/robot.fieldtest.json')
    places = Places('scenes/north_campus/places.demo.json', bundle)
    service = OutdoorService(bundle, places, None, lambda _: {}, cfg)
    base = {'scene_version': bundle['scene_version'], 'route_version': bundle['route_version'], 'frame_id': 'map'}
    target, _ = service.target({**base, 'place_id': 'P'})
    assert 'yaw' not in target
    point = bundle['network']['roads'][0]['points'][0]
    target, _ = service.target({**base, 'goal': {**point, 'yaw': .3}})
    assert target['yaw'] == .3

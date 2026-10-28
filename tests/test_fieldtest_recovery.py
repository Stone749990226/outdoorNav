"""Offline regressions from the outdoor report; never connect to robot inputs."""

import asyncio
import json
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from outdoornav.scan_health import healthy_scan


def scan(ranges, stamp=10):
    return SimpleNamespace(header=SimpleNamespace(frame_id='base_link',
        stamp=SimpleNamespace(sec=stamp, nanosec=0)), ranges=ranges,
        range_min=.6, range_max=8., angle_increment=.01)


@pytest.mark.parametrize('ranges,valid', [
    ([math.inf] * 360, True), ([2., math.inf], True), ([math.nan, 2.], True),
    ([], False), ([math.nan] * 10, False), ([-math.inf], False),
    ([0., -1.], False),
])
def test_scan_health_distinguishes_empty_scene_from_invalid_data(ranges, valid):
    assert healthy_scan(scan(ranges), 10.1, 1.) is valid


def test_stale_future_and_malformed_scan_are_not_healthy():
    message = scan([math.inf])
    assert not healthy_scan(message, 11., 1.)
    assert not healthy_scan(message, 9., 1.)
    message.angle_increment = 0.
    assert not healthy_scan(message, 10.1, 1.)
    message.angle_increment = .01
    message.header.frame_id = ''
    assert not healthy_scan(message, 10.1, 1.)


def ros_classes():
    pytest.importorskip('rclpy')
    from outdoornav_ros.lynx_navigation import LynxNavigation
    from outdoornav_ros.lynx_watchdog import Watchdog
    return LynxNavigation, Watchdog


def test_both_consumers_accept_all_inf_and_watchdog_passes_motion():
    navigation, watchdog = ros_classes()
    from geometry_msgs.msg import Twist
    from outdoornav.config import DEFAULTS
    node, guard = object.__new__(navigation), object.__new__(watchdog)
    node.config = guard.config = {**DEFAULTS, 'motion_enabled': True}
    clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000))
    node.get_clock = guard.get_clock = clock
    node.mark = Mock()
    guard.scan_at = -math.inf
    node.on_scan(scan([math.inf] * 360))
    guard.scan(scan([math.inf] * 360))
    node.mark.assert_called_once_with('lidar')
    assert guard.scan_at > 0
    guard.command = Twist()
    guard.command.linear.x = .2
    guard.command_at = guard.geo_at = time.monotonic()
    guard.ready, guard.conflict, guard.diagnostic = True, False, False
    guard.pub, guard.status_pub = Mock(), Mock()
    guard.pub.topic_name = '/cmd_vel'
    guard.tick()
    assert guard.pub.publish.call_args.args[0].linear.x == .2
    assert json.loads(guard.status_pub.publish.call_args.args[0].data)['blocked_reasons'] == []
    guard.scan_at = time.monotonic() - 2
    guard.tick()
    assert guard.pub.publish.call_args.args[0].linear.x == 0
    assert 'scan_stale' in json.loads(guard.status_pub.publish.call_args.args[0].data)['blocked_reasons']
    old = guard.scan_at
    guard.scan(scan([math.inf], stamp=1))
    assert guard.scan_at == old


def navigation_stub():
    pytest.importorskip('rclpy')
    from outdoornav_ros.navigation import Navigation
    node = object.__new__(Navigation)
    node.config = {'sensor_timeout_s': 1., 'navigation_recovery_timeout_s': .02}
    node.stopping = False
    node.clock_rewound = False
    node.command_pub = Mock()
    node.interrupted = lambda _: False
    node.status = lambda _: {'ready': True, 'health': {}}
    return node


def test_missing_pose_stops_immediately_then_recovers_without_crashing():
    node = navigation_stub()
    pose = {'x': 1., 'y': 2., 'yaw': .3, 'age_s': .1}
    values = iter([None, pose])
    node.pose = lambda: next(values)

    async def pause(_):
        assert node.stopping
        assert node.command_pub.publish.call_args.args[0].linear.x == 0
    node.pause = pause
    assert asyncio.run(node.wait_for_ready_pose('task')) == pose
    assert node.stopping is False


def test_persistent_missing_pose_and_cancel_keep_output_stopped():
    node = navigation_stub()
    node.pose = lambda: None
    async def pause(_):
        await asyncio.sleep(.01)
    node.pause = pause
    with pytest.raises(RuntimeError, match='unavailable after'):
        asyncio.run(node.wait_for_ready_pose('task'))
    assert node.stopping
    node.interrupted = lambda _: True
    with pytest.raises(InterruptedError):
        asyncio.run(node.wait_for_ready_pose('task'))
    assert node.command_pub.publish.call_args.args[0].linear.x == 0


def test_ownership_conflict_does_not_get_recovery_grace():
    node = navigation_stub()
    node.pose = lambda: None
    node.status = lambda _: {'ready': False, 'health': {'exclusive_owners': False}}
    with pytest.raises(RuntimeError, match='ownership/clock conflict'):
        asyncio.run(node.wait_for_ready_pose('task'))
    assert node.stopping


def test_mode_query_retries_even_without_data_and_duration_stays_bounded(monkeypatch):
    import sys
    from deploy.lynx import wit_stream
    clock = {'now': 10.}
    queries = []

    class Serial:
        def __init__(self, *_args, **_kwargs):
            self.port = 'offline-fake'
        def __enter__(self):
            return self
        def __exit__(self, *_):
            pass
        def read(self, **_):
            clock['now'] += 1.
            return b''
        def query_imu_algorithm(self):
            queries.append(clock['now'])

    class Socket:
        def __enter__(self):
            return self
        def __exit__(self, *_):
            pass

    monkeypatch.setattr(wit_stream, 'SerialInput', Serial)
    monkeypatch.setattr(wit_stream.socket, 'socket', lambda *_: Socket())
    monkeypatch.setattr(wit_stream.time, 'monotonic', lambda: clock['now'])
    monkeypatch.setattr(sys, 'argv', ['wit_stream', '--config', 'deploy/lynx/robot.json', '--duration', '12'])
    wit_stream.main()
    assert queries == [11., 16., 21.]
    assert clock['now'] == 23.


def test_generated_costmap_clears_inf_rays_without_increasing_near_blind_zone(tmp_path, monkeypatch):
    pytest.importorskip('launch_ros')
    import importlib.util
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[1]
    package = root / 'ros_ws/src/outdoornav_ros'
    spec = importlib.util.spec_from_file_location('offline_field_launch', package / 'launch/lynx.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    values = {'bundle': str(root/'scenes/north_campus/generated/bundle.json'),
              'robot_config': str(root/'deploy/lynx/robot.fieldtest.json'),
              'places': str(root/'scenes/north_campus/places.demo.json'),
              'run_dir': str(tmp_path), 'diagnostic': 'false'}
    monkeypatch.setattr(module, 'LaunchConfiguration', lambda name: SimpleNamespace(perform=lambda _: values[name]))
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(package))
    module.nodes(None)  # Construct launch actions and YAML; never execute nodes.
    params = yaml.safe_load((tmp_path/'lynx_nav2.yaml').read_text())
    for name in ('global_costmap', 'local_costmap'):
        assert params[name][name]['ros__parameters']['obstacle_layer']['scan']['inf_is_valid'] is True
    from outdoornav.config import load_robot_config
    assert load_robot_config(values['robot_config'])['scan_range_min_m'] == 3.0


@pytest.mark.parametrize('age', [None, 39., 120.])
def test_field_age_policy_is_explicit_and_still_rejects_degraded_solutions(age):
    from outdoornav.gnss import fix_grade
    fix = {'valid': True, 'differential': True, 'fixed': True, 'gga_quality': 4,
           'hacc_m': .02, 'latitude': 30., 'longitude': 120., 'correction_age_s': age}
    assert fix_grade(fix) is None
    policy = {'require_correction_age': False}
    assert fix_grade(fix, policy) == 'fixed'
    assert fix_grade({**fix, 'differential': False}, policy) is None
    assert fix_grade({**fix, 'gga_quality': 2}, policy) is None
    assert fix_grade({**fix, 'valid': False}, policy) is None
    assert fix_grade({**fix, 'hacc_m': 99.}, policy) is None

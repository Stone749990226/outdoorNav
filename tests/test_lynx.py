import json
import struct
import tempfile
from pathlib import Path

import pytest
from types import SimpleNamespace
from urllib.request import Request, urlopen

from outdoornav.bundle import load_bundle
from outdoornav.gnss import FixDecoder, usable_fix
from outdoornav.localization import GeoAlignment
from outdoornav.places import Places
from outdoornav.tasks import TaskStore
from outdoornav.outdoor import OutdoorService
from outdoornav.api import ApiError, serve


def nmea(body):
    checksum = 0
    for b in body.encode():
        checksum ^= b
    return f"${body}*{checksum:02X}\r\n".encode()


def pvt(carrier=2, tow=100):
    p = bytearray(92)
    struct.pack_into("<I", p, 0, tow)
    p[20], p[21], p[23] = 3, 3 | (carrier << 6), 20
    struct.pack_into("<iiiiII", p, 24, 1200000000, 300000000, 123456, 100000, 100, 150)
    data = bytes([1, 7, 92, 0]) + p
    a = b = 0
    for value in data:
        a = (a + value) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + data + bytes([a, b])


def fix(tow):
    return {
        "valid": True,
        "fixed": True,
        "differential": True,
        "gga_quality": 4,
        "hacc_m": 0.1,
        "correction_age_s": 1.0,
        "latitude": 30.0,
        "longitude": 120.0,
        "tow_ms": tow,
    }


def test_receiver_fixed_units_and_missing_age():
    decoder = FixDecoder()
    decoder.feed(
        nmea("GNGGA,123456,3000.0,N,12000.0,E,4,20,1.0,100,M,23.456,M,1.2,0000"), 0
    )
    result = decoder.feed(pvt(), 0.1)
    assert usable_fix(result)
    assert result["hacc_m"] == 0.1
    assert result["altitude_m"] == 123.456
    assert result["msl_m"] == 100
    assert not usable_fix(decoder.feed(pvt(carrier=0), 0.2))
    assert not usable_fix(decoder.feed(pvt(), 2))
    decoder.feed(
        nmea("GNGGA,123456,3000.0,N,12000.0,E,4,20,1.0,100,M,23.456,M,,0000"), 3
    )
    assert not usable_fix(decoder.feed(pvt(), 3.1))


def test_alignment_antenna_stability_loss_and_reset():
    align = GeoAlignment(
        {"latitude": 30.0, "longitude": 120.0},
        {
            "calibrated": True,
            "initial_heading_rad": 0.0,
            "antenna_xy_m": [1.0, 0.0],
            "map_offset_xy_m": [0.0, 0.0],
        },
    )
    for i in range(61):
        t = i / 10
        assert align.odom(10, 20, 0, 100 + t, t)
        align.update_fix(fix(i), t)
    assert align.ready(6)
    assert align.pose()["x"] == pytest.approx(-1)
    assert align.pose()["y"] == pytest.approx(0)
    assert not align.ready(7.1)
    assert not align.odom(0, 0, 0, 106.1, 6.1)
    assert not align.ready(6.1)


def test_repeated_epoch_cannot_keep_localization_fresh():
    align = GeoAlignment(
        {"latitude": 30.0, "longitude": 120.0},
        {
            "calibrated": True,
            "initial_heading_rad": 0.0,
            "antenna_xy_m": [0.0, 0.0],
            "map_offset_xy_m": [0.0, 0.0],
        },
    )
    for i in range(70):
        t = i / 10
        align.odom(0, 0, 0, t + 1, t)
        align.update_fix(fix(1), t)
    assert not align.ready(6.9)


def test_places_bind_existing_nodes_and_require_confirmation():
    bundle = load_bundle()
    places = Places("scenes/north_campus/places.json", bundle)
    assert places.resolve("公共路口")["id"] == "J"
    assert places.resolve("取餐点")["id"] == "P"
    assert not any(p["enabled"] for p in places.items)
    for p in places.items:
        node = next(n for n in bundle["network"]["nodes"] if n["id"] == p["id"])
        assert (p["x"], p["y"]) == (node["x"], node["y"])
    with pytest.raises(ValueError):
        Places(
            "scenes/north_campus/places.json", {**bundle, "route_version": "another"}
        )


def test_restart_records_failure_without_resuming_and_preserves_terminal():
    with tempfile.TemporaryDirectory() as directory:
        journal = Path(directory) / "tasks.json"
        store = TaskStore(journal=journal)
        done = store.submit({"x": 1.0, "y": 2.0})
        store.update(done["task_id"], state="succeeded")
        active = store.submit({"x": 3.0, "y": 4.0})
        recovered = TaskStore(journal=journal)
        assert recovered.snapshot(done["task_id"])["state"] == "succeeded"
        assert recovered.snapshot(active["task_id"])["state"] == "failed"
        assert "not resumed" in recovered.snapshot()["reason"]


def test_shared_service_rejects_unconfirmed_places_and_deduplicates_after_fix_loss():
    bundle = load_bundle()
    places = Places("scenes/north_campus/places.json", bundle)
    tasks = TaskStore(bundle["scene_version"], bundle["route_version"])
    state = {"ready": True, "pose": {**bundle["network"]["nodes"][0], "age_s": 0.1}}
    service = OutdoorService(bundle, places, tasks, lambda *_: state)
    payload = {
        "frame_id": "map",
        "scene_version": bundle["scene_version"],
        "route_version": bundle["route_version"],
        "place_id": "J",
        "blocked_road_ids": [],
        "request_id": "a" * 32,
    }
    assert service.preview(payload)["goal"] == {
        k: places.resolve("J")[k] for k in ("x", "y")
    }
    with pytest.raises(ApiError, match="confirmation"):
        service.submit(payload)
    places.items[1]["enabled"] = True
    first = service.submit(payload)
    state.update(ready=False, pose=None)
    assert service.submit(payload)["task_id"] == first["task_id"]
    assert len(tasks.tasks) == 1
    with pytest.raises(ApiError, match="not ready"):
        service.submit({**payload, "request_id": "b" * 32})
    with pytest.raises(ApiError, match="another goal"):
        service.submit({**payload, "place_id": "P"})
    tasks.cancel(first["task_id"])
    assert service.submit(payload)["state"] == "canceling"
    state.update(ready=True, pose={**bundle["network"]["nodes"][0], "age_s": 0.1})
    replacement = service.submit({**payload, "request_id": "b" * 32})
    assert replacement["state"] == "accepted"
    assert tasks.snapshot(first["task_id"])["cancel_requested"]
    with pytest.raises(ApiError, match="version"):
        service.submit({**payload, "route_version": "old"})


def test_http_places_submit_and_cancel_share_the_same_task():
    bundle = load_bundle()
    places = Places("scenes/north_campus/places.json", bundle)
    places.items[1]["enabled"] = True
    tasks = TaskStore(bundle["scene_version"], bundle["route_version"])
    status = lambda *_: {
        "ready": True,
        "pose": {**bundle["network"]["nodes"][0], "age_s": 0.1},
    }
    service = OutdoorService(bundle, places, tasks, status)
    backend = SimpleNamespace(
        places=places,
        tasks=tasks,
        obstacles=None,
        status=status,
        bundle=bundle,
        map_snapshot=lambda: bundle,
        submit=service.submit,
        preview=service.preview,
    )
    server = serve(backend, port=0)

    def request(path, body=None):
        req = Request(
            f"http://127.0.0.1:{server.server_port}" + path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Content-Type": "application/json"},
        )
        with urlopen(req, timeout=2) as response:
            return json.load(response)

    try:
        assert request("/api/places")["places"][1]["id"] == "J"
        result = request(
            "/api/tasks",
            {
                "frame_id": "map",
                "place_id": "J",
                "scene_version": bundle["scene_version"],
                "route_version": bundle["route_version"],
            },
        )
        path = "/api/tasks/" + result["task_id"]
        assert request(path)["goal"]["place_id"] == "J"
        assert request(path + "/cancel", {})["state"] == "canceling"
        assert tasks.snapshot()["cancel_requested"]
    finally:
        server.shutdown()
        server.server_close()


def test_float_quality_is_not_ordinary_differential_and_expires():
    from outdoornav.gnss import fix_grade
    good = {**fix(1), 'fixed': False, 'float': True, 'gga_quality': 5, 'hacc_m': 0.7}
    assert fix_grade(good) == 'float'
    for bad in ({'valid': False}, {'float': False}, {'hacc_m': 1.01},
                {'correction_age_s': None}, {'correction_age_s': 6}, {'gga_quality': 2}):
        assert fix_grade({**good, **bad}) is None
    align = GeoAlignment({'latitude': 30, 'longitude': 120}, {
        'calibrated': True, 'initial_heading_rad': 0, 'antenna_xy_m': [0, 0],
        'map_offset_xy_m': [0, 0]})
    for i in range(61):
        align.odom(0, 0, 0, i / 10 + 1, i / 10)
        align.update_fix({**good, 'tow_ms': i}, i / 10)
    assert align.ready(6) and align.grade == 'float'
    # Replayed packets cannot keep the estimate alive.
    align.update_fix({**good, 'tow_ms': 60}, 7.1)
    assert not align.ready(7.1)
    align.update_fix({**good, 'tow_ms': 61, 'valid': False}, 7.2)
    assert align.grade is None


def test_road_point_projection_closures_and_task_policy_are_durable(tmp_path):
    bundle = load_bundle()
    places = Places('scenes/north_campus/places.json', bundle)
    a, b = bundle['network']['roads'][0]['points'][:2]
    point = {k: (a[k] + b[k]) / 2 for k in ('x', 'y')}
    dx, dy = b['x'] - a['x'], b['y'] - a['y']
    length = (dx * dx + dy * dy) ** 0.5
    near = {'x': point['x'] - dy / length, 'y': point['y'] + dx / length}
    state = {'ready': True, 'localization_grade': 'fixed', 'pose': {**a, 'age_s': 0}}
    store = TaskStore(journal=tmp_path / 'tasks.json')
    service = OutdoorService(bundle, places, store, lambda *_: state)
    body = {'frame_id': 'map', 'scene_version': bundle['scene_version'],
            'route_version': bundle['route_version'], 'goal': near, 'request_id': 'c' * 32}
    route = service.preview(body)
    assert route['goal'] == pytest.approx(point)
    assert route['reference_path'] and route['length_m'] > 0
    task = service.submit(body)
    assert task['goal']['x'] == pytest.approx(point['x'])
    assert task['localization_grade'] == 'fixed' and task['arrival_tolerance_m'] == 0.5
    # Localization loss doesn't turn a retry into a second motion.
    state.update(ready=False, localization_grade='float')
    assert service.submit(body)['arrival_tolerance_m'] == 0.5
    state.update(ready=True)
    second = service.submit({**body, 'request_id': 'd' * 32})
    assert second['arrival_tolerance_m'] == 1.5
    assert store.snapshot(task['task_id'])['cancel_requested']
    recovered = TaskStore(journal=tmp_path / 'tasks.json')
    assert recovered.snapshot(second['task_id'])['state'] == 'failed'
    assert recovered.snapshot(second['task_id'])['localization_grade'] == 'float'
    with pytest.raises(ValueError, match='2 m'):
        service.preview({**body, 'goal': {'x': 1e6, 'y': 1e6}})
    with pytest.raises(ValueError, match='known road'):
        service.preview({**body, 'blocked_road_ids': ['unknown']})


def test_web_assets_and_api_are_same_origin_and_cannot_escape(tmp_path):
    from urllib.error import HTTPError
    web = tmp_path / 'web'
    web.mkdir()
    (web / 'index.html').write_text('<html>outdoor</html>')
    (tmp_path / 'private.txt').write_text('not a web asset')
    backend = SimpleNamespace(status=lambda: {'ready': False})
    server = serve(backend, port=0, web_root=web)
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        assert urlopen(base + '/').read() == b'<html>outdoor</html>'
        assert json.load(urlopen(base + '/api/status')) == {'ready': False}
        for path in ('/%2e%2e/private.txt', '/assets/missing.js'):
            with pytest.raises(HTTPError) as error:
                urlopen(base + path)
            assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()


def test_outdoor_profile_preserves_indoor_and_disables_duplicate_owners():
    import copy
    import runpy
    profile = runpy.run_path('deploy/lynx/profile.py')['outdoor_profile']
    original = {'name': 'lynx', 'system': {'atlas': {}, 'scene': {}},
                'primitive': [{'name': 'lynx_quadruped', 'config': {'odom_source_topic': '/outdoor/odom'}}],
                'service': [{'name': 'mapping'}, {'name': 'nav2'}, {'name': 'speech'}],
                'skill': [{'name': 'explore'}]}
    before = copy.deepcopy(original)
    output = profile(original, Path('/robot/outdoorNav'))
    assert original == before
    assert [s['name'] for s in output['service']] == ['speech', 'outdoor_navigation']
    assert output['primitive'][0]['config']['odom_source_topic'] == '/outdoor/odom'
    assert 'scene' not in output['system'] and not output['skill']
    original['service'].append({'name': 'retained', 'config': {'provider_ids': {'map': 'mapping'}}})
    with pytest.raises(ValueError, match='depends on indoor'):
        profile(original, Path('/robot/outdoorNav'))


@pytest.mark.skipif(__import__('importlib').util.find_spec('rclpy') is None, reason='requires ROS Humble')
def test_hardware_policy_downgrade_and_watchdog_output():
    from unittest.mock import Mock
    import time
    from geometry_msgs.msg import Twist
    from outdoornav.config import DEFAULTS
    from outdoornav_ros.lynx_navigation import LynxNavigation
    from outdoornav_ros.lynx_watchdog import Watchdog
    node = object.__new__(LynxNavigation)
    now = time.monotonic()
    node.config = {**DEFAULTS, **{k: True for k in ('calibrated', 'antenna_offset_confirmed',
                    'footprint_confirmed', 'cloud_frame_confirmed', 'motion_enabled', 'odometry_confirmed')}}
    node.config.update(localization_mode='external_odom',heading_source='configured')
    node.resolve_topic_name = lambda topic: topic
    node.geo = {'ready': True, 'grade': 'float', 'extrinsics': {'ready': True}}
    node.diagnostic = False
    node.guard, node.guard_at = {"output_topic": "/cmd_vel", "conflict": False}, now
    node.geo_at, node.velocity_at = now, now
    node.stats = {k: now for k in ('lidar', 'global_map', 'local_map')}
    node.tasks = TaskStore()
    task = node.tasks.submit({'x': 0, 'y': 0})
    node.owner = task['task_id']
    node.tasks.update(node.owner, localization_grade='fixed')
    node.service_active, node.startup_stopped, node.stop_failure = True, True, None
    node.lifecycle_clients = {}
    node.path_client = node.follow_client = SimpleNamespace(server_is_ready=lambda: True)
    node.pose = lambda: {'x': 0, 'y': 0, 'yaw': 0, 'age_s': 0}
    node.bundle = {'scene_version': 's', 'route_version': 'r'}
    assert not node.status()['health']['localization']
    node.tasks.update(node.owner, localization_grade='float')
    assert node.status()['ready']
    # Republishing must not keep an expired controller command alive.
    node.hardware, node.stopping = True, False
    node.tasks.update(node.owner, state='navigating')
    node.raw_command, node.raw_received = Twist(), now - 0.4
    node.raw_command.linear.x = 0.1
    node.command_pub = Mock()
    node.safety_tick()
    assert node.command_pub.publish.call_args.args[0].linear.x == 0
    node.raw_received = time.monotonic()
    node.safety_tick()
    assert node.command_pub.publish.call_args.args[0].linear.x == 0.1
    guard = object.__new__(Watchdog)
    guard.config = {**DEFAULTS, 'motion_enabled': False}
    guard.diagnostic = False
    guard.command, guard.command_at, guard.scan_at, guard.geo_at = Twist(), now, now, now
    guard.command.linear.x = 0.1
    guard.ready, guard.conflict, guard.pub = True, False, Mock()
    guard.pub.topic_name = "/cmd_vel"
    guard.status_pub = Mock()
    guard.tick()
    assert guard.pub.publish.call_args.args[0].linear.x == 0
    guard.config['motion_enabled'] = True
    guard.tick()
    assert guard.pub.publish.call_args.args[0].linear.x == 0.1
    guard.diagnostic = True
    guard.tick()
    assert guard.pub.publish.call_args.args[0].linear.x == 0
    guard.diagnostic = False
    guard.scan_at = now - 2
    guard.tick()
    assert guard.pub.publish.call_args.args[0].linear.x == 0
    guard.scan_at = now
    guard.command_at = now - 2
    guard.tick()
    assert guard.pub.publish.call_args.args[0].linear.x == 0


@pytest.mark.skipif(__import__('importlib').util.find_spec('rclpy') is None, reason='requires ROS Humble')
def test_native_odometry_keeps_source_time_measures_speed_and_rejects_replay():
    from unittest.mock import Mock
    from nav_msgs.msg import Odometry
    from outdoornav_ros.lynx_localizer import LynxLocalizer
    node = object.__new__(LynxLocalizer)
    node.config = {"sensor_timeout_s": 1.0}
    node.direct = None
    node.alignment = GeoAlignment({'latitude': 30, 'longitude': 120}, {
        'calibrated': False, 'initial_heading_rad': 0, 'antenna_xy_m': [0, 0],
        'map_offset_xy_m': [0, 0]})
    node.estimator = node.alignment
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_200_000_000))
    from collections import deque
    node.sent_transforms = deque(maxlen=200)
    node.odom_pub, node.tf = Mock(), Mock()
    node.ownership_conflict = False
    first = Odometry()
    first.header.stamp.sec = 10
    first.pose.pose.orientation.w = 1.0
    node.odom(first)
    node.odom_pub.publish.assert_not_called()
    second = Odometry()
    second.header.stamp.sec, second.header.stamp.nanosec = 10, 100_000_000
    second.pose.pose.orientation.w = 1.0
    second.pose.pose.position.x = 0.05
    node.odom(second)
    result = node.odom_pub.publish.call_args.args[0]
    assert result.twist.twist.linear.x == pytest.approx(0.5)
    assert result.header.stamp.nanosec == 100_000_000
    node.odom(second)
    assert node.odom_pub.publish.call_count == 1
    assert node.alignment.fault

    # Our TF survives a ROS serialization roundtrip; a foreign owner latches a fault.
    from rclpy.serialization import serialize_message, deserialize_message
    from tf2_msgs.msg import TFMessage
    import copy
    own = node.tf.sendTransform.call_args.args[0]
    roundtrip = deserialize_message(serialize_message(TFMessage(transforms=[own])), TFMessage)
    node.on_tf(roundtrip)
    assert not node.ownership_conflict
    foreign = copy.deepcopy(own)
    foreign.transform.translation.x += 1.0
    node.on_tf(TFMessage(transforms=[foreign]))
    assert node.ownership_conflict


@pytest.mark.skipif(__import__('importlib').util.find_spec('rclpy') is None, reason='requires ROS Humble')
def test_stationary_report_exposes_velocity_mismatch_and_nonfinite_values():
    import runpy
    import math
    from outdoornav.diagnostics import json_safe
    inspect = runpy.run_path('deploy/lynx/sensor_check.py')['odometry_summary']
    first = dict(stamp_s=1.0, received_s=1.0, age_s=0.0, x=0.0, y=0.0,
                 yaw=0.0, quaternion_norm=1.0, linear_mps=0.0, angular_rps=0.0)
    second = {**first, 'stamp_s': 1.1, 'received_s': 1.1, 'x': 1.0}
    result = inspect([first, second, {**second, 'x': math.nan}], 1.0)
    assert result['state'] == 'anomalous'
    assert result['speed_difference_mps']['max'] == pytest.approx(10)
    assert result['duplicate_stamps'] == 1
    assert result['nonfinite_samples'] == 1
    json.dumps(json_safe(result), allow_nan=False)
    assert inspect([], 1.0)['state'] == 'no_data'

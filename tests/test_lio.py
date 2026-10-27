import copy
import json
import math
from pathlib import Path

import numpy as np
import pytest

from outdoornav.config import load_robot_config
from outdoornav.lio import LioPose, canonical_points, fastlio_parameters, imu_si, transform, validate_lio


def configuration():
    config = json.loads(Path('deploy/lynx/robot.json').read_text())
    lio = json.loads(Path('deploy/lynx/lio.example.json').read_text())
    lio.update(enabled=True, extrinsics_confirmed=True, timing_confirmed=True,
               imu_units_confirmed=True, imu_frame='imu', imu_source_frame='imu', scan_lines=96,
               base_from_imu=np.eye(4).tolist(), base_from_lidar=np.eye(4).tolist())
    return {**config, 'localization_mode':'external_odom', 'heading_source':'configured', 'sensor_timeout_s': 1, 'odom_topic': '/outdoor/lio/odom', 'lio': lio}


def test_lio_requires_measured_contract_and_composes_backend_extrinsics(tmp_path):
    config = configuration()
    config['lio']['base_from_imu'][0][3] = 0.2
    config['lio']['base_from_lidar'][1][3] = 0.3
    params = fastlio_parameters(config)
    assert params['mapping.extrinsic_T'] == pytest.approx([-0.2, 0.3, 0])
    assert params['common.time_sync_en'] is False
    path = tmp_path / 'robot.json'
    path.write_text(json.dumps(config))
    assert load_robot_config(path)['motion_enabled'] is False
    for key, value in (('timing_confirmed', False), ('extrinsics_confirmed', False),
                       ('imu_units_confirmed', False), ('base_from_imu', None),
                       ('scan_lines', 129), ('time_unit', 'guessed')):
        invalid = copy.deepcopy(config)
        invalid['lio'][key] = value
        with pytest.raises(ValueError):
            validate_lio(invalid)
    config['odom_topic'] = '/ODOM'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_robot_config(path)


def test_timed_cloud_converts_airy_absolute_end_and_relative_units():
    lio = configuration()['lio']
    points = np.zeros((30, 6))
    points[:, 0] = 2
    points[:, 4] = np.arange(30)
    points[:, 5] = 100 + np.linspace(0, .1, 30)
    cloud, start = canonical_points(points[::-1], 100.1, lio)
    assert start == 100
    assert cloud[:, 5] == pytest.approx(np.linspace(0, .1, 30))
    # A relative microsecond stream with a scan-start header follows the same path.
    points[:, 5] = np.linspace(0, 100000, 30)
    relative = {**lio, 'point_time_absolute': False, 'time_unit': 'us', 'header_stamp_reference': 'start'}
    cloud, start = canonical_points(points, 100, relative)
    assert start == 100 and cloud[-1, 5] == pytest.approx(.1)
    for column, value in ((4, 128), (5, 0), (5, 1e9)):
        bad = points.copy()
        bad[:, column] = value
        with pytest.raises(ValueError):
            canonical_points(bad, 100, relative)


def test_airy_imu_g_and_degrees_are_converted_to_si():
    gyro, accel = imu_si([0, 0, 180], [0, -1, 0], {'gyro_unit': 'deg/s', 'accel_unit': 'g'})
    assert gyro == pytest.approx([0, 0, math.pi])
    assert accel == pytest.approx([0, -9.80665, 0])


def test_lio_base_lever_arm_rotation_and_reset():
    base_from_imu = transform([.4, -.1, .2], [0, 0, 0, 1])
    converter = LioPose(base_from_imu)
    for i in range(11):
        yaw = i*.05
        world_from_base = transform([10, 20, 0], [0, 0, math.sin(yaw/2), math.cos(yaw/2)])
        world_from_imu = world_from_base @ base_from_imu
        matrix, heading = converter.update(world_from_imu[:3, 3],
                                          [0, 0, math.sin(yaw/2), math.cos(yaw/2)], 1+i*.1)
        # The IMU moves during rotation, but the robot origin stays stationary.
        assert matrix[:3, 3] == pytest.approx([0, 0, 0], abs=1e-10)
        assert heading == pytest.approx(yaw)
    with pytest.raises(ValueError, match='reset'):
        converter.update([0, 0, 0], [0, 0, 0, 1], 2.1)


@pytest.mark.skipif(__import__('importlib').util.find_spec('rclpy') is None, reason='requires ROS Humble')
def test_ros_adapter_conversion_stale_imu_and_localizer_gate():
    from types import SimpleNamespace
    from unittest.mock import Mock
    import time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from outdoornav_ros.lynx_lio import LynxLio, quaternion
    from outdoornav_ros.lynx_localizer import LynxLocalizer

    node = object.__new__(LynxLio)
    node.config = configuration()
    node.lio = node.config['lio']
    node.pose = LioPose(node.lio['base_from_imu'])
    node.fault = None
    node.received, node.stamps = {}, {}
    node.scan_end, node.odom_at, node.stable_since = 10.1, -math.inf, None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_200_000_000))
    node.get_logger = lambda: Mock()
    node.count_publishers = lambda _: 1
    node.imu_pub, node.odom_pub, node.status_pub = Mock(), Mock(), Mock()
    node.cloud_pub = Mock()
    imu = Imu()
    imu.header.stamp.sec, imu.header.stamp.nanosec = 10, 100_000_000
    imu.header.frame_id = 'imu'
    imu.linear_acceleration.z = 1.
    node.imu(imu)
    assert node.imu_pub.publish.call_args.args[0].linear_acceleration.z == pytest.approx(9.80665)
    fields = [PointField(name=name, offset=i*8, datatype=8, count=1) for i, name in enumerate(
        ('x', 'y', 'z', 'intensity', 'ring', 'timestamp'))]
    header = Header(stamp=imu.header.stamp, frame_id=node.lio['lidar_frame'])
    cloud = point_cloud2.create_cloud(header, fields,
        [(2., 0., 0., 10., float(i), 10.+i*.1/29) for i in range(30)])
    node.cloud(cloud)
    assert node.fault is None
    canonical = node.cloud_pub.publish.call_args.args[0]
    assert canonical.header.stamp.sec == 10 and canonical.header.stamp.nanosec == 0
    assert [f.name for f in canonical.fields][-2:] == ['ring', 'time']
    raw = Odometry()
    raw.header.stamp = copy.deepcopy(imu.header.stamp)
    raw.header.frame_id, raw.child_frame_id = 'camera_init', 'body'
    raw.pose.pose.orientation.w = 1.
    node.odom(raw)
    result = node.odom_pub.publish.call_args.args[0]
    assert result.header.frame_id == 'odom' and result.child_frame_id == 'base_link'
    assert result.header.stamp.nanosec == 100_000_000
    node.received['imu'] -= 2
    raw.header.stamp.nanosec += 100_000_000
    node.odom(raw)
    assert node.odom_pub.publish.call_count == 1
    node.status()
    assert not json.loads(node.status_pub.publish.call_args.args[0].data)['ready']
    # Repeated IMU epochs are latched faults; they cannot restore output freshness.
    node.imu(imu)
    assert node.fault
    localizer = object.__new__(LynxLocalizer)
    localizer.config = node.config
    localizer.lio_status, localizer.lio_at = {'ready': True}, 100
    assert localizer.lio_ready(100.1)
    assert not localizer.lio_ready(101)
    for angle in (0, .7, math.pi):
        matrix = transform([0, 0, 0], [0, 0, math.sin(angle/2), math.cos(angle/2)])
        rebuilt = transform([0, 0, 0], quaternion(matrix[:3, :3]))
        assert rebuilt == pytest.approx(matrix)

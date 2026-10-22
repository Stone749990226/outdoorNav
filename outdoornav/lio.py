"""FAST-LIO input contract and IMU-origin to base-origin conversion.

Transforms are T_parent_child: they map child coordinates into parent coordinates.
The backend estimates the IMU pose, not the pose of base_link.
"""

import math

import numpy as np


def transform(xyz, xyzw):
    xyz, q = np.asarray(xyz, dtype=float), np.asarray(xyzw, dtype=float)
    if xyz.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[xyz, q]).all():
        raise ValueError('Invalid LIO translation/quaternion')
    if abs(float(q @ q) - 1) > 0.05:
        raise ValueError('LIO quaternion is not normalized')
    x, y, z, w = q / np.linalg.norm(q)
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    result[:3, 3] = xyz
    return result


def checked_matrix(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-6):
        raise ValueError('LIO extrinsics must be measured rigid 4x4 transforms')
    return matrix


def quaternion(rotation):
    """Rotation matrix to xyzw, including rotations near 180 degrees."""
    r = rotation
    k = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], r.trace()],
    ]) / 3
    _, vectors = np.linalg.eigh(k)
    return vectors[:, -1]


def validate_lio(config):
    """Disabled configurations need no guessed sensor model or extrinsics."""
    lio = config.get('lio', {'enabled': False})
    if not isinstance(lio, dict) or type(lio.get('enabled')) is not bool:
        raise ValueError('lio.enabled must be an explicit boolean')
    if not lio['enabled']:
        return lio
    for key in ('extrinsics_confirmed', 'timing_confirmed', 'imu_units_confirmed'):
        if lio.get(key) is not True:
            raise ValueError('LIO requires ' + key)
    for key in ('imu_topic',):
        if not isinstance(lio.get(key), str) or not lio[key].startswith('/') or lio[key].startswith('/outdoor/'):
            raise ValueError('LIO requires an external absolute ' + key)
    for key in ('imu_frame', 'lidar_frame', 'time_field', 'ring_field'):
        if not isinstance(lio.get(key), str) or not lio[key] or lio[key].startswith('/'):
            raise ValueError('LIO requires ' + key)
    if lio['imu_frame'] == 'base_link' and not np.allclose(checked_matrix(lio.get('base_from_imu')), np.eye(4)):
        raise ValueError('base_link IMU requires identity base_from_imu')
    for key in ('allow_empty_imu_frame', 'point_time_absolute'):
        if type(lio.get(key)) is not bool:
            raise ValueError('LIO requires an explicit ' + key)
    if not isinstance(lio.get('imu_source_frame'), str):
        raise ValueError('LIO requires the observed imu_source_frame')
    if not lio['imu_source_frame'] and not lio['allow_empty_imu_frame']:
        raise ValueError('An empty IMU source frame requires explicit confirmation')
    if lio.get('gyro_unit') not in ('rad/s', 'deg/s') or lio.get('accel_unit') not in ('m/s2', 'g'):
        raise ValueError('LIO requires explicit gyro/acceleration units')
    if lio.get('time_unit') not in ('s', 'ms', 'us', 'ns'):
        raise ValueError('LIO time_unit must be s, ms, us or ns')
    if lio.get('header_stamp_reference') not in ('start', 'end'):
        raise ValueError('LIO header_stamp_reference must be start or end')
    if type(lio.get('scan_lines')) is not int or not 1 <= lio['scan_lines'] <= 128:
        raise ValueError('LIO scan_lines must be in [1, 128]')
    for key in ('scan_period_s', 'max_scan_duration_s', 'input_timeout_s', 'acc_cov',
                'gyr_cov', 'b_acc_cov', 'b_gyr_cov'):
        value = lio.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('LIO requires positive measured ' + key)
    if not lio['scan_period_s'] <= lio['max_scan_duration_s'] < lio['input_timeout_s'] <= config['sensor_timeout_s']:
        raise ValueError('Invalid LIO scan duration/input timeout')
    checked_matrix(lio.get('base_from_imu'))
    checked_matrix(lio.get('base_from_lidar'))
    if lio['lidar_frame'] == 'base_link' or (lio['imu_frame'] == lio['lidar_frame'] and
            not np.allclose(lio['base_from_imu'], lio['base_from_lidar'])):
        raise ValueError('LIO sensor frames disagree with their extrinsics')
    if type(lio.get('start_driver')) is not bool:
        raise ValueError('LIO requires explicit start_driver')
    if lio['start_driver'] and not isinstance(lio.get('driver_config'), str):
        raise ValueError('LIO requires a driver_config path')
    if config.get('odom_topic') != '/outdoor/lio/odom':
        raise ValueError('Enabled LIO requires odom_topic=/outdoor/lio/odom')
    if not config.get('cloud_topic', '').startswith('/') or config['cloud_topic'].startswith('/outdoor/'):
        raise ValueError('LIO requires an external cloud_topic')
    return lio


def imu_si(gyro, accel, lio):
    values = np.asarray([*gyro, *accel], dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('Nonfinite IMU measurement')
    values[:3] *= 1 if lio['gyro_unit'] == 'rad/s' else math.pi/180
    values[3:] *= 1 if lio['accel_unit'] == 'm/s2' else 9.80665
    return values[:3], values[3:]


def fastlio_parameters(config):
    lio = validate_lio(config)
    if not lio['enabled']:
        raise ValueError('LIO is disabled')
    imu_from_lidar = np.linalg.inv(checked_matrix(lio['base_from_imu'])) @ checked_matrix(lio['base_from_lidar'])
    return {
        'common.lid_topic': '/outdoor/lio/points',
        'common.imu_topic': '/outdoor/lio/imu',
        'common.time_sync_en': False,
        'common.time_offset_lidar_to_imu': 0.0,
        'preprocess.lidar_type': 2,  # adapter emits Velodyne-compatible timed points
        'preprocess.scan_line': lio['scan_lines'],
        'preprocess.timestamp_unit': 0,  # seconds
        'preprocess.scan_rate': max(1, round(1 / lio['scan_period_s'])),
        'preprocess.blind': 0.1,
        'mapping.extrinsic_est_en': False,
        'mapping.extrinsic_T': imu_from_lidar[:3, 3].tolist(),
        'mapping.extrinsic_R': imu_from_lidar[:3, :3].ravel().tolist(),
        **{'mapping.' + key: float(lio[key]) for key in ('acc_cov', 'gyr_cov', 'b_acc_cov', 'b_gyr_cov')},
        'publish.path_en': False, 'publish.map_en': False,
        'publish.scan_publish_en': False, 'publish.effect_map_en': False,
        'pcd_save.pcd_save_en': False, 'runtime_pos_log_enable': False,
    }


def canonical_points(points, stamp, lio):
    """Nx6 x/y/z/intensity/ring/time -> timed points; never invent scan times."""
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 6:
        raise ValueError('Expected x/y/z/intensity/ring/time fields')
    values = values[np.isfinite(values).all(axis=1)].copy()
    if len(values) < 20:
        raise ValueError('Too few finite lidar points')
    rings = values[:, 4]
    if np.any((rings < 0) | (rings >= lio['scan_lines']) | (rings != np.floor(rings))):
        raise ValueError('Invalid lidar ring or scan_lines')
    values[:, 5] *= {'s': 1, 'ms': 1e-3, 'us': 1e-6, 'ns': 1e-9}[lio['time_unit']]
    scan_start = stamp
    if lio['point_time_absolute']:
        if lio['header_stamp_reference'] == 'end':
            if abs(float(np.max(values[:, 5]))-stamp) > 0.01:
                raise ValueError('Absolute point time does not match scan-end header')
            scan_start = float(np.min(values[:, 5]))
        values[:, 5] -= scan_start
    elif lio['header_stamp_reference'] == 'end':
        scan_start -= float(np.max(values[:, 5]))
    times = values[:, 5]
    if np.min(times) < -1e-6 or not 0 < np.max(times) <= lio['max_scan_duration_s']:
        raise ValueError('Point times must be offsets from the scan-start header')
    if np.ptp(times) <= 1e-6:
        raise ValueError('Lidar point times have no scan duration')
    values[:, 5] = np.maximum(times, 0)
    # FAST-LIO checks the last point time to select its deskew path.
    return values[np.argsort(values[:, 5], kind='stable')], scan_start


class LioPose:
    """Convert IMU poses to a gravity-aligned local frame with initial base yaw 0."""
    def __init__(self, base_from_imu):
        self.imu_from_base = np.linalg.inv(checked_matrix(base_from_imu))
        self.local_from_world = None
        self.previous = None

    def update(self, xyz, xyzw, stamp):
        world_from_base = transform(xyz, xyzw) @ self.imu_from_base
        if self.local_from_world is None:
            yaw = math.atan2(world_from_base[1, 0], world_from_base[0, 0])
            self.local_from_world = transform([0, 0, 0], [0, 0, math.sin(-yaw/2), math.cos(-yaw/2)])
            self.local_from_world[:3, 3] = -self.local_from_world[:3, :3] @ world_from_base[:3, 3]
        pose = self.local_from_world @ world_from_base
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        if self.previous is not None:
            old_stamp, old_pose, old_yaw = self.previous
            dt = stamp - old_stamp
            turn = abs(math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw)))
            if dt <= 0 or dt >= 1 or np.linalg.norm(pose[:3, 3]-old_pose[:3, 3]) > 0.25+2*dt or turn > 0.25+2*dt:
                raise ValueError('LIO odometry reset, gap or implausible motion; restart required')
        self.previous = stamp, pose.copy(), yaw
        return pose, yaw

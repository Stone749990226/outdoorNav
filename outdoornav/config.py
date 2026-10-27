"""Shared defaults for the outdoor deployment and its quality gates."""

import json
import math
from pathlib import Path

from .lio import validate_lio

DEFAULTS = {
    'gnss_fixed_hacc_m': 0.20,
    'gnss_float_hacc_m': 1.0,
    'gnss_stable_s': 5.0,
    'correction_max_age_s': 5.0,
    'sensor_timeout_s': 1.0,
    'command_timeout_s': 0.3,
    'fixed_arrival_m': 0.5,
    'float_arrival_m': 1.5,
    'gnss_speed_accuracy_mps': 0.10,
    'max_speed_mps': 0.4,
    'max_angular_speed_rps': 0.4,
    'scan_range_min_m': 0.0,
    'obstacle_layer_enabled': True,
    'local_track_unknown': True,
}


WIT_DEFAULTS = {
    'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    'baud': 9600, 'udp_port': 18767, 'timeout_s': 0.5,
    'base_from_imu': None, 'magnetic_declination_rad': None, 'yaw_std_deg': None,
    'mounting_confirmed': False, 'reference_confirmed': False, 'quality_confirmed': False,
    'course_heading': False,
}


def load_robot_config(path):
    config = {**DEFAULTS, **json.loads(Path(path).read_text())}
    for key in ('calibrated', 'antenna_offset_confirmed', 'cloud_frame_confirmed',
                'footprint_confirmed', 'odometry_confirmed', 'motion_enabled'):
        if type(config.get(key)) is not bool:
            raise ValueError(key + ' must be an explicit boolean')
    boolean_defaults = {key for key in DEFAULTS if type(DEFAULTS[key]) is bool}
    for key in (*DEFAULTS, 'initial_heading_rad', 'footprint_radius_m',
                'cloud_min_height_m', 'cloud_max_height_m'):
        if key in boolean_defaults:
            continue
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(key + ' must be finite')
    for key in DEFAULTS:
        if key in boolean_defaults or key == 'scan_range_min_m':
            continue
        if config[key] <= 0:
            raise ValueError(key + ' must be positive')
    for key in ('antenna_xy_m', 'map_offset_xy_m'):
        values = config.get(key)
        if not isinstance(values, list) or len(values) != 2 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values
        ):
            raise ValueError(key + ' must contain two finite measurements')
    if not (0 < config['max_speed_mps'] <= 0.4 and 0 < config['max_angular_speed_rps'] <= 0.4
            and 0.1 <= config['footprint_radius_m'] <= 2
            and config['cloud_min_height_m'] < config['cloud_max_height_m']
            and config['gnss_fixed_hacc_m'] <= config['gnss_float_hacc_m']
            and config['fixed_arrival_m'] <= config['float_arrival_m']):
        raise ValueError('Invalid outdoor speed, footprint, sensor band or GNSS limits')
    if not isinstance(config['scan_range_min_m'], (int, float)) or config['scan_range_min_m'] < 0:
        raise ValueError('scan_range_min_m must be a non-negative distance')
    if type(config['local_track_unknown']) is not bool:
        raise ValueError('local_track_unknown must be an explicit boolean')
    if type(config['obstacle_layer_enabled']) is not bool:
        raise ValueError('obstacle_layer_enabled must be an explicit boolean')
    config.setdefault('localization_mode', 'external_odom')
    config.setdefault('feedback_confirmed', False)
    if type(config['feedback_confirmed']) is not bool:
        raise ValueError('feedback_confirmed must be an explicit boolean')
    if config['localization_mode'] not in ('external_odom', 'rtk_imu'):
        raise ValueError('localization_mode must be external_odom or rtk_imu')
    config.setdefault('heading_source', 'configured')
    if config['heading_source'] not in ('configured', 'wit_imu'):
        raise ValueError('heading_source must be configured or wit_imu')
    wit = config.get('wit_imu', {})
    if not isinstance(wit, dict):
        raise ValueError('wit_imu must be an object')
    wit = config['wit_imu'] = {**WIT_DEFAULTS, **wit}
    if not isinstance(wit['port'], str) or not wit['port'].startswith('/dev/'):
        raise ValueError('WIT IMU needs an explicit serial path')
    if wit['baud'] not in (4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600):
        raise ValueError('Unsupported WIT baud rate')
    if type(wit['udp_port']) is not int or not 1024 <= wit['udp_port'] <= 65535:
        raise ValueError('Invalid WIT UDP port')
    if wit['udp_port'] == config['gnss_udp_port']:
        raise ValueError('WIT and position need separate UDP ports')
    if (isinstance(wit['timeout_s'], bool) or not isinstance(wit['timeout_s'], (int, float))
            or not math.isfinite(wit['timeout_s']) or not 0 < wit['timeout_s'] <= 2):
        raise ValueError('Invalid WIT timeout')
    for key in ('mounting_confirmed', 'reference_confirmed', 'quality_confirmed', 'course_heading'):
        if type(wit[key]) is not bool:
            raise ValueError('WIT ' + key + ' must be an explicit boolean')
    if wit['base_from_imu'] is not None:
        from .lio import checked_matrix
        checked_matrix(wit['base_from_imu'])
    if wit['mounting_confirmed'] and wit['base_from_imu'] is None:
        raise ValueError('WIT mounting confirmation requires measured base_from_imu')
    for key in ('magnetic_declination_rad', 'yaw_std_deg'):
        value = wit[key]
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value)):
            raise ValueError('WIT ' + key + ' must be finite or null')
    if wit['yaw_std_deg'] is not None and wit['yaw_std_deg'] <= 0:
        raise ValueError('WIT yaw uncertainty must be positive')
    if wit['reference_confirmed'] and wit['magnetic_declination_rad'] is None:
        raise ValueError('WIT north reference requires a measured magnetic correction')
    if wit['quality_confirmed'] and wit['yaw_std_deg'] is None:
        raise ValueError('WIT quality confirmation requires measured heading uncertainty')
    if config.get('odom_topic') in ('/ODOM', '/outdoor/odom'):
        raise ValueError('Unreliable /ODOM and our own odometry output cannot be localization sources')
    if config['localization_mode'] == 'rtk_imu':
        if config['heading_source'] != 'wit_imu' or config.get('odom_topic'):
            raise ValueError('RTK/IMU mode uses WIT heading and no external odometry source')
        if config.get('lio', {}).get('enabled'):
            raise ValueError('RTK/IMU mode must not start LIO')
        needed = ('mounting_confirmed',) if wit['course_heading'] else ('mounting_confirmed','reference_confirmed','quality_confirmed')
        if config['motion_enabled'] and not all(wit[k] for k in needed):
            raise ValueError('Motion requires measured WIT mounting, true-north reference and quality')
    elif config['heading_source'] == 'wit_imu':
        raise ValueError('WIT heading currently requires rtk_imu localization')
    source = config.get('odom_topic', '')
    if not isinstance(source, str) or (source and not source.startswith('/')):
        raise ValueError('odom_topic must be empty or an absolute ROS topic')
    if source in ('/ODOM', '/outdoor/odom'):
        raise ValueError('Unreliable /ODOM and our own odometry output cannot be localization sources')
    if config['odometry_confirmed'] and not source:
        raise ValueError('A verified odometry source must be configured before confirmation')
    validate_lio(config)
    if source == '/outdoor/lio/odom' and not config.get('lio', {}).get('enabled'):
        raise ValueError('LIO odometry source requires lio.enabled=true')
    feedback_gate = 'feedback_confirmed' if config['localization_mode'] == 'rtk_imu' else 'odometry_confirmed'
    gates = ('calibrated', 'antenna_offset_confirmed', 'cloud_frame_confirmed',
             'footprint_confirmed', feedback_gate)
    if config['motion_enabled'] and not all(config[k] for k in gates):
        raise ValueError('Motion requires measured calibration, footprint, coverage and verified state feedback')
    return config

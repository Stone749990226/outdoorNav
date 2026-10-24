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
    'max_speed_mps': 0.15,
    'max_angular_speed_rps': 0.4,
}


HEADING_DEFAULTS = {
    'port': '/dev/rtk',
    'baud': 115200,
    'udp_port': 18766,
    'timeout_s': 1.5,
    'baseline_to_body_yaw_rad': None,
    'mounting_confirmed': False,
    'quality_confirmed': False,
}


def load_robot_config(path):
    config = {**DEFAULTS, **json.loads(Path(path).read_text())}
    for key in ('calibrated', 'antenna_offset_confirmed', 'cloud_frame_confirmed',
                'footprint_confirmed', 'odometry_confirmed', 'motion_enabled'):
        if type(config.get(key)) is not bool:
            raise ValueError(key + ' must be an explicit boolean')
    for key in (*DEFAULTS, 'initial_heading_rad', 'footprint_radius_m',
                'cloud_min_height_m', 'cloud_max_height_m'):
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(key + ' must be finite')
    for key in DEFAULTS:
        if config[key] <= 0:
            raise ValueError(key + ' must be positive')
    for key in ('antenna_xy_m', 'map_offset_xy_m'):
        values = config.get(key)
        if not isinstance(values, list) or len(values) != 2 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values
        ):
            raise ValueError(key + ' must contain two finite measurements')
    if not (0 < config['max_speed_mps'] <= 0.15 and 0 < config['max_angular_speed_rps'] <= 0.4
            and 0.1 <= config['footprint_radius_m'] <= 2
            and config['cloud_min_height_m'] < config['cloud_max_height_m']
            and config['gnss_fixed_hacc_m'] <= config['gnss_float_hacc_m']
            and config['fixed_arrival_m'] <= config['float_arrival_m']):
        raise ValueError('Invalid outdoor speed, footprint, sensor band or GNSS limits')
    config.setdefault('heading_source', 'configured')
    if config['heading_source'] not in ('configured', 'dual_antenna'):
        raise ValueError('heading_source must be configured or dual_antenna')
    heading = config.get('dual_heading', {})
    if not isinstance(heading, dict):
        raise ValueError('dual_heading must be an object')
    heading = config['dual_heading'] = {**HEADING_DEFAULTS, **heading}
    if not isinstance(heading['port'], str) or not heading['port'].startswith('/dev/'):
        raise ValueError('Heading receiver must use an explicit /dev/ serial path')
    if heading['baud'] not in (9600, 38400, 57600, 115200, 230400, 460800, 921600):
        raise ValueError('Unsupported heading serial baud rate')
    if type(heading['udp_port']) is not int or not 1024 <= heading['udp_port'] <= 65535:
        raise ValueError('Heading UDP port must be 1024..65535')
    if heading['udp_port'] == config['gnss_udp_port']:
        raise ValueError('Position and heading require separate UDP inputs')
    if (isinstance(heading['timeout_s'], bool) or not isinstance(heading['timeout_s'], (int, float))
            or not math.isfinite(heading['timeout_s']) or not 0 < heading['timeout_s'] <= 5):
        raise ValueError('Heading timeout must be finite and in (0, 5] seconds')
    offset = heading['baseline_to_body_yaw_rad']
    if offset is not None and (isinstance(offset, bool) or not isinstance(offset, (int, float))
                               or not math.isfinite(offset)):
        raise ValueError('Heading mounting angle must be finite radians or null')
    for key in ('mounting_confirmed', 'quality_confirmed'):
        if type(heading[key]) is not bool:
            raise ValueError('Heading ' + key + ' must be an explicit boolean')
    if heading['mounting_confirmed'] and offset is None:
        raise ValueError('Confirmed heading mounting requires a measured angle')
    if (config['motion_enabled'] and config['heading_source'] == 'dual_antenna'
            and not (heading['mounting_confirmed'] and heading['quality_confirmed'])):
        raise ValueError('Motion requires measured heading mounting and verified heading quality')
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
    gates = ('calibrated', 'antenna_offset_confirmed', 'cloud_frame_confirmed',
             'footprint_confirmed', 'odometry_confirmed')
    if config['motion_enabled'] and not all(config[k] for k in gates):
        raise ValueError('Motion requires measured calibration, footprint, coverage and verified native odometry')
    return config

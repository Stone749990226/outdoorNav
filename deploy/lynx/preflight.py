"""Reject outdoor owners and unsafe static TF; public vendor TF is informational."""

import argparse
import json
from pathlib import Path
import socket
import time

import yaml

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from outdoornav.config import load_robot_config
from outdoornav.diagnostics import StaticFrames


def endpoints(node, topic):
    return [{'node': info.node_namespace.rstrip('/') + '/' + info.node_name,
             'gid': bytes(info.endpoint_gid).hex()} for info in node.get_publishers_info_by_topic(topic)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('robot_config')
    parser.add_argument('--diagnostic', action='store_true')
    args = parser.parse_args()
    config = load_robot_config(args.robot_config)
    if args.diagnostic and config['motion_enabled']:
        parser.error('Diagnostic startup requires motion_enabled=false')
    rclpy.init(args=[])
    node = rclpy.create_node('outdoor_preflight')
    frames, seen = StaticFrames(), set()
    node.create_subscription(TFMessage, '/tf_static', frames.receive,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/outdoor/tf',
                             lambda msg: seen.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms), 100)
    until = time.monotonic() + 3
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.1)
    conflicts = sorted(set(node.get_node_names()) & {
        'controller_server', 'planner_server', 'bt_navigator', 'rtabmap', 'icp_odometry',
        'navigation', 'gnss_odom', 'lynx_geo_localization', 'lynx_command_watchdog',
        'lynx_lio_adapter', 'outdoor_fastlio',
    })
    topics = {topic: endpoints(node, topic) for topic in (
        '/tf', '/tf_static', '/outdoor/tf', '/cmd_vel', '/outdoor/diagnostic_cmd_vel',
        '/outdoor/lio/odom', '/outdoor/lio/raw_odom', '/outdoor/lio/status')}
    output = '/outdoor/diagnostic_cmd_vel' if args.diagnostic else '/cmd_vel'
    driver_conflicts = []
    lio = config.get('lio', {})
    if lio.get('enabled') and lio.get('start_driver'):
        for topic in (config['cloud_topic'], lio['imu_topic']):
            if endpoints(node, topic):
                driver_conflicts.append('existing_sensor_publisher:' + topic)
        path = Path(lio['driver_config'])
        if not path.is_absolute():
            path = Path(args.robot_config).resolve().parent / path
        drivers = yaml.safe_load(path.read_text())['lidar']
        for item in drivers:
            for key in ('msop_port', 'difop_port', 'imu_port'):
                port = item['driver'][key]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    try:
                        probe.bind(('0.0.0.0', port))
                    except OSError:
                        driver_conflicts.append('occupied_udp_port:' + str(port))
    failed = bool(conflicts or frames.faults or topics['/outdoor/tf'] or topics[output]
                  or driver_conflicts
                  or any(topics[t] for t in ('/outdoor/lio/odom', '/outdoor/lio/raw_odom', '/outdoor/lio/status')))
    report = {'ready_to_start': not failed, 'diagnostic': args.diagnostic,
              'conflicting_nodes': conflicts, 'publishers': topics,
              'lidar_driver_conflicts': driver_conflicts,
              'outdoor_tf': sorted(seen), 'static_faults': sorted(frames.faults),
              'note': 'Public /tf is not consumed by outdoor navigation; no services were changed.'}
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    if failed:
        raise SystemExit('Outdoor startup rejected; resolve reported owners or static TF first')


if __name__ == '__main__':
    main()

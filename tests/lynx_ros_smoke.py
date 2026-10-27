"""Zero-motion ROS integration on an isolated domain; run after the ROS build."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from outdoornav.bundle import load_bundle
from outdoornav.coordinates import local_radii

ROOT = Path(__file__).resolve().parents[1]


def packet(lat, lon, epoch, grade="float"):
    payload = bytearray(92)
    struct.pack_into('<I', payload, 0, epoch)
    payload[20], payload[21], payload[23] = 3, 3 | ((2 if grade == "fixed" else 1) << 6), 20
    if grade == "invalid":
        payload[21] = 0
    struct.pack_into('<iiiiII', payload, 24, round(lon * 1e7), round(lat * 1e7), 0, 0, 100 if grade == "fixed" else 700, 1000)
    data = bytes([1, 7, 92, 0]) + payload
    a = b = 0
    for byte in data:
        a, b = (a + byte) & 255, (b + a + byte) & 255
    body = f'GNGGA,123456,3000.0,N,12000.0,E,{4 if grade == "fixed" else 5},20,1.0,100,M,23,M,1.0,0000'
    check = 0
    for byte in body.encode():
        check ^= byte
    return f'${body}*{check:02X}\r\n'.encode(), b'\xb5\x62' + data + bytes([a, b])


def main():
    # Neither the replay nor the command watchdog can reach the robot's domain 0.
    os.environ['ROS_DOMAIN_ID'] = '87'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['OUTDOORNAV_WEB_ROOT'] = str(ROOT / 'web/dist')
    config = json.loads((ROOT / 'deploy/lynx/robot.json').read_text())
    config.update({key: True for key in ('calibrated', 'antenna_offset_confirmed',
                  'footprint_confirmed', 'cloud_frame_confirmed', 'odometry_confirmed')})
    config.update(localization_mode='external_odom', heading_source='configured', motion_enabled=False, odom_topic='/test/native_odom', cloud_topic='/test/cloud',
                  gnss_udp_port=28765, http_port=18765)
    bundle = load_bundle(ROOT / 'scenes/north_campus/generated/bundle.json')
    x, y = bundle['spawn'][:2]
    origin = bundle['origin_wgs84']
    east, north = local_radii(origin['latitude'])
    lat, lon = origin['latitude'] + math.degrees(y/north), origin['longitude'] + math.degrees(x/east)
    rclpy.init(args=[])
    node = rclpy.create_node('outdoor_zero_motion_probe')
    odom = node.create_publisher(Odometry, config['odom_topic'], 10)
    cloud = node.create_publisher(PointCloud2, config['cloud_topic'], 10)
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy
    vendor = node.create_publisher(TFMessage, '/tf', 10)
    static = node.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = 'base_link', 'test_lidar'
    mount.transform.rotation.w = 1.0
    mount.transform.translation.z = 0.2
    static.publish(TFMessage(transforms=[mount]))
    output = []
    from geometry_msgs.msg import Twist
    node.create_subscription(Twist, '/outdoor/diagnostic_cmd_vel', lambda msg: output.append((msg.linear.x, msg.angular.z)), 10)
    client = node.create_client(SetParametersAtomically, '/controller_server/set_parameters_atomically')
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with tempfile.TemporaryDirectory(prefix='outdoor-smoke-') as directory:
        folder = Path(directory)
        (folder / 'robot.json').write_text(json.dumps(config))
        log = (ROOT / 'zero-motion-smoke.log').open('w')
        command = ['ros2', 'launch', 'outdoornav_ros', 'lynx.launch.py',
                   'bundle:=' + str(ROOT / 'scenes/north_campus/generated/bundle.json'),
                   'robot_config:=' + str(folder / 'robot.json'),
                   'places:=' + str(ROOT / 'scenes/north_campus/places.json'), 'run_dir:=' + directory, 'diagnostic:=true']
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        base = 'http://127.0.0.1:18765'
        status = None
        iteration, epoch = 0, 0

        def pump(seconds, *, gnss=True, cloud_on=True, odom_on=True, duplicate=False, grade='float'):
            nonlocal iteration, epoch, status
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert process.poll() is None, 'ROS launch exited; inspect zero-motion-smoke.log'
                stamp = node.get_clock().now().to_msg()
                message = Odometry()
                message.header.stamp, message.header.frame_id, message.child_frame_id = stamp, 'native', 'native_base'
                message.pose.pose.position.x, message.pose.pose.position.y = float(x), float(y)
                message.pose.pose.orientation.w = 1.0
                if odom_on:
                    odom.publish(message)
                if cloud_on:
                    header = Header(stamp=stamp, frame_id='test_lidar')
                    fields = [PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1)
                              for i, axis in enumerate(('x', 'y', 'z'))]
                    cloud.publish(point_cloud2.create_cloud(header, fields, [(3.0, 0.0, 0.5), (3.0, 0.1, 0.5)]))
                # Deliberately conflicting factory pose must never enter outdoor TF.
                transform = TransformStamped()
                transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, 'map', 'base_link'
                transform.transform.rotation.w, transform.transform.translation.x = 1.0, 1234.0
                vendor.publish(TFMessage(transforms=[transform]))
                if gnss:
                    if not duplicate:
                        epoch += 100
                    for raw in packet(lat, lon, epoch, grade):
                        udp.sendto(struct.pack('!d', time.monotonic()) + raw, ('127.0.0.1', 28765))
                iteration += 1
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    status = json.load(urlopen(base + '/api/status', timeout=0.5))
                except OSError:
                    pass

        try:
            began = time.monotonic()
            while time.monotonic() - began < 40:
                pump(0.4)
                if status and status['health']['localization'] and status['health'].get('controller_server') and output:
                    break
            assert status and status['localization_grade'] == 'float', status
            assert status['health']['localization'] and status['health']['controller_server'], status
            assert status['health']['sensor_extrinsics'] and status['health']['exclusive_owners'], status
            assert abs(status['pose']['x'] - x) < 1, status
            assert status['diagnostic'] and not status['ready']
            assert node.count_publishers('/cmd_vel') == 0
            assert node.count_subscribers('/tf') == 0, 'An outdoor listener still consumes public dynamic TF'
            assert node.count_subscribers('/outdoor/tf') >= 3
            assert output and all(values == (0.0, 0.0) for values in output)
            assert b'<!doctype html>' in urlopen(base + '/').read().lower()
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter('goal_checker.xy_goal_tolerance', value=1.5).to_parameter_msg()]
            assert client.wait_for_service(timeout_sec=2)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            assert future.done() and future.result().result.successful, ('pending' if not future.done() else str(future.result()))
            pump(1.5, gnss=False)
            assert not status['health']['localization'], status
            pump(6)
            assert status['health']['localization'], status
            pump(1.5, duplicate=True)
            assert not status['health']['localization'], status
            pump(6, grade='fixed')
            assert status['health']['localization'] and status['localization_grade'] == 'fixed', status
            pump(0.5, grade='invalid')
            assert not status['health']['localization'], status
            pump(6)
            assert status['health']['localization'], status
            pump(1.5, cloud_on=False)
            assert not status['health']['lidar'], status
            pump(0.5)
            assert status['health']['lidar'], status
            pump(1.5, odom_on=False)
            assert not status['health']['odometry'] and not status['health']['localization'], status
            pump(0.5)
            assert 'odometry_reset_or_gap' in status['localization'], status
            mount.transform.translation.z = 0.3
            static.publish(TFMessage(transforms=[mount]))
            pump(0.5)
            assert not status['health']['sensor_extrinsics'] and not status['health']['exclusive_owners'], status
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
            transform.transform.rotation.w = 1.0
            transform.transform.translation.x = 9000.0
            private = node.create_publisher(TFMessage, '/outdoor/tf', 10)
            private.publish(TFMessage(transforms=[transform]))
            pump(1.5)
            assert status['tf_conflicts']['dynamic'], status
            assert node.count_publishers('/cmd_vel') == 0
            assert output and all(values == (0.0, 0.0) for values in output)
            print(json.dumps({'float_localization': True, 'controller_active': True,
                              'same_origin_web': True, 'arrival_parameter_update': True,
                              'vendor_tf_isolated': True, 'static_conflict_detected': True,
                              'gnss_loss_replay_invalid': True, 'odom_reset_and_lidar_loss': True,
                              'zero_commands_observed': len(output), 'physical_command_publishers': 0,
                              'ros_domain': 87}))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            log.close()
    udp.close()
    node.destroy_node()
    rclpy.shutdown()


def live_diagnostic():
    """Real sensors only, observe-only GNSS, and no physical command publisher."""
    from outdoornav.config import load_robot_config
    from geometry_msgs.msg import Twist
    config = load_robot_config(ROOT / 'deploy/lynx/robot.json')
    assert not config['motion_enabled'], 'Live diagnostic requires motion disabled'
    rclpy.init(args=[])
    node = rclpy.create_node('outdoor_live_diagnostic_probe')
    observed = []
    node.create_subscription(Twist, '/outdoor/diagnostic_cmd_vel',
                             lambda msg: observed.append((msg.linear.x, msg.angular.z)), 10)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    before = {info.node_namespace + '/' + info.node_name for info in node.get_publishers_info_by_topic('/cmd_vel')}
    base = 'http://127.0.0.1:' + str(config['http_port'])
    report = {}
    output = ROOT / 'artifacts/lynx'
    output.mkdir(parents=True, exist_ok=True)
    log = (output / 'live-diagnostic.log').open('w')
    with tempfile.TemporaryDirectory(prefix='outdoor-live-') as directory:
        env = {**os.environ, 'LYNX_RUN_DIR': directory, 'LYNX_GNSS_MODE': 'observe'}
        command = ['bash', str(ROOT / 'deploy/lynx/start.sh'), '--diagnostic']
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=log, start_new_session=True)
        try:
            deadline, status = time.monotonic() + 40, None
            while time.monotonic() < deadline:
                assert process.poll() is None, 'Diagnostic startup failed; inspect artifacts/lynx/live-diagnostic.log'
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    status = json.load(urlopen(base + '/api/status', timeout=0.5))
                except OSError:
                    continue
                if status.get('diagnostic') and observed:
                    break
            assert status and status['diagnostic'] and not status['ready'], status
            assert status['odometry_source'] is None
            assert status['command_topic'] == '/outdoor/diagnostic_cmd_vel'
            assert b'<!doctype html>' in urlopen(base + '/').read().lower()
            duplicate = subprocess.run(command, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, timeout=5)
            assert duplicate.returncode != 0 and b'already running' in duplicate.stdout
            def post(path, body):
                return urlopen(Request(base + path, data=json.dumps(body).encode(),
                                       headers={'Content-Type': 'application/json'}), timeout=2)
            post('/api/activation', {'active': True}).close()
            bundle = load_bundle(ROOT / 'scenes/north_campus/generated/bundle.json')
            try:
                post('/api/tasks', {'frame_id': 'map', 'scene_version': bundle['scene_version'],
                                    'route_version': bundle['route_version'],
                                    'goal': bundle['network']['roads'][0]['points'][0]}).close()
                raise AssertionError('Diagnostic service accepted a physical navigation task')
            except HTTPError as error:
                assert error.code == 503
            post('/api/activation', {'active': False}).close()
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)
            current = {info.node_namespace + '/' + info.node_name for info in node.get_publishers_info_by_topic('/cmd_vel')}
            assert current == before
            assert not any(info.node_name in ('lynx_geo_localization', 'outdoor_sensor_inspection')
                           for info in node.get_subscriptions_info_by_topic('/ODOM'))
            assert observed and all(value == (0.0, 0.0) for value in observed)
            status = json.load(urlopen(base + '/api/status'))
            assert not status['health']['service_active']
            report = {'diagnostic_start_stop': True, 'same_origin_web': True, 'duplicate_start_rejected': True,
                      'navigation_rejected': True, 'activation_deactivation': True,
                      'physical_publishers_before': sorted(before), 'physical_publishers_during': sorted(current),
                      'zero_diagnostic_commands': len(observed), 'status': status}
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                raise AssertionError('Diagnostic process did not clean up within 30 seconds')
            log.close()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and node.count_publishers('/outdoor/diagnostic_cmd_vel'):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert node.count_publishers('/outdoor/diagnostic_cmd_vel') == 0
        report['diagnostic_publishers_after_stop'] = 0
        (output / 'live-diagnostic.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != 'status'}))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live-diagnostic', action='store_true', help='Real read-only inputs; never publish physical commands')
    args = parser.parse_args()
    live_diagnostic() if args.live_diagnostic else main()

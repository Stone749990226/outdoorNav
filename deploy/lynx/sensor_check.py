"""Read-only stationary sensor report. Never opens serial or publishes commands."""

import argparse
import fcntl
import itertools
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

from outdoornav.config import load_robot_config
from outdoornav.diagnostics import StaticFrames, json_safe, summary
from outdoornav.localization import wrap


def stream_summary(samples):
    intervals = [b['received_s'] - a['received_s'] for a, b in zip(samples, samples[1:])]
    return {'count': len(samples), 'received': bool(samples),
            'frequency_hz': (len(samples)-1) / sum(intervals) if intervals and sum(intervals) > 0 else None,
            'age_s': summary([s['age_s'] for s in samples]),
            'motion_verified': False}


def odometry_summary(samples, timeout):
    report = stream_summary(samples)
    steps, invalid, replay, reversed_stamp, gaps = [], 0, 0, 0, 0
    for s in samples:
        if not all(math.isfinite(s[k]) for k in ('stamp_s', 'x', 'y', 'yaw', 'quaternion_norm', 'linear_mps', 'angular_rps')):
            invalid += 1
    for a, b in zip(samples, samples[1:]):
        dt = b['stamp_s'] - a['stamp_s']
        if not math.isfinite(dt):
            continue
        replay += dt == 0
        reversed_stamp += dt < 0
        gaps += dt > timeout
        if dt > 0:
            speed = math.hypot(b['x'] - a['x'], b['y'] - a['y']) / dt
            steps.append({'stamp_s': b['stamp_s'], 'dt_s': dt, 'pose_speed_mps': speed,
                          'pose_angular_rps': wrap(b['yaw'] - a['yaw']) / dt,
                          'reported_speed_mps': b['linear_mps'],
                          'speed_difference_mps': abs(speed - b['linear_mps'])})
    drift = [math.hypot(s['x'] - samples[0]['x'], s['y'] - samples[0]['y']) for s in samples] if samples else []
    anomalies = []
    for code, bad in (
        ('nonfinite_data', invalid), ('duplicate_stamp', replay), ('reversed_stamp', reversed_stamp),
        ('source_gap', gaps), ('invalid_quaternion', any(not 0.95 <= s['quaternion_norm'] <= 1.05 for s in samples)),
        ('stale_or_future_stamp', any(not -0.1 <= s['age_s'] < timeout for s in samples)),
        ('stationary_position_drift', any(v > 0.10 for v in drift)),
        ('stationary_reported_motion', any(s['linear_mps'] > 0.05 or abs(s['angular_rps']) > 0.05 for s in samples)),
    ):
        if bad:
            anomalies.append(code)
    report.update(state='no_data' if not samples else 'anomalous' if anomalies else 'received_unverified',
                  anomalies=anomalies, duplicate_stamps=replay, reversed_stamps=reversed_stamp,
                  source_gaps=gaps, nonfinite_samples=invalid, stationary_drift_m=summary(drift),
                  pose_speed_mps=summary([s['pose_speed_mps'] for s in steps]),
                  reported_speed_mps=summary([s['linear_mps'] for s in samples]),
                  speed_difference_mps=summary([s['speed_difference_mps'] for s in steps]),
                  samples=samples, derived_steps=steps)
    return report


def gnss_processes(root):
    """Inspect PIDs and open tty descriptors, never read config/credentials or serial."""
    readers, processes = [], []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if 'gnss_stream.py' in command or (str(root) + '/client.py') in command:
                processes.append(int(entry.name))
            for fd in (entry / 'fd').iterdir():
                target = os.readlink(fd)
                if target.startswith(('/dev/ttyACM', '/dev/ttyUSB')):
                    readers.append({'pid': int(entry.name), 'device': target})
        except (OSError, PermissionError):
            continue
    locked = None
    lock_path = root / 'client.lock'
    if lock_path.exists():
        try:
            with lock_path.open('r') as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    locked = False
                    fcntl.flock(lock, fcntl.LOCK_UN)
                except BlockingIOError:
                    locked = True
        except OSError:
            pass
    return {'collector_pids': sorted(processes), 'serial_open_fds': readers,
            'collector_lock_held': locked, 'serial_opened_by_inspector': False,
            'serial_inspection_complete': os.geteuid() == 0,
            'multiple_visible_readers': len({r['pid'] for r in readers}) > 1}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--robot-config', type=Path, default=Path(__file__).with_name('robot.json'))
    parser.add_argument('--odom')
    parser.add_argument('--cloud')
    parser.add_argument('--imu', action='append', default=[], help='Read-only IMU topic; repeat to compare sources')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('--seconds must be in (0, 60]')
    config = load_robot_config(args.robot_config)
    odom_topic = args.odom if args.odom is not None else config['odom_topic']
    if odom_topic in ('/ODOM', '/outdoor/odom'):
        parser.error('Do not use unreliable /ODOM or the navigation odometry output as an input')
    rclpy.init(args=[])
    node = rclpy.create_node('outdoor_sensor_inspection')
    samples, clouds, transforms, localization = [], [], set(), []
    frames = StaticFrames()
    imus = {topic: [] for topic in args.imu}

    def odom(msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        samples.append({'stamp_s': stamp, 'received_s': time.monotonic(),
                        'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
                        'age_s': node.get_clock().now().nanoseconds / 1e9 - stamp,
                        'x': p.x, 'y': p.y, 'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                        'quaternion_norm': q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w,
                        'linear_mps': math.hypot(v.linear.x, v.linear.y), 'angular_rps': v.angular.z})

    def cloud(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        sample = {'frame_id': msg.header.frame_id, 'stamp_s': stamp, 'received_s': time.monotonic(),
                  'age_s': node.get_clock().now().nanoseconds / 1e9 - stamp,
                  'points': msg.width * msg.height, 'sample_limit': 10000}
        sample['fields'] = [{'name': f.name, 'datatype': f.datatype, 'count': f.count} for f in msg.fields]
        matrix = frames.matrix(msg.header.frame_id)
        sample['static_extrinsics_ready'] = matrix is not None
        try:
            points = np.array([tuple(p) for p in itertools.islice(
                point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False), 10000)], dtype=float).reshape(-1, 3)
            finite = points[np.isfinite(points).all(axis=1)]
            sample.update(sampled_points=len(points), valid_sampled_points=len(finite))
            if matrix is not None:
                base = finite @ matrix[:3, :3].T + matrix[:3, 3]
                sample['base_height_m'] = summary(base[:, 2].tolist())
                sample['height_band_points'] = int(((base[:, 2] >= config['cloud_min_height_m']) &
                                                    (base[:, 2] <= config['cloud_max_height_m'])).sum())
        except (ValueError, KeyError, AssertionError, TypeError) as error:
            sample['decode_error'] = str(error)
        clouds.append(sample)

    def geo(msg):
        try:
            localization.append(json.loads(msg.data))
        except ValueError:
            pass

    def imu(topic, msg):
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        imus[topic].append({'stamp_s': stamp, 'received_s': time.monotonic(),
                           'age_s': node.get_clock().now().nanoseconds / 1e9 - stamp,
                           'frame_id': msg.header.frame_id,
                           'orientation_xyzw': [q.x, q.y, q.z, q.w],
                           'gyro_rps': [w.x, w.y, w.z], 'accel_mps2': [a.x, a.y, a.z],
                           'orientation_covariance': list(msg.orientation_covariance),
                           'angular_velocity_covariance': list(msg.angular_velocity_covariance),
                           'linear_acceleration_covariance': list(msg.linear_acceleration_covariance)})

    for topic in imus:
        node.create_subscription(Imu, topic, lambda msg, topic=topic: imu(topic, msg), qos_profile_sensor_data)
    if odom_topic:
        node.create_subscription(Odometry, odom_topic, odom, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, args.cloud or config['cloud_topic'], cloud, qos_profile_sensor_data)
    node.create_subscription(String, '/outdoor/localization', geo, 10)
    node.create_subscription(TFMessage, '/tf_static', frames.receive,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/tf', lambda msg: transforms.update(
        (t.header.frame_id, t.child_frame_id) for t in msg.transforms), 100)
    until = time.monotonic() + args.seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.1)
    publishers = {topic: [info.node_namespace.rstrip('/') + '/' + info.node_name
                         for info in node.get_publishers_info_by_topic(topic)]
                  for topic in ('/tf', '/tf_static', '/outdoor/tf', '/cmd_vel', '/outdoor/diagnostic_cmd_vel',
                                odom_topic, args.cloud or config['cloud_topic']) if topic}
    lidar = stream_summary(clouds)
    faults = sorted(frames.faults)
    lidar_anomalies = []
    if clouds:
        last = clouds[-1]
        if not last.get('static_extrinsics_ready'):
            lidar_anomalies.append('missing_static_extrinsics')
        if last.get('valid_sampled_points', 0) == 0:
            lidar_anomalies.append('no_valid_points')
        if not -0.1 <= last['age_s'] < config['sensor_timeout_s']:
            lidar_anomalies.append('stale_or_future_stamp')
    lidar.update(state='no_data' if not clouds else 'anomalous' if lidar_anomalies or faults else 'received_unverified',
                 anomalies=lidar_anomalies, static_faults=faults, samples=clouds,
                 posture='lying_down', standing_coverage_verified=False)
    gnss = gnss_processes(Path(os.environ.get('LYNX_NTRIP_DIR', '/home/nvidia/ntrip')))
    gnss.update(localization_messages=len(localization), latest=localization[-1] if localization else None,
                state='no_localization_status' if not localization else 'received',
                no_rtk_expected_indoors=True)
    odometry = odometry_summary(samples, config['sensor_timeout_s'])
    odometry['source_topic'] = odom_topic or None
    if not odom_topic:
        odometry['state'] = 'source_unconfigured'
    imu_reports = {}
    for topic, readings in imus.items():
        item = stream_summary(readings)
        flags = []
        if readings:
            if any(not sample['frame_id'] for sample in readings):
                flags.append('missing_frame_id')
            if any(not -0.1 <= sample['age_s'] < config['sensor_timeout_s'] for sample in readings):
                flags.append('stale_or_future_stamp')
            if any(not all(math.isfinite(v) for key in ('orientation_xyzw', 'gyro_rps', 'accel_mps2')
                           for v in sample[key]) for sample in readings):
                flags.append('nonfinite_values')
            if any(all(v == 0 for v in sample['angular_velocity_covariance']) for sample in readings):
                flags.append('gyro_uncertainty_unknown')
            if any(all(v == 0 for v in sample['orientation_covariance']) for sample in readings):
                flags.append('orientation_uncertainty_unknown')
        item.update(state='no_data' if not readings else 'received_unverified', flags=flags,
                    gyro_rps=[summary([r['gyro_rps'][axis] for r in readings]) for axis in range(3)],
                    accel_mps2=[summary([r['accel_mps2'][axis] for r in readings]) for axis in range(3)],
                    source_stamps_increasing=bool(readings) and all(b['stamp_s'] > a['stamp_s']
                        for a, b in zip(readings, readings[1:])), samples=readings,
                    heading_reference_verified=False, mounting_verified=False)
        imu_reports[topic] = item
    report = {'duration_s': args.seconds, 'posture': 'lying_down', 'motion_verified': False,
              'odometry': odometry, 'lidar': lidar,
              'gnss': gnss, 'imus': imu_reports, 'public_dynamic_tf': sorted(transforms), 'publishers': publishers}
    text = json.dumps(json_safe(report), indent=2, allow_nan=False)
    print(text)
    if args.report:
        args.report.write_text(text + '\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

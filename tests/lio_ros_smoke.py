"""Synthetic FAST-LIO integration, isolated domain 89; no hardware inputs/motion."""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

# Set before ROS imports; synthetic publishers must never enter the robot domain.
os.environ.update(ROS_DOMAIN_ID='89', ROS_LOCALHOST_ONLY='1')

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
import yaml

from outdoornav.lio import fastlio_parameters
from outdoornav.config import DEFAULTS


def run():
    root = Path(__file__).resolve().parents[1]
    config = {**DEFAULTS, **json.loads((root/'deploy/lynx/robot.json').read_text())}
    lio = json.loads((root/'deploy/lynx/lio.example.json').read_text())
    lio.update(enabled=True, imu_topic='/synthetic/imu', imu_frame='synthetic_imu',
               imu_source_frame='synthetic_imu', lidar_frame='synthetic_lidar',
               accel_unit='m/s2', point_time_absolute=False, header_stamp_reference='start',
               time_unit='s', time_field='time', scan_lines=96,
               extrinsics_confirmed=True, timing_confirmed=True, imu_units_confirmed=True,
               base_from_imu=np.eye(4).tolist(), base_from_lidar=np.eye(4).tolist())
    config.update(lio=lio, cloud_topic='/synthetic/points', odom_topic='/outdoor/lio/odom')
    with tempfile.TemporaryDirectory(prefix='outdoor-lio-smoke-') as directory:
        directory = Path(directory)
        robot, params = directory/'robot.json', directory/'fastlio.yaml'
        robot.write_text(json.dumps(config))
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': fastlio_parameters(config)}}))
        processes, logs = [], []
        rclpy.init(args=[])
        node = rclpy.create_node('outdoor_synthetic_lio_test')
        observed, status = [], {}
        node.create_subscription(Odometry, '/outdoor/lio/odom', observed.append, 10)

        def receive(msg):
            status.clear()
            status.update(json.loads(msg.data))

        node.create_subscription(String, '/outdoor/lio/status', receive, 10)
        imu_pub = node.create_publisher(Imu, '/synthetic/imu', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/synthetic/points', 10)
        try:
            for name, command in (
                ('adapter', ['ros2', 'run', 'outdoornav_ros', 'lynx_lio', '--ros-args', '-p', 'robot_config:='+str(robot)]),
                ('backend', ['ros2', 'run', 'fast_lio', 'fastlio_mapping', '--ros-args', '--params-file', str(params),
                             '-r', '/Odometry:=/outdoor/lio/raw_odom', '-r', '/tf:=/outdoor/lio/tf']),
            ):
                log = (directory/(name+'.log')).open('w')
                logs.append(log)
                processes.append(subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True))
            axis = np.linspace(-3, 3, 24)
            a, b = np.meshgrid(axis, axis)
            planes = []
            for dim, bounds in ((0, (-4, 4)), (1, (-4, 4)), (2, (-1, 3))):
                for bound in bounds:
                    p = np.zeros((a.size, 3))
                    p[:, dim] = bound
                    p[:, [i for i in range(3) if i != dim]] = np.c_[a.ravel(), b.ravel()]
                    planes.extend(p.tolist())
            rows = [(x, y, z, 10., i % 96, i*.095/(len(planes)-1))
                    for i, (x, y, z) in enumerate(planes)]
            fields = [PointField(name=k, offset=o, datatype=t, count=1) for k, o, t in (
                ('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7), ('ring', 16, 4), ('time', 20, 7))]
            started, next_cloud = time.monotonic(), 0
            while time.monotonic()-started < 18:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('LIO process exited before completion')
                stamp = node.get_clock().now().to_msg()
                imu = Imu()
                imu.header = Header(stamp=stamp, frame_id='synthetic_imu')
                imu.linear_acceleration.z = 9.80665
                imu_pub.publish(imu)
                now = time.monotonic()
                if now >= next_cloud and now-started > 1:
                    header = Header(frame_id='synthetic_lidar')
                    ns = node.get_clock().now().nanoseconds-100_000_000
                    header.stamp.sec, header.stamp.nanosec = divmod(ns, 1_000_000_000)
                    cloud_pub.publish(point_cloud2.create_cloud(header, fields, rows))
                    next_cloud = now+.1
                rclpy.spin_once(node, timeout_sec=.005)
                if status.get('ready') and len(observed) > 20:
                    break
            assert status.get('ready'), status
            assert observed and max(abs(m.pose.pose.position.x) for m in observed) < .1
            before = len(observed)
            until = time.monotonic()+1.2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
            assert not status.get('ready'), status
            assert len(observed) <= before+2
            assert not node.count_publishers('/cmd_vel')
            assert not node.count_publishers('/tf') and not node.count_publishers('/outdoor/tf')
            print(json.dumps({'synthetic': True, 'odometry_messages': len(observed),
                              'stable_output': True, 'input_loss_detected': True,
                              'motion_publishers': 0, 'public_tf_publishers': 0}))
        except BaseException:
            for log in logs:
                log.flush()
            for path in directory.glob('*.log'):
                print(path.name, path.read_text()[-6000:])
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            for log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()

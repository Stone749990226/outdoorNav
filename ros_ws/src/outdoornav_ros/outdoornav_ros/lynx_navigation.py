"""Hardware observations around the shared road-route task executor."""

import json
import math
import os
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose

from outdoornav.api import serve
from outdoornav.config import load_robot_config
from outdoornav.diagnostics import readiness_reasons
from outdoornav.places import Places
from outdoornav.outdoor import OutdoorService
from outdoornav.tasks import TaskStore, TERMINAL
from outdoornav.localization import wrap
from .navigation import Navigation


class LynxNavigation(Navigation):
    def __init__(self):
        super().__init__(hardware=True)
        for key in ('robot_config', 'places'):
            self.declare_parameter(key, '')
        self.config = load_robot_config(self.get_parameter('robot_config').value)
        self.declare_parameter('diagnostic', False)
        self.diagnostic = self.get_parameter('diagnostic').value
        self.places = Places(self.get_parameter('places').value, self.bundle)
        self.tasks = TaskStore(self.bundle['scene_version'], self.bundle['route_version'],
                               Path(self.get_parameter('run_dir').value) / 'tasks.json')
        self.outdoor = OutdoorService(self.bundle, self.places, self.tasks, self.status, self.config)
        self.geo, self.geo_at = {}, -math.inf
        self.guard, self.guard_at = {}, -math.inf
        self.service_active = False
        self.startup_stopped, self.stationary_since = False, None
        self.create_subscription(String, '/outdoor/localization', self.on_geo, 10)
        self.create_subscription(String, '/outdoor/command_status', self.on_guard, 10)
        self.create_subscription(Odometry, '/outdoor/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/outdoor/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.check_stationary)
        self.controller_parameters = self.create_client(SetParametersAtomically, "/controller_server/set_parameters_atomically", callback_group=self.group)
        self.http = serve(self, port=self.config.get('http_port', 8765), host='0.0.0.0',
                          web_root=os.environ.get('OUTDOORNAV_WEB_ROOT'))

    def check_stationary(self):
        now = time.monotonic()
        stopped = now - self.velocity_at < self.config["sensor_timeout_s"] and max(self.velocity) <= 0.01
        self.stationary_since = (now if self.stationary_since is None else self.stationary_since) if stopped else None
        if self.stationary_since is not None and now - self.stationary_since >= 1:
            self.startup_stopped = True

    def on_geo(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError("Localization status must be an object")
            self.geo = value
            self.geo_at = time.monotonic()
        except (ValueError, TypeError):
            self.geo = {}

    def on_guard(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('Command status must be an object')
            self.guard, self.guard_at = value, time.monotonic()
        except (ValueError, TypeError):
            self.guard = {}

    def on_scan(self, msg):
        age = self.get_clock().now().nanoseconds / 1e9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if -0.1 <= age < self.config["sensor_timeout_s"] and any(math.isfinite(r) and msg.range_min < r < msg.range_max for r in msg.ranges):
            self.mark('lidar')

    def status(self, include_task=True):
        now, config, geo = time.monotonic(), self.config, self.geo
        timeout = config['sensor_timeout_s']
        grade = geo.get('grade')
        owned = self.tasks.snapshot(self.owner, include_paths=False) if self.owner else None
        required = owned.get('localization_grade') if owned else None
        health = {key: now - self.stats.get(key, -math.inf) < limit
                  for key, limit in (('lidar', timeout), ('global_map', 3), ('local_map', timeout))}
        health.update({
            'localization': bool(geo.get('ready')) and now - self.geo_at < timeout
                            and (required != 'fixed' or grade == 'fixed'),
            'heading': bool(geo.get('heading', {}).get('ready')) if config['heading_source'] == 'wit_imu' else True,
            'calibration': config['calibrated'] and config['antenna_offset_confirmed']
                           and config['footprint_confirmed'] and config['cloud_frame_confirmed'],
            'motion_enabled': config['motion_enabled'],
            'production_mode': not self.diagnostic,
            'sensor_extrinsics': bool(geo.get('extrinsics', {}).get('ready')),
            'service_active': self.service_active,
            'controller_stopped': self.startup_stopped and self.stop_failure is None,
            'odometry': (config['feedback_confirmed'] if config['localization_mode'] == 'rtk_imu' else config['odometry_confirmed'])
                        and now-self.velocity_at < timeout,
            'actions': self.path_client.server_is_ready() and self.follow_client.server_is_ready(),
            'exclusive_owners': not geo.get('ownership_conflict', False) and not self.guard.get('conflict', False),
            'command_guard': now - self.guard_at < timeout and self.guard.get('output_topic') == self.resolve_topic_name(
                '/outdoor/diagnostic_cmd_vel' if self.diagnostic else '/cmd_vel'),
        })
        for name in self.lifecycle_clients:
            value, stamp = self.lifecycle.get(name, (False, 0))
            health[name] = value and now - stamp < 5
        pose = self.pose()
        health['tf'] = pose is not None and pose['age_s'] < timeout
        if not health['tf'] and geo.get('pose'):
            pose = {**geo['pose'], 'frame_id': 'map', 'stamp_s': 0,
                    'age_s': max(geo.get('fix_age_s') or 0, now - self.geo_at)}
        return {'schema_version': 1, 'mode': 'lynx', 'run_id': 'lynx-outdoor',
                'scene_version': self.bundle['scene_version'], 'route_version': self.bundle['route_version'],
                'ready': all(health.values()), 'health': health,
                'diagnostic': self.diagnostic,
                'localization_mode':config['localization_mode'],
                'odometry_source': 'rtk_doppler+wit_imu' if config['localization_mode'] == 'rtk_imu' else (config.get('odom_topic') or None),
                'feedback':geo.get('feedback'),'velocity_source':geo.get('velocity_source'),
                'tf_conflicts': geo.get('tf_conflicts', {}),
                'command_topic': self.resolve_topic_name('/outdoor/diagnostic_cmd_vel' if self.diagnostic else '/cmd_vel'),
                'not_ready_reasons': readiness_reasons(health, geo),
                'localization': geo.get('fault') or ('ok' if health['localization'] else 'waiting_for_rtk'),
                'localization_grade': grade, 'gnss': geo.get('fix'),
                'position_source': geo.get('position_source'), 'heading': geo.get('heading'),
                'pose_source': geo.get('pose_source'), 'pose': pose,
                'task': self.tasks.snapshot() if include_task else None}

    def set_active(self, active):
        with self.outdoor.state_lock:
            self.service_active = active
            if not active:
                self.stopping = True
                with self.tasks.lock:
                    for task in list(self.tasks.tasks.values()):
                        if task.state not in TERMINAL:
                            self.tasks.cancel(task.task_id)

    def on_goal(self, request):
        p = request.pose.pose.position
        state = self.tasks.snapshot()
        if (state and state['state'] == 'accepted' and state['goal']['x'] == p.x
                and state['goal']['y'] == p.y and self.status(False)['ready']):
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def on_accepted(self, handle):
        if bytes(handle.goal_id.uuid).hex() not in self.tasks.tasks:
            handle.execute()
            return
        super().on_accepted(handle)

    async def execute(self, handle):
        if bytes(handle.goal_id.uuid).hex() not in self.tasks.tasks:
            handle.abort()
            return NavigateToPose.Result()
        return await super().execute(handle)

    def arrival_heading_ok(self, position, goal):
        return 'yaw' not in goal or abs(wrap(position['yaw'] - goal['yaw'])) <= 0.20

    async def prepare_route(self, route, task_id):
        if not await self.confirm_stop():
            raise RuntimeError('Cannot confirm stationary robot before route execution')
        tolerance = self.tasks.snapshot(task_id)['arrival_tolerance_m']
        if tolerance is None:
            raise RuntimeError('Task has no GNSS acceptance policy')
        if not self.controller_parameters.service_is_ready():
            raise RuntimeError('Controller parameter service is unavailable')
        # The controller's goal checker also ends every rolling-window leg, so a
        # meter-level arrival tolerance here would let it "succeed" at the first
        # horizon point and stall the task. Task-level arrival keeps the full
        # configured tolerance; the controller stays tight.
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('goal_checker.xy_goal_tolerance',
                                        value=float(min(tolerance, 0.5))).to_parameter_msg()]
        result = await self.result(self.controller_parameters.call_async(request), task_id)
        if not result.result.successful:
            raise RuntimeError('Controller rejected the task arrival tolerance')

    async def guided_path(self, guidance, window, position, task_id):
        path = await super().guided_path(guidance, window, position, task_id)
        goal = self.tasks.snapshot(task_id)['goal']
        if path and path.poses and window[-1] == len(guidance.points) - 1 and 'yaw' in goal:
            q = path.poses[-1].pose.orientation
            q.z, q.w = math.sin(goal['yaw'] / 2), math.cos(goal['yaw'] / 2)
        return path


def main():
    rclpy.init()
    node = LynxNavigation()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

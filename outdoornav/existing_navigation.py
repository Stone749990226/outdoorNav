"""Outdoor tasks delegate motion to the existing Robonix Navigation Service."""

import math
import re
import threading
import time

from .api import ApiError
from .georeference import MapAlignment
from .places import Places
from .routing import validate_point
from .tasks import TaskStore, TERMINAL


class ExistingNavigation:
    def __init__(self, bundle, places, config, client, journal=None, clock=time.monotonic):
        self.bundle, self.config, self.client, self.clock = bundle, config, client, clock
        self.places = Places(places, bundle)
        self.tasks = TaskStore(bundle['scene_version'], bundle['route_version'], journal)
        self.alignment = MapAlignment(bundle['origin_wgs84'], config['antenna_xy_m'])
        # Outdoor mode maps GNSS ENU straight onto the scene map: the odometry
        # node already publishes map/odom in that frame, so no baseline fit is
        # needed and a float solution is usable for a demo-level run.
        if config.get('frame_alignment') == 'enu_identity':
            self.alignment.transform = (0.0, 0.0, 0.0)
        self.lock = threading.RLock()
        self.obstacles = None
        self.service_active = False
        self.map_pose = None
        self.map_pose_at = self.odom_at = self.scan_at = -math.inf
        self.last_fix = None
        self.last_fix_at = -math.inf
        self.stopped_since = None
        self.navigation = {'active': False, 'registration_id': None}
        self.worker_task = None
        self.owned_run = None
        self.run_generation = None
        self.pending_terminal = None
        self.cancel_sent = False
        self.stop_deadline = None
        self.fault = None

    def observe(self, pose, speed, scan_ok, generation, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            self.alignment.generation_changed(generation)
            self.map_pose = pose
            if pose and 0 <= pose['age_s'] < 1:
                self.map_pose_at = now - pose['age_s']
            if speed is not None and all(math.isfinite(v) for v in speed):
                self.odom_at = now
                if abs(speed[0]) < 0.03 and abs(speed[1]) < 0.05 and pose and pose['age_s'] < 1:
                    if self.stopped_since is None:
                        self.stopped_since = now
                else:
                    self.stopped_since = None
            else:
                self.stopped_since = None
            if scan_ok:
                self.scan_at = now

    def on_fix(self, fix, received):
        with self.lock:
            if not 0 <= self.clock() - received < 1:
                return
            self.last_fix = fix
            self.last_fix_at = received
            if self.alignment.transform is None:
                self.alignment.update(fix, self.map_pose, received)

    def location_ready(self, now):
        if self.alignment.fault:
            return False
        if self.config.get('frame_alignment') == 'enu_identity':
            fix = self.last_fix
            if not fix or now - self.last_fix_at > self.config.get('gnss_max_age_s', 10):
                return False
            if not fix.get('differential'):
                return False
            hacc = fix.get('hacc_m')
            return bool(hacc is not None and math.isfinite(hacc)
                        and hacc <= self.config.get('gnss_max_hacc_m', 2.0))
        return self.alignment.ready(now)

    def stopped(self, now):
        return (self.stopped_since is not None and now - self.stopped_since >= 1
                and now - self.odom_at < 1 and now - self.map_pose_at < 1)

    def status(self):
        with self.lock:
            now = self.clock()
            health = {
                'localization': self.location_ready(now),
                'calibration': self.alignment.transform is not None and not self.alignment.fault,
                'antenna_extrinsics': self.config.get('antenna_offset_confirmed') is True,
                'tf': now - self.map_pose_at < 1,
                'odometry': now - self.odom_at < 1,
                'lidar': now - self.scan_at < 1,
                'actions': self.navigation['active'],
                'motion_enabled': self.config.get('motion_enabled') is True,
                'service_active': self.service_active,
                'controller_stopped': self.fault is None,
            }
            pose = self.alignment.to_enu(self.map_pose) if self.map_pose else None
            if pose:
                pose = {**pose, 'frame_id': 'map', 'age_s': max(pose['age_s'], now-self.map_pose_at)}
            return {'schema_version': 1, 'mode': 'lynx', 'run_id': 'lynx-existing-navigation',
                    'scene_version': self.bundle['scene_version'], 'route_version': self.bundle['route_version'],
                    'ready': all(health.values()), 'health': health,
                    'localization': self.fault or self.alignment.fault or ('ok' if all(health.values()) else 'not_ready'),
                    'pose': pose, 'pose_source': 'existing_map_georeferenced',
                    'gnss': self.last_fix or self.alignment.fix, 'task': self.tasks.snapshot(),
                    'alignment': {'samples': len(self.alignment.samples), 'residual_m': self.alignment.residual_m,
                                  'minimum_baseline_m': self.alignment.min_baseline,
                                  'valid': health['calibration']},
                    'navigation_provider': self.config['navigation_provider_id']}

    def map_snapshot(self):
        return self.bundle

    def set_active(self, active):
        with self.lock:
            self.service_active = active
            if not active:
                for task in list(self.tasks.tasks.values()):
                    if task.state not in TERMINAL:
                        self.tasks.cancel(task.task_id)

    def target(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('Goal must be an object')
        for key in ('scene_version', 'route_version'):
            if payload.get(key) != self.bundle[key]:
                raise ApiError(409, 'Map version mismatch')
        if payload.get('frame_id') != 'map':
            raise ValueError('Goal must use the frontend ENU map frame')
        if payload.get('blocked_road_ids'):
            raise ValueError('Road closures are not used by free-goal navigation')
        if payload.get('place_id'):
            place = self.places.resolve(payload['place_id'])
            if not place['enabled']:
                raise ApiError(409, 'Destination awaits field confirmation')
            return {k: place[k] for k in ('x', 'y', 'yaw')}
        point = validate_point(payload.get('goal'))
        yaw = payload['goal'].get('yaw')
        if yaw is None:
            return point
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
            raise ValueError('Goal yaw must be finite')
        return {**point, 'yaw': float(yaw)}

    def preview(self, payload):
        with self.lock:
            goal = self.target(payload)
            # This is a target check, NOT a collision-free route prediction.
            return {'schema_version': 1, 'scene_version': self.bundle['scene_version'],
                    'route_version': self.bundle['route_version'], 'goal': goal,
                    'start': self.status()['pose'], 'reference_path': [], 'segment_roads': [],
                    'road_ids': [], 'length_m': None, 'planning': 'deferred_to_existing_navigation'}

    def submit(self, payload):
        with self.lock:
            goal = self.target(payload)
            request_id = payload.get('request_id')
            if request_id is not None:
                if not isinstance(request_id, str) or not re.fullmatch('[a-f0-9]{32}', request_id):
                    raise ValueError('request_id must be 32 lowercase hex characters')
                if request_id in self.tasks.tasks:
                    old = self.tasks.snapshot(request_id)
                    if old['goal'] != goal:
                        raise ApiError(409, 'request_id was used for another goal')
                    return old
            if not self.status()['ready']:
                raise ApiError(503, 'RTK/map alignment, existing navigation or motion enable is not ready')
            return self.tasks.submit(goal, task_id=request_id)

    def tick(self):
        """One serial owner: cancel and confirm stop before dispatching a replacement."""
        try:
            navigation = self.client.inspect()
        except Exception:
            navigation = {'active': False, 'registration_id': None}
        with self.lock:
            self.navigation = navigation
            now = self.clock()
            if self.worker_task:
                task = self.tasks.snapshot(self.worker_task)
                lost = not self.status()['ready']
                if self.run_generation != navigation['registration_id']:
                    self.fault = 'Navigation provider restarted with an owned goal; inspect robot'
                    lost = True
                if (task['cancel_requested'] or lost) and not self.pending_terminal:
                    self.pending_terminal = ('failed', 'Readiness lost during navigation') if lost else ('canceled', 'Canceled')
                    self.stop_deadline = now + 15
                if self.pending_terminal and not self.cancel_sent:
                    try:
                        self.client.cancel(self.owned_run)
                        self.cancel_sent = True
                    except Exception:
                        pass
                try:
                    result = self.client.status(self.owned_run)
                except Exception:
                    result = {'known': False, 'state': 'UNKNOWN', 'detail': 'Navigation status unavailable'}
                if result['known'] and result['state'] in ('SUCCEEDED', 'FAILED', 'CANCELED'):
                    if not self.pending_terminal:
                        self.pending_terminal = ({'SUCCEEDED': 'succeeded', 'FAILED': 'failed', 'CANCELED': 'canceled'}[result['state']], result['detail'])
                        self.stop_deadline = now + 15
                    if self.stopped(now):
                        state, reason = self.pending_terminal
                        if state == 'succeeded':
                            pose = self.status()['pose']
                            if not pose or math.hypot(pose['x']-task['goal']['x'], pose['y']-task['goal']['y']) > self.config.get('arrival_tolerance_m', 0.5):
                                state, reason = 'failed', 'Provider succeeded but measured goal distance exceeds tolerance'
                        self.tasks.update(self.worker_task, state=state, reason=reason)
                        self.worker_task = self.owned_run = self.pending_terminal = None
                        self.cancel_sent = False
                        self.stop_deadline = None
                if self.worker_task and self.stop_deadline and now >= self.stop_deadline:
                    self.fault = 'Stop/status not confirmed; operator inspection required'
                    self.tasks.update(self.worker_task, state='failed', reason=self.fault)
                    # Retain ownership and keep attempting cancellation; never dispatch another goal.
                    self.stop_deadline = None
                    self.cancel_sent = False
                return
            if self.fault:
                return
            with self.tasks.lock:
                pending = [t.task_id for t in self.tasks.tasks.values() if t.state not in TERMINAL]
            for task_id in pending:
                task = self.tasks.snapshot(task_id)
                if task['cancel_requested']:
                    self.tasks.update(task_id, state='canceled', reason='Canceled before dispatch')
                    continue
                if not self.status()['ready'] or not self.stopped(now):
                    continue
                self.tasks.update(task_id, state='planning')
                try:
                    goal = self.alignment.to_map(task['goal'])
                    goal.setdefault('yaw', self.map_pose['yaw'])
                    response = self.client.navigate(goal)
                except Exception:
                    # An RPC timeout may have accepted the goal: never automatically retry it.
                    self.fault = 'Navigation dispatch outcome unknown; inspect robot before retrying'
                    self.tasks.update(task_id, state='failed', reason=self.fault)
                    return
                if not response['accepted']:
                    self.tasks.update(task_id, state='failed', reason=response['detail'])
                elif not response['run_id']:
                    self.fault = 'Navigation accepted without an addressable run_id'
                    self.tasks.update(task_id, state='failed', reason=self.fault)
                else:
                    self.worker_task, self.owned_run = task_id, response['run_id']
                    self.run_generation = navigation['registration_id']
                    self.tasks.update(task_id, state='navigating')
                return

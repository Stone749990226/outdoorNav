"""Shared road-goal validation and task submission for simulation and hardware."""

import math
import re
import threading

from .api import ApiError
from .routing import plan_route, snap, validate_point
from .config import DEFAULTS

# An off-road robot start is accepted up to this distance from the surveyed
# network: NavFn legs are planned from the robot's true position, so the first
# leg brings it straight back to the nearest road. Farther starts are refused
# to avoid long unobserved dashes across unvalidated terrain.
START_RESCUE_DISTANCE_M = 20


class OutdoorService:
    def __init__(self, bundle, places, tasks, status, config=None):
        self.bundle, self.places, self.tasks = bundle, places, tasks
        self.status = status
        self.config = {**DEFAULTS, **(config or {})}
        self.state_lock = threading.RLock()
        self.preference = None
        if self.config.get('road_preference_enabled'):
            from .road_preference import RoadPreference
            self.preference = RoadPreference(bundle, self.config['road_offroad_cost'],
                self.config['road_preference_resolution_m'], self.config.get('road_keepout_polygons', []),
                self.config.get('footprint_radius_m', .6))

    def planning_policy(self):
        return self.preference.policy() if self.preference else {'mode': 'centerline'}

    def planning_cost(self, payload):
        value = payload.get('road_offroad_cost')
        if value is None:
            return self.preference.offroad_cost if self.preference else None
        if not self.preference or type(value) is not int or not 1 <= value <= 99:
            raise ValueError('road_offroad_cost requires weighted mode and an integer 1..99')
        return value

    def plan_from(self, start, goal, blocked=(), offroad_cost=None):
        if self.preference:
            planner = self.preference
            if offroad_cost is not None and offroad_cost != planner.offroad_cost:
                from .road_preference import RoadPreference
                planner = RoadPreference(self.bundle, offroad_cost, planner.resolution,
                                         planner.keepouts, planner.robot_radius)
            return planner.plan(start, goal, blocked)
        return plan_route(self.bundle['network'], start, goal, blocked)

    def preview(self, payload):
        _, _, route = self.validated(payload)
        return {'schema_version': 1, 'scene_version': self.bundle['scene_version'],
                'route_version': self.bundle['route_version'], **route}

    def target(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('Request must be an object')
        if set(payload) - {'scene_version', 'route_version', 'frame_id', 'place_id',
                           'goal', 'blocked_road_ids', 'request_id', 'road_offroad_cost'}:
            raise ValueError('Unknown navigation request fields')
        self.planning_cost(payload)
        for key in ('scene_version', 'route_version'):
            if payload.get(key) != self.bundle[key]:
                raise ApiError(409, 'Map version mismatch')
        if payload.get('frame_id') != 'map':
            raise ValueError('Goal must use map coordinates')
        if payload.get('place_id'):
            if self.places is None:
                raise ValueError('Named places are unavailable')
            place = self.places.resolve(payload['place_id'])
            keys = ('x', 'y', 'yaw') if self.config.get('respect_place_heading', True) else ('x', 'y')
            goal = {k: place[k] for k in keys}
            goal.update(place_id=place['id'], enabled=place['enabled'])
        else:
            raw = payload.get('goal')
            point = validate_point(raw)
            if self.preference:
                self.preference.check_point(point)
                goal = {**point, 'enabled': True}
            else:
                projected = snap(self.bundle['network'], point)
                if projected['distance_m'] > 2:
                    raise ValueError('Select a goal within 2 m of the road network')
                goal = {**projected['point'], 'enabled': True}
            if 'yaw' in raw:
                angle = raw['yaw']
                if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle):
                    raise ValueError('Goal yaw must be finite')
                goal['yaw'] = float(angle)
        blocked = payload.get('blocked_road_ids', [])
        roads = {r['id'] for r in self.bundle['network']['roads']}
        if not isinstance(blocked, list) or any(not isinstance(r, str) or r not in roads for r in blocked):
            raise ValueError('blocked_road_ids must contain known road IDs')
        return goal, sorted(set(blocked))

    def validated(self, payload):
        goal, blocked = self.target(payload)
        position = self.status(False)['pose']
        if position is None or not 0 <= position.get('age_s', math.inf) < self.config['sensor_timeout_s']:
            raise ApiError(503, 'No fresh robot position for planning')
        if not self.preference and snap(self.bundle['network'], position)['distance_m'] > START_RESCUE_DISTANCE_M:
            raise ValueError(
                f'Robot is more than {START_RESCUE_DISTANCE_M:.0f} m from the surveyed road network'
            )
        return goal, blocked, self.plan_from(position, goal, blocked, self.planning_cost(payload))

    def submit(self, payload):
        with self.state_lock:
            goal, blocked = self.target(payload)
            offroad_cost = self.planning_cost(payload)
            request_id = payload.get('request_id')
            if request_id is not None:
                if not isinstance(request_id, str) or not re.fullmatch(r'[a-f0-9]{32}', request_id):
                    raise ValueError('request_id must be 32 lowercase hexadecimal characters')
                if request_id in self.tasks.tasks:
                    old = self.tasks.snapshot(request_id)
                    if (old['goal'] != goal or old['blocked_road_ids'] != blocked
                            or old.get('road_offroad_cost') != offroad_cost):
                        raise ApiError(409, 'request_id was used for another goal')
                    return old
            if not goal['enabled']:
                raise ApiError(409, 'Destination awaits field confirmation')
            status = self.status(False)
            if not status['ready']:
                raise ApiError(503, 'Localization, calibration, sensors or motion enable is not ready')
            goal, blocked, route = self.validated(payload)
            result = self.tasks.submit(goal, blocked, request_id)
            grade = status.get('localization_grade')
            tolerance = self.config[grade + '_arrival_m'] if grade in ('fixed', 'float') else None
            self.tasks.update(result['task_id'], reference_path=route['reference_path'],
                              localization_grade=grade, arrival_tolerance_m=tolerance,
                              road_offroad_cost=offroad_cost, state='accepted')
            return self.tasks.snapshot(result['task_id'])

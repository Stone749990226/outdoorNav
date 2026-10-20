"""Shared semantic routing and task submission, independent of ROS transports."""

import math
import re
import threading

from .api import ApiError
from .routing import plan_route, snap, validate_point


class OutdoorService:
    def __init__(self, bundle, places, tasks, status):
        self.bundle, self.places, self.tasks = bundle, places, tasks
        self.status = status
        self.state_lock = threading.RLock()

    def preview(self, payload):
        _, _, route = self.validated(payload)
        return {
            "schema_version": 1,
            "scene_version": self.bundle["scene_version"],
            "route_version": self.bundle["route_version"],
            **route,
        }

    def target(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("Request must be an object")
        for key in ("scene_version", "route_version"):
            if payload.get(key) != self.bundle[key]:
                raise ApiError(409, "Map version mismatch")
        if payload.get("frame_id") != "map":
            raise ValueError("Goal must use map coordinates")
        if payload.get("place_id"):
            place = self.places.resolve(payload["place_id"])
        else:
            point = validate_point(payload.get("goal"))
            choices = [
                p
                for p in self.places.items
                if math.dist((p["x"], p["y"]), (point["x"], point["y"])) <= 0.25
            ]
            if len(choices) != 1:
                raise ValueError("Select a named, surveyed stopping place")
            place = choices[0]
        goal = {key: place[key] for key in ("x", "y", "yaw")}
        goal["place_id"] = place["id"]
        goal["enabled"] = place["enabled"]
        blocked = payload.get("blocked_road_ids", [])
        if not isinstance(blocked, list) or not all(
            isinstance(v, str) for v in blocked
        ):
            raise ValueError("blocked_road_ids must be a list of road IDs")
        return goal, blocked

    def validated(self, payload):
        goal, blocked = self.target(payload)
        position = self.status(False)["pose"]
        if position is None or position["age_s"] is None or position["age_s"] > 1:
            raise ApiError(503, "No fresh robot position for planning")
        if snap(self.bundle["network"], position)["distance_m"] > 2:
            raise ValueError("Robot is outside the surveyed road network")
        return (
            goal,
            blocked,
            plan_route(self.bundle["network"], position, goal, blocked),
        )

    def submit(self, payload):
        with self.state_lock:
            goal, blocked = self.target(payload)
            # A caller-supplied request ID gives retries the same task, never a new move.
            request_id = payload.get("request_id")
            if request_id:
                if not isinstance(request_id, str) or not re.fullmatch(
                    r"[a-f0-9]{32}", request_id
                ):
                    raise ValueError("request_id must be 32 lowercase hex characters")
                if request_id in self.tasks.tasks:
                    old = self.tasks.snapshot(request_id)
                    if old["goal"] != goal or old["blocked_road_ids"] != blocked:
                        raise ApiError(409, "request_id was used for another goal")
                    return old
            if not goal["enabled"]:
                raise ApiError(409, "Destination awaits field confirmation")
            if not self.status(False)["ready"]:
                raise ApiError(
                    503, "RTK, calibration, sensors or motion enable is not ready"
                )
            goal, blocked, route = self.validated(payload)
            result = self.tasks.submit(goal, blocked, request_id)
            self.tasks.update(result["task_id"], reference_path=route["reference_path"])
            return self.tasks.snapshot(result["task_id"])

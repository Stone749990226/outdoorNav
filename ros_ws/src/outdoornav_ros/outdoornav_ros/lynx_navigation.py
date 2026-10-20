"""Hardware profile for the existing ordered NavFn/FollowPath task owner."""

import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose

from outdoornav.api import serve
from outdoornav.bundle import route_mask, is_free
from outdoornav.places import Places
from outdoornav.outdoor import OutdoorService
from outdoornav.tasks import TaskStore
from outdoornav.localization import wrap
from .maps import LATCHED
from .navigation import Navigation


class LynxNavigation(Navigation):
    def __init__(self):
        super().__init__(hardware=True)
        for key in ("robot_config", "places"):
            self.declare_parameter(key, "")
        self.config = json.loads(
            Path(self.get_parameter("robot_config").value).read_text()
        )
        self.places = Places(self.get_parameter("places").value, self.bundle)
        self.tasks = TaskStore(
            self.bundle["scene_version"],
            self.bundle["route_version"],
            Path(self.get_parameter("run_dir").value) / "tasks.json",
        )
        self.outdoor = OutdoorService(self.bundle, self.places, self.tasks, self.status)
        self.geo, self.geo_at = {}, -math.inf
        self.service_active = True
        self.corridor = None
        self.mask_pub = self.create_publisher(
            OccupancyGrid, "/outdoor/corridor", LATCHED
        )
        self.info_pub = self.create_publisher(
            CostmapFilterInfo, "/outdoor/corridor_info", LATCHED
        )
        info = CostmapFilterInfo()
        info.header.frame_id = "map"
        info.filter_mask_topic, info.type, info.base, info.multiplier = (
            "/outdoor/corridor",
            0,
            0.0,
            1.0,
        )
        self.info_pub.publish(info)
        grid = self.bundle["grid"]
        self.mask_pub.publish(
            self.maps.message(
                np.full((grid["height"], grid["width"]), 100, dtype=np.int8)
            )
        )
        self.create_subscription(String, "/outdoor/localization", self.on_geo, 10)
        self.create_subscription(Odometry, "/outdoor/odom", self.on_odom, 10)
        self.create_subscription(
            LaserScan, "/outdoor/scan", self.on_scan, qos_profile_sensor_data
        )
        self.http = serve(self)

    def on_geo(self, msg):
        try:
            self.geo = json.loads(msg.data)
            self.geo_at = time.monotonic()
        except (ValueError, TypeError):
            self.geo = {}

    def on_scan(self, msg):
        age = (
            self.get_clock().now().nanoseconds / 1e9
            - msg.header.stamp.sec
            - msg.header.stamp.nanosec / 1e9
        )
        # No-return rays cannot certify blind sectors as observed free space.
        if -0.1 <= age < 1 and any(
            math.isfinite(r) and msg.range_min < r < msg.range_max for r in msg.ranges
        ):
            self.mark("lidar")

    def status(self, include_task=True):
        now = time.monotonic()
        config = getattr(self, "config", {})
        geo = getattr(self, "geo", {})
        health = {
            key: now - self.stats.get(key, -math.inf) < limit
            for key, limit in (("lidar", 1), ("global_map", 3), ("local_map", 1))
        }
        health.update(
            {
                "localization": bool(geo.get("ready"))
                and now - getattr(self, "geo_at", -math.inf) < 1,
                "calibration": bool(
                    config.get("calibrated") and config.get("cloud_frame_confirmed")
                ),
                "motion_enabled": bool(config.get("motion_enabled")),
                "service_active": getattr(self, "service_active", False),
                "controller_stopped": self.stop_failure is None,
                "odometry": now - self.velocity_at < 1,
                "actions": self.path_client.server_is_ready()
                and self.follow_client.server_is_ready(),
            }
        )
        for name in self.lifecycle_clients:
            value, stamp = self.lifecycle.get(name, (False, 0))
            health[name] = value and now - stamp < 5
        pose = self.pose()
        health["tf"] = pose is not None and pose["age_s"] < 1
        if (pose is None or pose["age_s"] >= 1) and geo.get("pose"):
            pose = {
                **geo["pose"],
                "frame_id": "map",
                "stamp_s": 0,
                "age_s": max(
                    geo.get("fix_age_s") or 0, now - getattr(self, "geo_at", -math.inf)
                ),
            }
        return {
            "schema_version": 1,
            "mode": "lynx",
            "run_id": "lynx-outdoor",
            "scene_version": self.bundle["scene_version"],
            "route_version": self.bundle["route_version"],
            "ready": all(health.values()),
            "health": health,
            "localization": "ok"
            if health["localization"]
            else geo.get("fault") or "waiting_for_rtk",
            "gnss": geo.get("fix"),
            "pose_source": geo.get("pose_source"),
            "pose": pose,
            "task": self.tasks.snapshot() if include_task else None,
        }

    def validated(self, payload):
        return self.outdoor.validated(payload)

    def set_active(self, active):
        with self.outdoor.state_lock:
            self.service_active = active
            if not active:
                self.stopping = True
                with self.tasks.lock:
                    for task in self.tasks.tasks.values():
                        if task.state not in ("succeeded", "failed", "canceled"):
                            self.tasks.cancel(task.task_id)

    def submit(self, payload):
        return self.outdoor.submit(payload)

    def on_goal(self, request):
        # Only dispatch() may use this action: external goals must pass the shared API.
        p = request.pose.pose.position
        state = self.tasks.snapshot()
        if (
            state
            and state["state"] == "accepted"
            and state["goal"]["x"] == p.x
            and state["goal"]["y"] == p.y
            and self.status(False)["ready"]
        ):
            return GoalResponse.ACCEPT
        return GoalResponse.REJECT

    def on_accepted(self, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        if task_id not in self.tasks.tasks:
            handle.execute()
            return
        super().on_accepted(handle)

    async def execute(self, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        if task_id not in self.tasks.tasks:
            handle.abort()
            return NavigateToPose.Result()
        return await super().execute(handle)

    def arrival_heading_ok(self, position, goal):
        return abs(wrap(position["yaw"] - goal["yaw"])) <= 0.20

    async def prepare_route(self, route, task_id):
        if not await self.confirm_stop():
            raise RuntimeError("Cannot confirm stationary robot before route execution")
        widths = self.config["road_widths_m"]
        if any(road not in widths for road in route["road_ids"]):
            raise ValueError("Survey the selected road widths before driving")
        radius = self.config["footprint_radius_m"]
        adjusted = {r: float(w) - 2 * radius for r, w in widths.items()}
        if any(adjusted[r] <= 0.2 for r in route["road_ids"]):
            raise ValueError("Road is too narrow for the configured footprint")
        grid = self.bundle["grid"]
        self.corridor = route_mask(
            {**self.bundle, "road_widths_m": adjusted},
            np.ones((grid["height"], grid["width"]), dtype=bool),
            route,
            self.pose(),
        )
        self.mask_pub.publish(
            self.maps.message(np.where(self.corridor < 100, 0, 100).astype(np.int8))
        )
        # Let both costmaps receive the new mask before requesting a path. The
        # resulting path is independently checked below, including every segment.
        await self.pause(1.1)

    async def guided_path(self, guidance, window, position, task_id):
        path = await super().guided_path(guidance, window, position, task_id)
        if path is None:
            return None
        points = [p.pose.position for p in path.poses]
        allowed = self.corridor < 100
        for a, b in zip(points, points[1:]):
            count = max(1, math.ceil(math.hypot(b.x - a.x, b.y - a.y) / 0.025))
            for i in range(count + 1):
                if not is_free(
                    allowed,
                    self.bundle["grid"],
                    {
                        "x": a.x + (b.x - a.x) * i / count,
                        "y": a.y + (b.y - a.y) * i / count,
                    },
                ):
                    return None
        if window[-1] == len(guidance.points) - 1:
            angle = self.tasks.snapshot(task_id)["goal"]["yaw"]
            q = path.poses[-1].pose.orientation
            q.z, q.w = math.sin(angle / 2), math.cos(angle / 2)
        return path


def main():
    rclpy.init()
    node = LynxNavigation()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

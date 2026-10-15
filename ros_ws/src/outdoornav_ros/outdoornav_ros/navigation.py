"""Single task owner: standard NavigateToPose, constrained Nav2 and goal HTTP API."""

import math
import os
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from unique_identifier_msgs.msg import UUID

from outdoornav.api import ApiError, serve
from outdoornav.bundle import is_free, load_bundle, masks, route_mask
from outdoornav.routing import plan_route, validate_point
from outdoornav.tasks import TaskStore, TERMINAL, ProgressBudget
from .maps import LATCHED, Maps
from .obstacles import Obstacles
from .motion import MotionPrediction


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def length(points):
    return sum(
        math.dist((a["x"], a["y"]), (b["x"], b["y"]))
        for a, b in zip(points, points[1:])
    )


def lateral_difference(a, b):
    """Maximum new-path distance to the old polyline, rather than old vertices."""
    if not b:
        return math.inf
    old = np.array([(p["x"], p["y"]) for p in b])
    starts, vectors = old[:-1], old[1:] - old[:-1]
    denom = np.maximum((vectors * vectors).sum(axis=1), 1e-12)
    distances = []
    for point in a[:: max(1, len(a) // 100)]:
        delta = np.array([point["x"], point["y"]]) - starts
        t = np.clip((delta * vectors).sum(axis=1) / denom, 0, 1)
        distances.append(
            np.sqrt(((delta - t[:, None] * vectors) ** 2).sum(axis=1)).min()
        )
    return max(distances, default=0)


class Navigation(Node):
    def __init__(self):
        super().__init__("navigation")
        self.declare_parameter("bundle", "")
        self.bundle_path = self.get_parameter("bundle").value
        self.bundle = load_bundle(self.bundle_path)
        self.free = masks(self.bundle_path)
        self.tasks = TaskStore(
            self.bundle["scene_version"], self.bundle["route_version"]
        )
        self.group = ReentrantCallbackGroup()
        self.declare_parameter("run_dir", "")
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.maps = Maps(self, self.bundle, self.free)
        self.stats, self.lifecycle, self.lifecycle_pending = {}, {}, set()
        self.clock_value, self.clock_changed = None, 0.0
        self.localization = "no_fix"
        self.global_map, self.local_map = None, None
        self.raw_command, self.raw_received = Twist(), 0.0
        self.velocity, self.velocity_at = (math.inf, math.inf), 0.0
        self.owner, self.stopping = None, True
        self.stop_failure = None
        self.handles, self.requests, self.cancel_sent = {}, {}, set()
        self.state_lock = threading.RLock()
        self.last_sim_time = 0.0
        self.path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=self.group
        )
        self.follow_client = ActionClient(
            self, FollowPath, "follow_path", callback_group=self.group
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.group
        )
        self.cancel_client = self.create_client(
            CancelGoal,
            "navigate_to_pose/_action/cancel_goal",
            callback_group=self.group,
        )
        self.lifecycle_clients = {
            name: self.create_client(
                GetState, f"/{name}/get_state", callback_group=self.group
            )
            for name in ("planner_server", "controller_server")
        }
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(
            Path, "/navigation/execution_path", LATCHED
        )
        from rclpy.qos import qos_profile_sensor_data

        for message_type, topic, key in [
            (Image, "/rtk/camera/depth_image", "depth"),
            (PointCloud2, "/rtk/camera/points", "points"),
            (LaserScan, "/rtk/scan", "lidar"),
        ]:
            self.create_subscription(
                message_type,
                topic,
                lambda msg, key=key: self.mark(key),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            ClockMessage, "/clock", self.on_clock, qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/rtk/localization_status", self.on_localization, LATCHED
        )
        # Status is published on transitions; freshness comes from localization output.
        self.create_subscription(
            Odometry, "/odometry/global", lambda msg: self.mark("localization"), 10
        )
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_global, LATCHED
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_local, LATCHED
        )
        self.create_subscription(Odometry, "/odometry/local", self.on_odom, 10)
        self.create_subscription(
            Twist, "/navigation/controller_command", self.on_command, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.on_rviz_goal, 1)
        self.server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            handle_accepted_callback=self.on_accepted,
            callback_group=self.group,
        )
        self.create_timer(
            0.05, self.safety_tick, clock=self.steady, callback_group=self.group
        )
        self.create_timer(
            0.1, self.dispatch, clock=self.steady, callback_group=self.group
        )
        self.create_timer(
            1.0, self.poll_lifecycle, clock=self.steady, callback_group=self.group
        )
        self.obstacles = Obstacles(self, self.get_parameter("run_dir").value)
        self.prediction = MotionPrediction(self)
        self.http = serve(self)

    def mark(self, key):
        with self.state_lock:
            self.stats[key] = time.monotonic()

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec / 1e9
        with self.state_lock:
            if value != self.clock_value:
                self.clock_changed = time.monotonic()
            if self.clock_value is not None and value < self.clock_value:
                if self.owner:
                    self.tasks.update(
                        self.owner,
                        state="canceling",
                        cancel_requested=True,
                        reason="Simulation clock moved backward",
                    )
                self.stats.clear()
                self.stopping = True
            self.clock_value = value

    def on_localization(self, msg):
        self.localization = msg.data

    def on_global(self, msg):
        self.global_map = msg
        self.mark("global_map")

    def on_local(self, msg):
        self.local_map = msg
        self.mark("local_map")

    def on_odom(self, msg):
        t = msg.twist.twist
        self.velocity = (math.hypot(t.linear.x, t.linear.y), abs(t.angular.z))
        self.velocity_at = time.monotonic()

    def on_command(self, msg):
        self.raw_command, self.raw_received = msg, time.monotonic()

    def poll_lifecycle(self):
        for name, client in self.lifecycle_clients.items():
            if name in self.lifecycle_pending or not client.service_is_ready():
                continue
            self.lifecycle_pending.add(name)
            future = client.call_async(GetState.Request())

            def done(future, name=name):
                self.lifecycle_pending.discard(name)
                try:
                    self.lifecycle[name] = (
                        future.result().current_state.id == 3,
                        time.monotonic(),
                    )
                except Exception:
                    self.lifecycle[name] = (False, time.monotonic())

            future.add_done_callback(done)

    def pose(self):
        try:
            transform = self.tf.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
        return {
            "frame_id": "map",
            "x": t.x,
            "y": t.y,
            "yaw": yaw(q),
            "stamp_s": stamp,
            "age_s": max(0, self.get_clock().now().nanoseconds / 1e9 - stamp),
        }

    def status(self, include_task=True):
        now = time.monotonic()
        with self.state_lock:
            health = {
                key: now - self.stats.get(key, -math.inf) < 3.0
                for key in (
                    "depth",
                    "points",
                    "lidar",
                    "localization",
                    "global_map",
                    "local_map",
                )
            }
            health["localization"] &= self.localization == "ok"
            health["obstacle_services"] = all(
                client.service_is_ready()
                for client in (
                    self.obstacles.create,
                    self.obstacles.remove,
                    self.obstacles.move,
                )
            )
            health["clock"] = (
                self.clock_value is not None and now - self.clock_changed < 3.0
            )
            health.update(
                {
                    name: self.lifecycle.get(name, (False, 0))[0]
                    and now - self.lifecycle.get(name, (False, 0))[1] < 5
                    for name in self.lifecycle_clients
                }
            )
        pose = self.pose()
        health["tf"] = pose is not None and pose["age_s"] < 1.0
        health["controller_stopped"] = self.stop_failure is None
        health["actions"] = (
            self.path_client.server_is_ready() and self.follow_client.server_is_ready()
        )
        return {
            "schema_version": 1,
            "run_id": os.environ.get("OUTDOORNAV_RUN_ID"),
            "scene_version": self.bundle["scene_version"],
            "route_version": self.bundle["route_version"],
            "ready": all(health.values()),
            "health": health,
            "localization": self.localization,
            "pose": pose,
            "task": self.tasks.snapshot() if include_task else None,
        }

    def map_snapshot(self):
        return self.bundle

    def validated(self, payload):
        if not isinstance(payload, dict):
            raise ValueError("Request must be a JSON object")
        if set(payload) - {
            "frame_id",
            "scene_version",
            "route_version",
            "goal",
            "blocked_road_ids",
        }:
            raise ValueError("Unknown navigation request fields")
        if payload.get("frame_id") != "map":
            raise ValueError("Goal frame must be map")
        for key in ("scene_version", "route_version"):
            if payload.get(key) != self.bundle[key]:
                raise ApiError(409, key + " does not match the loaded scene")
        goal = validate_point(payload.get("goal"))
        blocked = payload.get("blocked_road_ids", [])
        if not isinstance(blocked, list) or not all(
            isinstance(v, str) for v in blocked
        ):
            raise ValueError("blocked_road_ids must be a list of road IDs")
        if not self.status(False)["ready"]:
            raise ApiError(503, "Localization, sensors or Nav2 are not ready")
        if not is_free(self.free, self.bundle["grid"], goal):
            raise ValueError("Destination is not in traversable scene space")
        route = plan_route(self.bundle["network"], self.pose(), goal, blocked)
        # Projection is a selection tolerance, not permission to jump to another road.
        if (
            math.dist((goal["x"], goal["y"]), (route["goal"]["x"], route["goal"]["y"]))
            > 0.25
        ):
            raise ValueError("Destination must lie on the measured road centerline")
        route_mask(self.bundle, self.free, route, self.pose())
        return goal, blocked, route

    def preview(self, payload):
        _, _, route = self.validated(payload)
        return {
            "schema_version": 1,
            "scene_version": self.bundle["scene_version"],
            "route_version": self.bundle["route_version"],
            **route,
        }

    def submit(self, payload):
        goal, blocked, route = self.validated(payload)
        result = self.tasks.submit(goal, blocked)
        self.tasks.update(result["task_id"], reference_path=route["reference_path"])
        return self.tasks.snapshot(result["task_id"])

    def on_rviz_goal(self, msg):
        try:
            self.submit(
                {
                    "frame_id": msg.header.frame_id,
                    "scene_version": self.bundle["scene_version"],
                    "route_version": self.bundle["route_version"],
                    "goal": {"x": msg.pose.position.x, "y": msg.pose.position.y},
                }
            )
        except ValueError as error:
            self.get_logger().warn(str(error))

    def on_goal(self, request):
        try:
            self.validated(
                {
                    "frame_id": request.pose.header.frame_id,
                    "scene_version": self.bundle["scene_version"],
                    "route_version": self.bundle["route_version"],
                    "goal": {
                        "x": request.pose.pose.position.x,
                        "y": request.pose.pose.position.y,
                    },
                }
            )
            return GoalResponse.ACCEPT
        except ValueError:
            return GoalResponse.REJECT

    def on_accepted(self, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        try:
            self.tasks.snapshot(task_id, include_paths=False)
        except KeyError:
            p = handle.request.pose.pose.position
            self.tasks.submit({"x": p.x, "y": p.y}, task_id=task_id)
        self.handles[task_id] = handle
        self.requests.setdefault(task_id, None)
        handle.execute()

    def on_cancel(self, handle):
        self.tasks.cancel(bytes(handle.goal_id.uuid).hex())
        return CancelResponse.ACCEPT

    def dispatch(self):
        with self.tasks.lock:
            for task_id, task in self.tasks.tasks.items():
                if task.state == "canceling" and task_id not in self.requests:
                    self.tasks.update(
                        task_id, state="canceled", reason="Canceled before dispatch"
                    )
        current = self.tasks.snapshot(include_paths=False)
        if (
            current
            and current["state"] == "accepted"
            and current["task_id"] not in self.requests
        ):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = (
                current["goal"]["x"],
                current["goal"]["y"],
            )
            goal.pose.pose.orientation.w = 1.0
            uid = UUID(uuid=list(bytes.fromhex(current["task_id"])))
            future = self.navigate_client.send_goal_async(goal, goal_uuid=uid)
            self.requests[current["task_id"]] = future

            def sent(future, task_id=current["task_id"]):
                try:
                    if not future.result().accepted:
                        self.tasks.update(
                            task_id,
                            state="failed",
                            reason="Navigation action rejected the goal",
                        )
                except Exception as error:
                    self.tasks.update(task_id, state="failed", reason=str(error))

            future.add_done_callback(sent)
        for task_id, handle in list(self.handles.items()):
            try:
                state = self.tasks.snapshot(task_id, include_paths=False)
            except KeyError:
                self.handles.pop(task_id, None)
                continue
            if (
                state["cancel_requested"]
                and state["state"] not in TERMINAL
                and task_id not in self.cancel_sent
            ):
                request = CancelGoal.Request()
                request.goal_info.goal_id = handle.goal_id
                self.cancel_client.call_async(request)
                self.cancel_sent.add(task_id)

    def safety_tick(self):
        state = (
            self.tasks.snapshot(self.owner, include_paths=False) if self.owner else None
        )
        safe = (
            not self.stopping
            and state
            and not state["cancel_requested"]
            and state["state"] in ("navigating", "replanning")
            and self.status(False)["ready"]
            and time.monotonic() - self.raw_received < 0.5
        )
        self.command_pub.publish(self.raw_command if safe else Twist())

    def path_obstructed(self, points, position):
        if not points or self.global_map is None:
            return False
        message = self.global_map
        meta = message.info
        closest = min(
            range(len(points)),
            key=lambda i: math.hypot(
                points[i]["x"] - position["x"], points[i]["y"] - position["y"]
            ),
        )
        for point in points[closest:]:
            if math.hypot(point["x"] - position["x"], point["y"] - position["y"]) > 8:
                break
            col = int(
                math.floor((point["x"] - meta.origin.position.x) / meta.resolution)
            )
            row = int(
                math.floor((point["y"] - meta.origin.position.y) / meta.resolution)
            )
            if (
                0 <= row < meta.height
                and 0 <= col < meta.width
                and message.data[row * meta.width + col] >= 70
            ):
                return True
        return False

    async def pause(self, seconds=0.05):
        future = Future()

        def finish():
            timer.cancel()
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(
            seconds, finish, clock=self.steady, callback_group=self.group
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def interrupted(self, task_id):
        try:
            state = self.tasks.snapshot(task_id, include_paths=False)
            return state["cancel_requested"] or state["state"] in TERMINAL
        except KeyError:
            return True

    async def result(self, future, task_id, timeout=10):
        began = self.get_clock().now().nanoseconds / 1e9
        while not future.done():
            if self.interrupted(task_id):
                raise InterruptedError("Task canceled or superseded")
            if not self.status(False)["ready"]:
                raise RuntimeError(
                    "Localization, clock, sensor or Nav2 became unavailable"
                )
            if self.get_clock().now().nanoseconds / 1e9 - began > timeout:
                raise RuntimeError("Nav2 action timed out")
            await self.pause()
        if self.interrupted(task_id):
            raise InterruptedError("Task canceled or superseded")
        return future.result()

    async def confirm_stop(self):
        self.stopping = True
        deadline, stable = time.monotonic() + 15, None
        while time.monotonic() < deadline:
            now = self.get_clock().now().nanoseconds / 1e9
            stopped = (
                time.monotonic() - self.velocity_at < 3
                and self.velocity[0] <= 0.01
                and self.velocity[1] <= 0.01
            )
            stable = (now if stable is None else stable) if stopped else None
            if stable is not None and now - stable >= 0.5:
                return True
            await self.pause()
        return False

    async def release_controller(self, requests):
        """Resolve pending acceptances and cancel goals before releasing task ownership."""
        deadline = time.monotonic() + 5
        for pending in requests:
            while not pending.done() and time.monotonic() < deadline:
                await self.pause()
            if not pending.done():
                return False
            try:
                follower = pending.result()
                if not follower.accepted:
                    continue
                result = follower.get_result_async()
                if result.done():
                    continue
                follower.cancel_goal_async()
                while not result.done() and time.monotonic() < deadline:
                    await self.pause()
                if not result.done():
                    return False
            except Exception:
                return False
        return True

    async def execute(self, handle):
        task_id = bytes(handle.goal_id.uuid).hex()
        follow_requests = []
        outcome, reason = "failed", None
        try:
            # A successor cannot activate a mask or command while its predecessor is stopping.
            while True:
                if self.stop_failure:
                    raise RuntimeError(self.stop_failure)
                if self.interrupted(task_id):
                    raise InterruptedError("Task canceled before execution")
                with self.state_lock:
                    if self.owner is None:
                        self.owner = task_id
                        self.stopping = True
                        break
                await self.pause()
            state = self.tasks.snapshot(task_id, include_paths=False)
            if self.interrupted(task_id):
                raise InterruptedError("Task canceled before planning")
            self.tasks.update(task_id, state="planning")
            route = plan_route(
                self.bundle["network"],
                self.pose(),
                state["goal"],
                state["blocked_road_ids"],
            )
            goal_point = route["goal"]
            self.maps.activate(route_mask(self.bundle, self.free, route, self.pose()))
            self.tasks.update(task_id, reference_path=route["reference_path"])
            began = time.monotonic()
            while not (
                self.maps.applied(self.global_map, self.tf)
                and self.maps.applied(self.local_map, self.tf)
            ):
                if self.interrupted(task_id):
                    raise InterruptedError(
                        "Task canceled while activating road constraints"
                    )
                if time.monotonic() - began > 15:
                    raise RuntimeError(
                        "Costmaps did not acknowledge the selected road constraints"
                    )
                await self.pause(0.1)
            current_points, follow_result = [], None
            last_plan, last_send = -math.inf, -math.inf
            budget = ProgressBudget()
            started_sim = self.get_clock().now().nanoseconds / 1e9
            while True:
                if self.interrupted(task_id):
                    raise InterruptedError("Task canceled or superseded")
                if not self.status(False)["ready"]:
                    raise RuntimeError(
                        "Localization, clock, sensor or Nav2 became unavailable"
                    )
                now = self.get_clock().now().nanoseconds / 1e9
                position = self.pose()
                budget.moved(position["x"], position["y"], now)
                remaining = math.dist(
                    (position["x"], position["y"]), (goal_point["x"], goal_point["y"])
                )
                self.tasks.update(task_id, distance_remaining_m=remaining)
                feedback = NavigateToPose.Feedback()
                feedback.current_pose.header.frame_id = "map"
                feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
                (
                    feedback.current_pose.pose.position.x,
                    feedback.current_pose.pose.position.y,
                ) = position["x"], position["y"]
                (
                    feedback.current_pose.pose.orientation.z,
                    feedback.current_pose.pose.orientation.w,
                ) = math.sin(position["yaw"] / 2), math.cos(position["yaw"] / 2)
                feedback.distance_remaining = remaining
                handle.publish_feedback(feedback)
                if (
                    follow_result is not None
                    and follow_result.done()
                    and follow_result.result().status == GoalStatus.STATUS_SUCCEEDED
                    and remaining <= 0.25
                ) or (not current_points and remaining <= 0.25):
                    outcome = "succeeded"
                    break
                if now - started_sim > 1800:
                    raise RuntimeError(
                        "Navigation exceeded the 1800 simulation-second task limit"
                    )
                if follow_result is not None and follow_result.done():
                    current_points, follow_result = [], None
                    if not budget.controller_ended(now):
                        raise RuntimeError(
                            "Controller could not make progress on this road"
                        )
                if now - last_plan >= 1.0:
                    last_plan = now
                    request = ComputePathToPose.Goal()
                    request.goal.header.frame_id = "map"
                    request.goal.header.stamp = self.get_clock().now().to_msg()
                    request.goal.pose.position.x, request.goal.pose.position.y = (
                        goal_point["x"],
                        goal_point["y"],
                    )
                    if len(route["reference_path"]) >= 2:
                        a, b = route["reference_path"][-2:]
                        heading = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
                    else:
                        heading = position["yaw"]
                    request.goal.pose.orientation.z, request.goal.pose.orientation.w = (
                        math.sin(heading / 2),
                        math.cos(heading / 2),
                    )
                    request.use_start = False
                    planner = await self.result(
                        self.path_client.send_goal_async(request), task_id
                    )
                    if not planner.accepted:
                        raise RuntimeError("Planner rejected the constrained goal")
                    try:
                        result = await self.result(planner.get_result_async(), task_id)
                    except Exception:
                        planner.cancel_goal_async()
                        raise
                    path = result.result.path
                    points = [
                        {"x": p.pose.position.x, "y": p.pose.position.y}
                        for p in path.poses
                    ]
                    if result.status != GoalStatus.STATUS_SUCCEEDED or len(points) < 2:
                        if not budget.planned(False):
                            raise RuntimeError(
                                "No traversable path inside the selected road corridor"
                            )
                        await self.pause(0.2)
                        continue
                    budget.planned(True)
                    if current_points and follow_result and not follow_result.done():
                        if not self.path_obstructed(current_points, position) and (
                            now - last_send < 6
                            or (
                                lateral_difference(points, current_points) < 0.45
                                and abs(length(points) - length(current_points)) < 1.2
                            )
                        ):
                            await self.pause(0.1)
                            continue
                    self.tasks.update(
                        task_id, state="replanning" if current_points else "planning"
                    )
                    command = FollowPath.Goal()
                    command.path = path
                    command.controller_id = "FollowPath"
                    command.goal_checker_id = "goal_checker"
                    pending = self.follow_client.send_goal_async(command)
                    follow_requests.append(pending)

                    def accepted(future):
                        # Even a response arriving after cancellation must be stopped.
                        try:
                            follower = future.result()
                            if self.interrupted(task_id) and follower.accepted:
                                follower.cancel_goal_async()
                        except Exception:
                            pass

                    pending.add_done_callback(accepted)
                    follower = await self.result(pending, task_id)
                    if not follower.accepted:
                        raise RuntimeError("Controller rejected the planned path")
                    follow_result = follower.get_result_async()
                    current_points, last_send = points, now
                    revision = (
                        self.tasks.snapshot(task_id, include_paths=False)[
                            "path_revision"
                        ]
                        + 1
                    )
                    self.tasks.update(
                        task_id,
                        state="navigating",
                        path_revision=revision,
                        execution_path=points,
                    )
                    self.stopping = False
                    self.path_pub.publish(path)
                await self.pause(0.1)
        except InterruptedError as error:
            outcome, reason = "canceled", str(error)
            if (
                self.tasks.snapshot(task_id, include_paths=False)["reason"]
                == "Simulation clock moved backward"
            ):
                outcome, reason = "failed", "Simulation clock moved backward"
        except Exception as error:
            reason = str(error)
            self.tasks.update(task_id, state="canceling", reason=reason)
            self.get_logger().error(f"Task {task_id}: {reason}")
        finally:
            if self.owner == task_id:
                self.stopping = True
                released = await self.release_controller(follow_requests)
                stopped = await self.confirm_stop()
                if not released or not stopped:
                    self.stop_failure = "Could not confirm controller release and robot stop; restart required"
                    outcome, reason = "failed", self.stop_failure
                self.tasks.update(
                    task_id,
                    state=outcome,
                    reason=reason,
                    completed_sim_s=self.get_clock().now().nanoseconds / 1e9,
                )
                with self.state_lock:
                    self.owner = None
            else:
                self.tasks.update(
                    task_id,
                    state=outcome,
                    reason=reason,
                    completed_sim_s=self.get_clock().now().nanoseconds / 1e9,
                )
            self.handles.pop(task_id, None)
            self.requests.pop(task_id, None)
            self.cancel_sent.discard(task_id)
            if handle.is_cancel_requested:
                handle.canceled()
            elif outcome == "succeeded":
                handle.succeed()
            else:
                handle.abort()
        return NavigateToPose.Result()

    def destroy_node(self):
        if rclpy.ok():
            self.command_pub.publish(Twist())
        self.http.shutdown()
        self.http.server_close()
        self.obstacles.close()
        self.server.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = Navigation()
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

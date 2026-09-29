"""Local HTTP/ROS2 bridge; every submitted route is simulation-only."""

from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def valid_point(point):
    return isinstance(point, dict) and all(
        isinstance(point.get(key), (int, float))
        and not isinstance(point.get(key), bool)
        and math.isfinite(point[key])
        for key in ("x", "y")
    )


def validate_route(route, calibration):
    if not isinstance(route, dict):
        raise ValueError("Route must be a JSON object")
    if route.get("frame_id") != "map":
        raise ValueError("Route frame must be map")
    if route.get("calibration_version") != calibration["version"] or route.get("calibration_status") != calibration["status"]:
        raise ValueError("Calibration version or status does not match the simulation map")
    if route.get("map_version") != calibration["map_version"] or route.get("executable_on_robot") is not False:
        raise ValueError("This bridge only accepts the current simulation map and non-executable routes")
    path, waypoints = route.get("path"), route.get("waypoints")
    if not isinstance(path, list) or not 2 <= len(path) <= 20000 or not all(valid_point(p) for p in path):
        raise ValueError("Complete path must contain 2-20000 finite map positions")
    if not any(math.dist((path[0]["x"], path[0]["y"]), (p["x"], p["y"])) > 1e-8 for p in path[1:]):
        raise ValueError("Zero-length tasks must complete locally")
    if route.get("origin_wgs84") != calibration["origin_wgs84"]:
        raise ValueError("Route coordinate origin does not match the simulation map")
    if not isinstance(waypoints, list) or not 2 <= len(waypoints) <= len(path) + 100 or not all(valid_point(p) for p in waypoints):
        raise ValueError("Waypoints must be finite map positions")
    if not all(isinstance(p.get("yaw"), (int, float)) and not isinstance(p["yaw"], bool) and math.isfinite(p["yaw"]) for p in waypoints):
        raise ValueError("Every waypoint needs a finite map yaw")
    if math.dist((path[0]["x"], path[0]["y"]), (waypoints[0]["x"], waypoints[0]["y"])) > 0.01:
        raise ValueError("First waypoint does not match path start")
    if math.dist((path[-1]["x"], path[-1]["y"]), (waypoints[-1]["x"], waypoints[-1]["y"])) > 0.01:
        raise ValueError("Last waypoint does not match path goal")


def pose(x, y, yaw):
    result = Pose()
    result.position.x = float(x)
    result.position.y = float(y)
    result.orientation.z = math.sin(yaw / 2)
    result.orientation.w = math.cos(yaw / 2)
    return result


def dense_points(points, spacing=0.25):
    result = [points[0]]
    for start, end in zip(points, points[1:]):
        length = math.dist((start["x"], start["y"]), (end["x"], end["y"]))
        if length < 1e-9:
            continue
        segments = max(1, math.ceil(length / spacing))
        for index in range(1, segments + 1):
            portion = index / segments
            result.append(
                {
                    "x": start["x"] + (end["x"] - start["x"]) * portion,
                    "y": start["y"] + (end["y"] - start["y"]) * portion,
                }
            )
    return result


class Bridge(Node):
    def __init__(self):
        super().__init__("outdoor_nav_bridge")
        self.calibration = json.loads(
            (Path(get_package_share_directory("outdoor_nav_bridge")) / "maps/calibration.json").read_text()
        )
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.path_publisher = self.create_publisher(RosPath, "/outdoor_nav/planned_path", retained)
        self.waypoint_publisher = self.create_publisher(PoseArray, "/outdoor_nav/waypoints", retained)
        self.status_publisher = self.create_publisher(String, "/outdoor_nav/status", retained)
        self.reset_publisher = self.create_publisher(PoseStamped, "/outdoor_nav/sim_reset", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.action = ActionClient(self, FollowPath, "follow_path")
        self.lock = threading.Lock()
        self.state = {
            "phase": "idle",
            "message": "Nav2 simulation ready",
            "pose": None,
            "distance_remaining": None,
            "cmd_vel_count": 0,
            "calibration_version": self.calibration["version"],
            "map_version": self.calibration["map_version"],
            "calibration_status": self.calibration["status"],
        }
        self.pending = None
        self.cancel_requested = False
        self.goal_handle = None
        self.wait_until = 0.0
        self.create_timer(0.1, self.tick)
        self.create_timer(0.5, self.publish_status)

    def snapshot(self):
        with self.lock:
            return deepcopy(self.state)

    def update(self, **changes):
        with self.lock:
            self.state.update(changes)

    def on_odom(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.update(pose={"x": round(p.x, 3), "y": round(p.y, 3), "yaw": round(yaw, 4)})

    def on_command(self, _message):
        with self.lock:
            self.state["cmd_vel_count"] += 1

    def submit(self, route):
        validate_route(route, self.calibration)
        with self.lock:
            if self.state["phase"] in ("queued", "running", "canceling"):
                raise RuntimeError("A simulation route is already active")
            self.pending = route
            self.cancel_requested = False
            self.goal_handle = None
            self.wait_until = 0.0
            self.state.update(phase="queued", message="Waiting for Nav2", distance_remaining=None)

    def cancel(self):
        with self.lock:
            if self.state["phase"] not in ("queued", "running"):
                raise RuntimeError("No active route to cancel")
            self.cancel_requested = True

    def tick(self):
        with self.lock:
            route = self.pending
            cancel = self.cancel_requested
            goal_handle = self.goal_handle
            until = self.wait_until
        if cancel and goal_handle:
            self.update(phase="canceling", message="Cancel requested")
            goal_handle.cancel_goal_async()
            with self.lock:
                self.cancel_requested = False
            return
        if cancel and route:
            with self.lock:
                self.pending = None
                self.cancel_requested = False
            self.update(phase="canceled", message="Canceled before submission")
            return
        if not route or goal_handle:
            return
        if not self.action.server_is_ready():
            return
        if until == 0:
            first, second = dense_points(route["path"])[:2]
            heading = math.atan2(second["y"] - first["y"], second["x"] - first["x"])
            reset = PoseStamped()
            reset.header.frame_id = "map"
            reset.header.stamp = self.get_clock().now().to_msg()
            reset.pose = pose(first["x"], first["y"], heading)
            self.reset_publisher.publish(reset)
            with self.lock:
                self.wait_until = time.monotonic() + 0.8
            self.update(message="Setting simulated start pose")
            return
        if time.monotonic() < until:
            return
        self.send_route(route)
        with self.lock:
            self.pending = None

    def send_route(self, route):
        stamp = self.get_clock().now().to_msg()
        path = RosPath()
        path.header.frame_id = "map"
        path.header.stamp = stamp
        control_points = dense_points(route["path"])
        for i, point in enumerate(control_points):
            previous = control_points[max(0, i - 1)]
            following = control_points[min(len(control_points) - 1, i + 1)]
            yaw = math.atan2(following["y"] - previous["y"], following["x"] - previous["x"])
            item = PoseStamped()
            item.header = path.header
            item.pose = pose(point["x"], point["y"], yaw)
            path.poses.append(item)
        points = PoseArray()
        points.header = path.header
        points.poses = [pose(p["x"], p["y"], p["yaw"]) for p in route["waypoints"]]
        self.path_publisher.publish(path)
        self.waypoint_publisher.publish(points)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "goal_checker"
        future = self.action.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)
        self.update(phase="running", message="FollowPath sent to Nav2")

    def on_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.update(phase="failed", message="Nav2 rejected FollowPath")
                return
            with self.lock:
                self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_result)
        except Exception as error:
            self.update(phase="failed", message=str(error))

    def on_feedback(self, message):
        self.update(distance_remaining=round(message.feedback.distance_to_goal, 3))

    def on_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.update(phase="completed", message="Nav2 FollowPath completed", distance_remaining=0)
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.update(phase="canceled", message="Nav2 FollowPath canceled")
        else:
            self.update(phase="failed", message=f"Nav2 FollowPath ended with status {result.status}")
        with self.lock:
            self.goal_handle = None

    def publish_status(self):
        message = String()
        message.data = json.dumps(self.snapshot())
        self.status_publisher.publish(message)


class Handler(BaseHTTPRequestHandler):
    bridge = None

    def respond(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/api/status", "/api/health"):
            self.respond(200, self.bridge.snapshot())
        else:
            self.respond(404, {"error": "Unknown endpoint"})

    def do_POST(self):
        if self.path not in ("/api/routes", "/api/cancel"):
            self.respond(404, {"error": "Unknown endpoint"})
            return
        try:
            if self.path == "/api/cancel":
                self.bridge.cancel()
            else:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4_000_000:
                    raise ValueError("Invalid route size")
                route = json.loads(self.rfile.read(length))
                self.bridge.submit(route)
            self.respond(202, self.bridge.snapshot())
        except (ValueError, json.JSONDecodeError) as error:
            self.respond(400, {"error": str(error)})
        except RuntimeError as error:
            self.respond(409, {"error": str(error)})

    def log_message(self, _format, *_args):
        pass


def main():
    rclpy.init()
    bridge = Bridge()
    Handler.bridge = bridge
    server = ThreadingHTTPServer(("127.0.0.1", 8765), Handler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    bridge.get_logger().info("Simulation API listening on 127.0.0.1:8765")
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

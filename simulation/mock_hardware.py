"""Atlas mock primitive. No serial, robot sockets, native DDS or /ODOM access."""

import json
import math
import os
from pathlib import Path
import random
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String, UInt32
from robonix_api import Primitive, Ok, Err

from .world import World, coordinates

ROOT = Path(__file__).resolve().parent
primitive = Primitive(id="mock_hardware", namespace="robonix/primitive", pkg_root=ROOT)
resources = {}
settings = {}
TOPICS = {
    "gnss/fix": "/mock/gps/fix", "gnss/velocity": "/mock/gps/velocity",
    "gnss/rtk_status": "/mock/gps/rtk_status", "imu/imu": "/mock/imu/data",
    "imu/angular_velocity": "/mock/imu/gyro", "imu/inertial": "/mock/imu/inertial",
    "imu/status": "/mock/imu/status", "lidar/lidar3d": "/mock/free_cloud",
    "chassis/twist_in": "/mock/cmd_vel", "chassis/gait_in": "/mock/gait_request",
}
DEFAULT_FAULTS = {"gnss": "fixed", "imu_connected": True, "chassis_feedback": True,
                  "gait_ack": True, "heading_bias_deg": 0.0, "velocity_noise_mps": 0.0}


class Hardware(Node):
    def __init__(self, config, context):
        super().__init__("outdoor_mock_hardware", context=context)
        self.config = config
        self.world = World(config["start"], config["max_speed_mps"])
        self.lock = threading.RLock()
        self.faults = dict(DEFAULT_FAULTS)
        self.rng = random.Random(config["seed"])
        self.generation = uuid.uuid4().hex
        self.last_tick = time.monotonic()
        self.imu_count = 0
        self.last_imu = None
        self.pending_gait = None
        self.pending_gait_at = 0.0
        self.fix = self.create_publisher(NavSatFix, TOPICS["gnss/fix"], 10)
        self.velocity = self.create_publisher(TwistStamped, TOPICS["gnss/velocity"], 10)
        self.rtk = self.create_publisher(String, TOPICS["gnss/rtk_status"], 10)
        self.imu = self.create_publisher(Imu, TOPICS["imu/imu"], 10)
        self.gyro = self.create_publisher(Vector3Stamped, TOPICS["imu/angular_velocity"], 20)
        self.inertial = self.create_publisher(Imu, TOPICS["imu/inertial"], 500)
        self.imu_status = self.create_publisher(String, TOPICS["imu/status"], 10)
        self.chassis = self.create_publisher(String, "/mock/chassis_status", 10)
        self.gait = self.create_publisher(UInt32, "/mock/actual_gait", 10)
        self.cloud = self.create_publisher(PointCloud2, TOPICS["lidar/lidar3d"], 10)
        self.create_subscription(Twist, TOPICS["chassis/twist_in"], self.command, 10)
        self.create_subscription(UInt32, TOPICS["chassis/gait_in"], self.gait_request, 10)
        self.create_timer(0.01, self.tick)
        self.create_timer(0.1, self.publish_gnss)
        self.create_timer(0.05, self.publish_chassis)
        self.create_timer(0.2, self.publish_imu_status)
        self.create_timer(0.1, self.publish_cloud)

    def command(self, msg):
        with self.lock:
            self.world.command(msg.linear.x, msg.angular.z, time.monotonic())

    def gait_request(self, msg):
        with self.lock:
            if msg.data in (0x3002, 0x3003) and self.pending_gait != msg.data:
                self.pending_gait, self.pending_gait_at = msg.data, time.monotonic()

    def tick(self):
        with self.lock:
            now = time.monotonic()
            dt = now - self.last_tick
            self.last_tick = now
            self.world.tick(max(0.001, min(dt, 0.1)), now)
            if (self.faults["gait_ack"] and self.pending_gait is not None
                    and now - self.pending_gait_at >= 0.2
                    and abs(self.world.v) < 0.01 and abs(self.world.w) < 0.01):
                self.world.gait = self.pending_gait
                if self.world.gait_history[-1] != self.world.gait:
                    self.world.gait_history.append(self.world.gait)
                self.pending_gait = None
            if not self.faults["imu_connected"]:
                return
            yaw, acceleration, gyro = self.world.imu(self.faults["heading_bias_deg"])
            stamp = self.get_clock().now().to_msg()
            msg = Imu()
            msg.header = Header(stamp=stamp, frame_id="mock_imu_link")
            msg.orientation.z, msg.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acceleration
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
            # Absolute angle at 20 Hz; independent raw inertial samples at 100 Hz.
            if self.imu_count % 5 == 0:
                self.imu.publish(msg)
            rate = Vector3Stamped(header=msg.header)
            rate.vector.z = gyro[2]
            self.gyro.publish(rate)
            msg.orientation_covariance[0] = -1.0
            self.inertial.publish(msg)
            self.last_imu, self.imu_count = now, self.imu_count + 1

    def publish_gnss(self):
        with self.lock:
            mode = self.faults["gnss"]
            if mode == "off":
                return
            w = self.world
            stamp = self.get_clock().now().to_msg()
            stamp_s = stamp.sec + stamp.nanosec / 1e9
            epoch = round(stamp_s * 1000) % 604800000
            msg = NavSatFix(header=Header(stamp=stamp, frame_id="mock_gnss_link"))
            msg.latitude, msg.longitude = coordinates(w.x, w.y, self.config["origin"])
            msg.status.status, msg.status.service = NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.SERVICE_GPS
            sigma = 0.02 if mode == "fixed" else 0.2
            msg.position_covariance = [sigma * sigma, 0.0, 0.0, 0.0, sigma * sigma, 0.0, 0.0, 0.0, 0.04]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            vel = TwistStamped(header=Header(stamp=stamp, frame_id="gnss_enu"))
            noise = self.faults["velocity_noise_mps"]
            vel.twist.linear.x = w.v * math.cos(w.yaw) + self.rng.gauss(0, noise)
            vel.twist.linear.y = w.v * math.sin(w.yaw) + self.rng.gauss(0, noise)
            self.fix.publish(msg)
            self.velocity.publish(vel)
            self.rtk.publish(String(data=json.dumps({
                "state": "RTK_FIXED" if mode == "fixed" else "RTK_FLOAT",
                "receiver_quality": 4 if mode == "fixed" else 5, "rtk_fixed": mode == "fixed",
                "fix_age_s": 0.0, "measurement_age_s": 0.0, "fix_stamp_s": stamp_s,
                "epoch_source": "UBX_iTOW_ms", "measurement_id": epoch,
                "differential_age_s": 0.0, "satellites": 20,
                "serial_state": "OPEN", "position_covariance_source": "UBX_NAV_PVT",
                "velocity": {"fresh": True, "age_s": 0.0, "stamp_s": stamp_s,
                             "measurement_id": epoch, "source": "UBX_NAV_PVT",
                             "accuracy_source": "UBX_NAV_PVT", "accuracy_mps": max(0.01, noise),
                             "reference_frame": "ENU", "covariance_known": True},
                "simulated": True,
            })))

    def publish_imu_status(self):
        with self.lock:
            age = time.monotonic() - self.last_imu if self.last_imu else None
            self.imu_status.publish(String(data=json.dumps({
                "serial_connected": self.faults["imu_connected"],
                "valid": age is not None and age < 0.5, "generation": self.generation,
                "imu_age_s": age, "gyro_age_s": age, "counts": {"imu": self.imu_count},
                "frame_id": "mock_imu_link", "simulated": True,
            })))

    def publish_chassis(self):
        with self.lock:
            if not self.faults["chassis_feedback"]:
                return
            w = self.world
            ready = w.state == 17 and w.gait in (0x3002, 0x3003) and w.usage_mode == 1
            self.chassis.publish(String(data=json.dumps({
                "ready": ready, "state": w.state, "gait": w.gait, "usage_mode": w.usage_mode,
                "motion_feedback_age_s": 0.0, "usage_mode_age_s": 0.0,
                "motion_output_enabled": True, "gait_output_enabled": True,
                "blocked_reasons": [] if ready else ["mock_chassis_not_ready"], "simulated": True,
            })))
            self.gait.publish(UInt32(data=w.gait))

    def publish_cloud(self):
        # Fixed returns outside the 6 m marking range clear the free-space grid.
        # This provides liveness only, without claiming obstacle simulation.
        fields = [PointField(name=a, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, a in enumerate(("x", "y", "z"))]
        points = [(7.9 * math.cos(i * math.tau / 360), 7.9 * math.sin(i * math.tau / 360), 0.0)
                  for i in range(360)]
        self.cloud.publish(point_cloud2.create_cloud(
            Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"), fields, points))

    def snapshot(self):
        with self.lock:
            return {"simulated": True, "world": self.world.snapshot(), "faults": dict(self.faults),
                    "imu_samples": self.imu_count, "scene_port": self.config["scene_port"],
                    "mode": "SIMULATION — NO ROBOT CONNECTION"}

    def update_faults(self, body):
        if set(body) - DEFAULT_FAULTS.keys():
            raise ValueError("Unknown fault key")
        for key, value in body.items():
            if key == "gnss":
                if value not in ("fixed", "float", "off"):
                    raise ValueError("gnss must be fixed, float or off")
            elif type(DEFAULT_FAULTS[key]) is bool:
                if type(value) is not bool:
                    raise ValueError(key + " must be boolean")
            elif (type(value) not in (float, int) or not math.isfinite(value)
                  or abs(value) > (180 if key == "heading_bias_deg" else 1)
                  or key == "velocity_noise_mps" and value < 0):
                raise ValueError("Invalid " + key)
        with self.lock:
            if body.get("imu_connected") is True and not self.faults["imu_connected"]:
                self.generation = uuid.uuid4().hex
            self.faults.update(body)


def control_server(node):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/state":
                self.reply(node.snapshot())
            elif self.path == "/":
                data = (ROOT / "control.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/faults":
                self.send_error(404)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("Invalid body size")
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError("Expected a JSON object")
                node.update_faults(body)
                self.reply(node.snapshot())
            except (ValueError, TypeError) as error:
                self.reply({"error": str(error)}, 400)

        def reply(self, value, code=200):
            data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *_):
            pass

    return ThreadingHTTPServer(("127.0.0.1", node.config["control_port"]), Handler)


@primitive.on_init
def init(config):
    global settings
    if resources:
        return Err("Deactivate mock hardware before reconfiguration")
    if os.environ.get("ROS_DOMAIN_ID") != str(config["domain"]) or config["domain"] in (0, 80):
        return Err("Mock requires its own isolated ROS domain")
    settings = dict(config)
    for contract, topic in TOPICS.items():
        primitive.declare_ros2_topic("robonix/primitive/" + contract, topic, qos="reliable")
    return Ok()


@primitive.on_activate
def activate():
    if resources:
        return Ok()
    context = Context()
    rclpy.init(args=[], context=context)
    node = Hardware(settings, context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    server = control_server(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    resources.update(context=context, node=node, executor=executor, server=server, thread=thread)
    thread.start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return Ok()


@primitive.on_deactivate
@primitive.on_shutdown
def close():
    if resources:
        resources["server"].shutdown()
        resources["server"].server_close()
        resources["executor"].shutdown(timeout_sec=2)
        resources["thread"].join(2)
        resources["node"].destroy_node()
        rclpy.try_shutdown(context=resources["context"])
        resources.clear()
    return Ok()


if __name__ == "__main__":
    primitive.run()

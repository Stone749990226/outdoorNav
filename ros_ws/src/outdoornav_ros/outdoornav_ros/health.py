"""Bounded readiness probe for the navigation API's measured component health."""

import argparse
import json
import sys
import time
from urllib.request import urlopen


def simulation_ready(timeout):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, LaserScan, PointCloud2
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node("simulation_readiness")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    buffer = Buffer()
    _listener = TransformListener(buffer, node)
    received = {}
    for kind, topic in [
        (Clock, "/clock"),
        (Odometry, "/odometry/global"),
        (LaserScan, "/rtk/scan"),
        (Image, "/rtk/camera/depth_image"),
        (PointCloud2, "/rtk/camera/points"),
    ]:
        node.create_subscription(
            kind,
            topic,
            lambda msg, key=topic: received.update({key: time.monotonic()}),
            qos_profile_sensor_data,
        )
    deadline, stable = time.monotonic() + timeout, None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            ready = len(received) == 5 and all(now - t < 2 for t in received.values())
            ready &= buffer.can_transform("map", "base_footprint", rclpy.time.Time())
            stable = (now if stable is None else stable) if ready else None
            if stable is not None and now - stable >= 1:
                print("Simulation, localization and sensors are ready", flush=True)
                return 0
        print(
            "Simulation readiness timeout; received: " + str(received), file=sys.stderr
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=175)
    parser.add_argument("--simulation", action="store_true")
    args = parser.parse_args()
    if args.simulation:
        return simulation_ready(args.timeout)
    deadline, stable, last = time.monotonic() + args.timeout, None, {}
    while time.monotonic() < deadline:
        try:
            with urlopen("http://127.0.0.1:8765/api/status", timeout=1) as response:
                last = json.load(response)
            now = time.monotonic()
            stable = (now if stable is None else stable) if last["ready"] else None
            if stable is not None and now - stable >= 3:
                print(json.dumps(last, ensure_ascii=False))
                return 0
        except (OSError, ValueError, KeyError) as error:
            last = {"error": str(error)}
            stable = None
        time.sleep(0.2)
    print(json.dumps(last, ensure_ascii=False), file=sys.stderr)
    return 1

"""Read-only launch checks; do not stop or reconfigure existing robot services."""

import json
import math
import sys
import time

import rclpy


def main():
    config = json.load(open(sys.argv[1]))
    for key in ("calibrated", "cloud_frame_confirmed", "motion_enabled"):
        if type(config[key]) is not bool:
            raise ValueError(key + " must be a boolean")
    if any(not math.isfinite(w) or w <= 0 for w in config["road_widths_m"].values()):
        raise ValueError("Road widths must be finite positive measurements")
    for key in (
        "initial_heading_rad",
        "footprint_radius_m",
        "max_speed_mps",
        "cloud_min_height_m",
        "cloud_max_height_m",
    ):
        if not math.isfinite(config[key]):
            raise ValueError(key + " must be finite")
    for key in ("antenna_xy_m", "map_offset_xy_m"):
        if len(config[key]) != 2 or not all(math.isfinite(v) for v in config[key]):
            raise ValueError(key + " must contain two finite values")
    if config["motion_enabled"] and not (
        config["calibrated"]
        and config["cloud_frame_confirmed"]
        and config["road_widths_m"]
    ):
        raise ValueError(
            "Motion requires calibration, cloud coverage and measured road widths"
        )
    rclpy.init()
    node = rclpy.create_node("outdoor_preflight")
    until = time.monotonic() + 3
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.1)
    names = set(node.get_node_names())
    conflicts = names & {
        "controller_server",
        "planner_server",
        "bt_navigator",
        "rtabmap",
        "navigation",
        "lynx_geo_localization",
        "lynx_command_watchdog",
    }
    if conflicts or node.count_publishers("/cmd_vel"):
        raise RuntimeError(
            "Stop existing navigation/mapping before outdoor startup: "
            + ", ".join(sorted(conflicts))
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Outdoor-only Nav2 profile; no BT navigator, SLAM or Gazebo dependencies."""

import json
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    bundle = json.loads(Path(value("bundle")).read_text())
    robot = json.loads(Path(value("robot_config")).read_text())
    if (
        not 0 < robot["max_speed_mps"] <= 0.15
        or not 0.1 <= robot["footprint_radius_m"] <= 2
    ):
        raise ValueError("Invalid speed limit or measured footprint radius")
    share = Path(get_package_share_directory("outdoornav_ros"))
    params = yaml.safe_load((share / "config/lynx_navigation.yaml").read_text())
    grid = bundle["grid"]
    for name, rolling, frame in (
        ("global_costmap", False, "map"),
        ("local_costmap", True, "odom"),
    ):
        costmap = {
            "use_sim_time": False,
            "global_frame": frame,
            "robot_base_frame": "base_link",
            "rolling_window": rolling,
            "track_unknown_space": True,
            "update_frequency": 5.0 if rolling else 2.0,
            "publish_frequency": 5.0 if rolling else 2.0,
            "always_send_full_costmap": True,
            "transform_tolerance": 0.2,
            "resolution": float(grid["resolution"]),
            "robot_radius": robot["footprint_radius_m"],
            "width": 8 if rolling else math.ceil(grid["width"] * grid["resolution"]),
            "height": 8 if rolling else math.ceil(grid["height"] * grid["resolution"]),
            "plugins": ["obstacle_layer", "inflation_layer"],
            "obstacle_layer": {
                "plugin": "nav2_costmap_2d::ObstacleLayer",
                "enabled": True,
                "observation_sources": "scan",
                "scan": {
                    "topic": "/outdoor/scan",
                    "data_type": "LaserScan",
                    "marking": True,
                    "clearing": True,
                    "inf_is_valid": False,
                    "obstacle_max_range": 6.0,
                    "raytrace_max_range": 8.0,
                },
            },
            "inflation_layer": {
                "plugin": "nav2_costmap_2d::InflationLayer",
                "inflation_radius": robot["footprint_radius_m"] + 0.2,
                "cost_scaling_factor": 3.0,
            },
            "filters": ["road_boundary"],
            "road_boundary": {
                "plugin": "nav2_costmap_2d::KeepoutFilter",
                "enabled": True,
                "filter_info_topic": "/outdoor/corridor_info",
                "transform_tolerance": 0.2,
            },
        }
        if not rolling:
            costmap.update(
                origin_x=float(grid["origin"][0]), origin_y=float(grid["origin"][1])
            )
        params[name] = {name: {"ros__parameters": costmap}}
    params["controller_server"]["ros__parameters"]["FollowPath"].update(
        max_vel_x=robot["max_speed_mps"], max_speed_xy=robot["max_speed_mps"]
    )
    params["velocity_smoother"]["ros__parameters"]["max_velocity"][0] = robot[
        "max_speed_mps"
    ]
    output = Path(value("run_dir"))
    output.mkdir(parents=True, exist_ok=True)
    effective = output / "lynx_nav2.yaml"
    effective.write_text(yaml.safe_dump(params))
    shared = {
        "bundle": value("bundle"),
        "robot_config": value("robot_config"),
        "use_sim_time": False,
    }
    return [
        RegisterEventHandler(
            OnProcessExit(
                on_exit=[EmitEvent(event=Shutdown(reason="An outdoor process exited"))]
            )
        ),
        Node(
            package="outdoornav_ros",
            executable="lynx_localizer",
            parameters=[shared],
            output="screen",
        ),
        Node(
            package="outdoornav_ros",
            executable="lynx_navigation",
            parameters=[shared, {"places": value("places"), "run_dir": str(output)}],
            output="screen",
        ),
        Node(package="outdoornav_ros", executable="lynx_watchdog", output="screen"),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            remappings=[("cloud_in", robot["cloud_topic"]), ("scan", "/outdoor/scan")],
            parameters=[
                {
                    "target_frame": "base_link",
                    "transform_tolerance": 0.2,
                    "min_height": robot["cloud_min_height_m"],
                    "max_height": robot["cloud_max_height_m"],
                    "range_min": robot["footprint_radius_m"],
                    "range_max": 8.0,
                    "use_inf": True,
                }
            ],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[str(effective)],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[str(effective)],
            remappings=[("cmd_vel", "/outdoor/controller_command")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            parameters=[str(effective)],
            remappings=[
                ("cmd_vel", "/outdoor/controller_command"),
                ("cmd_vel_smoothed", "/navigation/controller_command"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="outdoor_lifecycle",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": [
                        "planner_server",
                        "controller_server",
                        "velocity_smoother",
                    ],
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *(
                DeclareLaunchArgument(name)
                for name in ("bundle", "robot_config", "places", "run_dir")
            ),
            OpaqueFunction(function=nodes),
        ]
    )

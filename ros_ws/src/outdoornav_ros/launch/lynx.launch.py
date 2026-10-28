"""Outdoor-only Nav2 profile; no BT navigator, SLAM or Gazebo dependencies."""

import json
from outdoornav.config import load_robot_config
from outdoornav.lio import fastlio_parameters, checked_matrix, quaternion
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
    GroupAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    diagnostic = value("diagnostic").lower()
    if diagnostic not in ('true', 'false'):
        raise ValueError('diagnostic must be true or false')
    diagnostic = diagnostic == 'true'
    bundle = json.loads(Path(value("bundle")).read_text())
    robot = load_robot_config(value("robot_config"))
    if (
        not 0 < robot["max_speed_mps"] <= 0.4
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
            # Unknown cells ahead of the robot must not be treated as collisions;
            # the planner keeps its own allow_unknown policy on the global grid.
            "track_unknown_space": robot["local_track_unknown"] if rolling else True,
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
                "enabled": robot["obstacle_layer_enabled"],
                "observation_sources": "scan" if robot["obstacle_layer_enabled"] else "",
                "scan": {
                    "topic": "/outdoor/scan",
                    "data_type": "LaserScan",
                    "marking": True,
                    "clearing": True,
                    "inf_is_valid": True,
                    "obstacle_max_range": 6.0,
                    "raytrace_max_range": 8.0,
                },
            },
            "inflation_layer": {
                "plugin": "nav2_costmap_2d::InflationLayer",
                "inflation_radius": robot["footprint_radius_m"] + 0.2,
                "cost_scaling_factor": 3.0,
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
    params['controller_server']['ros__parameters']['FollowPath']['max_vel_theta'] = robot['max_angular_speed_rps']
    params['velocity_smoother']['ros__parameters']['max_velocity'][2] = robot['max_angular_speed_rps']
    params['velocity_smoother']['ros__parameters']['min_velocity'][2] = -robot['max_angular_speed_rps']
    output = Path(value("run_dir"))
    output.mkdir(parents=True, exist_ok=True)
    effective = output / "lynx_nav2.yaml"
    shared = {
        "bundle": value("bundle"),
        "robot_config": value("robot_config"),
        "use_sim_time": False,
        "diagnostic": diagnostic,
    }
    params['velocity_smoother']['ros__parameters']['velocity_timeout'] = robot['command_timeout_s']
    effective.write_text(yaml.safe_dump(params))
    lio_nodes = []
    if robot.get('lio', {}).get('enabled'):
        # FAST-LIO owns a private IMU-frame TF tree. Only the localizer owns outdoor TF.
        get_package_share_directory('fast_lio')
        lio_nodes = [
            Node(package='outdoornav_ros', executable='lynx_lio', parameters=[shared], output='screen'),
            Node(package='fast_lio', executable='fastlio_mapping', name='outdoor_fastlio',
                 parameters=[fastlio_parameters(robot)], output='screen',
                 remappings=[('/tf', '/outdoor/lio/tf'), ('/Odometry', '/outdoor/lio/raw_odom'),
                             ('/path', '/outdoor/lio/path'), ('/cloud_registered', '/outdoor/lio/registered'),
                             ('/cloud_registered_body', '/outdoor/lio/registered_body'),
                             ('/cloud_effected', '/outdoor/lio/effected'), ('/Laser_map', '/outdoor/lio/map')]),
        ]
        lio = robot['lio']
        if lio['start_driver']:
            get_package_share_directory('rslidar_sdk')
            path = Path(lio['driver_config'])
            if not path.is_absolute():
                path = Path(value('robot_config')).resolve().parent / path
            driver = yaml.safe_load(path.read_text())
            sensor = driver['lidar'][0]['ros']
            if (sensor['ros_send_point_cloud_topic'] != robot['cloud_topic'] or
                    sensor['ros_send_imu_data_topic'] != lio['imu_topic'] or
                    sensor['ros_frame_id'] != lio['lidar_frame']):
                raise ValueError('LIO driver topics/frame differ from robot configuration')
            lio_nodes.insert(0, Node(package='rslidar_sdk', executable='rslidar_sdk_node',
                                     name='outdoor_lidar_driver', parameters=[{'config_path': str(path)}], output='screen'))
        for child, key in ((lio['lidar_frame'], 'base_from_lidar'), (lio['imu_frame'], 'base_from_imu')):
            if child == 'base_link':
                continue
            matrix = checked_matrix(lio[key])
            xyz, xyzw = matrix[:3, 3].tolist(), quaternion(matrix[:3, :3]).tolist()
            arguments = [arg for name, number in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), [*xyz, *xyzw])
                         for arg in ('--'+name, str(number))]
            lio_nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                  arguments=[*arguments, '--frame-id', 'base_link', '--child-frame-id', child]))
    return [GroupAction(actions=[
        SetRemap(src='/tf', dst='/outdoor/tf'),
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
        Node(package="outdoornav_ros", executable="lynx_watchdog",
             parameters=[shared], output="screen"),
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
                    "range_min": robot["scan_range_min_m"] or robot["footprint_radius_m"],
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
    ]), GroupAction(actions=lio_nodes)]


def generate_launch_description():
    return LaunchDescription(
        [
            *(
                DeclareLaunchArgument(name)
                for name in ("bundle", "robot_config", "places", "run_dir")
            ),
            DeclareLaunchArgument("diagnostic", default_value="false"),
            OpaqueFunction(function=nodes),
        ]
    )

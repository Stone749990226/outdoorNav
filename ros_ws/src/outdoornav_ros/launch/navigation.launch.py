"""Nav2 and the unique goal/task owner, with task-level road constraints."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    bundle_path = LaunchConfiguration("bundle").perform(context)
    bundle = json.loads(Path(bundle_path).read_text())
    share = Path(get_package_share_directory("outdoornav_ros"))
    params = str(share / "config/navigation.yaml")
    common = {"use_sim_time": True}
    grid = bundle["grid"]
    # Emit a final parameter file, not unsupported YAML aliases or dotted parameter overrides.
    import yaml

    effective = yaml.safe_load(Path(params).read_text())
    controller = effective["controller_server"]["ros__parameters"]
    pursuit = controller["FollowPath"]
    if (
        pursuit["use_rotate_to_heading"]
        and pursuit["min_lookahead_dist"]
        <= controller["goal_checker"]["xy_goal_tolerance"]
    ):
        raise ValueError(
            "RPP minimum lookahead must exceed arrival tolerance when heading alignment is enabled"
        )
    # Costmap filters run after ordinary plugins. Inflate the route boundary
    # after KeepoutFilter as well, using the existing inflation settings.
    for name in ("global_costmap", "local_costmap"):
        costmap_params = effective[name][name]["ros__parameters"]
        costmap_params["filters"].extend(
            (["motion_prediction"] if name == "global_costmap" else [])
            + ["route_inflation"]
        )
        if name == "local_costmap":
            costmap_params["route_keepout"]["filter_info_topic"] = (
                "/navigation/local_keepout_info"
            )
        else:
            costmap_params["motion_prediction"] = {
                "plugin": "nav2_costmap_2d::KeepoutFilter",
                "enabled": True,
                "filter_info_topic": "/navigation/prediction_info",
                "transform_tolerance": 0.5,
            }
        costmap_params["route_inflation"] = dict(costmap_params["inflation_layer"])
    global_params = effective["global_costmap"]["global_costmap"]["ros__parameters"]
    global_params.update(
        width=int(round(grid["width"] * grid["resolution"])),
        height=int(round(grid["height"] * grid["resolution"])),
        origin_x=float(grid["origin"][0]),
        origin_y=float(grid["origin"][1]),
        resolution=grid["resolution"],
    )
    output = Path(LaunchConfiguration("run_dir").perform(context)) / "navigation.yaml"
    output.write_text(yaml.safe_dump(effective, sort_keys=False))
    params = str(output)
    return [
        Node(
            package="outdoornav_ros",
            executable="navigation",
            parameters=[
                common,
                {
                    "bundle": bundle_path,
                    "run_dir": LaunchConfiguration("run_dir").perform(context),
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=[params],
            remappings=[("cmd_vel", "/navigation/controller_command")],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="navigation_lifecycle",
            parameters=[
                common,
                {
                    "autostart": True,
                    "bond_timeout": 20.0,
                    "node_names": ["planner_server", "controller_server"],
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            parameters=[common],
            arguments=["-d", str(share / "config/campus.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bundle"),
            DeclareLaunchArgument("run_dir"),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=nodes),
        ]
    )

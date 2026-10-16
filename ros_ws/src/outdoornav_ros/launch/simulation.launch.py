"""Fortress physics and sensors, shared by native Ubuntu and WSL."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    bundle_path = Path(LaunchConfiguration("bundle").perform(context))
    bundle = json.loads(bundle_path.read_text())
    run_dir = Path(LaunchConfiguration("run_dir").perform(context))
    share = Path(get_package_share_directory("outdoornav_ros"))
    common = {"use_sim_time": True}
    origin = bundle["origin_wgs84"]
    gazebo_share = get_package_share_directory("ros_gz_sim")
    world = bundle_path.parent / "world.sdf"
    model = share / "models/robot.urdf.xacro"
    result = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(gazebo_share) / "launch/gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-r -v 2 --gui-config {share / 'config/gui.config'} {world}"
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[common, {"robot_description": Command(["xacro ", str(model)])}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                "robot_rtk",
                "-topic",
                "robot_description",
                "-x",
                str(bundle["spawn"][0]),
                "-y",
                str(bundle["spawn"][1]),
                "-z",
                "0.039",
                "-Y",
                str(bundle["spawn"][2]),
            ],
        ),
    ]
    core = [
        "/world/north_campus/create@ros_gz_interfaces/srv/SpawnEntity",
        "/world/north_campus/remove@ros_gz_interfaces/srv/DeleteEntity",
        "/world/north_campus/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/rtk/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/rtk/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
        "/wheel_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/rtk/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]
    camera = [
        f"/rtk/camera/{topic}@sensor_msgs/msg/{kind}[gz.msgs.{gz_kind}"
        for topic, kind, gz_kind in [
            ("image", "Image", "Image"),
            ("depth_image", "Image", "Image"),
            ("camera_info", "CameraInfo", "CameraInfo"),
        ]
    ]
    for name, topics in [("core_bridge", core), ("camera_bridge", camera)]:
        result.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=name,
                arguments=topics,
                parameters=[common],
            )
        )
    for executable in ("rtk_sim", "wheel_odom_bridge", "absolute_localizer"):
        result.append(
            Node(package="outdoornav_ros", executable=executable, parameters=[common])
        )
    result += [
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            parameters=[
                str(share / "config/navsat.yaml"),
                common,
                {"datum": [origin["latitude"], origin["longitude"], 0.0]},
            ],
            remappings=[
                ("imu", "/rtk/imu"),
                ("gps/fix", "/rtk/sim_fix"),
                ("odometry/filtered", "/odometry/local"),
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            parameters=[str(share / "config/ekf_local.yaml"), common],
            remappings=[("odometry/filtered", "/odometry/local")],
        ),
        Node(
            package="outdoornav_ros",
            executable="depth_display",
            parameters=[common],
        ),
        Node(
            package="outdoornav_ros",
            executable="depth_points",
            parameters=[common, {"emit_far_points": True, "far_clip": 8.0}],
        ),
        Node(
            package="outdoornav_ros",
            executable="scan_points",
            parameters=[
                common,
                {
                    "emit_far_points": True,
                    "far_range": 7.9999,
                    "obstacle_max_range": 7.5,
                },
            ],
        ),
        Node(
            package="outdoornav_ros",
            executable="tilt_frames",
            parameters=[
                common,
                {
                    "camera_frame": "robot_rtk/base_footprint/depth_camera",
                    "camera_x": 0.055,
                    "camera_y": 0.0,
                    "camera_z": 0.19,
                    "camera_roll": -1.5707963,
                    "camera_pitch": 0.0,
                    "camera_yaw": -1.5707963,
                    "scan_frame": "robot_rtk/base_footprint/near_lidar",
                    "scan_x": 0.0,
                    "scan_y": 0.0,
                    "scan_z": 0.11,
                },
            ],
        ),
        Node(
            package="outdoornav_ros",
            executable="trajectory_logger",
            parameters=[common, {"output_csv": str(run_dir / "trajectory.csv")}],
        ),
    ]
    for name, child, xyz in [
        ("imu_tf", "robot_rtk/base_footprint/imu_sensor", (0, 0, 0.05)),
        ("gps_tf", "robot_rtk/base_footprint/navsat_sensor", (-0.03, 0, 0.13)),
    ]:
        result.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=name,
                parameters=[common],
                arguments=[
                    "--x",
                    str(xyz[0]),
                    "--y",
                    str(xyz[1]),
                    "--z",
                    str(xyz[2]),
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    child,
                ],
            )
        )
    return result


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bundle"),
            DeclareLaunchArgument("run_dir"),
            OpaqueFunction(function=nodes),
        ]
    )

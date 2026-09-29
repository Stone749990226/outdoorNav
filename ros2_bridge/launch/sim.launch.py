from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("outdoor_nav_bridge"))
    config = str(share / "config/nav2_sim.yaml")
    map_yaml = str(share / "maps/campus_sim.yaml")
    return LaunchDescription(
        [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[config, {"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[config],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_sim",
                parameters=[{"use_sim_time": False, "autostart": True, "node_names": ["map_server", "controller_server"]}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "map", "--child-frame-id", "odom"],
            ),
            Node(package="outdoor_nav_bridge", executable="sim_robot"),
            Node(package="outdoor_nav_bridge", executable="bridge"),
        ]
    )

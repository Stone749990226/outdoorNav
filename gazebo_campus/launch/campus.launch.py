"""Self-contained Gazebo 11 / Humble navigation; separate from the web demo."""
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    generated=root/'generated'
    scene=json.loads((generated/'scene_geometry.json').read_text())
    spawn=scene['default_spawn']
    gui=LaunchConfiguration('gui'); rviz=LaunchConfiguration('rviz')
    params=str(root/'config/nav2.yaml')
    shared={'use_sim_time':True}
    nodes=[DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true')]
    nodes += [DeclareLaunchArgument(key,default_value=str(value)) for key,value in zip(['x','y','yaw'],spawn)]
    nodes += [SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI',''),
              SetEnvironmentVariable('GAZEBO_MODEL_PATH',str(generated/'models')+os.pathsep+os.environ.get('GAZEBO_MODEL_PATH','')),
              IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros'))/'launch/gazebo.launch.py')),
                                       launch_arguments={'world':str(generated/'campus.world'),'gui':gui,'verbose':'true'}.items()),
              Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','campus_robot','-file',str(root/'models/robot.sdf'),'-x',LaunchConfiguration('x'),'-y',LaunchConfiguration('y'),'-z','0.02','-Y',LaunchConfiguration('yaw')],output='screen'),
              ExecuteProcess(cmd=['/usr/bin/python3',str(root/'scripts/scene_node.py'),'--ros-args','-p','use_sim_time:=true','-p',f'scene_dir:={generated}'],output='screen'),
              Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[shared,{'robot_description':(root/'models/robot.urdf').read_text()}]),
              Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[params,{'yaml_filename':str(generated/'campus.yaml')}],output='screen')]
    for package,executable in [('nav2_controller','controller_server'),('nav2_planner','planner_server'),('nav2_bt_navigator','bt_navigator')]:
        extra={'default_nav_to_pose_bt_xml':str(root/'config/navigate.xml'),
               'default_nav_through_poses_bt_xml':str(root/'config/navigate_through.xml')} if executable=='bt_navigator' else {}
        nodes.append(Node(package=package,executable=executable,name=executable,parameters=[params,extra],output='screen'))
    nodes += [Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[shared,{'autostart':True,'node_names':['map_server','controller_server','planner_server','bt_navigator'],'bond_timeout':10.0}],output='screen'),
              Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/campus.rviz')],parameters=[shared],condition=IfCondition(rviz),output='screen')]
    return LaunchDescription(nodes)

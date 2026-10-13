#!/usr/bin/env python3
"""RTK 第四步（目标点导航：规划 — 跟随 — 实时重规划）的完整 launch。

第三阶段（复用 nav2 建图与规划、出路径但不接管控制）的组件仍在，但**默认不启动**：
`route_follower`（`route:=true`）、`danger`（`danger:=true`）、
自写局部图宿主 `local_costmap_host`（`legacy_costmap_host:=true`）——保留是为了让第三阶段的判据能复跑。

数据流（验收做来源审计时对照这张图）：

    Gazebo GPS ──/rtk/navsat──> rtk_sim ──/rtk/sim_fix──> navsat_transform
                                                              │ /gps/filtered
    Gazebo IMU ──/rtk/imu──────────────────────────────────────┤
                                                              ▼
                       absolute_localizer ──> TF: map→odom（位置直接取 RTK、航向直接取 IMU）
    DiffDrive ──/wheel_odom_raw──> wheel_odom_bridge ──> ekf_local ──> TF: odom→base_footprint

    Gazebo 深度相机 ──/rtk/camera/{depth_image,camera_info}──> depth_image_proc/point_cloud_xyz
                                                              │ /rtk/camera/points
                                                              ▼
                        ┌─────────────────────────────────────┴──────────────────────┐
                        ▼                                                            ▼
        planner_server 内的 global_costmap                        controller_server 内的 local_costmap
        （obstacle_layer + inflation_layer，map 帧，40×40 m，      （obstacle_layer + inflation_layer，base_link 帧，
          track_unknown_space + Overwrite，从零累积）               8×8 m 滚动）
                        │                                                            │
                        │ /global_costmap/costmap                    /local_costmap/costmap
                        │ （nav2 原生命名：相对名 "costmap" 解析到节点命名空间）
                        ▼                                                            ▼
                  NavFn 规划（ComputePathToPose）                     RPP 跟随（FollowPath）
                        │                                                            │
                        └──────────────> goal_navigator（本仓库自写，薄层）<──────────┘
                                          规划→跟随→三种触发重规划                     │
                                                                                       ▼ /cmd_vel
                                                                              （只有 controller_server 发）

    Gazebo 真值 ──/ground_truth/odom──> 只给离线记录器（不进 TF、不进任何一环）

真值不桥接 /ground_truth_tf，也不进 TF。

**第三阶段相对第二阶段删掉的东西**（AuYang 已定，brief §1）：
近场 360° 激光（传感器、`/rtk/scan` 桥接、`near_field` 停车通路、`scan_clearing`）全部移除。
相机水平视场 60°，**视场之外（侧后方）的障碍只有等机器人转过去才会进图**——
v1 不考虑侧向/后方贴障碍通过的剐蹭（用户已确认的口径）。

**局部图的宿主在第四步换成了 nav2 的 `controller_server`**（第四步 brief §2 裁定 1，AuYang 已确认
"可以接管控制"）。第三阶段之所以自写宿主，是为了"不接管控制"时避开 `controller_server` 的 cmd_vel 通路；
现在控制本来就交给它，那个前提不存在了。自写宿主保留、默认不起。
**不上 `bt_navigator`/行为树**：重规划由自写的 `goal_navigator` 负责，时机可控可解释。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'robonix_rtk_nav2'

# gz 话题 <-> ROS 话题。**故意不桥接** /ground_truth_tf：真值不进 TF 是一条硬边界。
# 也不再桥接 /rtk/scan（近场激光已移除）。
#
# **拆成两个桥接节点**：相机单独一个。理由是验收里要测"断掉相机数据 → 停车"，
# 如果相机和 /clock 挂在同一个桥接上，杀掉桥接会连仿真时钟一起杀掉——时钟一停，
# 所有基于仿真时间的看门狗全部失效，测出来的是"时钟没了"而不是"相机没了"。
CAMERA_BRIDGE_TOPICS = [
    '/rtk/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
    '/rtk/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
    '/rtk/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
]

CORE_BRIDGE_TOPICS = [
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    '/rtk/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    '/rtk/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
    '/wheel_odom_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry',
    '/wheel_tf_diag@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
]


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(package_share, 'worlds', 'rtk_courtyard.sdf')
    default_model = os.path.join(package_share, 'models', 'robot_rtk_nav2.urdf.xacro')
    default_navsat = os.path.join(package_share, 'config', 'navsat.yaml')
    default_ekf_local = os.path.join(package_share, 'config', 'ekf_local.yaml')
    default_route = os.path.join(package_share, 'config', 'route.yaml')
    default_nav2 = os.path.join(package_share, 'config', 'nav2_params.yaml')
    default_danger = os.path.join(package_share, 'config', 'danger.yaml')
    default_rviz = os.path.join(package_share, 'config', 'demo.rviz')
    tf_gate = os.path.join(package_share, 'scripts', 'wait_tf_gate.py')

    # **Gazebo 默认带 GUI 起**（`gz_gui:=true`）。`-s`（只起 server）在这台机器上
    # **等于"没有相机"**：实测 `ign topic -l | grep camera` 计数 = 0 → 没有 `/rtk/camera/depth_image`
    # → 没有点云 → 两张代价地图没有数据。**传感器走渲染，`-s` 起不来**。
    # 这条是实测结论，保留；reviewer 原来"默认无界面"的建议**已撤回**。
    #
    # **接线曾经写反过**（reviewer 2026-09-20 复验时从产物里读出来的）：
    # `gz_sim_headless` 带着 `-s` 却挂了 `IfCondition(gz_gui)`、`gz_sim_gui` 挂了 `UnlessCondition(gz_gui)`，
    # 于是在 `gz_gui` 默认 true 时**走的正是无界面那支**——`nav4-3`/`nav4-4` 两次跑的 Gazebo 命令行
    # 里都有 `-s` 就是证据。写反之后现象极其像"参数没生效"，我因此误判过一轮（见下）。
    #
    # 另外两条仍然成立的代价（属于**演示现场**要处理的事，不是起栈默认值）：
    # GUI 是这台机器上最不稳的一环（第三阶段记录过它会盖住 RViz 面板），且实测 **172% CPU**。
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 -s ', LaunchConfiguration('world')]}.items(),
        condition=UnlessCondition(LaunchConfiguration('gz_gui')),
    )
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 ', LaunchConfiguration('world')]}.items(),
        condition=IfCondition(LaunchConfiguration('gz_gui')),
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', LaunchConfiguration('model_file')])}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='log',
        arguments=['-name', 'robot_rtk',
                   '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x_pose'),
                   '-y', LaunchConfiguration('y_pose'),
                   '-z', LaunchConfiguration('z_pose')],
        parameters=[{'use_sim_time': True}],
    )

    bridge_core = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ros_gz_bridge_rtk_core', output='log',
        arguments=CORE_BRIDGE_TOPICS,
        parameters=[{'use_sim_time': True}],
    )

    bridge_camera = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='ros_gz_bridge_rtk_camera', output='log',
        arguments=CAMERA_BRIDGE_TOPICS,
        parameters=[{'use_sim_time': True}],
    )

    rtk_sim = Node(
        package=PACKAGE, executable='rtk_sim', name='rtk_sim', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odom_bridge = Node(
        package=PACKAGE, executable='wheel_odom_bridge', name='wheel_odom_bridge',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    navsat = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform', output='screen',
        parameters=[default_navsat, {'use_sim_time': True}],
        remappings=[('imu', '/rtk/imu'),
                    ('gps/fix', '/rtk/sim_fix'),
                    ('odometry/filtered', '/odometry/local')],
    )

    ekf_local = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_local', output='screen',
        parameters=[default_ekf_local, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odometry/local')],
    )

    localizer = Node(
        package=PACKAGE, executable='absolute_localizer', name='absolute_localizer',
        output='screen', parameters=[{'use_sim_time': True}],
    )

    # ---- 相机深度 -> 点云（brief §2 指定，不再自写反投影）----
    # 用 point_cloud_xyz（**沿光轴**的深度）而不是 point_cloud_xyz_radial：
    # 第一阶段自写的反投影用的就是"深度 = 沿光轴距离"这一假设，Z_FLOOR=0.12 那个标定值
    # 是在那个假设下标出来的。换径向假设会让标定失效，所以这里保持一致。
    point_cloud = Node(
        package='depth_image_proc', executable='point_cloud_xyz_node',
        name='depth_point_cloud', output='log',
        parameters=[{'use_sim_time': True}],
        remappings=[('image_rect', '/rtk/camera/depth_image'),
                    ('camera_info', '/rtk/camera/camera_info'),
                    ('points', '/rtk/camera/points')],
    )

    # ---- 前向危险距离（自写，收缩后只做这一件事）----
    # `danger`（第三阶段的前向危险距离节点）：**退役，默认不起**。
    # 它会给 route_follower 判停车，而本步避障交给 costmap + 控制器 + 重规划；
    # 留着它只会多一条抢控制的通路。代码保留、开关可开，供第三阶段判据复跑。
    danger = Node(
        package=PACKAGE, executable='danger', name='danger', output='screen',
        parameters=[default_danger, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('danger')),
    )

    depth_display = Node(
        package=PACKAGE, executable='depth_display', name='depth_display', output='log',
        parameters=[{'use_sim_time': True}],
    )

    # ---- navbar 两张图的宿主 ----
    # 全局图由 nav2 原生的 planner_server 承载（不发控制，3.2 本来就需要它）。
    # 局部图由**本仓库自写的 C++ 宿主**承载（裁定 2：不用 controller_server）。
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen',
        parameters=[default_nav2, {'use_sim_time': True}],
    )
    # **controller_server 接管控制，并承载 local_costmap**（第四步 brief §2 裁定 1）。
    # 参数段 `local_costmap: local_costmap:` 原样复用——那本来就是 nav2 惯例布局
    # （Costmap2DROS 的全限定名就是 /local_costmap/local_costmap），换宿主不改结构。
    controller_server = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen',
        parameters=[default_nav2, {'use_sim_time': True}],
        remappings=[('cmd_vel', '/cmd_vel')],
    )
    # 第三阶段自写的局部图宿主：**退役，默认不起**。
    # 保留开关的原因是"第三阶段的判据还要能复跑"（reviewer 裁定 2），代码与脚本一律不删。
    # 注意它和 controller_server **不能同时起**：两者都会创建 `local_costmap` 节点、
    # 抢同一个话题与服务，会打架。
    local_costmap_host = Node(
        package='robonix_rtk_nav2_costmap_host', executable='local_costmap_host',
        name='local_costmap_host', output='screen',
        parameters=[default_nav2, {'use_sim_time': True,
                                   'param_dump_path': LaunchConfiguration('param_dump_path')}],
        condition=IfCondition(LaunchConfiguration('legacy_costmap_host')),
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_nav2', output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     # bond 由宿主建（见 costmap_host 的注释）。给足超时：
                     # 全局图激活时要等 map→base_footprint 这条 TF 就绪。
                     'bond_timeout': 20.0,
                     'node_names': ['planner_server', 'controller_server']}],
        condition=UnlessCondition(LaunchConfiguration('legacy_costmap_host')),
    )
    # 复跑第三阶段（`legacy_costmap_host:=true`）时换这一个：管 planner + 自写局部图宿主。
    # **为什么用两个节点而不是把 node_names 做成 LaunchConfiguration**：nav2 的 `node_names` 要的是
    # **字符串数组**，而 launch 参数传过去是**字符串**（`"['a','b']"`），类型对不上——
    # 这是 `global_map/local_map` 一直 False 的一个独立嫌疑（nav2 没被激活）。
    # 直接给 Python list 最省事，也不用引入 OpaqueFunction。
    lifecycle_manager_legacy = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_nav2', output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'bond_timeout': 20.0,
                     'node_names': ['planner_server', 'local_costmap_host']}],
        condition=IfCondition(LaunchConfiguration('legacy_costmap_host')),
    )

    # nav2 只在 `map → base_footprint` 真的可查之后再起。
    #
    # 为什么必须显式等：`Costmap2DROS::on_activate()` 里有一个 2 Hz 的死等循环
    # （等 global↔base 的 TF 能变换），TF 没就绪它就**一直不返回**——而
    # lifecycle_manager 的 change_state 是带超时的，超时后 autostart 会失败，
    # 全局图就永远是死的。靠"睡几秒"来过这一关是脆的（定位链就绪时间随机），
    # 所以这里用一个真的去查 TF 的闸门进程，它的退出码决定 nav2 起不起。
    nav2_group = [planner_server, controller_server, local_costmap_host,
                  lifecycle_manager, lifecycle_manager_legacy]
    tf_gate_proc = ExecuteProcess(
        cmd=['python3', tf_gate,
             '--target', 'map', '--source', 'base_footprint', '--timeout', '180'],
        output='screen',
    )

    route_follower = Node(
        package=PACKAGE, executable='route_follower', name='route_follower',
        output='screen',
        parameters=[default_route, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('route')),
    )

    # ---- 导航入口（第四步的核心新组件）----
    # 规划 → 跟随 → 三种触发实时重规划；不用 bt_navigator/行为树（brief §2 裁定 3）。
    #
    # **抖动抑制阈值/最小重发间隔在这里显式给值，是对 brief 建议值的实测修正**（偏离了要说明）：
    # brief §2 建议 横向<0.25 m 且 长度差<0.30 m 不重发。实测正常行驶中、**仅仅因为地图在长**
    # 造成的路径变化就在 横向 0.14~0.18 m / 长度差 0.31~0.45 m —— 也就是**长度差每次都在阈值之上**，
    # 于是几乎每张新图都触发一次"取消+重发"，车每隔 1.5~4 s 就被打断一次（`/cmd_vel` 一次会话里
    # 只有 3.7% 的时间有输出），表现就是"走一段就停住"。
    # 所以把阈值抬到**日常抖动带之上**：只有路径**真的变了**（例如路上新出现一堵墙、
    # 路径必须绕开）才重发。这既让车能连续走完，也正好是演示要的那个效果——
    # "增删障碍后路径实时改变"，而不是"每秒都变一点点"。
    goal_navigator = Node(
        package=PACKAGE, executable='goal_navigator', name='goal_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'jitter_lateral_m': 0.45,
            'jitter_length_m': 1.20,
            'min_resend_interval_s': 6.0,
        }],
    )

    trajectory_logger = Node(
        package=PACKAGE, executable='rtk_trajectory_logger', name='rtk_trajectory_logger',
        output='log',
        parameters=[{'use_sim_time': True,
                     'output_csv': LaunchConfiguration('trajectory_csv')}],
    )

    # Gazebo 的传感器帧名是 "<模型>/<链路>/<传感器>"，而 TF 里没有这一帧。
    # 第一阶段为 IMU 与 GPS 补过两个静态 TF（数值与 URDF 一致）；第三阶段相机也进了
    # TF 链路（nav2 的障碍层必须把点云从它的 frame_id 变换到代价地图帧，
    # 而第二阶段自写反投影用的是硬编码外参、不需要 TF），所以相机这一帧同样要补。
    #
    # 相机的旋转是**光学约定**（z 前 / x 右 / y 下），与 URDF 里
    # `camera_optical_joint` 的 rpy 一致（-90°, 0, -90°）。
    # 只给 TF 补旋转是不够的：`depth_image_proc` 输出的点云按光学约定排列，
    # 所以这一帧必须被声明成光学帧，否则点云会整体转 90°、地面点跑到墙上去。
    imu_frame_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='imu_sensor_frame_tf', output='log',
        arguments=['--x', '0', '--y', '0', '--z', '0.05',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', LaunchConfiguration('imu_frame')],
        parameters=[{'use_sim_time': True}],
    )
    gps_frame_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='navsat_sensor_frame_tf', output='log',
        arguments=['--x', '-0.03', '--y', '0', '--z', '0.13',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', LaunchConfiguration('gps_frame')],
        parameters=[{'use_sim_time': True}],
    )
    camera_frame_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_sensor_frame_tf', output='log',
        arguments=['--x', '0.055', '--y', '0', '--z', '0.19',
                   '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                   '--frame-id', 'base_footprint',
                   '--child-frame-id', LaunchConfiguration('camera_frame')],
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    def _nav2_after_gate(event, context):
        """**闸门非 0 就不起 nav2**（reviewer 指出的缺陷，第四步范围内）。

        原来 `OnProcessExit(target_action=tf_gate_proc, on_exit=nav2_group)` **不看退出码**——
        闸门超时（`map` 一直不存在）时 nav2 照样起，然后 `global_costmap` 无限
        "Timed out waiting for transform ... Invalid frame ID map"，把
        "上游没起来"伪装成"nav2 卡住"。现在按退出码分流；具体是哪一类上游问题，
        由 `wait_tf_gate.py` 在它自己的输出里分好类（仿真上游 / 定位链 / nav2 自身）。
        """
        rc = getattr(event, 'returncode', None)
        if rc == 0:
            return nav2_group
        return [LogInfo(msg='** TF 闸门未通过（rc=%s），不启动 nav2。**'
                            '请按闸门输出里的分类找上游（仿真上游 / 定位链 / nav2 自身）；'
                            '不要把它当成 nav2 的问题查。' % rc)]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('model_file', default_value=default_model),
        # 出生点与第二阶段一致：(2.6,-2.6) 是路线起点，也避开了中央方块（|x|,|y| <= 1.25）。
        DeclareLaunchArgument('x_pose', default_value='2.6'),
        DeclareLaunchArgument('y_pose', default_value='-2.6'),
        DeclareLaunchArgument('z_pose', default_value='0.01'),
        DeclareLaunchArgument('trajectory_csv', default_value='/tmp/trajectory.csv'),
        # 局部图宿主在 on_activate 时把内部 costmap 的实际参数落一份到这里。
        # 为什么要在进程内落：`/local_costmap/local_costmap` 的参数服务在本接线下
        # 不应答（ListParameters 超时、`ros2 param dump` 返回 0 字节），而两张图的
        # 实际参数是 brief §4 要的证据，不能因为外部查询工具不通就缺一份。
        DeclareLaunchArgument('param_dump_path', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        # 路线跟随默认不随 launch 启动（由 run-route.sh 显式启动）：栈刚起来时代价地图
        # 还在瞬态，它会在几秒内以 costmap_no_pose 退出，在日志里留下误导性的非零退出码。
        DeclareLaunchArgument('route', default_value='false'),
        # ---- 第四步的开关（默认值 = 演示用的一套）----
        # `danger` / `route_follower` / `local_costmap_host` 都是第三阶段的组件，
        # 本步默认全部不起（brief 裁定 2），但**代码与脚本保留**，给开关就能复跑第三阶段判据。
        DeclareLaunchArgument('danger', default_value='false'),
        # Gazebo 是否开 3D 窗口。默认 **不开**（见上面 gz_sim_headless 的注释）。
        # 默认 **True**：无界面会把相机链路弄没（见 gz_sim_headless 处的实测说明）。
        # **订正一条我写错的因果**（reviewer 读了容器里 Humble 的实现后纠正）：
        # launch 的 `IfCondition` **不是** `eval`，而是**先 `lower()` 再查表**
        # （`launch/conditions/evaluate_condition_expression_impl.py`），
        # 所以小写 `'true'` / `'false'` **完全合法**，`'True'` 也一样。
        # 我先前写的"必须写 True，否则恒为假"是**错的**，这里保留 `'True'` 只是无害。
        # 我那次"对调之后还是走无界面"的观察也是错的——reviewer 从产物里看到
        # nav4-5/nav4-6 的 Gazebo 命令行已经**没有 `-s`**（nav4-3/4 才有）。
        DeclareLaunchArgument('gz_gui', default_value='True'),
        DeclareLaunchArgument('legacy_costmap_host', default_value='false'),
        # 生命周期管理器管哪些节点：由 `legacy_costmap_host` 选（见上面两个 lifecycle_manager 的说明）。
        # Gazebo 给的帧名带链路前缀，且**随机型/写法变**，所以做成参数并显式记录；
        # 首轮实测后按实际 frame_id 钉死（写进 docs/rtk-nav2.md）。
        DeclareLaunchArgument('imu_frame',
                              default_value='robot_rtk/base_footprint/imu_sensor'),
        DeclareLaunchArgument('gps_frame',
                              default_value='robot_rtk/base_footprint/navsat_sensor'),
        # **实测值**（2026-09-20，nav2-d2）：相机帧是 `robot_rtk/base_footprint/depth_camera`
        # ——Gazebo 给传感器的帧名是 `<模型>/<规范链路>/<传感器名>`，与 IMU 的
        # `robot_rtk/base_footprint/imu_sensor` 同一规律。**不是** URDF 里那个 camera_link。
        # 猜错时的现象很有迷惑性：局部图那边刷
        # "Message Filter dropping message: frame '...' for reason 'the timestamp on the
        # message is earlier than all the data in the transform cache'"（看起来像时间同步问题，
        # 实际是"这一帧根本不在 TF 里"）。
        DeclareLaunchArgument('camera_frame',
                              default_value='robot_rtk/base_footprint/depth_camera'),
        gz_sim_headless,
        gz_sim_gui,
        robot_state_publisher,
        spawn_entity,
        bridge_core,
        bridge_camera,
        rtk_sim,
        odom_bridge,
        navsat,
        ekf_local,
        localizer,
        point_cloud,
        danger,
        depth_display,
        imu_frame_tf,
        gps_frame_tf,
        camera_frame_tf,
        tf_gate_proc,
        # **不要把 handler 包进 `OpaqueFunction`**：`OpaqueFunction.execute()` 只传 `context`
        # （`self.__function(context, *args, **kwargs)`），于是 `(event, context)` 少一个参数 →
        # `TypeError` → **`ros2 launch` 捕获异常后把整条 launch 拆掉**（所有 ROS 节点被 SIGINT，
        # 而 Gazebo 忽略 SIGINT 继续活着）。
        # 现象就是我这几轮反复看到的"只有相机链路没了、别的都在"——
        # 而且因为异常正好抛在"该起 nav2"那一刻，看起来还像"nav2 没激活"这个独立问题。
        # `OnActionEventBase.handle()` 在 `on_exit` 直接给 callable 时会带 `(event, context)` 调它，
        # 所以**直接把函数交出去**就是对的（签名本来就是 `(event, context)`）。
        # 这个 bug 由 reviewer 从 `rev4-hl` 的产物里读出来并读了 launch 源码定位（2026-09-20）。
        RegisterEventHandler(OnProcessExit(
            target_action=tf_gate_proc,
            on_exit=_nav2_after_gate)),
        route_follower,
        goal_navigator,
        trajectory_logger,
        rviz,
    ])

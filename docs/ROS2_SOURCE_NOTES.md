# ROS 2 Humble / Nav2 源码阅读笔记

本地 `ros2_src/` 包含 20 个 ROS 2 Humble 上游浅克隆，`nav2/` 是 Navigation2 的 Humble 浅克隆。这些目录由各上游项目维护，不随 OutdoorNav 仓库发布。下面的路径用于已有本地源码时阅读。

## 通信栈

1. `ros2_src/ros2/rcutils/src/logging.c`：底层日志工具。
2. `ros2_src/ros2/rcl/rcl/src/rcl/node.c`：C 层节点初始化。
3. `ros2_src/ros2/rcl/rcl/src/rcl/publisher.c`：C 层发布者。
4. `ros2_src/ros2/rclcpp/rclcpp/src/rclcpp/node.cpp`：C++ 客户端封装。
5. `ros2_src/ros2/rmw/rmw/include/rmw/rmw.h`：中间件抽象接口。
6. `ros2_src/ros2/rmw_fastrtps/rmw_fastrtps_cpp/src/rmw_node.cpp`：Fast DDS 实现。

## 消息与坐标变换

- `ros2_src/ros2/rosidl/rosidl_runtime_c/src/string_functions.c`：消息运行时。
- `ros2_src/ros2/rcl_interfaces/`、`ros2_src/ros2/common_interfaces/`：标准接口定义。
- `ros2_src/ros2/geometry2/tf2/src/buffer_core.cpp`：变换树与插值。
- `ros2_src/ros2/geometry2/tf2_ros/`：ROS 坐标变换封装。

## Nav2 导航栈

1. `nav2/nav2_bringup/launch/bringup_launch.py`：系统启动入口。
2. `nav2/nav2_bringup/params/nav2_params.yaml`：节点参数。
3. `nav2/nav2_util/src/lifecycle_node.cpp`：生命周期节点基类。
4. `nav2/nav2_bt_navigator/src/bt_navigator.cpp`：行为树调度。
5. `nav2/nav2_planner/src/planner_server.cpp`：全局路径规划。
6. `nav2/nav2_controller/src/controller_server.cpp`：局部轨迹跟踪。
7. `nav2/nav2_costmap_2d/src/costmap_2d_ros.cpp`：代价地图。

Ubuntu 24.04 不是 ROS 2 Humble 的官方目标系统；若需要构建 Humble，请使用支持的 Ubuntu 22.04 环境或官方容器。

# 智慧谷北区 Gazebo 三维园区

基于 `survey/north_building/` 的原始轨迹、滤波轨迹和米制路网生成场景。道路与楼前广场可以自由导航；主楼、裙楼、花坛和树干具有碰撞体。原始日志和旧网页演示不被改写。

## 启动

环境：Ubuntu 22.04 / WSLg、ROS2 Humble、Gazebo Classic 11，安装 `gazebo_ros_pkgs`、`navigation2`、`nav2_bringup`、`robot_state_publisher` 、`rviz2` 和 `ros-humble-rmw-cyclonedds-cpp`。当前工作环境已具备这些组件。模型和地图已随仓库提供，无需在线下载 Gazebo 模型，也无需构建 ROS 工作区。

在仓库根目录运行：

```bash
bash gazebo_campus/run.sh
```

同时打开 Gazebo 和 RViz，两个窗口均显示三维建筑、道路、广场和机器人。Gazebo 中拖动旋转、滚轮缩放；RViz 默认使用三维 Orbit 视角，使用 **Move Camera** 拖动旋转、滚轮缩放，使用 **Nav2 Goal** 在地面道路或广场上按住鼠标拖出目标朝向。机器人从实测环线起点出发，收到新目标后在地面自由空间规划，Gazebo 显示实际运动。

这是三维场景与物理仿真，地面机器人使用二维代价地图规划；第一版不支持上楼、越过台阶或飞行。RViz 的 `3D campus` 图层与 Gazebo 直接共用几何模型。勾选 `Traversable map` 可显示黑白规划地图；需要检查遮挡处时，可以在 `3D campus → Namespaces` 关闭 `buildings`。Navigation 2 面板的 `Localization: inactive` 表示没有启动 AMCL，本项目使用 Gazebo 理想定位，并不表示机器人位置失效。

RViz 的 `Raw measured traces`、`Filtered traces` 和 `Original road network` 可分别开关；黄色为原始测量，青色为滤波结果，橙色为旧中心线路网。`Global costs` 可显示障碍膨胀范围。目标须落在可容纳机器人身体的自由区域。

自定义出生位置，单位米、弧度；选择道路或广场内的空地：

```bash
bash gazebo_campus/run.sh x:=49 y:=-70 yaw:=0.7
```

无窗口启动：

```bash
bash gazebo_campus/run.sh gui:=false rviz:=false
```

只查看三维窗口：`bash gazebo_campus/run.sh rviz:=false`。Ctrl+C 关闭本次启动的全部节点。请勿同时启动两份园区实例。

三种 Gazebo 相机预设：`python gazebo_campus/scripts/set_camera.py oblique`、`top`、`plaza`（这里的脚本只使用 Python 标准库）。

## 在路中央放置障碍物

仿真运行时，在另一个终端添加临时箱体：

```bash
# 默认放在东侧 8 米宽道路中央，尺寸 1.6 × 1.6 × 1 米
bash gazebo_campus/obstacle.sh add

# 自定义位置、尺寸及名称；单位均为米
bash gazebo_campus/obstacle.sh add --name plaza_box --x 61 --y -70 --width 1.2 --depth 1.2 --height 1.0

# 删除默认箱体或指定名称的箱体
bash gazebo_campus/obstacle.sh remove
bash gazebo_campus/obstacle.sh remove --name plaza_box
```

橙色实体同时出现在 Gazebo 和 RViz；激光发现它后更新导航代价地图并重新规划。它不预先写入静态地图，专门用于测试感知避障。箱体在仿真重启后消失；通过此工具创建的箱体若在 Gazebo 中移动，RViz 也会跟随其位置。避免把箱体直接生成在机器人身上；完全封路时导航可能返回不可达。

**基础园区的 RTK 保护带始终保持可走；临时测试障碍物允许有意占用原轨迹，让机器人验证绕行。** 永久调整建筑高度、道路和花坛请修改 `config/scene.json`，重新生成后重启仿真。主楼默认 90 米、裙楼默认 12 米。

## 场景生成与约束

`config/scene.json` 是米制布局配置，定义道路宽度、建筑高度、广场、障碍物和验收任务。与实测数据共同生成 `generated/` 下的世界文件、COLLADA 路面、碰撞体、PGM/YAML、轨迹和几何检查报告。坐标原点与现有实测地图一致，X 向东、Y 向北。

```bash
# 当前仓库没有 venv：首次安装生成工具依赖
conda run -n torch python -m pip install -r gazebo_campus/requirements.txt
bash gazebo_campus/build.sh
```

`build.sh` 优先使用仓库 `.venv` 或 `venv` 对应的 uv 环境，否则使用 conda `torch`。ROS 运行脚本固定使用 Humble 所需的系统 Python，不受 conda 激活状态影响。

- 环线 A / B 分别宽 6 / 8 米，支路和原始引入段宽 4 米；原始轨迹与滤波轨迹的包络并入道路。
- 实测轨迹的连续 0.85 米保护带必须位于可通行区，并远离全部实体障碍；实际生成检查略加裕量。不得移动实测点来迁就建筑。
- 碰撞矩形与保护带冲突时，生成器向矩形中心逐次收缩，并记录调整；无法安全放置则生成失败。
- 全部道路和广场共用平坦地面。草坪由静态地图禁行，没有不可见围墙。二维激光能检测实体障碍，草坪通行规则由静态地图保留。
- 0.1 米栅格按整格可通行原则保守生成，避免障碍边缘误标自由。三维地面网格保留道路环内部的孔洞。
- 主楼 90 米、裙楼 12 米，建筑和新增宽度为示意仿真假设；实测定位质量标记沿用原始报告，不宣称独立测量过建筑边界。

## 导航接口与通信

`NavigateToPose` 接收 `map` 坐标目标；NavFn A* 规划、DWB 控制，通过 `/cmd_vel` 驱动差速轮。最大指令线速度 0.45 米/秒，规划半径 0.5 米，实际机器人水平外接半径小于 0.5 米。

Gazebo P3D 提供世界位姿，`scene_node.py` 将速度变换到机体坐标，发布 `/odom`、动态 `odom → base_link` 和静态 `map → odom`；这是一种理想仿真定位。`robot_state_publisher` 发布车体到激光等固定 TF。全部节点使用 `/clock`。

Gazebo 原始激光为 `/campus/raw_scan`。场景节点等对应 TF 已发布 0.2 秒后，将原时间戳的扫描转发到 `/scan`，避免此 Humble 版本中激光异步 TF 过滤器的锁等待；超出 1 秒的旧扫描丢弃。地图为 `/map`，全局路径为 `/plan`，碰撞观测为 `/campus/contacts`。RViz 的机器人轮子仅显示外形，Gazebo 中的轮子采用真实转动关节。

启动脚本默认使用 ROS 域 **42**、Gazebo 端口 **11456**，并将 ROS 通信限定在本机回环 UDP。项目使用 Cyclone DDS 的回环 UDP 配置，隔离局域网 ROS 节点；控制器 TF 停更的实际线程检查结果及扫描缓冲处理见验证报告。`GAZEBO_IP` 和 `IGN_IP` 同时设为 `127.0.0.1`，避免 WSL 多接口下 Gazebo 渲染服务超时而黑屏。旧网页桥接仍使用其原启动入口，第一版不将自由导航接到网页 HTTP API。

其他 ROS 终端需要同样环境：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/gazebo_campus/config/cyclonedds.xml"
ros2 topic list --no-daemon
```

可在启动前设置 `ROS_DOMAIN_ID` 和 `GAZEBO_MASTER_URI` 更换域和端口，所有测试终端需使用相同设置。Gazebo 和 RViz 必须具备可工作的 WSLg 图形环境；无界面模式可用于导航自动化。

## 验证

```bash
conda run -n torch python -m unittest discover -s gazebo_campus/tests -v
```

仿真运行后，在另一个终端执行下面命令。它会在各案例之间重置仿真机器人，依次执行环线路段、支路进环线、道路进广场、广场绕花坛，以及两个不可达目标；运行时请勿同时手动发送目标。

```bash
bash gazebo_campus/check.sh
conda run -n torch python gazebo_campus/scripts/validate_navigation.py
```

实时测试用系统 ROS Python，几何后验检查用生成环境。测试记录真实 Gazebo 里程计、规划路径、碰撞事件、激光接收量和动作结果；后验检查实际连续运动是否留在可通行区域、是否保持机器人半径净空，以及广场任务是否真正离开旧中心线并绕开花坛。失败会返回非零退出码。

单个案例：`bash gazebo_campus/check.sh --case plaza_diagonal --output /tmp/plaza.json`。完整结果保存在 `evidence/`；场景修改后应重新生成并重跑验收，不能沿用其他场景版本的结果。

动态路障验收：先 `bash gazebo_campus/obstacle.sh add`，再运行 `bash gazebo_campus/check.sh --case road_obstacle --output gazebo_campus/evidence/road_obstacle.json`，最后用 `validate_navigation.py` 检查该 JSON。它验证实际运动绕开临时箱体，且不驶出道路。详细结果和实际窗口截图见 [验证报告](VALIDATION.md)。

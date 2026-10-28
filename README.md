# OutdoorNav 室外导航

从实测 UBX 轨迹还原校园路网，在 Gazebo Fortress 三维场景中模拟小车、GNSS、IMU、雷达和深度相机。网页选择目标后，后端沿路网选路，Nav2 使用传感器代价地图，沿道路参考路线分段规划、控制和绕障。

山猫实机使用同一套路网执行器，部署和现场验收见 [山猫部署说明](deploy/lynx/README.md)；实施状态见 [计划](docs/plan.md)。默认关闭实机运动。

## 环境

Ubuntu 22.04 原生 Linux 或 WSL2，ROS 2 Humble、Gazebo Fortress、Node.js ≥22.12。WSL2 需要可用的 WSLg 显示环境。

先按 ROS/Gazebo 官方说明安装 Humble 与 Fortress，再安装所需 ROS 包：

```bash
sudo apt install python3-colcon-common-extensions mesa-utils \
  ros-humble-ros-gz ros-humble-navigation2 ros-humble-robot-localization \
  ros-humble-turtlebot3-description ros-humble-xacro ros-humble-laser-geometry \
  ros-humble-sensor-msgs-py ros-humble-rviz2
```

轨迹与场景处理优先使用仓库 uv venv，否则使用 conda `torch` 环境。新建环境示例：

```bash
uv venv --python python3.10
uv pip install --python .venv/bin/python -e .
```

ROS 节点始终使用 Humble 对应的系统 Python。启动程序不会安装依赖或修改驱动。

## 启动与使用

在仓库根目录执行，两种平台命令相同：

```bash
./nav doctor
./nav up
```

等待 `OutdoorNav is ready`，打开 **http://localhost:5174/**：

1. 选择 **A→B 规划**；在线时 A 自动使用小车当前位置。
2. 在道路上设置终点，点击 **规划 A→B**。
3. 点击 **执行导航**，在 Gazebo 中观察真实物理运动。

绿色及圆点为路网参考路线与途经点，蓝色虚线为 Nav2 当前滚动执行路径。路网用于选路与终点选择，不作为代价地图的静态先验；未观测区域保持未知，由雷达和深度相机逐步观测。执行时保留道路弯折，中间引导点不逐个停车。默认 RViz 同时显示 RGB 与 Depth。可以取消任务或替换目标。**离线动画演示只播放网页动画**。

到达后只停车，Gazebo、RViz 和网页保持开启，可继续设置下一个目标。任务失败或导航暂时不可用时也保留界面和失败原因；未恢复就绪之前不接受新的导航任务。

只有主动按 `Ctrl+C` 或在另一个终端运行 `./nav down` 才结束正常运行的整套演示。

行驶中可以在右侧 **场景障碍物** 面板选择静止或移动障碍物：点击“选障碍物位置”后在地图上选点；移动障碍物再选移动终点、设置速度；点击“添加到 Gazebo”。橙色方块是真实场景障碍，移动障碍物在两点之间往返，可随时删除。小车通过传感器感知障碍并重新规划；传感器地图无法找到可行路径时明确失败。

## 常用命令

| 命令 | 用途 |
| --- | --- |
| `./nav up` | 默认同时起 RViz（看传感器、地图和路径）|
| `./nav up --no-rviz` | 不起 RViz，只起仿真与前端 |
| `./nav up --render software` | 使用软件渲染；默认 auto 会检测并尝试回退 |
| `./nav build` | 显式构建；up 也会按源码与依赖指纹检查重建 |
| `./nav status` / `./nav logs navigation` | 查看健康、任务和日志 |
| `./nav survey --loop 1.ubx --branch 2.ubx` | 重新处理实测轨迹 |
| `./nav scene build` | 根据处理结果生成统一场景包 |
| `./nav obstacle add --x 1 --y -5 --width 0.6 --depth 0.6` | 添加临时静态障碍 |
| `./nav obstacle remove` | 删除该障碍 |
| `./nav obstacle add --kind moving --x 3 --y -10 --end-x 4 --end-y -10 --speed 0.2` | 添加两点往返的移动障碍 |
| `./nav capture --seconds 60` | 同步采集栅格、点云与 TF 供诊断 |
| `./nav diagnose sticky-history --help` | 查看离线诊断参数 |
| `./nav validate --scenario short` | 执行真实运动验收并生成独立真值报告 |
| `./nav record` | 记录 ROS bag；Ctrl+C 结束记录 |

运行日志、实际参数、进程身份与真值/估计轨迹统一写入 `artifacts/runs/<run_id>/`。

[架构](docs/ARCHITECTURE.md) · [实测数据](docs/DATA.md) · [验证记录](docs/VALIDATION.md) · [相机网页](docs/camera-view.md)

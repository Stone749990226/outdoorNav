# OutdoorNav 室外导航仿真

从实测 UBX 轨迹还原校园路网，在 Gazebo Fortress 三维场景中模拟小车、GNSS、IMU、雷达和深度相机。网页选择目标后，后端沿路网选路，Nav2 在允许道路内规划、控制和绕障。

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

绿色为后端参考路线，蓝色为 Nav2 执行路径。可以取消任务或替换目标。**离线动画演示只播放网页动画**。

用 `Ctrl+C` 或另一个终端的 `./nav down` 停止整套系统。

行驶中可以在右侧 **场景障碍物** 面板选择静止或移动障碍物：点击“选障碍物位置”后在地图上选点；移动障碍物再选移动终点、设置速度；点击“添加到 Gazebo”。橙色方块是真实场景障碍，移动障碍物在两点之间往返，可随时删除。小车通过传感器感知、在允许道路内绕行，并重新接回参考路线；道路完全堵塞时明确失败。

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

[架构](docs/ARCHITECTURE.md) · [实测数据](docs/DATA.md) · [验证记录](docs/VALIDATION.md)

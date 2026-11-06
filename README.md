# OutdoorNav 室外导航

在山猫四足机器人上适配室外导航：RTK + WIT IMU 定位，沿实测校园路网分段规划，雷达避障，网页选点导航。

同一套路网执行器有两种运行方式：

- **山猫实机**（当前重点）：真车、真传感器、真运动，见 [山猫部署说明](deploy/lynx/README.md)
- **Gazebo 仿真**（开发调试）：小车、GNSS、IMU、雷达、深度相机，一键起整套演示

> 注意：室外导航**不使用山猫自带的 `/ODOM` 里程计**（经常失效），定位靠 RTK 位置 + WIT IMU 航向/角速度。

## 山猫实机快速开始

连接机器人（内网穿透）：

```bash
ssh -p 6003 nvidia@svr.gddg08.top
```
山猫WiFi切换命令：
```bash
sudo nmcli connection up ZTE_B3000D
nmcli connection show --active
```

在机器人上的仓库目录里：

```bash
# 真实导航（网页端口 8765，真实速度指令经桥转发到底盘）
bash deploy/lynx/fieldtest/start_production.sh

# 室内模拟（虚拟底盘 + 合成 RTK，网页端口 8766，不产生真实运动）
bash start_mock.sh
curl -s -X POST -H 'Content-Type: application/json' -d '{"active":true}' http://127.0.0.1:8766/api/activation

# 只读诊断（拒绝导航任务，速度只发到诊断话题）
bash deploy/lynx/start.sh --diagnostic
```

打开网页 `http://<机器人地址>:8765/`，现场档可选择地图范围内的路外终点：参考规划从道路中心低代价连续增加至路边／路外代价，网页滑条可调，先规划再执行；通用配置未启用此策略时仍使用原路网约束。网页可直接切换只读代价图（局部/全局图 + 车位姿 + 执行路径），无需 RViz，见 [网页代价地图](docs/costmap-web.md)；远程看车状态用 [相机网页](docs/camera-view.md)。

**让车动起来的前提**（缺一车就不会走）：

1. 底盘站起且处于 RL 控制（`/MOTION_STATE` 先 `state 1` 再 `state 17`）
2. 导航步态（`/GAIT` 0x3002）与使用模式 `usage_mode=1`
3. 网页上所有健康项变绿（RTK 固定/浮点解、IMU 航向就绪、雷达数据新鲜、服务已激活）

现场档配置为 `deploy/lynx/robot.fieldtest.json`（默认 `motion_enabled` 放行）；通用配置 `deploy/lynx/robot.json` 保持严格默认（未确认项全关，车不会动）。出门前的限制审查见 [室外准备检查](docs/outdoor-readiness-audit-2026-10-29.md)，10 月 28 日实机复盘见 [调试总结](docs/fieldtest-postmortem-2026-10-28.md) 与 [修复记录](docs/fieldtest-fixes-2026-10-28.md)。

## Gazebo 仿真

Ubuntu 22.04 或 WSL2，ROS 2 Humble、Gazebo Fortress、Node.js ≥22.12。先按官方说明安装 Humble 与 Fortress，再安装所需 ROS 包：

```bash
sudo apt install python3-colcon-common-extensions mesa-utils \
  ros-humble-ros-gz ros-humble-navigation2 ros-humble-robot-localization \
  ros-humble-turtlebot3-description ros-humble-xacro ros-humble-laser-geometry \
  ros-humble-sensor-msgs-py ros-humble-rviz2
```

在仓库根目录：

```bash
./nav up          # 一键构建并启动仿真 + RViz + 前端
./nav down        # 结束整套演示
```

打开 **http://localhost:5174/**：

1. 选择 **A→B 规划**；在线时 A 自动使用小车当前位置
2. 在道路上设置终点，点击 **规划 A→B**
3. 点击 **执行导航**，在 Gazebo 中观察真实物理运动

绿色及圆点为路网参考路线与途经点，蓝色虚线为 Nav2 当前滚动执行路径。可以取消任务或替换目标；任务失败时保留界面和失败原因，未恢复就绪前不接受新任务。行驶中可在右侧 **场景障碍物** 面板添加静止或移动障碍物，小车通过传感器感知并重新规划。

常用命令：`./nav status` 看健康与任务，`./nav logs navigation` 看日志，`./nav survey` 重新处理实测轨迹，`./nav scene build` 生成场景包，`./nav capture` 采集诊断数据，`./nav validate` 跑真实运动验收，`./nav record` 录 ROS bag。运行日志与真值/估计轨迹统一写入 `artifacts/runs/<run_id>/`。

## 测试

```bash
.venv/bin/python -m pytest tests/     # 离线单元/回归测试（ROS 相关用例自动跳过）
tests/lynx_ros_smoke.py               # 隔离 ROS 域联调（需 source ROS 与 .ros-install）
tests/isolated_task_flow.py --rtk-imu # 合成 RTK+IMU 的完整网页→任务→Nav2 流程
```

## 文档

[架构](docs/ARCHITECTURE.md) · [实测数据](docs/DATA.md) · [验证记录](docs/VALIDATION.md) · [实施计划](docs/plan.md) · [山猫部署](deploy/lynx/README.md) · [相机网页](docs/camera-view.md) · [网页代价地图](docs/costmap-web.md) · [出门前检查](docs/outdoor-readiness-audit-2026-10-29.md) · [前雷达实测](docs/lidar-field-notes.md) · [主路优先与路外终点](docs/road-preference.md)

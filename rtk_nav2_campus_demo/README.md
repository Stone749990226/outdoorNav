# Robonix 二维 SLAM 演示

**WSL + RTK/Nav2 校园演示**：使用 [WSL 原生启动说明](docs/wsl-native.md)。在仓库根目录分别运行 `bash rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-start.sh` 和 `bash rtk_nav2_campus_demo/scripts/rtk-nav2/wsl-frontend.sh`，打开 `http://localhost:5174`。启动脚本会检测 Ogre2 所需的 OpenGL 扩展，不兼容时自动使用软件渲染。下文的 `scripts/start-demo.sh` 是另一套二维 SLAM 的 Docker 入口。

在 Ubuntu 图形桌面（远程桌面会话）中同时显示 **Gazebo 场景** 与 **RViz 地图**，
用 TurtleBot3 burger 的二维激光雷达 + 里程计跑 **slam_toolbox** 在线建图，
支持键盘遥控、固定路线自动巡游、安全停车、保存地图与重新加载。

- 需求：`docs/superpowers/specs/2026-09-17-2d-slam-demo.md`
- 实施计划：`docs/superpowers/plans/2026-09-17-2d-slam-demo.md`
- 环境与版本：`docs/environment.md`
- 自测记录与误差报告：`docs/validation.md`
- 独立验收记录：`docs/review.md`（reviewer 维护）

**第一版范围**：不接入 Go2、深度相机、高德或 Robonix API；不做三维建图、Nav2 点到点导航与自主探索。
巡游脚本是固定路线 + 里程计闭环控制 + 近障停车，**不是**自主导航或绕障规划。

> **另有两套独立演示**（入口与本文的激光版完全分开：各自独立容器、工作区、ROS domain、
> Gazebo 分区、产物目录，互不影响）：
>
> - **RTK 版（当前主线）**：模拟 RTK（+IMU）提供**绝对定位**，深度相机只出**代价地图**、
>   不参与定位；路线跟随读绝对位姿 + 代价地图。见
>   [`docs/rtk-demo.md`](docs/rtk-demo.md)（使用说明）、
>   [`docs/rtk-environment.md`](docs/rtk-environment.md)（环境、版本与隔离、
>   **模拟 RTK 不代表真实精度**）、[`docs/rtk-validation.md`](docs/rtk-validation.md)（自测与误差）。
> - **RGB-D 版（已按用户指示停止维护）**：虚拟深度相机 + 视觉里程计 + RTAB-Map 建二维图。
>   源码仍在（`rgbd_ws/`、`scripts/rgbd/`），产物已清理，见
>   [`docs/rgbd-validation.md`](docs/rgbd-validation.md) 开头的说明。

---

## 1. 一分钟理解这套演示

```
        Gazebo Classic 11                    RViz2
   ┌───────────────────────┐          ┌───────────────────────┐
   │  courtyard.sdf 场地    │          │  Fixed Frame = map    │
   │  TurtleBot3 burger     │          │  Map / LaserScan /    │
   │  ├ 二维激光 /scan      │          │  RobotModel / TF /    │
   │  ├ 轮式里程计 /odom    │          │  Odometry / PatrolPath│
   │  └ 仿真时钟 /clock     │          └───────────────────────┘
   └───────────┬───────────┘                      ▲
               │                                  │ /map, map→odom
               ▼                                  │
        ┌──────────────────────────────────────────┴──┐
        │              slam_toolbox（在线异步建图）      │
        │   消费 /scan + TF + /clock → 发布 /map        │
        └──────────────────────────────────────────────┘
```

数据流：Gazebo 产生激光、里程计与时钟 → slam_toolbox 估计位姿并输出占据栅格地图 →
RViz 把地图、机器人、激光点与巡游轨迹画在同一坐标系里。
巡游节点只读 `/odom` 和 `/scan`，发布 `/cmd_vel`；它**不使用** `map` 坐标，因此 SLAM 的回环修正不会干扰运动控制。

### 窗口说明

启动后远程桌面上会有两个窗口，左右各占半屏：

| 窗口 | 内容 |
|------|------|
| 左侧 **Gazebo** | 3D 场景：10×10 m 场地、外框墙、中央方块、两道隔墙形成的通道、三根固定立柱，以及 TurtleBot3 小车。小车顶部的激光传感器默认开启射线可视化。 |
| 右侧 **RViz2** | 俯视地图视图：占据栅格地图（灰=未知、白=空闲、黑=障碍）、红色激光点、机器人模型与 TF 坐标轴、里程计轨迹、巡游路线与当前目标点。 |

RViz 左侧 Displays 面板里可以逐项确认数据是否正常：
`Map → Status: Ok`（地图）、`LaserScan → Status: Ok`（激光）、`TF`（坐标系树）。

---

## 2. 环境与前置条件

这套演示在**两台不同的机器**上都跑通了，脚本会按机器自动选择对应实现：

| | 6001 | 6002 |
|---|---|---|
| 入口 | `ssh -p 6001 xjy@svr.gddg08.top` | `ssh -p 6002 xjy@svr.gddg08.top` |
| 架构 / 系统 | x86_64 / Ubuntu 22.04 | **aarch64（Jetson Thor）** / Ubuntu 24.04（容器内仍是 22.04） |
| 仿真器 | Gazebo Classic 11 | **Gazebo Sim（Fortress）** |
| 容器镜像 | `robonix/slam-demo:humble` | `robonix/slam-demo:humble-arm64` |
| 图形会话 | xrdp + Xorg | **xrdp + Xorg**（原为 gnome-remote-desktop，2026-09-18 已切换，见 `docs/environment-arm64-6002.md`） |

两台机器**不同**的只有仿真器后端与图形会话的处理方式；ROS 2 Humble、slam_toolbox、RViz、
二维激光参数、场地几何、巡游与安全逻辑、四个入口、产物结构与误差评估方法完全相同。

为什么 6002 不用 Gazebo Classic：aarch64 上根本没有它的二进制包；
详情与两台机器各自的实测记录见 `docs/environment.md`（索引）、
`docs/environment-x86_64-6001.md`、`docs/environment-arm64-6002.md`。

### 关于 6001（下表以它为例）

| 项目 | 说明 |
|------|------|
| 服务器 | `ssh -p 6001 xjy@svr.gddg08.top`（Ubuntu 22.04 LTS） |
| 远程项目路径 | `/home/xjy/projects/robonix`（容器内挂载为 `/work`） |
| 本地项目路径 | `C:\Users\xjy\projects\robonix`（WSL：`/mnt/c/Users/xjy/projects/robonix`） |
| 图形显示 | 两台都是 xrdp + Xorg（6001 `:10`，6002 也是 `:10`）。脚本自动探测显示号，可用 `DEMO_DISPLAY` 覆盖 |
| 截图方式 | 两台都是 `ffmpeg -f x11grab`（脚本仍保留 Wayland 下用 `gnome-screenshot` 的分支，但当前用不到） |
| 运行方式 | 全部进程跑在专用容器 `robonix-slam-demo` 内；**宿主机不安装 ROS** |
| 依赖 | Docker（xjy 已在 `docker` 组，无需 sudo） |

版本组合、安装来源与渲染性能记录见 `docs/environment.md`。

> 说明：宿主机 `sudo` 需要密码，因此本项目不改动宿主机系统，全部依赖装在容器镜像里，
> 只挂载本项目目录、X11 socket 与 `.Xauthority`。这同时保证 `stop-demo.sh`
> 不可能影响服务器上其他项目。

---

## 3. 首次准备

```bash
cd ~/projects/robonix

# 1) 构建容器镜像（基础镜像 robonix-osrf-ros:humble-desktop-full 需已在服务器上）
scripts/build-image.sh

# 2) 之后每次演示只需下面四个入口；scripts/start-demo.sh 会自动编译工作区
```

---

## 4. 四个操作入口

### 4.1 启动演示

```bash
scripts/start-demo.sh            # run-id 自动取时间戳
scripts/start-demo.sh myrun      # 指定 run-id
```

做的事情：确认远程桌面显示存在 → 启动/复用容器 → 干净重编工作区（`build/`、`install/`、`__pycache__` 清空后 `colcon build`，约 1~2 秒）→
启动 Gazebo + RViz + slam_toolbox + 轨迹记录 + rosbag → 等待关键话题就绪 →
把 Gazebo/RViz 左右排开 → 抓启动截图。

如果远程桌面会话处于锁定状态，脚本会先调用 `loginctl unlock-session` 解锁
**当前用户自己的会话**（这只是让窗口可见，不修改 X 访问控制）。

### 4.2 键盘遥控

```bash
scripts/teleop.sh
```

在能接收键盘输入的终端里运行。按键：`i` 前进、`,` 后退、`j` 左转、`l` 右转、`k` 停止、
`q`/`z` 增减速度。速度被限制在 0.15 m/s、0.8 rad/s，低速验证模型与传感器。

### 4.3 自动巡游

```bash
scripts/run-patrol.sh myrun
```

机器人沿 `config/patrol.yaml` 里的 4 个 odom 坐标路线点走一圈并回到起点，
用于观察地图增长与回环闭合。退出码：

| 退出码 | 含义 |
|--------|------|
| 0 | 正常走完全部路线点并停车 |
| 3 | 激光数据缺失或过期，停车中止 |
| 4 | 里程计数据缺失或过期，停车中止 |
| 5 | 前方 25° 扇区内出现小于阈值的障碍，停车中止 |
| 6 | 超过任务总时限，停车中止 |
| 130 | 被 Ctrl-C 中断 |

### 4.4 保存地图与产物

```bash
scripts/save-map.sh myrun
```

保存到 `artifacts/<run-id>/`：

```
map/map.yaml, map/map.pgm          导航栅格地图（nav2_map_server 格式）
map/slam_posegraph.posegraph       slam_toolbox 内部序列化数据
map/slam_posegraph.data
map/map_metrics.json               分辨率/原点/尺寸/占据比例 + 尺度检查
bag/run/                           rosbag（clock/scan/odom/tf/map/cmd_vel/patrol_path/真值）
trajectory.csv                     同步记录的真值轨迹与 SLAM 估计轨迹
patrol_report.json                 巡游结果（到达点、里程、退出码）
logs/                              各进程日志
screenshots/                       各阶段界面截图
```

每次运行一个独立目录，不覆盖历史结果。

### 4.5 停止

```bash
scripts/stop-demo.sh             # 只停容器内本项目进程
scripts/stop-demo.sh --remove    # 顺带删除容器（产物在宿主机目录，不受影响）
```

### 4.6 传感器与运动链路自检

```bash
docker exec robonix-slam-demo bash -lc \
  "bash /work/scripts/container/verify_sensors.sh <run-id> 0.15"
```

统计 `/scan` 的测距分布、对照 `/odom` 与真值的位移，并以 0.15 m/s 驱动一小段验证
“速度指令 → 底盘运动 → 里程计变化”。输出写到 `artifacts/<run-id>/verification/`。

### 4.7 验证保存的地图可以重新加载

```bash
scripts/stop-demo.sh
scripts/load-map.sh myrun
```

用 `nav2_map_server` 在干净进程里加载 `map.yaml`（不依赖 slam_toolbox 状态），
打印 `/map` 元数据并截图，与保存时的元数据对照。

---

## 5. 安全行为与参数依据

`config/patrol.yaml` 里每个阈值都有选择依据，摘要如下：

| 参数 | 取值 | 依据 |
|------|------|------|
| `max_linear_speed` | 0.18 m/s | TurtleBot3 burger 官方最大线速度 0.22 m/s，留出余量降低建图抖动 |
| `max_angular_speed` | 1.0 rad/s | 与线速度匹配，避免原地打转导致激光匹配退化 |
| `goal_tolerance` | 0.12 m | 约 2.4 个栅格（分辨率 0.05 m） |
| `obstacle_stop_distance` | 0.35 m | 约为车身半径（0.105 m）的 3 倍；0.18 m/s 下约 1.9 s 反应时间 |
| `forward_sector_deg` | 25° | 略宽于车身宽度对应的张角，只看正前方不误判侧向墙壁 |
| `scan_timeout` | 1.0 s | 仿真激光 5 Hz（0.2 s/帧），相当于连续漏 5 帧才判失效 |
| `odom_timeout` | 0.5 s | 里程计 30 Hz，相当于漏 15 帧 |
| `total_timeout` | 300 s | 路线周长约 24 m，0.18 m/s 约需 133 s，留 2 倍余量 |
| `startup_grace` | 30 s | 节点刚启动、DDS 还在做发现时收不到第一帧属于正常现象，30 s 内保持零速度等待，超过才判失效 |
| `clock_jump_threshold` | 2.0 s | 控制回路 20 Hz（tick 间隔 0.05 s）；相邻 tick 的仿真时间差超过该值即判定为时钟跳变，重置新鲜度基准 |

安全判定顺序固定为 **激光 → 里程计 → 障碍 → 总超时**，任一条件触发即发送零速度并中止。

行为测试与异常场景：

```bash
python3 -m pytest tests/test_patrol_safety.py -v     # 34 条纯逻辑用例（含启动竞态回归）
scripts/test-patrol-behavior.sh <run-id>             # 在真实仿真里跑四类异常场景
scripts/test-startup-race.sh <run-id> 20             # 启动竞态回归：连续启动 20 次不得误报失效
                                                     # 同时跑一组绕过时钟守卫的负对照，证明用例能检出该竞态
```

### 实测结果（2026-09-17，完整记录见 `docs/validation.md`）

| 项目 | 实测 |
|------|------|
| 自动巡游 | 退出码 0，4/4 路线点，里程 23.6~23.8 m，用时 137.7~145.9 s |
| 启动竞态回归 | 带时钟守卫 40 次启动 0 次误报；负对照（绕过守卫）40 次里 8 次误报 |
| 位置误差（Umeyama 刚体对齐后，巡游窗口内） | run-a RMSE 0.098 m / max 0.153 m；run-c RMSE 0.069 m / max 0.097 m |
| 朝向误差 RMSE | 1.36° / 0.78° |
| 地图尺度（空闲外包 vs 9.85 m） | 误差 ≤ 0.15 m（3 个栅格） |
| 闭环残差 | 0.017 m / 0.007 m |
| 地图重新加载 | 分辨率 / 尺寸 / 原点与保存时完全一致 |
| 异常停车 | 激光丢失→3、里程计丢失→4、遇障→5、超时→6，全部符合预期 |
| 渲染 | llvmpipe 软件渲染，Gazebo Real Time Factor 1.00，界面 25~32 fps |

---

## 6. 故障排查

| 现象 | 原因与处理 |
|------|-----------|
| `显示 :10 不存在` | 没有活动的 xrdp 会话。先连接一次远程桌面；或用 `DEMO_DISPLAY=:N` 指定实际显示号（`ls /tmp/.X11-unix/` 查看） |
| 启动后 RViz 里 Map 一直是 `No map received` | slam_toolbox 还没收到第一帧激光。确认 `/scan` 有数据（`ros2 topic hz /scan`），再看 `artifacts/<run>/logs/demo_launch.log` |
| `patrol` 退出码 3/4 | 激光或里程计话题名与 `config/patrol.yaml` 不一致，或仿真未启动。用 `ros2 topic list` 核对 |
| 退出码 5 | 路线上确实有障碍；确认场景中没有残留上一次测试用的临时障碍（重新启动演示即可清掉） |
| Gazebo 窗口卡顿 | 容器内使用软件渲染（llvmpipe）。可以降低 Gazebo 窗口尺寸或关闭射线可视化；本演示场景很小，实测帧率见 `docs/environment.md` |
| 容器起不来 | `docker rm -f robonix-slam-demo` 后重跑 `scripts/start-demo.sh` |
| 启动即报退出码 3 / 4（误报传感器失效） | 已由 `ClockGuard` 修复（见 `docs/validation.md` 第 8 节）；若仍出现，用 `scripts/test-startup-race.sh <run-id> 20` 复现并附日志 |
| 改了代码但行为没变 | `scripts/start-demo.sh` 每次都会清掉 `build/ install/ __pycache__` 后重新 `colcon build`，正常不会出现；若手工改过源码，重跑一次 `start-demo.sh` 即可 |
| 截图全黑但窗口在 `xwininfo` 里能看到 | GNOME 会话被锁屏。脚本会在截图前自动 `loginctl unlock-session`；需要**实时观看**时请先在远程桌面里操作一下 |

诊断命令（在容器内执行）：

```bash
docker exec -it robonix-slam-demo bash -lc '
  source /opt/ros/humble/setup.bash && source /work/demo_ws/install/setup.bash
  ros2 topic list
  ros2 topic hz /scan
  ros2 run tf2_ros tf2_echo map base_footprint'
```

---

## 7. 目录结构

```
robonix/
├── README.md                     本文件
├── agent.md / AGENTS.md / CLAUDE.md   项目与智能体协作规则
├── docker/Dockerfile             演示容器镜像
├── demo_ws/src/robonix_slam_demo/
│   ├── package.xml, setup.py, setup.cfg
│   ├── launch/demo.launch.py         总启动（Gazebo+RViz+slam_toolbox）
│   ├── launch/load_map.launch.py     加载已保存地图（干净进程）
│   ├── config/slam.yaml              slam_toolbox 参数
│   ├── config/patrol.yaml            巡游参数与阈值依据
│   ├── config/demo.rviz              RViz 配置（fixed frame = map）
│   ├── config/map_view.rviz          只显示地图的 RViz 配置
│   ├── worlds/courtyard.sdf          10×10 m 场地
│   ├── worlds/test_obstacle.sdf      异常演示用的临时障碍
│   ├── models/burger/model.sdf       TurtleBot3 burger 官方模型（含真值插件）
│   ├── scripts/patrol.py             巡游节点的直接入口
│   └── robonix_slam_demo/
│       ├── patrol.py                 巡游 ROS 节点
│       ├── patrol_core.py            与 ROS 无关的控制/安全纯逻辑（便于测试）
│       ├── trajectory_logger.py      真值与估计轨迹的同步记录
│       ├── trajectory_error.py       离线对齐与误差评估
│       └── map_metrics.py            地图元数据与尺度检查
├── scripts/                      四个入口 + 辅助脚本
│   ├── start-demo.sh / run-patrol.sh / save-map.sh / stop-demo.sh   四个入口
│   ├── teleop.sh                 键盘遥控
│   ├── load-map.sh               在干净进程中加载已保存地图
│   ├── build-image.sh            构建容器镜像
│   ├── test-patrol-behavior.sh   四类异常场景验证
│   ├── lib.sh                    公共函数
│   └── container/                容器内执行的脚本
├── tests/test_patrol_safety.py   运动控制安全行为测试
├── artifacts/<run-id>/           每次运行的产物
└── docs/                         环境、自测、验收文档
```

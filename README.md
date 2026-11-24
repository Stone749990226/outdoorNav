# OutdoorNav：山猫室外导航

本工作区把室外定位、道路采集和导航接入 Robonix。整机部署源码统一在
`robot-deep_robotics-lynx_s10/outdoor/`，使用原生 `rbnx build / boot / shutdown`。

定位使用 G70 RTK＋WIT IMU，不使用山猫本体 `/ODOM`。Scene 提供道路录制、
地图保存、路线预览、执行和取消；导航服务负责跟随路线和雷达避障。

## 从哪里开始

- [系统分工](docs/ARCHITECTURE.md)：原语、服务、部署配置分别做什么。
- [部署和使用](docs/DEPLOYMENT.md)：准备依赖、启动、停止、建图和现场参数。
- [验证和现场复验](docs/VALIDATION.md)：离线检查与实机待办。
- [室内模拟联调](simulation/README.md)：mock RTK、IMU 和底盘，运行真实 Scene、定位和导航服务。
- [相机查看工具](robot-deep_robotics-lynx_s10/tools/camera_view/README.md)：通过 SSH 隧道实时查看山猫四路画面。

## 山猫仿真

山猫 S10 已开展两类仿真。Isaac Sim 用简化底盘测试到点、绕障、堵塞停车和任务取消。MuJoCo 用 S10 关节模型测试坡道、楼梯和移动障碍。两套仿真都复用 Robonix 的地图、定位和导航服务，由虚拟底盘和传感器接入，记录轨迹、碰撞和到点误差。

## 源码位置

| 目录 | 职责 |
|---|---|
| `robot-deep_robotics-lynx_s10/outdoor` | 室外部署清单和现场配置 |
| `robot-deep_robotics-lynx_s10/primitives/primitive-lynx_chassis` | 底盘消息转换、通信隔离和反馈保护 |
| `robot-deep_robotics-lynx_s10/primitives/primitive-lynx_lidar` | 雷达驱动和点云接口 |
| `robot-deep_robotics-lynx_s10/primitives/primitive-lynx_camera` | 山猫相机原语唯一维护位置，包含实现、测试和说明 |
| `primitive-g70-gnss-rbnx`、`primitive-wit-imu-rbnx` | 外接传感器原语 |
| `service-localization-outdoor-rbnx` | RTK＋IMU 融合定位 |
| `service-map-outdoor-rbnx` | 道路录制、补采、地图保存和路线规划 |
| `service-navigation-outdoor-rbnx` | 路线执行、避障、停车和步态切换流程 |
| `robonix/system/scene` | 操作网页，调用地图与导航服务 |
| `simulation` | 隔离的模拟原语、故障开关及完整流程检查 |

旧校园地图资料归档在 `artifacts/archive/north_campus/`，不参与当前运行。

## 获取工作区

各独立仓库不随本仓库提交，用 `scripts/prepare.py` 一次拉取全部源码（只按分支取最新代码，不锁 commit）：

```bash
git clone git@github.com:Stone749990226/outdoorNav.git
cd outdoorNav
python3 scripts/prepare.py --dry-run   # 可选：先预览
python3 scripts/prepare.py             # 拉取/更新所有子仓库
```

脚本每次运行都会把每个子仓库更新到其分支的最新代码，本地未提交修改会被覆盖。
各仓库的分支在 `tools/sources.json` 里维护，留空表示跟随该仓库的远程默认分支。

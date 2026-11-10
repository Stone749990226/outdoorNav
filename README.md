# OutdoorNav：山猫室外导航

本工作区把室外定位、道路采集和导航接入 Robonix。整机部署源码统一在
`robot-deep_robotics-lynx_s10/outdoor/`，使用原生 `rbnx build / boot / shutdown`。

定位使用 G70 RTK＋WIT IMU，不使用山猫本体 `/ODOM`。Scene 提供道路录制、
地图保存、路线预览、执行和取消；导航服务负责跟随路线和雷达避障。

## 从哪里开始

- [系统分工](docs/ARCHITECTURE.md)：原语、服务、部署配置分别做什么。
- [部署和使用](docs/DEPLOYMENT.md)：准备依赖、启动、停止、建图和现场参数。
- [验证和现场复验](docs/VALIDATION.md)：离线检查与实机待办。
- [相机查看工具](robot-deep_robotics-lynx_s10/tools/camera_view/README.md)：通过 SSH 隧道实时查看山猫四路画面。

## 源码位置

| 目录 | 职责 |
|---|---|
| `robot-deep_robotics-lynx_s10/outdoor` | 室外部署清单和现场配置 |
| `robot-deep_robotics-lynx_s10/primitives/primitive-lynx_chassis` | 底盘消息转换、通信隔离和反馈保护 |
| `robot-deep_robotics-lynx_s10/primitives/primitive-lynx_lidar` | 雷达驱动和点云接口 |
| `primitive-g70-gnss-rbnx`、`primitive-wit-imu-rbnx` | 外接传感器原语 |
| `service-localization-outdoor-rbnx` | RTK＋IMU 融合定位 |
| `service-map-outdoor-rbnx` | 道路录制、补采、地图保存和路线规划 |
| `service-navigation-outdoor-rbnx` | 路线执行、避障、停车和步态切换流程 |
| `robonix/system/scene` | 操作网页，调用地图与导航服务 |

旧校园地图资料归档在 `artifacts/archive/north_campus/`，不参与当前运行。

## 获取工作区

各独立仓库通过 submodule 固定版本，一次拉取全部源码：

```bash
git clone --recurse-submodules git@github.com:Stone749990226/outdoorNav.git
# 已有工作区更新后：
git submodule update --init --recursive
```

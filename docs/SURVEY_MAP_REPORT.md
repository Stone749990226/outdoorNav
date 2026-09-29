# 智慧谷北区实测路网报告

## 数据与结果

输入 `1.ubx` 与 `2.ubx`，分别有 2135、368 个 GGA 位置。NMEA 校验均通过，GGA 全为质量 `2`；NAV-PVT 载波状态也没有固定解。接收器 hAcc 中位数约 0.36、0.385 米，仅为设备估计，不是独立地图误差。

地图版本 `survey-388df4ebbef8`。共同原点来自 `1.ubx` 末尾约 10 秒稳定位置中位数，保存在 `survey/north_building/origin.json`，X 向东、Y 向北。

| 道路 | 连接 | 长度 |
| --- | --- | --- |
| 环线 A | S → J | 76.63 米 |
| 环线 B | J → S | 268.72 米 |
| 中央支路 | P → J | 36.27 米 |

环线合计约 345.35 米。环线前导段裁切至约 `03:07:01.80 UTC`；闭环接缝修正 0.473 米。支路末端到环线中段的连接由用户确认，末端连接修正 2.980 米，未移动整条路线。

## 处理与生成物

处理时间重复和乱序；11 点 Hampel 窗口与速度一致性检查识别孤立跳点；超过 2 秒的缺测或持续异常切段。停留至少 10 秒且集中在 0.5 米半径内的轨迹压缩为中位数点，再进行保留转角的 5 点加权平滑。

最终按 0.5 米弧长重采样，使用 0.15 米容差简化；公共路口和接缝作为分段端点保留。本次没有剔除孤立跳点或跨越缺口，压缩环线 2 段、支路 2 段停留。非停留点最大滤波位移约 0.158 米和 0.046 米。

`survey/north_building/` 保存完整原始点、滤波段、米制路网、参数、源日志哈希、处理报告及独立 SVG。网页显示坐标与 ROS2 校准元数据由同一次生成写入，版本与原点一致。

```bash
conda run -n torch python tools/build_survey_map.py
```

当前脚本仅处理“一圈加一条已确认接入的支路”，不自动推断任意轨迹交叉是否连通。闭环超过 2 米、路口修正超过 5 米或轨迹存在缺口时拒绝更新地图。

## Nav2 与边界

网页在中心线上选点，拓扑 A* 输出完整路径；HTTP 桥接校验当前地图版本、配准状态及原点，发布完整路径和关键点，再调用 Nav2 `FollowPath`。每段路径约 0.25 米加密，保留原端点和路口。零长度任务仅在网页完成。

仿真地图为 **610 × 595、0.2 米/格**的均匀自由空间，原点 `[-11, -99, 0]`。它覆盖路网并留至少 10 米余量，专门供现有控制器联调；不代表实测道路净宽或真实可通行区域。当前控制器仍关闭碰撞检测，`map→odom` 为仿真静态变换。

输出 `scale_is_mock: false` 仅表示尺度来自测量；`calibration_status: survey_dgps_unverified`、`executable_on_robot: false`。实机需要同一坐标框架下的定位、机器人外参、障碍感知与代价地图。

## 验证命令

```bash
conda run -n torch python -m unittest discover -s tools -p 'test_*.py'
source /opt/ros/humble/setup.bash
PYTHONPATH=ros2_bridge:/usr/lib/python3/dist-packages:$PYTHONPATH /usr/bin/python3 -m unittest discover -s ros2_bridge/test -p 'test_*.py'
cd outdoor-patrol-demo
npm test
npm run build
BRIDGE_REAL=1 npm run test:e2e
```

最后一条需要已启动 `bash ros2_bridge/run_sim.sh`。验收覆盖滤波与断段、路网、选点与封路、导出、桌面及手机布局，以及真实 Nav2 控制器的短环线路径、支路过路口、取消与地图版本拒绝。

## 本次验收结果

2026-09-29：生成器及历史解析器 9 项、ROS2 桥接校验 5 项、前端 16 项单元测试通过，生产构建通过。Chromium 浏览器验收 13 项全部通过，包括实际运行的 Nav2 控制器。

实际 Nav2 已完成约 2 米环线路径与约 7 米支路经公共路口进入环线的路径；长路径在收到运动控制指令后取消，结果为 `Nav2 FollowPath canceled`。旧地图版本请求返回 HTTP 400。

检查了 1366×768、1920×1080 和 390×844 的布局与截图。没有验证真实机器狗、碰撞避障或完整 345 米环线的实机执行。可选系统 Firefox 验收未运行。

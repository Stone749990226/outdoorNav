# 复用山猫现有导航的室外接入

该路径**不启动第二套 Nav2、建图、TF 或速度输出**。室外应用只做四件事：自由选点、地理坐标与现有地图对齐、任务所有权、通过 Atlas 调用现有 Navigation Service。

## 运行边界

- 复用：现有建图（RTAB-Map）、`/scanner/scan`、现有 Nav2／DWB／velocity guard／底盘链、已有 `map→odom→base_link`。
- 新增：`deploy/lynx/start_existing.sh` 启动的室外应用（HTTP＋任务＋地理对齐），以及只读 ROS 观察节点。
- 只读：本应用不发布任何 TF，不发布 `/cmd_vel`，不写地图；地图位姿来自 TF，速度只用于判定是否停稳。
- 不启动：不调用 `rbnx boot`，不启停原 nav2／mapping；部署须由现有 rbnx 启动流程拉起本包。

## 配置（`deploy/lynx/existing_robot.json`）

- `navigation_provider_id`／`mapping_provider_id`／`odometry_provider_id`：Atlas 中的提供者 ID，默认 `nav2`／`mapping`／`lynx_quadruped`。
- `scan_topic`：现有链的二维扫描话题，默认 `/scanner/scan`。
- `antenna_xy_m`＋`antenna_offset_confirmed`：GNSS 天线相对 base_link 的前／左偏移；未实测量时保持 false，此时不放行行驶。
- `motion_enabled`：默认 false；现场准备运动测试时显式启用。
- `arrival_tolerance_m`：成功判据＝导航服务终态成功 **且** 实测停稳 **且** 位姿到目标直线距离在该容差内。

## 地理对齐

被动标定，不新增 TF 发布者：

1. 读取接收机 NAV-PVT／GGA，要求固定解、hAcc≤0.20 m、GGA 改正数年龄 0–5 s。
2. 同一时刻取现有 `map→base_link` 位姿，把天线点分别换到 ENU 与 map。
3. 样本需覆盖至少 5 m 基线后，最小二乘求旋转＋平移；残差 >0.25 m 视为失败，不放行。
4. 建图服务重新注册（新 registration_id）即清空标定，必须重新标定；GNSS 与地图冲突 >0.75 m 立即锁止。

静止样本**不会**产生绝对航向：没有 5 m 基线时始终不放行，这是刻意行为。

## 任务所有权

- 每次导航都记录本应用创建的 `run_id`；取消、状态只对该 ID 生效，绝不取消其他客户端的目标。
- 替换任务：先取消并确认原目标终态，再等待实测停稳，之后才下发下一个目标。
- 取消被接受 ≠ 已停车：必须等终态、速度小于阈值且位姿新鲜满 1 秒。
- 下发 RPC 超时视为结果未知，锁止且不自动重试。
- 导航提供者重启（registration_id 变化）时锁止，不重放目标。
- 过程退出时最多等待 15 秒确认停车；未确认就打印告警并保留锁止状态。

## 已知限制

- 终点可为地图任意点，但预览只做坐标校验，真实可达性由现有 Nav2 在执行时规划；目标若在障碍内会由现有导航失败返回。
- 网页尚未体现“标定基线不足”的进度条，只在未就绪原因里给出。
- 没有 Robonix 的连续途经点接口；仍是一次一个目标。
- 实际运动验收（到达、取消停车、RTK 断流、里程计重置）尚未进行。

## 旧的替换式配置

`deploy/lynx/start.sh`、`lynx.launch.py`、`lynx_watchdog.py` 属于先前交接的独立 Nav2 方案，保留作参考与仿真，不再作为山猫接入路径，也不应由 Robonix 包启动。

## RTK-only odometry (2026-10-20)

Indoor SLAM odometry cannot register in open outdoor space: the front Airy is a
vertical fan, so ICP sees sparse far returns and reports `ratio=0`. The robot
consequently has no `map->odom`. The outdoor path therefore uses RTK position
plus IMU yaw rate instead of SLAM:

- `deploy/lynx/gnss_odom.py` publishes `/outdoor/odom` and the TF
  `map->odom` (identity; the scene bundle's ENU frame *is* the map) and
  `odom->base_link` from the GNSS fix and the integrated IMU yaw rate.
- `deploy/lynx/gnss_stream.py` now tees each receiver message to two local UDP
  ports: 18765 for the task runtime, 18766 for the odometry node.
- `frame_alignment: enu_identity` skips the surveyed-baseline fit; the GNSS
  position already is the map position.
- A float solution is accepted (`gnss_max_hacc_m: 2.0`, `gnss_max_age_s: 10`)
  because a fixed solution is not reliably available; the fixed-only gate stays
  available by removing `frame_alignment`.

Heading still needs one initial value. It is taken from the GNSS course over
ground once the robot has moved 1 m over 2 s; a wrong initial heading rotates
every goal, so verify it after the first move.

Robot-side changes made on 山猫 (not in this repository):

- `robonix_manifest.yaml`: `lynx_quadruped.odom_source_topic` ->
  `/outdoor/odom`; `deskew_lidar: false`. Backup:
  `robonix_manifest.yaml.bak-20261020-1700`.
- `config/nav2_params_lite3.yml`: global costmap is rolling 24x24 m, the static
  layer is removed, `allow_unknown: true`, obstacle ranges 6 m / raytrace 8 m,
  `xy_goal_tolerance: 0.50`. Backup: `nav2_params_lite3.yml.bak-20261020-1700`.
- `/home/nvidia/ntrip/ntrip.conf`: `rtkggaint=1` (was 5). Backup:
  `ntrip.conf.bak-20261020`.

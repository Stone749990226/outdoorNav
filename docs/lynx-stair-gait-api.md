# 山猫楼梯步态 API 调研

本文保留 2026-11-03 的调研记录；当前 service 部署和步态确认策略见 `service-map-outdoor-rbnx/docs/navigation-handoff.md`。

依据：2026-11-03 SSH 检查山猫机载 SDK 文档（`robot-deep_robotics-lynx_s10/.sdk/S10软件开发指南202607.md`，
与仓库根目录 `S10软件开发指南202607.pdf` 内容一致）、机载 `robonix_manifest.yaml`、`primitive-lynx_quadruped` 源码。
当天在室内，**没有实机下发过楼梯步态**；所有结论都来自文档和代码阅读。

## 结论

上下楼 API 存在：向 ROS2 话题 `/GAIT` 发布 `gait = 0x3003`（楼梯步态·导航运动模式），
切回 `0x3002`（平地步态）即恢复。切换结果可从 `/MOTION_INFO`（20Hz）的 `gait` 字段确认。

**受力检测没有单独 API**。"碰到台阶 → 检测受力 → 自己抬脚上楼"是楼梯步态的内置行为，
切到 0x3003 后正常下发前进速度即可，不需要额外代码。文档未写台阶高度限制，实机从低台阶试起。

## 步态值速查

| 值 | 含义 |
|---|---|
| 0x1001 | 基础（标准运动模式，手动控制用） |
| 0x1003 | 楼梯（标准运动模式） |
| 0x3002 | 平地（导航运动模式，现在的导航步态） |
| 0x3003 | **楼梯（导航运动模式）** |

注意：不同运动模式支持不同步态；**切换步态会自动切换到对应运动模式**，反之亦然。
所以只发 `/GAIT 0x3003` 一步即可，不用先切运动模式。

## 前提条件

和现在平地导航完全一样：RL 控制（`/MOTION_STATE` state=17）+ 使用模式=导航（usage_mode=1）。
`/NAV_CMD` 速度指令在导航模式下生效，楼梯步态属于导航运动模式，速度流可以照常下发
（`/cmd_vel` → primitive 桥接 → `/NAV_CMD` 这条链路不检查步态，见下）。

## 现网代码情况（2026-11-03）

机载 robonix primitive `primitive-lynx_quadruped/lynx_quadruped/main.py`：

- `_nav_gait = 0x3002` 写死（`main.py:97`）。`posture('stand'/'rl')` 会重新发布 0x3002，
  **楼梯区里不能调 posture**，否则会被切回平地。
- `move()` 检查 `_gait != _nav_gait` 即拒绝（`main.py:579`）——楼梯步态下 move 工具不可用。
- `_on_twist_in`（`/cmd_vel` → `/NAV_CMD` 桥接，`main.py:170`）**不检查步态**，
  导航速度流在楼梯步态下照常生效。我们的导航链路走的是这条，不需要改。

本地仓库 `outdoornav/chassis_status.py:45` 的 `chassis_blockers` 已把 `0x3003` 算作合法导航步态，不用改。

## 建议实现方案

1. **地图标楼梯**：`scenes/north_campus/layout.json` 增加 `"stairs"` 区域（位置、范围、朝向），
   `scene build` 生成进场景 bundle，前端地图画楼梯标记。
2. **自动切步态**：加一个步态切换节点，订阅 `/outdoor/odom`，
   机器人**进入楼梯区域前**发 `/GAIT 0x3003`，**离开后**发 `0x3002`；
   切换后等 `/MOTION_INFO` 确认（等待逻辑可抄 primitive 里的 `_wait_for_gait`）。
   注意不要和 primitive 的 posture 重断言打架——切步态由这个节点独占。
3. **楼梯区降速**（比如 0.15 m/s），走稳一点。

## 未验证项（实机待办）

- 台阶高度上限（文档没写）。
- 楼梯步态下 `/NAV_CMD` 的速度上限是否和平地一致。
- 0x3003 下 `_MOTION_INFO` 的 height 等字段是否正常。
- 楼梯步态切回 0x3002 是否需要先停车（建议停车后再切）。

## 参考位置

- 文档：仓库根目录 `S10软件开发指南202607.pdf` §2.2.2「运控步态切换」、§2.2.3「获取运动基本状态和步态」
- 机载 SDK：`/home/nvidia/robot-deep_robotics-lynx_s10/.sdk/S10软件开发指南202607.md`（同内容）
- 机载 primitive：`/home/nvidia/robot-deep_robotics-lynx_s10/primitives/primitive-lynx_quadruped/lynx_quadruped/main.py`
- 机载配置：`/home/nvidia/robot-deep_robotics-lynx_s10/robonix_manifest.yaml`（gait_topic=/GAIT, nav_gait=12290）

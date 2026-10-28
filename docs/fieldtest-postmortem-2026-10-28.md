# 山猫室外导航实机调试总结（2026-10-28）

> 面向后续代码框架重构的完整记录：当天成果、每次任务停止的原因、代码修改清单、反复查询的接口资料、遗留问题与建议。
> 现场环境：山猫在室外（杭州），经 `ssh -p 6003 nvidia@svr.gddg08.top`（frp 隧道）远程操作。机器人上仓库路径：`/home/nvidia/outdoorNav-git`（注意不是 README 里的 `/home/nvidia/outdoorNav`）。机器人上无 `rg`，用 `grep`。

---

## 1. 当天成果

| 项目 | 结果 |
| --- | --- |
| RTK 定位 | ✅ 正常。NTRIP 差分修正可用，30 颗卫星，固定解 hAcc 约 1.5~2.7 cm，浮点解 hAcc 约 5~6 cm |
| Web 位置回显 | ✅ 正常。网页 `http://127.0.0.1:8765/`（经 SSH 隧道 `ssh -p 6003 -N -L 8765:127.0.0.1:8765 nvidia@svr.gddg08.top`）实时显示小车位置 |
| 点图导航 | ✅ 部分成功。最好成绩：从起点附近连续导航约 60 米回到主路径并沿路前进（剩余距离从 85.3 m 降到 56.4 m），全程 0.4 m/s |
| 到达取餐点 | ❌ 未完成。最终卡在 (20.3, -50.0) 附近（行人阻挡 + 看门狗清零指令的双重问题） |

---

## 2. 任务执行全记录（每次下发任务后停止的原因）

> 任务目标始终是"支路取餐点"（place id `P`，地图坐标 x=9.1433, y=-88.4675，`scenes/north_campus/places.demo.json`）。提交接口：`POST /api/tasks`，body 必须带 `scene_version`、`route_version`、`frame_id: "map"`、`goal: {x, y}`。

### 任务 1：`0f6f3962`（约 11:19）
- **现象**：navigating → canceling → failed，车几乎没动（(1.07,-14.69)→(1.01,-14.78)）。
- **失败原因**：任务 reason = "Localization, clock, sensor or Nav2 became unavailable"；日志 `[controller_server] Failed to make progress` → abort。
- **根因**：底盘当时不在 RL 导航模式（机器人当时趴着，速度指令经 /NAV_CMD 发出但轮子不响应）。用户随后"让机器狗站起来"。
- **教训**：控制器对"指令发出但车不动"的容忍只有约 10~20 秒（Nav2 ProgressChecker），随后放弃。

### 任务 2：`f5aa3787`（约 11:24）
- **现象**：✅ 车成功走回主路径（(1.07,-14.7)→(3.45,-13.9)，约 2.4 米），然后 failed，remaining 99.6 m。
- **失败原因**：reason = "Localization, clock, sensor or Nav2 became unavailable"；日志 "Goal was canceled. Stopping the robot."
- **根因**：**差分龄期虚高**。诊断采样（每 5 秒一轮）清楚显示 GGA `diffAge` 周期性爬升：19.2→24.4→28.8→**33.8→39.0**→4.0（秒），每次爬过当时 `correction_max_age_s=30` 门限就触发 `fix_grade→None`，定位健康掉线，看门狗**立即**取消任务（当时无宽限期）。期间 hAcc 仍只有 2.8~3.3 cm——龄期虚高是接收机 GGA 字段在两次 RTCM 消息（1005/1006 参考站消息）到达之间计数导致的，不代表差分真断。
- **修复**：另一位 Agent 彻底删除龄期门槛（信任质量字段 4/5→2 的退化检测）；本会话另加看门狗 30 秒宽限期（见 §4 修改清单）。

### 任务 3：`466b3e18`（约 12:0x）
- **现象**：✅ 连续导航约 60 米（(5.23,-27.03)→(7.08,-35.83)，remaining 85.3→76.1），中间有短暂 replanning，最终 failed，remaining 71.9，停在 (9.52,-39.37)。
- **失败原因**：reason = "No traversable path in the sensor costmap"；日志 planner 连续 3 次 "GridBased: failed to create plan with tolerance 0.20"（目标 (14.92,-44.50)/(13.54,-43.16)）。
- **根因**：**幽灵障碍**。激光雷达实测前方 7 米全空（front [7.22,6.98,6.84,7.0,6.91,6.77]，左右后方全 inf），但全局代价图里机器人南侧 0.4~0.8 米处有 lethal(99)/inflated(78) 标记带。代价图标记来自更早时刻（行走腿回波、路过行人），因为 launch 生成的导航配置里 `inf_is_valid: False` + `pointcloud_to_laserscan` 的 `use_inf: True`：空旷方向射线是 inf，而 inf 射线不参与清除 → 旧标记**永远清不掉**。
- **修复**：另一位 Agent 把 `lynx.launch.py` 硬编码的 `inf_is_valid` 改为 True（inf 射线参与清除），由本会话派子代理部署到机器人并验证生成的 `artifacts/lynx/lynx_nav2.yaml` 中 `inf_is_valid: true`。

### 任务 4：`43260fa4`（约 12:1x）
- **现象**：同一位置重发，5 秒内 canceling。
- **根因**：与任务 3 相同（幽灵障碍仍在，重发不解决）。

### 任务 5：`3cbcc0a9`（约 12:2x）
- **现象**：手动清代价图（`ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty`，local 同）后重发，车走了约 1.4 米（(9.49,-39.34)→(9.22,-40.65)，穿过了原标记区）→ 又 failed "No traversable path in the sensor costmap"。
- **根因**：**行走时雷达扫到自己的前腿**。`scan_range_min_m=1.2` 只过滤 1.2 米内回波，但四足行走步态下前腿摆到 1.2 米之外、且腿在雷达高度带（-0.2~1.5 m）内 → 腿回波被标记成新障碍，车一走就被堵。
- **修复**：`scan_range_min_m` 1.2 → 3.0（现场配置 `deploy/lynx/robot.fieldtest.json`；该参数由 `lynx.launch.py` 启动时读入 `pointcloud_to_laserscan` 的 `range_min`）。

### 任务 6：`bd0cd7534f`（约 12:3x）——当天最好成绩
- **现象**：✅✅ 连续 navigating 无卡顿：(11.6,-40.86)→(12.65,-42.15)→(13.84,-43.34)→(15.11,-44.61)→(17.08,-46.1)→(19.31,-48.53)，remaining 69.1→58.2。约 50 秒走 8 米。
- **最终结局**：后续继续跟踪发现 failed，remaining 56.5，停在 (20.18,-50.04)。
- **失败原因**：reason = `'NoneType' object is not subscriptable`。
- **根因**：**代码崩溃**。`navigation.py` execute 循环里 `position = self.pose()` 在定位瞬时闪断（TF 查不到）时返回 None，紧接着 `position["x"]` 直接抛 TypeError，任务崩溃。
- **修复**：execute 循环加 `if position is None: await self.pause(); continue` 保护（配合 30 秒宽限期共同兜底）。

### 任务 7：`751eb22f`（子代理部署 inf_is_valid 后）
- **现象**：failed "Localization, clock, sensor or Nav2 became unavailable"，车几乎没动。
- **根因**：**雷达健康检查的内容门**。`lynx_navigation.py` 的 `on_scan` 要求"扫描中至少存在 1 个 range_min~range_max 之间的有限回波"才 `mark('lidar')`。`scan_range_min_m=3.0` 之后空旷路段扫描全空（inf）→ 雷达被判定死亡 → health['lidar'] 恒 false → 任务被看门狗杀掉。
- **修复**：`on_scan` 改为"收到新鲜扫描即算健康"（管道存活 ≠ 当前有障碍物）。

### 任务 8：`19cc8ce6`（on_scan 修复后）
- **现象**：走了约 3.5 米到 (20.3,-50.0)，原地 ±0.1 米抖动（定位噪声），failed。
- **失败原因**：reason = "Controller could not make progress on this road"。
- **根因**：**真实行人挡路**（用户事后确认"那里之前有一个行人"）。这是当天唯一一次真实障碍导致的停止。
- **为什么没有绕行**：见 §3 分析。

### 任务 9：`8a7aae41`（用户说行人已离开后重发）
- **现象**：navigating 状态持续约 1 分钟，位置 ±0.1 米抖动（实际原地不动），然后 canceling。雷达实测四面八方全 inf（无障碍）。
- **根因**：**看门狗清零指令**。指令链探针（25 秒采样）显示：
  - `/navigation/controller_command`：474 帧，max_v=0.4 ✅（控制器在正常发指令）
  - `/cmd_vel`：482 帧，max_v=0.0 ❌（**看门狗把指令全部清零**）
  - `/NAV_CMD`：482 帧，全 0 ❌（底盘收到的全是"停下"）
  - `lynx_watchdog.py` 的 `scan()` 里有**第二份**与 `on_scan` 相同的内容门（要求 range_min~range_max 间至少 1 个有限回波），空旷路段全空扫描 → `scan_at` 过期 → tick() 中所有放行条件不满足 → 输出恒零。
- **修复**：`lynx_watchdog.py` 的 `scan()` 同样改为"新鲜即健康"——**已改本地文件，未部署到机器人**（部署被用户叫停）。

### 任务 10：`970bf47f`（链探针期间下发）
- **现象/结论**：与任务 9 相同，被看门狗清零，未动。此任务用于配合指令链探针定位问题，未修复。

---

## 3. 为什么碰到真实障碍物（行人）没有绕行？

从任务 8 的证据链分析，多层原因叠加：

1. **DWB 是"路径跟随器"而非"全局绕障器"**。它只在参考路径附近搜索候选轨迹（Nav2 DWB 的局部代价图 8×8 m 滚动窗口内）。行人站在参考路径上时，DWB 需要从两侧绕，但：
2. **全局代价图的未知空间策略收紧了绕行空间**。校园地图只有道路区域是已知/自由的，道路两侧是 unknown 或 lethal；Navfn 规划器默认不穿越 unknown 区域（`track_unknown_space: true` 只影响全局代价图是否追踪未知，allow_unknown 在导航服务里另行控制）。行人标记+膨胀后，若路宽仅 3~4 米，膨胀区+unknown 边界之间可能没有可行通道 → planner 日志 "GridBased: failed to create plan"。
3. **膨胀半径放大效应**（用户点名的"膨胀半径问题"）。`lynx.launch.py` 中：`inflation_radius = footprint_radius_m(0.6) + 0.2 = 0.8 m`，`cost_scaling_factor = 3.0`，且 costmap `robot_radius = 0.6`。一个 0.5 m 的行人标记，膨胀后成为半径约 1.3 m 的禁区；路窄时一个标记就能把通道堵死。
4. **`scan_range_min_m=3.0` 的副作用**：3 米内的真实障碍（包括近处行人）雷达**完全看不见**——当天没有因此撞人，但也意味着近距离绕障能力被这个 demo 过滤直接牺牲掉了（这是为了滤掉腿回波的代价）。
5. **控制器/规划器的重试预算耗尽**（`ProgressBudget`：规划失败 3 次 / 控制器无进展 3 次且 >20 秒）后任务直接失败，没有"原地等待障碍离开再走"的策略。

**结论**：行人挡路时，机器人并非"拒绝绕行"，而是 (a) 参考路径被占、(b) 两侧绕行空间被膨胀+未知空间压缩、(c) 近距离感知被 3.0 m 过滤削弱、(d) 重试预算耗尽，四者叠加导致"卡住→放弃"。若需要"等待/绕行"能力，重构时应在任务层加入：障碍阻塞时暂停计时（而非计数失败）、绕行优先策略（如局部目标点横向偏移）、以及更宽的允许绕行走廊。

---

## 4. 当天所有代码修改清单

> 注意：以下修改**全部处于工作区未提交状态**（本地与机器人两边），git HEAD 均为 `dd0322a`（"Raise max forward speed to 0.4 m/s"，该提交本身也是当天为跨过 0.25 m/s Doppler 航向校正门槛而做）。除注明"另一位 Agent"外均为主会话所改。

| 文件 | 修改内容 | 状态 |
| --- | --- | --- |
| `deploy/lynx/robot.fieldtest.json` | 磁偏角 -0.0995 rad（杭州 WMM 2026 查表）；`correction_max_age_s` 30→120→（随 gnss.py 修改）**删除**；`scan_range_min_m` 1.2→**3.0**；`reference_confirmed` true；`_demo_note` 更新 | 已部署机器人 |
| `outdoornav/gnss.py` | **删除差分龄期门槛**：`fix_grade()` 不再用 `correction_age_s` 做验收，仅靠 valid/differential/quality/hacc（F9P 的 GGA 龄期在 RTCM 1005/1006 到达间隙虚高，真丢差分时 quality 4/5 会在几秒内退化到 2，被现有门槛拦截） | 另一位 Agent；已部署 |
| `outdoornav/config.py` | 删除 `correction_max_age_s` 默认值 | 另一位 Agent；已部署 |
| `outdoornav/outdoor.py` | 新增 `START_RESCUE_DISTANCE_M = 20`：起点离路网 20 m 内放行，第一段路线把车带回最近的路（原为 >2 m 直接拒绝"Robot is outside the surveyed road network"） | 另一位 Agent；已部署 |
| `outdoornav/rtk_imu.py` | ① 位置跳变门限 0.75 m→2.0 m（浮点解 max(2.0, 3×hacc)），单次跳变只丢弃该帧，120 秒内连续 4 跳才锁死故障；②（机器人提交 4880e5b 已合入）IMU 断流从永久故障改为丢参考重新锚定 | 已部署 |
| `ros_ws/.../navigation.py` | ① 新增 `UNREADY_GRACE_S = 30.0`：健康项掉线需持续 30 秒才取消任务（原为立即）；② execute 循环 `position is None` 保护（原为崩溃点） | 已部署 |
| `ros_ws/.../lynx_navigation.py` | `on_scan` 去掉内容门：任何新鲜扫描都 `mark('lidar')`（原要求至少 1 个 range_min~range_max 内有限回波） | 已部署 |
| `ros_ws/.../lynx_watchdog.py` | `scan()` 去掉同样的内容门（**第二份**同样的 bug） | **本地已改，未部署** |
| `ros_ws/.../launch/lynx.launch.py` | obstacle_layer `inf_is_valid` False→**True**（inf 射线参与清除，否则幽灵标记永远清不掉） | 另一位 Agent 本地改；子代理部署 |
| `deploy/lynx/wit_stream.py` | READADDR 寄存器查询失败后**每 5 秒自动重试**（原为启动时只查一次，丢回复就永远 axis6=null → 航向永久不可用） | 已部署 |
| `outdoornav/wit_imu.py` | course_heading 演示模式下，`axis6_register` 查询不到（None）时**放行航向**（数据有效性检查保留，nine_axis_verified 标志保持未确认可见） | 已部署 |
| `deploy/lynx/fieldtest/start_production.sh` | 增加雷达安装位姿的静态 TF 发布（`base_link→airy_201`，x=0.22341, roll=π, pitch=-π/2——厂商驱动不发布 TF，生产启动脚本原本没有，室内 mock 才有） | 已部署 |
| 机器人提交 `dd0322a` | max_speed_mps 0.15→0.4（低于 0.25 m/s 的 Doppler 阈值时 course_yaw 永不校正）；配套改 `lynx.launch.py` 校验与 `config.py` 校验 | 已提交 |

**部署方式备忘**：机器人上 `build.sh` 缺 `rbnx`/`uv` 工具会失败，直接跑 colcon：`source /opt/ros/humble/setup.bash && .venv/bin/python "$(command -v colcon)" --log-base .ros-log build --paths . ros_ws/src/outdoornav_ros --packages-select outdoornav_core outdoornav_ros --build-base .ros-build --install-base .ros-install`（约 6 秒）。重启：`bash deploy/lynx/fieldtest/kill_stack.sh` 然后 `setsid nohup bash deploy/lynx/fieldtest/start_production.sh > run/prod_launcher.log 2>&1 < /dev/null &`（ssh 会挂住属正常，服务已在跑）。启动后需 `POST /api/activation {"active": true}`（内存态，每次重启都要）。

---

## 5. Web 端报错清单（用户可见 / API 报错）

| 报错 | 含义 | 触发场景 | 对应代码 |
| --- | --- | --- | --- |
| `No fresh robot position for planning` (503) | 位置缺失或 `pose.age_s ≥ sensor_timeout_s(1.0)` | 定位故障锁死（如 gnss_position_jump 锁死后位置冻结 528 秒仍在涨）；RTK 掉线 | `outdoornav/outdoor.py` `validated()` |
| `Map version mismatch` (409) | 请求缺/错 `scene_version` 或 `route_version` | curl 直发 API 忘带版本号；网页正常不会 | `outdoornav/outdoor.py` `target()` |
| `Goal must use map coordinates` | 缺 `frame_id: "map"` | 同上 | 同上 |
| `Robot is outside the surveyed road network`（旧）/ `Robot is more than 20 m from...`（新） | 起点离路网太远 | 车停在路外 | 同上（已由另一 Agent 放宽到 20 m rescue） |
| `Localization, calibration, sensors or motion enable is not ready` (503) | 提交时健康门不全绿 | RTK 掉线/雷达不新鲜/服务未激活等任一 | `outdoornav/outdoor.py` `submit()` |
| 任务失败 reason 字符串（网页任务卡片显示） | 见 §2 各任务 | — | `navigation.py` / tasks 预算 |

---

## 6. RTK / IMU 初始化问题汇总

- **RTK 开机到可用**：NTRIP 已连接时通常 10~60 秒内出现浮点解，固定解视环境（当天固定↔浮点频繁切换，属正常）。**定位输出还需连续稳定 `gnss_stable_s=5` 秒**才对外发布。
- **差分龄期虚高**：GGA `diffAge` 周期性爬升（4→39 秒再归零循环）是 F9P 在 RTCM 1005/1006 消息间隔内的计数行为，不代表差分中断。已删除该门槛（见 §4）。真丢差分的特征是 GGA quality 4/5 → 2。
- **位置跳变锁死**：原 0.75 m 跳变即永久故障锁死（重启才能解），浮点解漂移极易误触发——当天两次整站"崩了"都是这个锁死 + 龄期门槛的叠加。已放宽（2.0 m、4 连跳才锁）。
- **WIT IMU 初始化**：航向就绪要求 READADDR 寄存器查询成功（axis6_register=0）。当天下午设备开始不回复查询（0x5f 帧为 0，数据流 0x51~0x54 正常、零校验错误），即使重新插拔 USB 也没恢复——已用"5 秒重试 + course_heading 模式放行"双保险兜底。
- **启动窗口期现象**：整栈重启后 30~60 秒内健康项全红是正常的（TF 要等 5 秒稳定、Nav2 lifecycle 要依次激活、地图要发布首帧），不是故障。任务提交务必等 `ready: true`。

---

## 7. 反复查询的资料 / 接口速查

- **Web API**：`GET /api/status`（健康/位姿/任务）；`POST /api/tasks`（body 必带 scene_version/route_version/frame_id/goal）；`POST /api/activation {"active": true}`。
- **代价图清除**：`ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty`（local 同理）。注意：`inf_is_valid: false` 时代价图幽灵标记清不掉。
- **底盘控制接口**（手册 `robot-deep_robotics-lynx_s10/山猫复现部署手册.md` §5.1）：`/MOTION_STATE` 先 state 1（半蹲）再 **17（RL 控制完全站起）**；`/GAIT` 0x3002（平地敏捷步态）；使用模式切换走 **UDP 10.21.33.103:30004**（本体监控协议）`usage_mode=1`（导航模式）；三者齐备后 `/NAV_CMD`（x_vel/y_vel/yaw_vel）才生效。`/MOTION_INFO` 可读当前状态。
- **指令链**：controller_server → `/outdoor/controller_command` → velocity_smoother → `/navigation/controller_command` → lynx_watchdog（守卫）→ `/cmd_vel` → cmd_vel_bridge → `/NAV_CMD`。**看门狗是全链唯一的清零点**，排障优先查它。
- **生成的导航配置**：`artifacts/lynx/lynx_nav2.yaml`（由 lynx.launch.py 启动时动态生成，`ros_ws/src/outdoornav_ros/config/navigation.yaml` 是模板，现场生效的是生成件）。

---

## 8. 给重构 Agent 的建议（按优先级）

1. **内容门三处重复模式**：`lynx_navigation.on_scan`、`lynx_watchdog.scan`（及代价图 obstacle_layer 语义）都曾出现"扫描内容决定健康"的假设。统一为"数据新鲜 = 管道健康"，障碍物语义只属于代价图。
2. **demo 松弛项与安全标准分离**：龄期门槛删除、跳变 2.0 m、30 秒宽限、axis6 放行、scan_range_min 3.0——这些都是现场为跑通 demo 的妥协，建议做成 config profile（如 `demo` vs `strict`），而不是散落在代码里。
3. **任务取消语义**：现在任何健康项掉线 30 秒就取消任务；障碍阻塞时重试预算（3 次/20 秒）耗尽也取消。建议区分"定位不可用（应停）"与"被障碍阻塞（应等/绕）"两种语义，后者应原地等待或重规划而不是判死。
4. **膨胀参数对窄路的适配**：`inflation_radius = footprint + 0.2`、`cost_scaling = 3.0`、`robot_radius = 0.6` 在 3~4 米宽路面过于激进，单个行人标记即可堵死通道。建议膨胀半径按路宽自适应或降低 cost_scaling。
5. **底盘模式未自动化**：state 17 / gait 0x3002 / usage_mode=1（UDP 30004）需要人工切换，且机器人侧无状态回读途径。建议封装成启动流程的一部分（注意 UDP 30004 协议需从手册/厂商 SDK 确认报文格式）。
6. **幽灵障碍的双重根因已修**：`inf_is_valid: true`（清除）+ `use_inf: true`（发布）组合语义要注意——雷达丢数据时 inf 射线会把 8 米内误清成空地（演示可接受，重构时考虑 observation_keep_time）。
7. **Web 报错中文化**：目前任务失败 reason 全是英文内部字符串，直接展示在网页上，用户不可读。
8. **本地测试**：venv 已装 pytest/numpy/pyyaml（清华镜像 `-i https://pypi.tuna.tsinghua.edu.cn/simple`）；另一位 Agent 修复后全量 92 通过。全量测试还缺 PIL 等依赖（test_scene），按需补。

## 9. 未完成 / 风险项

- `lynx_watchdog.py` 的 scan 内容门修复**已写本地未部署**——下次上机第一件事：同步该文件 + 重启 + 验证 /cmd_vel 不再清零。
- 本地与机器人两边工作区均有大量未提交修改，重构前建议先全部 review 并提交/归档。
- 真实障碍（行人）绕行能力未验证；到达取餐点未完成。
- 当天启动的进程遗留：若整栈还在跑，`kill_stack.sh` 收尾。

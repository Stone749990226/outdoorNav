# 接入他们的前端页面 —— 第 0 步（只读侦察）接口说明

★ 场景 ✓：★ 业主点"**我在前端选目标点 ⇒ 点规划路径 ⇒ 仿真里的小车沿中间点走到目标**" ✓ ⇒ ★ **提交为新 commit** ✓
★ 侦察基准 ✓：★ 他们仓 **`87ed89d`** 的**干净克隆**（★ ✗ 没碰 `/home/xjy/projects/outdoorNav`（活挂载源）✓、✗ 没进 `outdoornav-demo` 容器 ✓）
★ 每条都带**文件路径 ＋ 行号** ✓；★ 查不到的**明写"没查到"** ✓

---

## 一、四条问答

### ① 前端**怎么把路径送出去** ✓

★ **不是 rosbridge，是本地 HTTP** ✓ —— ★ 前端 `fetch` 一个 HTTP API ✓：

| 调用 | 出处 | 说明 |
|---|---|---|
| `GET /api/status` | `outdoor-patrol-demo/src/rosBridge.ts:25` | 取状态（含车位姿 ✓） |
| `POST /api/routes` | `outdoor-patrol-demo/src/rosBridge.ts:26` | ★ **送路径** ✓，body ＝ `exportAB(...)` 的返回值 ✓ |
| `POST /api/cancel` | `outdoor-patrol-demo/src/rosBridge.ts:27` | 取消 |

★ **谁提供这个 HTTP 服务** ✓：★ **就是那份桥自己** ✓ —— `ros2_bridge/outdoor_nav_bridge/bridge.py:4` `ThreadingHTTPServer` ✓；★ 监听 ★ **`127.0.0.1:8765`** ✓（`bridge.py:296` ✓）；★ 路由见 `bridge.py:263–286` ✓
★ 前端 dev server 用 **vite 代理**把它接上 ✓：`outdoor-patrol-demo/vite.config.ts:5` `proxy: { "/api": "http://127.0.0.1:8765" }` ✓

★★ **路径点的字段与单位** ✓（★ 这是最容易搞错的地方 ✓）：
★ 每个点由 `map.ts:56` `toGeoreferenced(p) = { ...toMeters(p), ...mapToGeo(toMap(p)) }` 生成 ✓ ⇒ ★ **两种表示同时带着** ✓：
| 字段 | 单位／系 | 出处 |
|---|---|---|
| `{x, y}` | ★ **米**，frame `map`，★ **X 东 / Y 北** ✓ | `map.ts:31` `toMeters` ✓ |
| `{latitude, longitude}` | WGS84 度 | `map.ts:35` `mapToGeo` ✓ |
★ 整包还带 ★ `frame_id: "map"` ✓、`units: "meters"`（`abNavigation.ts:18` ✓）、`axes: "X east, Y north"`（`:19` ✓）、`calibration_version` ✓、`map_version` ✓、`origin_wgs84` ✓、`executable_on_robot: false` ✓

★★ ★ **接线时只许用 `{x,y}`（米）那一半** ✗ —— ★ **别用 `latitude/longitude` 中转** ✗：
　★ 实测 ✓：★ 用经纬度按"**他们 origin ＋ 我们的常数**"与"**我们的 origin ＋ 我们的常数**"两条路各换一次 ⇒ ★ **两条都对不上他们自己的 (x,y)** ✗（前者差 **0.935 m** ✓、后者差 **0.442/0.151 m** ✓）⇒ ★ 因为**两套 origin 的基准点不同** ✓ ⇒ ★ **混用就是本项目栽过的那个"4.031 m"** ✗ 的同族 ✓（见 §三）。

### ② 前端**从哪读车的位置** ✓

★ **不经 ROS 话题** ✓ —— ★ 走 **HTTP** ✓：`GET /api/status` ⇒ 响应里的 ★ **`pose: {x, y, yaw}`** ✓（类型定义 `rosBridge.ts:6` ✓；★ 字段名就是 `x`／`y`／`yaw` ✓）
★ 前端渲染 ✓：`ABWorkspace.tsx:78` 轮询 `bridgeStatus()` ✓；`:200–203` 用 `mapToSvg(bridge.pose)` 画车标 ✓（`data-testid="ros-robot-pose"` ✓）；`:348` 显示"仿真位置 x, y" ✓

★★ ★ `pose` 的来源 ✓：★ `bridge.py:125–129` `on_odom()` 直接取 ★ **`/odom`**（`nav_msgs/Odometry` ✓，订阅见 `bridge.py:96` ✓）的 `pose.pose.position.x/y` ＋ 由四元数算 `yaw` ✓ —— ★ **一个字的变换都没做** ✗
★ ⇒ ★ **所以我们的车要发 `/odom`** ✓，且 ★ **position 直接就是 frame `map` 的米值** ✓
★ 他们的 TF 也印证这一点 ✓：`ros2_bridge/launch/sim.launch.py:34` 静态 `map → odom`（0,0,0，identity ✓）⇒ ★ 在他们的设定里 **`map` ≡ `odom`** ✓

### ③ **那份桥是不是必需** ✓ ⇒ ★ **必需** ✓

★ 前端**只认 HTTP `:8765`** ✓（`rosBridge.ts:14–23` ✓，vite 也只代理 `/api` ✓）⇒ ★ **只有 `bridge.py` 提供它** ✓ ⇒ ★ **必须跑 `bridge.py`** ✓
★ 而 ★ **`sim_robot.py` 不必须** ✗ —— ★ 它是**我们要顶掉的那台假车** ✓（见 ④）

### ④ 三份标定的**数值与坐标约定** ✓

| 文件 | `origin_wgs84`（lat / lon） | `meters_per_unit` | `rotation_rad` | 其它 |
|---|---|---|---|---|
| 前端 `outdoor-patrol-demo/src/calibration.json` | **30.22413029 / 120.25979315833334** | **0.125** | 0 | `version = map_version = survey-388df4ebbef8` ✓、`status = survey_dgps_unverified` ✓、`map_origin_svg = (88,160)` ✓ |
| 桥 `ros2_bridge/maps/calibration.json` | **同上**（★ 逐字相同 ✓：两份 `sha256` 前 20 位都是 **`ef689574187c757f1f0e`** ✓） | 0.125 | 0 | 同上 |
| 我们 `scripts/rtk-nav2/demo-data/geo_calib.json` | ★ **30.224128929 / 120.259788563**（★ **不同** ✓） | —（用 `m_per_deg_lon=96269.768571` ✓／`m_per_deg_lat=110856.417701` ✓） | `theta_deg ≈ −5.59e−05` ✓ | `translation_m = (−0.442564, −0.150836)` ✓ |

★★ **三者的关系（实测 ✓）**：
- ★ 两份 `calibration.json` **完全相同** ✓ ⇒ ★ 权威那份**只有一份** ✓
- ★ 我们的 `geo_calib.origin` ＝ **他们 41 点里第 1 个点的经纬度** ✓（他们 CSV 第 1 行 `30.224128929 / 120.259788563` ✓）⇒ ★ 差的正好是 `translation_m` ✓（实测：Δlon·m_lon ＝ **0.442392** vs `−tx` ＝ **0.442564** ✓；Δlat·m_lat ＝ **0.150876** vs `−ty` ＝ **0.150836** ✓ ⇒ ★ 互为相反数、到 0.0002 m ✓）
- ⇒ ★ **两份标定描述的是同一个 ENU 系，只是取的"原点"不同一点** ✓

★ **坐标约定** ✓（由 `map.ts:22–30` `toMap` 反推 ✓）：★ **米** ✓、★ **X 东 / Y 北** ✓、★ SVG 的 y 轴**向下** ⇒ 转换里有一次**翻转** ✓（`dy = origin_svg.y − p.y` ✓）

★ **≥3 个可辨认点** ✓（★ 取自他们自己的导出 `waypoints_export.csv` ✓，与 `campus.json` 的路网同名 ✓）：
| 点 | 名字 | 他们 map (x, y) 米 | 我们世界 (gt_x, gt_y) 实测 |
|---|---|---|---|
| wp1 | 环线起点 | (−0.442, −0.151) | **(−0.442, −0.151)** ✓ |
| wp21 | 支路端点 | (9.143, −88.468) | (9.200, −88.265) |
| wp41 | 返回起点 | (−0.442, −0.151) | (−0.393, −0.261) |

---

## 二、★★ 那条**唯一**的变换 ＋ 残差（有数 ✓）

★ 决定 ✓（reviewer）：★ **以前端那份标定为权威** ✓ ⇒ ★ 路径活在**它的坐标系**里 ⇒ ★ **一切换算到它** ✓、★ **只许一条** ✓

★★ **公式** ✓（★ **从他们的 payload 到我们的 Fortress 世界** ✓）：
```
world_x = s·(cosθ·x − sinθ·y) + tx
world_y = s·(sinθ·x + cosθ·y) + ty
```
★ **实测拟合值** ✓（★ 用某轮实跑的到达点配对他们 CSV ✓，n = **41** 对 ✓）：
★ **s ＝ 0.999287952** ✓ ｜ ★ **θ ＝ −0.013267°** ✓ ｜ ★ **t ＝ (−0.028797, +0.022321) m** ✓
★★ ⇒ ★ **它 ≈ 恒等变换** ✓ ⇒ ★ **代码里取恒等即可** ✓（★ 上式的 s/θ/t 留在注释与产物里 ✓ —— ★ 它们是"**这条链成立**"的证据 ✓）

★★ ★ **残差要说清它量的是什么** ✗：
★ 拟合残差 ★ **max 0.4404 m ／ RMS 0.2305 m ／ 中位 0.2063 m** ✓
★★ ★ 但那**不是变换误差** ✗ —— ★ 它含**车的停车精度** ✓（★ 本条链用 `--tol 0.40` ✓ ⇒ ★ max 0.44 m **正好落在容差上** ✓）
★ ⇒ ★ 能代表"两系对齐"的**纯几何证据**是这两条 ✓（★ 无动力学 ✓）：
　★ ① ★ wp1 的 ★ **`gt = (−0.442, −0.151)`** ✓ **＝** 他们 CSV 的 `(x, y)` ✓ **到毫米** ✓
　★ ② ★ 他们的 `mapToGeo`（`map.ts:35` ✓）把 `map(−0.442,−0.151)` 映到 `(30.224128929, 120.259788563)` ✓ ＝ ★ **我们 `geo_calib` 的原点** ✓（★ 用他们 origin ＋ 他们的 `meters_per_unit`／度数换算复算 ✓）

★★ ★ **结论与红线** ✓：★ **路径用米、恒等变换、直接喂 nav2** ✓；★ ✗ **绝不用经纬度中转** ✗（★ 混用两套 origin 会偏 **0.44/0.15 m** ✓，与"闸读 `_world_xy` vs 驱动读 lat/lon ⇒ 差 4.031 m"是同一条病 ✓）

---

## 三、★ 那条链（每跳带话题名 ＋ 消息类型 ✓）

```
[浏览器] 地图上点目标点（起点＝车的当前位置）
   │ ① HTTP POST /api/routes      body = {frame_id:"map", units:"meters", path:[{x,y}…], waypoints:[{x,y,yaw}…], …}
   ▼
[bridge.py（ROS 节点；HTTP :8765 由它提供）]
   │ ② 校验（bridge.py:32–55）：frame 必须 map ✓／版本与 origin 必须与 calibration.json 一致 ✓／path 2–20000 点 ✓
   │ ③ 先发 /outdoor_nav/sim_reset   nav_msgs/PoseStamped   起点＝path 第 1 点（bridge.py:177–181）★ 等 0.8 s
   │ ④ 再发 FollowPath 动作（nav2 的 controller_server，controller_id="FollowPath"）（bridge.py:211–215）
   ▼
[nav2 controller_server] ──/cmd_vel──▶ geometry_msgs/Twist ──▶ [车]
   ▲                                                              │
   └──────────── /odom ── nav_msgs/Odometry ◀────────────────────┘
   │ ⑤ bridge 把 /odom 的 x,y,yaw 原样放进状态（bridge.py:125–129）
   ▼
[浏览器] GET /api/status ⇒ {pose:{x,y,yaw}, distance_remaining, cmd_vel_count, phase}
   车标位置（ABWorkspace.tsx:200–203）／"仿真位置 x, y"（:348）／剩余距离
   ★ 停在哪：FollowPath 的 result（bridge.py:234–243）⇒ phase = completed／failed ✓
```

★ 另一条**只做事后取证**的路 ✓（不影响上面）：`/outdoor_nav/planned_path`（`nav_msgs/Path` ✓，retained ✓，`bridge.py:92` ✓）／`/outdoor_nav/waypoints`（`PoseArray` ✓，`:93` ✓）／`/outdoor_nav/status`（`std_msgs/String` ✓，`:94` ✓）—— ★ 都是 **bridge 往外发**的 ✓，可以订阅来做离线复算 ✓

---

## 四、★ 建议的接入点（我们这边加什么、放哪）

★ ★ **要顶掉的两样** ✓（`sim_robot.py` 的角色 ✓ —— `sim_robot.py:24,26` ✓）：
| 它做的 | 我们要做的 |
|---|---|
| 订阅 `/cmd_vel`（Twist ✓ `sim_robot.py:24` ✓） | ★ 我们的车**必须收 `/cmd_vel`** ✓ —— ★ 现有链已有此桥 ✓（`demo.launch.py:91` `/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist` ✓） |
| 发 `/odom`（Odometry ✓ `sim_robot.py:26` ✓，`frame_id="odom"` ✓／`child_frame_id="base_link"` ✓ `:60–61` ✓）＋ TF `odom→base_link` ✓ | ★ 我们**必须发 `/odom`** ✓，且 ★ **position 就是 frame `map` 的米值** ✓（★ 因为 bridge **不换算** ✓ 而他们的 `map→odom` 是 identity ✓） |

★★ **一处必须处理的冲突** ✗：★ 他们的 `sim.launch.py:34` 会起 ★ **静态 `map→odom`（identity）** ✓，★ 而我们自己的栈由 ★ **`navsat_transform` 发 `map→odom`**（RTK 修正 ✓）⇒ ★ ★ **两者不能同时在跑** ✗（★ TF 一个 frame 只能有一个父 ✓）⇒ ★ 接入时**用我们那条 TF** ✓、★ **不起他们的 static TF 那行** ✓

★ **落点建议** ✓（★ ✗ 不动现有演示行为 ✓）：★ 新增**一处**接入脚本（例如 `scripts/rtk-nav2/frontend-bridge.sh` ✓）＋ ★ nav2 配置里**改哪几个参数**由实现那一步定 ✓ —— ★ 本条**只报接口** ✗、★ 不批任何代码改动 ✓

### ★ 最小可跑组合（跑通"前端点目标 ⇒ 车动"）

| 侧 | 起什么 | 出处 |
|---|---|---|
| 他们 | ★ `bridge.py` ✓（★ **必需** ✓ —— 提供 HTTP :8765 ✓） | `ros2_bridge/run_sim.sh` ✓（`colcon build --packages-select outdoor_nav_bridge` ⇒ `ros2 launch outdoor_nav_bridge sim.launch.py` ✓） |
★ ★ ★ **这一格已废止** ✗（2026-10-13 ✓）：★ 前端已**搬进容器** ⇒ ★ 不用在宿主／Windows 侧起 ✗ ⇒ ★ 见 `docs/frontend-integration.md` §五 ✓（★ 本文件其余部分仍是当时那份**只读侦察**的记录 ✓，✗ 不改 ✗）
| 他们 | ★ frontend dev server ✓（★ **从我们自己的副本起** ✓ —— 在他们那份跑会往 `node_modules/.vite` 写缓存 ✓） | `outdoor-patrol-demo/` ✓（★ `node_modules` 已装好 ✓ ⇒ ✗ 不用 `npm install` ✓） |
| 他们 | ★ ✗ **不起 `sim_robot.py`** ✗ | `sim.launch.py:36` ✓ —— ★ **那是我们要顶掉的假车** ✓ |
| 他们 | ★ ✗ **不起 static TF `map→odom`** ✗ | `sim.launch.py:34` ✓ —— ★ 我们那条 TF 取代它 ✓ |
| 他们 | ★ nav2 `map_server` ＋ `controller_server`（`sim.launch.py:13–30` ✓）★ 是否用他们的 `config/nav2_sim.yaml` ⇒ ★ 待定 ✓（★ 用我们的 `campus_nav2.yaml` 也行 ⇒ ★ 但 `map_server` 的地图与我们的代价地图**必须只有一个** ✗） | — |
| 我们 | ★ 我们现在那套起栈 ✓（Gazebo ＋ 传感器 ＋ RTK 定位链 ＋ nav2 两张图 ＋ 控制器 ✓） | `scripts/rtk-nav2/start-northcampus.sh` ✓ |
| 我们 | ★ 接入脚本 ✓（★ 待实现 ✓）：★ 把路径收下／把 `/odom` 按他们的约定发出去 ✓ | — |

---

## 五、★ 边界与未决

★ **没查到** ✗（★ 如实列出 ✓）：
1. ★ `mapToGeo` 的**下半段常数**（`map.ts:35` 起被我的读取截断 ✓）⇒ ★ 我用**他们的 origin ＋ 我们的度换算常数**复算过 ✓、★ 结论一致 ✓，★ 但**他们自己用的那个 deg→m 常数我没读到** ✗
2. ★ 他们的 `config/nav2_sim.yaml` 里 **`FollowPath` 插件的完整参数**（我只读到 `robot_base_frame: base_link` ✓ 等 5 行 ✓）⇒ ★ 与我们的 `campus_nav2.yaml`（`odom_topic: /odometry/local` ✓ 等 ✓）**是否兼容** ✗ 待核
3. ★ 前端**目标点选取**是否受"必须落在路上"约束（`planner.ts:52–53` 要求距离 ≤1e-6 ✓）⇒ ★ 用户点偏一点就 `route()` 返回 null ✓（`:55` ✓）⇒ ★ UI 上怎么提示的**没查到** ✗

★ **边界** ✓：★ ✗ 不改他们仓库 ✓；★ ✗ 不动 `/home/xjy/projects/outdoorNav`（活挂载源 ✓）；★ ✗ 不起他们的进程（★ 业主已点名 ✓，★ 但**执行时若被权限层拦 ⇒ 报原文、不绕** ✓）；★ ✗ 现有 41/41 不受影响 ✓

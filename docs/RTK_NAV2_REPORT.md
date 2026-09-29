# G70 日志配准与 ROS2/Nav2 仿真接入报告

> 本文保留旧手绘地图两点暂定配准的历史记录。当前地图已由 `1.ubx`、`2.ubx` 重新生成，采用共同米制原点和中心线路网，状态为 `survey_dgps_unverified`。当前数据与验收见 [实测地图报告](SURVEY_MAP_REPORT.md)，重新生成使用 `tools/build_survey_map.py`。

## 1. 结论与边界

本次已经把现有手绘园区图做成**全图暂定坐标配准**，网页 A/B 规划可导出 `map` 米制完整路径、关键途经点及推算 WGS84 经纬度。ROS2 桥接发布路径与途经点话题，向真正运行的 Nav2 Humble `FollowPath` Action 发送完整路径，并回传里程计位置、进度及结果。在 WSL 上，3 米验收路径已由 Nav2 控制器驱动仿真机器人完成。

这**不是实机可用地图**。日志首尾仅在“北侧入口 N”和“东侧楼前 E”附近，均为差分定位质量 `2`，并非现场精确标记的 RTK 固定解控制点；后段还出现明显跳变。两点拟合的零残差只是数学构造，无法证明地图几何或绝对位置精度。所有输出带 `calibration_status: provisional` 和 `executable_on_robot: false`。

## 2. 原始数据与解析

输入：`北侧路口到东侧楼前.ubx`，658711 字节。它是 NMEA 文本与 UBX 二进制报文混合的接收机串流。`tools/prepare_demo.py`：

1. 扫描 `$...*HH\r\n` NMEA 报文，对 `$` 与 `*` 之间的字节计算 XOR 校验；从 `GGA` 读 UTC、纬度、经度、定位质量、卫星数、HDOP，从 `RMC` 可取得日期。
2. 将 NMEA 度分转换为十进制度，例如 `3013.43580,N → 30 + 13.43580/60 = 30.223930°`。
3. 扫描 `B5 62` UBX 帧，按长度和双字节校验解 `NAV-PVT`，交叉核对差分标志、载波解状态及接收机报告的 `hAcc`。`hAcc` 是设备估计值，不是独立测量出的地图误差。
4. 检测相邻点的大位移；本轮地图核对图层只显示固定解点，未把异常差分解轨迹接成可走道路。

识别到 9361 条校验正确的 NMEA 语句、0 条坏校验；GGA 共 748 条，其中 1 条无效且无坐标。有效定位包括：差分解 `2` 为 467 点、RTK 固定解 `4` 为 150 点、RTK 浮点解 `5` 为 130 点。匹配到 747 条 `NAV-PVT`。固定解主要在 **02:51:19.00–02:51:39.20 UTC** 和 **02:51:45.00–02:51:54.40 UTC**；实际数据约 5 Hz。北京时间为 UTC+8。最大异常在 **02:52:45.00–02:52:45.20 UTC**，相邻位置变化约 **28.61 米/0.2 秒**，不可解释为正常步行。

可复算：

```bash
conda run -n torch python tools/prepare_demo.py
conda run -n torch python -m unittest discover -s tools -p 'test_*.py'
```

生成物包括网页配准文件与固定解图层，以及 `ros2_bridge/maps/` 下的仿真占据栅格、Map Server YAML 和含跳变记录的 `survey_summary.json`。原始日志不被改写。

## 3. 两点暂定配准

图形原点在左上，Y 向下；先令图形局部向量 `u=x−480, v=290−y`。选用手绘节点 `N=(480,290)`、`E=(865,660)`，对应日志第一条与最后一条有效 GGA：

| 锚点 | UTC | GGA 质量 | 纬度 | 经度 |
| --- | --- | --- | --- | --- |
| 北侧附近 | 02:50:42.40 | 2，差分解 | 30.224080333° | 120.259848000° |
| 东侧附近 | 02:53:12.60 | 2，差分解 | 30.223227500° | 120.260783167° |

经纬度差使用 WGS84 椭球在北侧附近的东西、南北曲率半径转换为局部米制差量，再解相似变换：

```text
[X, Y] = s · R(θ) · [x_svg − 480, 290 − y_svg]
s = |实测首尾位移| / |手绘首尾位移| = 0.244487434 米/图形单位
θ = 实测首尾方位 − 手绘首尾方位 = −0.044319918 rad = −2.5393°
map 原点 ≈ 第一条 GGA 天线位置，X 近似向东，Y 近似向北
```

旧演示比例为 `0.25 米/图形单位`，且 X/Y 未绑定东/北。新变换应用于导出路径、途经点、距离、朝向与仿真栅格；SVG 道路、建筑轮廓及路宽仍是原手绘假设。与先前用球形地球估算得到的约 `0.24465`、`−2.68°` 略有区别，此处使用 WGS84 局部曲率计算值。首尾“附近”的误差、末段跳变后的偏移都未被独立测量；不要报告为厘米级配准。

固定解轨迹作为 A/B 地图中的可开关核对层。它只覆盖前段；不能由一条天线轨迹推断道路两侧净宽、建筑占地或台阶通行性。为真正修图，需要在明确地面标志处记录多个稳定固定解控制点，留独立检查点验证残差，并实测道路边界和机器狗净空。

## 4. 系统接口与数据流

```text
网页 A/B 选点 → 现有区域 A* → 完整路径 + 关键点
  → 暂定 map 米制坐标 / 推算 WGS84
  → 本机 HTTP POST /api/routes
  → ROS2 /outdoor_nav/planned_path  (nav_msgs/Path)
  → ROS2 /outdoor_nav/waypoints     (geometry_msgs/PoseArray)
  → Nav2 /follow_path               (nav2_msgs/action/FollowPath)
  → /cmd_vel → 仿真运动学节点 → /odom、odom→base_link
  → 桥接读取反馈与 /odom → /outdoor_nav/status、GET /api/status → 网页
```

Map Server 发布 `/map`；静态 `map→odom` 和仿真节点的 `odom→base_link` 组成 TF。Nav2 控制器使用地图、TF、里程计和完整路径输出速度指令。关键点单独发布供检查，`FollowPath` 只保证控制器按完整路径跟踪，并不承诺每个稀疏关键点都被严格停止。浏览器显示的是仿真回传，不是机器狗反馈。

`ab-route.json` 的 `frame_id` 为 `map`，`path` 和 `waypoints` 均包含米制 `x/y` 与**推算**经纬度；关键点还有 `yaw`（相对于 `map` +X、逆时针为正）。起终点与版本信息一同发给桥接。桥接只接受当前配准版本、`map` 坐标和 `executable_on_robot: false` 的有限数值路径；`POST /api/cancel` 可取消任务。网页通过 Vite 的 `/api` 代理访问仅监听 `127.0.0.1:8765` 的本机服务。没有连接机器狗底层控制器。

## 5. 运行与验收记录

Ubuntu 22.04 / ROS2 Humble，两个终端：

```bash
bash ros2_bridge/run_sim.sh
```

```bash
cd outdoor-patrol-demo
npm ci
npm run dev
```

打开 Vite 地址，进入 A→B，点起终点并规划；下载 JSON 可检查坐标，点击“发送仿真路径”执行，页面显示反馈并可取消。在 ROS 终端检查：

```bash
source /opt/ros/humble/setup.bash
ros2 action list -t
ros2 topic list -t
ros2 lifecycle get /map_server
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map base_link
```

本机验收：Map Server 加载 **520×482、0.488975 米/格**的仿真地图；旋转已烘焙到栅格图像，YAML 原点朝向为零，局部代价地图在测试路段显示自由空间。Map Server 与控制器均为 `active [3]`，存在 `/follow_path [nav2_msgs/action/FollowPath]`、`/map`、`/odom`、`/cmd_vel`、规划路径、关键点和状态话题。向桥接发送一条 3 米局部测试路径后，Nav2 返回 `FollowPath completed`，终点里程计约 `(-0.026,-2.620)` 米，处于配置的 0.4 米终点容差内，桥接累计收到 **61 条 `/cmd_vel`**。Python ROS2 订阅者收到 4 个完整路径顶点；`PoseArray` 收到 2 个关键点，TF 可查 `map→base_link`。网页真实选点生成的短路径也已由 Nav2 完成；桥接可取消运行中的 Action，错误坐标系请求返回 HTTP 400。控制器收到的是在原完整折线间按约 0.25 米插值后的密集路径，原端点和关键点不变。这验证的是 Nav2 控制器和仿真机器人闭环，不是实机行走能力。

测试命令：

```bash
conda run -n torch python -m unittest discover -s tools -p 'test_*.py'
source /opt/ros/humble/setup.bash
PYTHONPATH=ros2_bridge:/usr/lib/python3/dist-packages:$PYTHONPATH /usr/bin/python3 -m unittest discover -s ros2_bridge/test -p 'test_*.py'
cd outdoor-patrol-demo
npm test
npm run build
```

本轮分别通过解析器 3 项、桥接输入校验 4 项、前端 30 项单元测试和生产构建。Playwright 使用本机已有 Chromium，常规套件 7 项通过、1 项真实 Nav2 测试按需跳过；另单独运行真实网页到 Nav2 的测试并通过。包括桌面与移动端截图检查。

## 6. 实机前仍需完成

- 在清晰物理地标处采多个稳定固定解控制点，明确天线位置与停留时段；留未参与拟合的检查点报告平均、最大和薄弱路段误差。
- 实测道路边界、台阶、坡度、障碍以及机器人外形与定位误差预算；更新真实可通行地图，并把未知地面继续当作未知。
- 在 Ubuntu 持续提供 NTRIP 改正数据，G70 驱动发布 `NavSatFix`；融合 IMU/里程计、标定天线到 `base_link` 外参，验证 `map→odom→base_link` 定位一致性与退化处理。
- 与机器狗底层控制接口、实时障碍感知、停止机制及任务反馈完成联调；通过现场低速、逐段验证后才设计实机放行开关。

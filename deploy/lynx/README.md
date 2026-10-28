# 山猫室外导航

网页和 Robonix 共用本项目的路线执行器，沿已有校园路网分段规划；雷达更新障碍地图，Nav2 执行绕障。
首版支持道路附近选点及已确认地点，不支持任意离路终点。参考路线不是已确认无障碍路径。

## 当前室内静止调试

机器人保持趴下；`motion_enabled=false`，现场确认项保持未放行。构建后在仓库根目录执行：

```bash
bash deploy/lynx/start.sh --diagnostic
```

打开 **http://10.29.1.4:8765/** 查看真实传感器状态和全部未就绪原因。诊断模式拒绝导航任务，
最终速度只发布到 `/outdoor/diagnostic_cmd_vel`，不向真实 `/cmd_vel` 发布，包括退出时的零速。
该模式默认 `LYNX_GNSS_MODE=observe`、`LYNX_IMU_MODE=observe`：不启动位置或 IMU 串口采集者，不占用现有 GNSS 端口；
网页没有定位数据时先检查采集链路，不要降低 RTK 门槛。正常生产默认由服务管理唯一采集进程。

`Ctrl+C` 结束本次诊断并清理本服务子进程。不会停止厂商底盘程序或当前正式 Robonix 部署。
通过 Robonix 包启动诊断时设置 `LYNX_DIAGNOSTIC=true`；完整正式部署切换留待后续验收。

只读采样与联调命令见后文；最新实测异常见 `docs/lynx-validation.md`。

## 构建和选择运行模式

在山猫上使用 ROS 2 Humble、系统 Python 3.10、uv、colcon、Node.js ≥22.12 和已有 rbnx。
需要安装 Nav2（含 DWB）、pointcloud_to_laserscan、tf2 和 sensor_msgs_py。

以下用于后续正式部署；当前静止调试不执行整套 `rbnx boot`。
将本仓库放在 `/home/nvidia/outdoorNav`。先从原室内部署生成独立室外 manifest：

```bash
cd /home/nvidia/outdoorNav
python3 deploy/lynx/profile.py /home/nvidia/robot-deep_robotics-lynx_s10/robonix_manifest.yaml
cd /home/nvidia/robot-deep_robotics-lynx_s10
rbnx build -f robonix_manifest.outdoor.yaml
rbnx boot -f robonix_manifest.outdoor.yaml
```

生成器不修改原 manifest，已有室外 manifest 时拒绝覆盖。切换前先用原部署的停止流程结束上一模式。
室外配置保留底盘和传感器，移除室内 mapping/nav2、Scene 及相关 skill，加入 `outdoor_navigation` Service。
原 manifest 用于回到室内；一次只启动一种。禁止只修改缓存目录。

构建入口 `bash deploy/lynx/build.sh` 构建 ROS 包、网页和 MCP 类型；运行入口由 Robonix 包自动调用。
单独运行 `bash deploy/lynx/start.sh` 只供诊断，Service 未激活时不接受运动。

## 可选旧路径：雷达＋IMU 连续里程计（非当前默认）

当前默认已经改为原 u-blox RTK＋新十轴 WIT IMU，见下文“RTK＋新 IMU 直接运动反馈”；LIO 仅作为保留的候选路径。
以下 LIO 仍为保留候选。

新增 FAST-LIO2 ROS 2 后端，使用固定版本 `Ericsii/FAST_LIO_ROS2@2fffc570`。
链路为 Airy 原始点云／IMU → 输入适配 → FAST-LIO → 机身原点里程计 → 现有 GNSS 地图对齐。
后端动态 TF 隔离到 `/outdoor/lio/tf`；只有原定位节点发布室外动态 TF。

10 月 22 日在隔离域临时运行接收驱动，确认前 Airy 点云约 10 Hz、内置 IMU 约 200 Hz。
点云有 `ring:uint16`、`timestamp:float64`；时间戳为绝对秒，消息头对应扫描结束。
内置 IMU 的驱动输出为加速度 g、角速度 rad/s，适配器转换为 SI 单位。
驱动给 IMU 标注 `airy_201` 不代表其物理轴与点云坐标轴相同，不能因此填写单位外参。

在山猫上构建（只构建，不启动驱动或运动）：

```bash
# 先 source 已安装的真实 livox_ros_driver2 包；这是后端的编译依赖，Airy 不使用 Livox 设备。
export LYNX_LIVOX_SETUP=/absolute/path/to/livox_workspace/install/setup.bash
LYNX_BUILD_LIO=true bash deploy/lynx/build.sh
```

构建还需要 PCL、Eigen、pcl_ros、pcl_conversions、tf2_ros 和系统 Python 开发库。
脚本将固定版本源码和子模块放到 `.lio-src`，产物放到 `.lio-install`；不更改系统安装。
运行时可用 `LYNX_LIO_SETUP` 指定已构建的后端，用 `LYNX_LIDAR_SETUP` 指向 rslidar 工作空间。

配置步骤：

1. 将 `lio.example.json` 的对象填入独立机器人配置的 `lio` 字段，保留 `motion_enabled=false`。
2. 确认话题、96 线配置、逐点时间语义、扫描周期和 IMU 噪声；示例噪声仅为初始参数。
3. 测量 `base_from_imu`、`base_from_lidar`，分别填写 4×4 刚体变换，含义为传感器坐标转换到 `base_link`。
   `imu_frame` 表示实际 IMU 物理轴；`imu_source_frame` 记录驱动原始标签。不能直接照搬未核实的 URDF。
4. 确认单位、时间和外参后，将三个 `*_confirmed` 与 `enabled` 设为 true，`odom_topic` 设为 `/outdoor/lio/odom`。
5. 若由本服务管理接收驱动，设置 `start_driver=true`，`driver_config` 指向 `airy_lio.yaml`（相对机器人配置文件）。
   启动前检查原始话题发布者及 UDP 端口占用；不得与原驱动同时启动。

先执行 `start.sh --diagnostic`。输入适配器检查时间递增、断流、点时间、坐标系和非有限值；
将 FAST-LIO 的 IMU 原点位姿通过外参转换为机身原点，保留源时间，检查重置／异常运动。
`/outdoor/lio/status` 未稳定就绪或过期时，现有定位及看门狗保持未就绪。
适配器不向实机发布速度；诊断模式仍隔离最终速度输出。它不提供完整的几何退化评分，稳定输出不代表精度已验收。

当前默认 `lio.enabled=false`，外参未确认。真实接收采样不等于融合已通过；
还需隔离回放、静止漂移、直行／转弯、退化环境及 GNSS 对齐验收，之后才能确认里程计与运动条件。

后端与 ROS 包构建后，可运行隔离的合成数据接入检查：

```bash
source .lio-install/setup.bash
source .ros-install/setup.bash
.venv/bin/python tests/lio_ros_smoke.py
```

该脚本固定 ROS 域 89，只运行适配器、FAST-LIO 和合成输入，验证连续输出、断流及 TF／速度隔离；不使用实机传感器。

## GNSS／IMU 路径试验

当前新增的是**离线可行性原型**，尚未作为 Nav2 的定位提供者，也不会改变正式配置。
原型使用 GNSS 的 ENU 位置和接收机多普勒速度约束平移，用校正零偏后的陀螺仪跟踪转向；
不积分加速度，不根据静止／低速 GNSS 推断朝向。初始 ENU 朝向、误差范围、陀螺零偏及安装偏移必须显式提供。
这是平面 GNSS／角速度融合试验，不是完整 GNSS／惯导系统，GNSS 不能在此模型中消除航向漂移。
原型始终报告 `motion_allowed=false`，不发布 ROS 话题或 TF。

只读比较两个 IMU（不订阅 `/ODOM`）：

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python deploy/lynx/sensor_check.py --seconds 8 --imu /IMU --imu /IMU_103 --report /tmp/outdoor-imu.json
```

离线验证 `tests/test_gnss_imu.py` 覆盖已知弯道、旋转天线偏移、速度单位、断流、异常跳变和航向不确定性增长。
当前原型仅接受同一时刻的 IMU／GNSS 数据；实际接入前还须处理接收机与 ROS 时钟对齐、延迟补偿，
确认 IMU 坐标系及噪声、初始朝向和零偏标定，增加航向误差放行门槛，再做隔离 ROS 联调。

## 配置和定位

室外复盘的离线修复说明见 [2026-10-28 修复记录](../../docs/fieldtest-fixes-2026-10-28.md)。
现场演示档 `robot.fieldtest.json` 显式关闭 GGA 改正年龄门槛；通用／默认配置仍保留该门槛，其他定位质量检查不变。

`deploy/lynx/robot.json` 只保存部署值，通用默认值集中在 `outdoornav/config.py`：

- **不使用 `/ODOM`。**`odom_topic` 默认留空，诊断服务不订阅该话题，配置加载也拒绝将它设为定位源。
- 当前 `localization_mode=rtk_imu` 不订阅外部里程计，`odom_topic` 保持空；`odometry_confirmed` 是旧路径字段，不作为新模式放行项。新模式的 `feedback_confirmed` 只有验证真实多普勒速度、角速度和停车反馈后才可确认。
- 实测新 IMU 到车体的安装外参、磁北到真北参考、磁场校准与实际航向精度，以及原 RTK 天线杆臂和地图原点；未知值保持 null，确认项保持 false。当前模式不使用 `initial_heading_rad` 或运动航迹角做默认航向。
- 实测保守包围机器人运动外形的半径，验证雷达高度带和覆盖后设置 `footprint_confirmed`、`cloud_frame_confirmed`。
- `motion_enabled` 默认 false；完成以上确认后才允许现场运动。
- NTRIP 凭据继续保存在 `/home/nvidia/ntrip/ntrip.conf`，由其虚拟环境运行；文件锁确保单一串口读取者。

当前 `robot.json` 为米级演示档：固定解和浮点解均要求接收机报告的 hAcc≤3.0 m，到达距离容差均为 3.0 m。仍要求有效定位、差分标志、对应 GGA 解类型及改正年龄≤5 s，稳定 5 s。普通差分／单点／无效解未因此放开。通用 `DEFAULTS` 保留更严格值，部署值以 `robot.json` 覆盖。
缺少改正年龄保持未就绪，不能用 RTCM 到达时间替代。数据重复不刷新有效期，传感器过期停止运动。
任务固定验收档位；固定解任务降为浮点解时仍停车。当前演示档固定／浮点到达容差均为 3.0 m，但仍需终态与停稳确认；它不是实际地理误差≤3 m的保证。测试目标应明显远于3 m，避免起点已在容差内而无需行驶。

当前定位直接消费 RTK 位置／多普勒速度与新 IMU 姿态／角速度。内部生成 Nav2 的 Odometry 消息及局部连续参考，不是额外依赖本体里程计，也不是完整 GNSS／惯导融合；真实误差与延迟仍须实测。
数据时间倒退、异常位置或航向跳变、IMU 输入中断／采集重启会锁止，检查后重启。原本只靠位置差分或控制指令推算运动的路径不启用；不自动回退到厂商 `/ODOM` 或 ICP。

## RTK＋新 IMU 直接运动反馈（当前默认）

默认 `localization_mode=rtk_imu`、`heading_source=wit_imu`，不订阅本体、轮腿、ICP 或 LIO 里程计。位置唯一来源仍是原 u-blox NAV-PVT；速度使用同一接收机的北／东多普勒速度和报告的速度精度。新 WIT 十轴 IMU 提供磁参考欧拉角与三轴角速度；不积分加速度、不把控制指令当实测速度，也不靠位置差分估计速度。

内部 `odom→base_link` 由有效多普勒速度和角速度在局部参考系连续推进；`map→odom` 由接受的 RTK 位置与 IMU 绝对朝向计算，GNSS 校正不直接跳动局部里程计。世界位置取最近 RTK 测量，最多在数据新鲜期内用其多普勒速度作短时外推，状态中保留实际 `fix_age_s` 和 `feedback.prediction_s`；不重复刷新旧 RTK 有效期。任一必要数据不可用时不放行导航。

`wit_imu` 的关键配置：

- `port`：明确指定新 CP2102 串口 by-id 路径，不回退到原 RTK 端口；默认 9600 波特率、UDP 18767。
- `base_from_imu`：实测 4×4 传感器到车体变换，旋转同时用于角速度轴转换和姿态转换。未知保持 null，不能默认 IMU Z 轴就是车体竖直轴。
- `magnetic_declination_rad`：东偏为正的磁偏角；厂家九轴角度按磁参考 ENU 解读，真 ENU 角为磁 ENU 角减去该值。必须结合实际地点、日期和独立方向参照核对，不默认零偏。
- `yaw_std_deg`：验证后的航向不确定度，不把厂家标称 1°或静止抖动当作实测绝对误差；未知保持 null。
- `mounting_confirmed`、`reference_confirmed`、`quality_confirmed`：安装、真北参考及磁校准／航向质量的现场确认，默认全部 false。

`wit_stream.py` 仅独占采集并发送 `FF AA 27 24 00`（READADDR 查询 AXIS6）确认九轴模式，先丢弃旧缓冲回复；不解锁、不切换算法、不标定、不保存设置。节点从 collector 元数据和原始校验帧确认当前模式，六轴、未知模式或重启后的旧数据不作为有效绝对航向。

速度要求为合格 RTK 解、有效多普勒速度及报告的速度不确定度不超过 `gnss_speed_accuracy_mps`（默认质量策略 0.10 m/s，非实测性能保证）。缺失或零精度字段不当成“零速度”。原 RTK 天线旋转杆臂的速度也会扣除，再转换成车体速度。

默认 `motion_enabled=false`、`feedback_confirmed=false`。旧 `odometry_confirmed=false` 不需要为新模式改成 true。角度帧失效时撤销导航许可；若陀螺与多普勒仍可靠，保留测量反馈以确认停车。全部 IMU 或 GNSS 丢失时不能伪造零速度来宣称已停，按看门狗及执行器的超时／锁止处理。

诊断默认 `LYNX_GNSS_MODE=observe`、`LYNX_IMU_MODE=observe`，不抢串口；确认两接收机无占用后，才可显式选择 managed 在诊断中检查真实输入。动态 TF 仍隔离到 `/outdoor/tf`，正式 manifest 和底盘程序不自动修改。

本地 Docker 完整测试用 `python3 tests/isolated_task_flow.py --rtk-imu`：只提供 5 Hz 合成 RTK 位置／多普勒与 10 Hz 二进制 IMU，不提供外部里程计；真实网页、任务与 Nav2 控制虚拟底盘。它不是实车精度验收。

## 室内模拟 RTK（调试全流程） 

`deploy/lynx/mock_virtual_base.py` 按真实接收机的同一格式（UBX NAV-PVT＋NMEA GGA，同一 UDP 端口与主机单调时间前缀）输出合成定位，定位与导航代码不改；
虚拟底盘只积分被重映射的指令话题 `/mock_base/cmd_vel`，**不产生真实速度、不启动速度桥**，适合室内验证“网页选点→任务→Nav2→到达”。
真实 IMU 仍可用；航向使用 `course_heading`（磁航向起步、按实测航迹方向校正），页面上显示的是虚拟位姿。

启动（山猫，独立工作目录）：

```bash
bash start_mock.sh   # 内部：mock_virtual_base + wit_stream + ros2 launch run/mock.launch.py
curl -s -X POST -H 'Content-Type: application/json' -d '{"active":true}' http://127.0.0.1:8766/api/activation
```

网页端口 8766。真实模式仍用 `start_production.sh`（端口 8765、真实 `/cmd_vel`＋`cmd_vel_bridge.py` 转发到 `/NAV_CMD`）。
演示配置 `deploy/lynx/robot.fieldtest.json`（真实）与 `robot.mock.json`（模拟）记录了三米档、安装矩阵、以及**未实测项**（天线杆臂、磁偏角、航向绝对精度）；
`scan_range_min_m=3.0`（真实演示档）用于剔除前雷达 3 m 内的自车腿部回波（行走时前腿摆过 1.2 m，1.2 m 过滤会让 DWB 所有轨迹都判碰撞）；mock 档保持 1.2。

## 网页和客户端

远程看车状态用相机网页工具，说明见 [相机网页](../../docs/camera-view.md)。

启动后访问 **http://10.29.1.4:8765/**，无需电脑运行 Vite 或 SSH 转发。
网页静态文件与 `/api/*` 同源，默认监听 `0.0.0.0:8765`。

- 点击道路附近目标（最多偏离 2 m），或选择已确认地点；规划后可执行、取消或替换任务。
- 绿色为参考路线，蓝色虚线为当前执行路径；支持封路后重新选路。
- Robonix 先查询地点，以地点 ID 调用导航，再以返回的 `run_id` 查询或取消。
- 未确认地点可以预览，但不能通过地点 ID 执行。重启将旧活动任务标为中断，确认停稳后才接受新任务。

## 只读检查与现场验收

当前机器人在室内，没有 RTK 浮点／固定解属于预期情况。先使用模拟数据与零输出联调；不要因此降低定位门槛。

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python deploy/lynx/sensor_check.py --seconds 8 --report /tmp/outdoor-sensors.json
```

该工具不发布运动指令、不占 GNSS 接收端口。静态采样不能证明里程计可用于导航。
室外所有定位、规划、控制及雷达投影共用 `/outdoor/tf`；厂商公共 `/tf` 不进入室外导航。
共享 `/tf_static` 仅提供传感器静态外参，必须能够连接雷达与 `base_link`。
静态定位根冲突、外参不一致、室外 TF 重复发布或所选速度输出被占用时拒绝启动。
诊断输出与真实底盘分开；生产模式仍要求 `/cmd_vel` 唯一。检查不会关闭其他进程。

检查报告包含静止漂移、推算／报告速度差、时间异常和点云采样统计（每帧最多 10000 点）。
0.10 米静止漂移、0.05 米/秒及弧度/秒仅用于标记静止异常，不是运动验收标准。
趴下时的点云高度统计不能确认站立后的避障覆盖，非有限数值会在 JSON 中写为 null 并计入异常。

构建后可运行隔离域故障联调，以及真实输入诊断启动／退出检查：

```bash
source /opt/ros/humble/setup.bash
source .ros-install/setup.bash
.venv/bin/python tests/lynx_ros_smoke.py
.venv/bin/python tests/lynx_ros_smoke.py --live-diagnostic
```

前者固定使用 ROS 域 87、localhost 和模拟输入；后者使用当前实机域及原始配置，
强制诊断输出和 GNSS observe 模式，不注入定位、不接受导航、不修改正式部署。

先做定位检查和短距离到达，再测试取消停车、障碍绕行、堵路停车、GNSS／雷达断流和进程退出。
最后测量独立目标误差并验证重启。真实运动由现场人员执行，记录见 `docs/lynx-validation.md`。

## 本地隔离完整任务测试（不连接山猫）

`tests/isolated_task_flow.py` 只允许在 Docker 容器中运行，固定 ROS 域 94 和 localhost。
它使用真实网页、HTTP／任务执行器、NavFn、DWB 与速度看门狗，传感器和底盘运动学是合成的；
不读取串口、不调用本体姿态接口、不发布规范 `/cmd_vel` 或 `/NAV_CMD`。
仅最终看门狗出口与执行器的输出校验采用按节点限定的 ROS 重映射，发送到 `/isolated_task_flow/cmd_vel`；
内部控制器、平滑器的连线保持生产方式，避免全局重映射制造多发布者冲突。

先在本地测试容器构建／加载 ROS 包和网页，确保 `pointcloud_to_laserscan` 及 Nav2 可用，随后运行：

```bash
# 以下命令在本地测试容器内执行，不在山猫执行。
source /opt/ros/humble/setup.bash
source /path/to/test-install/setup.bash
export PYTHONPATH="$PWD:$PWD/ros_ws/src/outdoornav_ros:${PYTHONPATH:-}"
python3 tests/isolated_task_flow.py
```

脚本启动独立端口 19865（测试网页／API）和 19866（仅虚拟环境控制），自动运行
`web/playwright.live.config.ts` 的浏览器测试；不拦截或伪造业务 API 响应。
到达由真实导航结果和虚拟底盘的位姿／速度反馈共同判定，不直接把任务改成成功。
临时配置中的标定确认只针对合成场景，不能复制到实机配置。

产物在 `artifacts/validation/isolated-task-flow-20261023/`：任务历史、虚拟轨迹、浏览器截图／日志及报告。
退出会停用测试服务并清理本轮标记的子进程，正常模式的 `robot.json` 保持运动关闭。
该测试只验证软件任务流程，不代表实机定位、避障精度或真实停车验收。

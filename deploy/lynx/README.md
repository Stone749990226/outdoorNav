# 山猫室外接入

本仓库新增独立室外运行配置，复用现有有序路线执行器，使用 NavFn、DWB、速度平滑和独立速度看门狗。
Robonix 核心和机器人现有 NTRIP 文件无需修改。室外运行时必须停用原 nav2 和 mapping，不能同时启动两个导航执行器或两个 `map→odom` 发布者。

## 构建与接入

在山猫上克隆本仓库。依赖 ROS 2 Humble、colcon、uv、rbnx，以及
`nav2_planner`、`nav2_controller`、`dwb_core`、`nav2_velocity_smoother`、
`nav2_lifecycle_manager`、`pointcloud_to_laserscan`。

```bash
bash deploy/lynx/build.sh
```

脚本创建使用系统 Python 3.10 的 `.venv`，构建两个 ROS 包及 MCP 类型；不启动机器人。
GNSS 仍使用 `/home/nvidia/ntrip/.venv`。`gnss_stream.py` 取得原 `client.lock` 后包装原 `client.py`，
保留 NTRIP 补发 GGA 和串口唯一读取；通过本机 UDP 把接收机原始数据交给定位节点。
不要同时运行原 `ntrip/start.sh`。凭据不进入本仓库。

在山猫主工程的 `robonix_manifest.yaml` 中，用以下条目替换原 nav2、mapping 服务；保留底盘、雷达和机身 TF：

```yaml
service:
  - name: outdoor_navigation
    path: /home/nvidia/outdoorNav/integrations/robonix_outdoor
```

路径按实际克隆位置修改。包启动脚本同时管理室外 ROS 栈、GNSS 包装进程和 MCP Service。
Atlas 由 rbnx 从包内 `capabilities/` 加载五个契约；须重新启动部署才能发现新增契约。
直接调试 ROS 栈可运行 `bash deploy/lynx/start.sh`，此时不要再启动上述 Service。
启动预检发现已有导航、建图节点或 `/cmd_vel` 发布者会拒绝启动，不会替你关闭其他进程。

## 地点与现场配置

`scenes/north_campus/places.json` 提供出发点 S、公共路口 J、支路取餐点 P。
坐标来自现有路网，不代表已经现场确认可达；默认 `enabled=false`。
在现场调整停车坐标、ENU 朝向 `yaw` 后才启用。地图采用 WGS84 局部 ENU，不是 GCJ-02。
地点文件与 `route_version` 绑定；修改地点或道路后重启服务，并重新构建网页，任务不会自动恢复。

复制 `robot.json` 为现场配置，用 `LYNX_ROBOT_CONFIG` 指定绝对路径。需填写：

- `initial_heading_rad`：**本次启动首帧**机器人朝向，东为 0、北为 π/2；机器人重定位或重新启动后重新确认。
- `antenna_xy_m`：天线在 base_link 平面内的前、左偏移；`map_offset_xy_m`：独立测量得到的地图平移校正。
- `road_widths_m`：loop-a、loop-b、branch 的实测可行驶宽度；当前示意道路宽度不会用于实车放行。
- `footprint_radius_m`：覆盖整机外轮廓的实测半径，含必要间隙；不是仿真机器人的尺寸。
- 点云话题、相对 base_link 的高度截取范围，以及已有 URDF/TF 中正确的雷达外参。
- `calibrated`、`cloud_frame_confirmed`：标定和雷达有效覆盖通过检查后设置为 true。
- `motion_enabled`：默认 false；现场准备运动测试时显式启用。

定位以原 `/ODOM` 时间戳和接收时刻对齐 GNSS；目前采用固定初始航向、GNSS 平移渐进校正。
这不是完整惯导融合器，不能自动消除长期航向漂移；需要现场验证时间延迟和航向误差。
连续 5 秒固定解、hAcc≤0.20m、GGA 改正数年龄≤5秒才放行。GGA 缺少年龄时保持未就绪，不能用 RTCM 到达时间代替。
里程计重置、超过阈值的位置跳变会锁止，需检查标定后重启。

## 网页与客户端

后端 HTTP 只监听山猫回环地址。你的电脑可使用隧道：

```bash
ssh -N -L 8765:127.0.0.1:8765 nvidia@10.29.1.4
cd web && npm run dev
```

也可设置 `OUTDOORNAV_API_URL` 指向另一个受控 HTTP 转发地址；浏览器始终通过同源 `/api` 访问。
网页自动进入 A→B 工作区，显示位置、RTK 质量、命名地点及任务。定位未就绪时仍显示收到的位置，但禁止执行。
选择已启用地点，预览后执行；取消后等待“已取消并停车”。实机模式不提供 Gazebo 障碍物注入。

Robonix 客户端输入“走到公共路口”。Pilot 先查询 `outdoor/places`，再以 ID 调用 `outdoor/navigate`；
Executor 通过同名 `/status`、`/cancel` 契约跟踪整次任务。不要绕过室外 Service 调用原单点导航。
任务状态落盘；重启把未完成任务标记为失败，不恢复运动。

## 验收边界

本地已通过 26 项 Python 核心测试、16 项前端单元测试、网页构建和相关浏览器流程检查；
18 项依赖 ROS 环境的既有测试跳过。包清单校验和五个 MCP 类型生成/适配器检查通过，尚未验证实际 Atlas/Executor 传输。

本地单元测试和浏览器接口测试不代表实车验收。最后由现场验证：独立实测到达误差、目标朝向、
弯道连续跟随、边界内绕障、堵路停车、取消与替换、GNSS/雷达断流、里程计重置，以及进程退出后的真实停车。
看门狗独立于导航进程，输入超时输出零速度；整机断电或看门狗本身退出后的制动仍依赖厂商底盘命令超时机制，必须现场验证。

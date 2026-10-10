# 三维园区与导航验证报告

验证日期：2026-10-09（北京时间）。场景几何版本 `campus-06b547b8ec03`，实测路网版本 `survey-388df4ebbef8`。

## 已实现和验证

Gazebo 11 运行三维园区、实体碰撞、差速轮关节及激光雷达；RViz 默认显示相同三维建筑、地面、绿化、机器人和轨迹，可旋转缩放并发送 Nav2 Goal。Nav2 在地面二维自由空间规划并控制三维仿真机器人。主楼 90 米、裙楼 12 米，建筑尺寸与路宽为仿真假设。

环境为 WSL Ubuntu 22.04 / ROS2 Humble / Gazebo Classic 11.10.2。WSLg OpenGL 报告为 Microsoft D3D12 / Intel UHD Graphics，支持加速。Gazebo 和 RViz 的窗口均已实际启动和检查，Gazebo 相机在斜视、俯视、广场近景之间切换成功；截图中的 Gazebo 显示约 62 FPS，RViz 约 31 FPS，仅代表该次窗口观察。

## 实测轨迹与几何

- 原始、滤波、最终路网的连续轨迹均被可通行区域覆盖，包含原路网裁掉的引入段。
- 所有轨迹覆盖率为 100%；最小中心线到禁行边界距离 **1.1613 米**，大于要求的 0.85 米保护距离。
- 可通行面积约 **3675.40 平方米**。导航栅格 **1380 × 1310**，分辨率 **0.1 米**，原点 **(-20, -103)**，X 东 / Y 北。
- 6 项场景测试通过：连续轨迹保护带、栅格整格保守通行、三维碰撞体与地图一致、道路网格孔洞及朝上法向、广场绕障测试有效性、不合法障碍物布局拒绝。
- 原有采样处理工具的 9 项测试通过。原始日志、测绘输出、网页代码和旧 ROS 桥接地图没有修改。

详见 [几何报告](generated/validation.json) 和 [测试输出](evidence/geometry_tests.txt)。

## 实际导航结果

下表记录真实 Gazebo 位姿和 Nav2 动作结果；并对记录的连续运动折线检查机器人 0.5 米半径所需净空。

| 案例 | 结果 | 终点误差 | 实际运动最小边界净空 |
| --- | --- | --- | --- |
| 环线路段 | 成功 | 0.267 米 | 1.569 米 |
| 支路进入环线 | 成功 | 0.266 米 | 0.997 米 |
| 道路进入楼前广场 | 成功 | 0.282 米 | 3.393 米 |
| 广场斜向绕花坛 | 成功 | 0.289 米 | 1.095 米 |
| 建筑内部目标 | 正确中止，无移动指令 | 不适用 | 不适用 |
| 园区外目标 | 正确中止，无移动指令 | 不适用 | 不适用 |

全部任务碰撞事件为 **0**，指令线速度不超过 **0.45 米/秒**。完整无界面测试接收 **1804 帧有效激光**，以及 **3610 条接触传感器消息**。广场斜向路径的起终点直线与花坛相交，但实际运动绕开花坛，最远离原中心线 **26.03 米**，证明已支持区域自由导航。

另外在桌面模式执行广场绕障任务成功，终点误差约 0.292 米。修改三维显示后，带正常 Gazebo 渲染窗口再次验证道路临时箱体绕行：在 **(98.8, -42)** 加入 **1.6 × 1.6 × 1 米**实体箱体，未写入静态地图，机器人通过激光发现并绕行成功，终点误差约 **0.279 米**、碰撞事件 **0**。具体净空以 [路障后验报告](evidence/road_obstacle_validation.json) 为准。

原始记录及可视化：

- [完整无界面验收](evidence/navigation_validation.json) · [实际运动路径图](evidence/navigation_paths.png)
- [桌面广场验收](evidence/desktop_navigation_validation.json)
- [临时道路障碍物验收](evidence/road_obstacle_validation.json) · [实际绕行路径](evidence/road_obstacle_paths.png)

## 三维窗口截图

以下图片是实际运行窗口截图，未使用绘图替代 Gazebo：

- [Gazebo 斜视](evidence/gazebo_oblique.png)
- [Gazebo 俯视](evidence/gazebo_top.png)
- [Gazebo 广场近景](evidence/gazebo_plaza.png)
- [RViz 三维导航视图](evidence/rviz_overview.png)

WSLg 普通窗口抓图曾返回全黑，最终通过 XComposite 抓取窗口像素；复现工具为 `scripts/capture_window.py`。Gazebo 相机预设使用 `scripts/set_camera.py`，不需要额外 GUI 插件。

## 运行时修复与适用范围

- **ROS Python：** Gazebo 的部分 ROS 脚本使用 `/usr/bin/env python3`。启动时将系统 Python 放在 conda 之前，避免 ROS Python 3.10 扩展被 Python 3.12 加载。
- **Gazebo 黑屏：** 最终将服务端和客户端的 `GAZEBO_IP`、`IGN_IP` 都设为 `127.0.0.1`，修复本次 WSL 多接口下渲染服务超时、窗口无帧的问题。
- **TF 停更：** 调试线程栈发现 `tf2::BufferCore::testTransformableRequests` 与 `tf2_ros::Buffer::waitForTransform` 的锁等待，发生于激光过滤与 TF 更新并行时。单独改 DDS 无法解决。场景节点现在缓冲激光，待后续 0.2 秒 TF 发布后再转发，保留原扫描时间戳，并丢弃超过 1 秒的旧扫描；完成跨起点重置的连续完整测试后未复现。
- **本机通信：** 使用 Cyclone DDS 的回环 UDP 配置，消息按较小数据报分片，ROS 域默认为 42。已安装 `ros-humble-rmw-cyclonedds-cpp`。这不是定位算法改进，定位仍为 Gazebo 理想位姿。
- **显示一致性：** RViz 三维 Marker 从 Gazebo SDF visual 读取，地面网格使用相同 COLLADA 文件。道路与广场显示网格去除重叠、三角面法向朝上，避免黑面和重叠闪烁。

这是静态平坦园区中的仿真验证，未模拟四足步态、坡道、台阶、定位漂移、动态行人或真实 RTK 接收器噪声。临时路障专门允许占用原实测轨迹；基础地图依然保持 RTK 通行保护带。操作入口和复现命令见 [使用说明](README.md)。

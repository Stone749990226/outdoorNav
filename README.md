# OutdoorNav - 实测路网与 Nav2 路径跟踪

智慧谷北区的闭合环线和中央支路，来自 `1.ubx`、`2.ubx`。网页支持中心线选点、A→B 最短路径、多点巡航、封路重规划、路径导出和 Nav2 仿真反馈。

两份日志均为差分解质量 `2`，没有 RTK 固定解。道路具有实测米制尺度，绝对位置精度与道路宽度未独立验证；当前不连接真实机器狗。

新增独立的 [Gazebo 三维园区与自由导航](gazebo_campus/README.md)：实测轨迹周围保留连续通行空间，增加不同宽度的道路、楼前广场、建筑与障碍物，通过 Gazebo + RViz + Nav2 在二维区域内选点导航。运行 `bash gazebo_campus/run.sh`；此模式的建筑与宽度为仿真假设，使用独立导航地图，保留下面的网页中心线演示。

## 重建地图

没有仓库 venv 时使用 conda 的 torch 环境：

```bash
conda run -n torch python tools/build_survey_map.py
```

脚本校验日志、识别跳点和停留、平滑并简化轨迹，裁掉环线前导段，闭环并接入已确认的支路路口。原点保存在 `survey/north_building/origin.json`，重复生成继续使用同一原点。

输出原始及滤波点、米制路网、处理报告和 [独立 SVG 图](survey/north_building/map.svg)，并同步网页与 ROS2 地图。环线约 345 米，支路约 36 米；闭环修正约 0.47 米，路口修正约 2.98 米。

`--no-install --output /tmp/survey-preview` 可生成独立结果而不替换网页和 ROS2 地图。旧的 `prepare_demo.py` 仅用于历史手绘图，不能覆盖当前实测地图。

## 运行

Node.js 22.12 或更新版本：

```bash
cd outdoor-patrol-demo
npm ci
npm run dev
```

打开终端显示的地址。A/B 点击位置投影到中心线；只沿已记录路线规划，不推断道路两侧自由区域。

Ubuntu 22.04 / ROS2 Humble 另一个终端启动：

```bash
bash ros2_bridge/run_sim.sh
```

A→B 规划后点击“发送仿真路径”，通过 `FollowPath` 控制器驱动仿真机器人，并显示位置、剩余距离、结果及取消操作。

**仿真 PGM 是无障碍测试区域，不代表真实可通行地面。** 路径、网页和桥接使用同一 `map` 原点及地图版本；导出始终标记 `executable_on_robot: false`。

## 验证

```bash
conda run -n torch python -m unittest discover -s tools -p 'test_*.py'
cd outdoor-patrol-demo
npm test
npm run build
npx playwright install chromium
npm run test:e2e
```

Nav2 已启动时，用 `BRIDGE_REAL=1 npm run test:e2e` 验证真实控制器的环线路径、支路过路口、取消和版本拒绝。

## 文档

- [项目说明](outdoor-patrol-demo/README.md)：交互、算法、坐标及导出格式。
- [RTK 采图指南](docs/RTK_MAPPING_GUIDE.md)：采集、多日志处理与后续背景配准。
- [实测地图报告](docs/SURVEY_MAP_REPORT.md)：当前数据、生成物及验收。
- [历史配准与 Nav2 报告](docs/RTK_NAV2_REPORT.md)：旧手绘地图的两点暂定配准记录。
- [实机导航路线图](outdoor-patrol-demo/MAPPING_ROADMAP.md)：定位、传感器与机器狗接口。
- [ROS2 源码笔记](docs/ROS2_SOURCE_NOTES.md)：上游源码学习入口。

独立克隆的上游源码和本机机器狗手册不属于本项目提交内容。

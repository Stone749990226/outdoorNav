# 室外导航模拟

在具备 ROS 2 Humble、Nav2 和已构建 Robonix 依赖的机器上运行。
只替换底层原语，使用真实定位、地图、导航和 Scene 服务；不连接真实串口、底盘或 `/ODOM`。
默认 ROS domain 176、Scene 50118，与实机 domain 0/80、Scene 50108 分开。

## 启动

依赖来自此工作区时直接执行；Python 需能导入 ROS，推荐用 uv 创建带系统包的环境：

```bash
uv venv --system-site-packages --python /usr/bin/python3 simulation/.venv
uv pip install --python simulation/.venv/bin/python -e robonix/pylib/robonix-api \
  -r robot-deep_robotics-lynx_s10/outdoor/requirements.txt
# 在已有部署上可直接复用其 uv 环境和已构建依赖：
SIM_DEPS_ROOT=/home/nvidia/robot-deep_robotics-lynx_s10/outdoor bash simulation/start.sh
```

在山猫上运行时，电脑转发两个模拟端口：

```bash
ssh -p 6003 -N -L 19890:127.0.0.1:19890 -L 50118:127.0.0.1:50118 nvidia@svr.gddg08.top
```

打开 `http://127.0.0.1:19890/`。黄色栏始终标识模拟模式，下面嵌入真实 Scene。
默认加载 `mock-demo` 地图，并提供“快递点”；在 Scene 中预览、执行和取消。
控制栏可以切换 FIXED/FLOAT、拔出 IMU、断开底盘反馈、停止步态响应、加入速度噪声。
Ctrl-C 停止本次模拟；重新启动后虚拟山猫回到配置起点。

`simulation/config/default.json` 配置起点（x 向东、y 向北，yaw 东为 0°、逆时针）、
端口、速度和到点容差。默认模拟到点容差为 0.5 米、滚动空场为 20×20 米，实机仍使用自己的现场配置。
机头沿 IMU 负 Y 轴，模拟数据精确采用该安装关系，不借用实机未验证的磁航向误差。

## 旧地图与检查

```bash
# 复制旧地图到模拟目录；不修改原文件。起点需在该图道路附近。
SIM_DEPS_ROOT=/home/nvidia/robot-deep_robotics-lynx_s10/outdoor bash simulation/start.sh \
  --map /home/nvidia/robot-deep_robotics-lynx_s10/outdoor/data/outdoor_mapping/maps/outdoor-20261106-dff1ba32/map.json

# 执行到点、步态、取消、IMU 断流与恢复检查，完成后退出：
SIM_DEPS_ROOT=/home/nvidia/robot-deep_robotics-lynx_s10/outdoor bash simulation/start.sh --smoke
```

更换地图或同时开第二套模拟时，使用独立 `--run-dir` 和不同端口/domain 的 `--config`。
模拟数据、标注、日志及 `smoke-result.json` 默认保存在 `simulation/run/`。
启动只读依赖源码和现有生成文件，不运行真实原语的启动脚本，不读取 NTRIP 凭据。
依赖需提前完成自己的构建；此工具不自动升级或修改服务仓库。
需要比较导航服务版本时，可用 `SIM_NAVIGATION_ROOT` 指定另一份已构建的导航仓库，
不覆盖实机正在使用的服务。

山猫上的独立模拟实例位于 `/home/nvidia/outdoorNav-simulation`，复用正式部署的服务源码：

```bash
cd /home/nvidia/outdoorNav-simulation
SIM_DEPS_ROOT=/home/nvidia/robot-deep_robotics-lynx_s10/outdoor \
bash simulation/start.sh
```

旧 `dependencies/service-navigation-outdoor-rbnx` 和 `navigation-update` 已改为指向正式
导航包的链接，不再维护第二份导航源码；模拟运行数据仍单独保存。

虚拟底盘根据收到的速度计算运动，独立发布 RTK/IMU，真值不直接传给定位服务。
固定无障碍输入只让真实导航链路运转，不验证避障、真实磁干扰、USB 连接可靠性或楼梯通过能力。
模拟通过后再切换实机启动配置验收。

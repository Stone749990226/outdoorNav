# 部署和使用

以下在山猫上执行。源码位置为 `robot-deep_robotics-lynx_s10/outdoor`。
本地工作区的同名服务目录用于开发；首次获取子仓库源码使用
`scripts/prepare.py`（配置见 `tools/sources.json`），现场部署统一同步本地当前源码。

## 同步到山猫

本地当前源码（含未提交修改）统一通过以下命令同步。先在山猫用 `rbnx shutdown`
停止正在运行的清单，再在电脑执行：

```bash
bash tools/sync_robot.sh --dry-run
bash tools/sync_robot.sh
```

唯一正式目录为 `/home/nvidia/robot-deep_robotics-lynx_s10/outdoor`。
同步覆盖部署、服务、框架与三个山猫原语，保留地图、Scene 标注、日志、虚拟环境
和私密配置；另将整个工作区源码镜像到 `/home/nvidia/outdoorNav-workspace/`，包含未提交文件及根目录工具、文档。
不会启动服务。简明步骤见根目录[同步到山猫.md](../同步到山猫.md)。之后在山猫加载环境、构建，再使用下面的启动命令。
`prepare.py` 按 `tools/sources.json` 把各子仓库更新到分支最新，会覆盖未提交修改；开发中的修改请在同步到山猫之后再运行。
旧 `./outdoor` 和 `robot_outdoor/cli.py` 已移除。

## 现场路线预览

实现与模块分工见[室外语义地点路线预览](OUTDOOR_PLACE_PREVIEW.md)。

此模式直接读取山猫已有的地图和地点标注，不启动传感器、底盘或导航执行。需要使用当前工作区源码构建的 Robonix；在山猫的 `framework/robonix` 目录运行 `RUSTUP_TOOLCHAIN=stable make install CARGO_FLAGS="--locked --debug"` 替换旧 CLI 和组件，然后回到正式部署目录。

空地图库会自动导入并加载 `example-campus`。已有地图时不会自动导入或切图。

停止其他部署后，在正式部署目录执行。预览与日常检查复用同一份 `robonix_manifest.yaml`：

```bash
export OUTDOOR_MODE=preview
source scripts/environment.sh
export ROBONIX_VLM_UPSTREAM=https://api.deepseek.com
export ROBONIX_VLM_MODEL=deepseek-flash
rbnx build -f robonix_manifest.yaml
read -rs -p '模型 API Key: ' ROBONIX_VLM_API_KEY; printf '\n'
export ROBONIX_VLM_API_KEY
flock -n run/boot.lock rbnx boot -f robonix_manifest.yaml --no-update-check
unset ROBONIX_VLM_API_KEY
```

Robonix 随清单启动 Pilot，不用另开 Pilot 脚本。换 OpenAI 兼容服务时，换 API 基础地址（不含 `/chat/completions`）、模型名和密钥即可。

1. 在电脑运行 `ssh -p 6003 -N -L 50108:127.0.0.1:50108 nvidia@svr.gddg08.top`，打开 `http://127.0.0.1:50108/outdoor`，加载地图并标注地点。
2. 在山猫另开终端运行 `rbnx chat --server 127.0.0.1:51551`，例如输入「帮我在室外导航到快递点」。Scene 从当前地图第一个已录道路点画出路线，只报告距离，不执行。

## 首次准备

需要 ROS 2 Humble、drdds 消息、RoboSense SDK、Nav2（含 regulated pure pursuit）、
Robonix CLI/codegen 和 uv。工作空间路径可用 `ROS_SETUP`、`DRDDS_SETUP`、
`RSLIDAR_WS` 覆盖，默认分别是 `/opt/ros/humble/setup.bash`、
`$HOME/drdds_build_ws/install/setup.bash`、`$HOME/rslidar_ws`。

```bash
cd /home/nvidia/outdoorNav-workspace   # 主仓库根目录，拉取各子仓库分支最新
python3 scripts/prepare.py
cd /home/nvidia/robot-deep_robotics-lynx_s10/outdoor
uv venv --system-site-packages --python /usr/bin/python3 .venv
uv pip install --python .venv/bin/python -e framework/robonix/pylib/robonix-api -r requirements.txt
```

`prepare.py` 只按分支拉取/更新各子仓库源码（不锁 commit，见 `tools/sources.json`），
不生成部署内 Robonix 配置和 GNSS 配置模板。Robonix CLI 与 codegen 应来自同一
锁定框架版本；安装方法见该框架 README。

填写 `config/gnss.base.yaml` 的 NTRIP 地址、用户和挂载点。密码放在
`config/secrets.json`，格式为 `{"NTRIP_PASSWORD":"实际密码"}`，权限设为 600。
这两个文件不提交。设备串口、IMU 安装方向、磁偏角、天线偏移均须现场核对。
航向参数已固化（2026-11-13）：`yaw_std_deg=20`、`max_yaw_2sigma_deg=45`，
取自 2026-11-11 现场授权（磁航向实测误差约 18°，20° 为假设不确定度）。
不再有逐次航向精度确认门槛：RTK 固定解且连续静止 5 秒后定位自动初始化。
安装旋转、磁偏角等现场核对项确认一次后随清单持久保存，无需重复配置。

## 构建、启动和停止

每个终端先加载环境。更新源码或配置后构建；启动不再自动重复构建。

```bash
source scripts/environment.sh
rbnx build -f robonix_manifest.yaml

# 默认检查模式：真实输入，但不接通运动和步态出口。
flock -n run/boot.lock rbnx boot -f robonix_manifest.yaml --no-update-check -v
```

在另一个终端，进入同一目录并加载环境后：

```bash
rbnx caps -v
rbnx shutdown -f robonix_manifest.yaml
```

服务 ACTIVE 表示生命周期启动成功，位置和底盘是否允许导航要看 Scene 的状态。
`flock` 是系统文件锁，两个清单共用 `run/boot.lock`，避免重复启动覆盖 Robonix
的进程记录。保持前台运行；Ctrl-C 也会走 Robonix 的退出流程。不再有第二份 PID 文件。

建图完成并停止检查模式后，显式选择真实运动配置：

```bash
flock -n run/boot.lock rbnx boot -f robonix_manifest.field.yaml --no-update-check -v
# 另一个已加载环境的终端停止：
rbnx shutdown -f robonix_manifest.field.yaml
```

两种模式及室内导航不要同时运行。启动不会自动站起、切控制姿态或下发导航目标。

## 网页和操作顺序

在电脑转发网页端口：

```bash
ssh -p 6003 -N -L 50110:127.0.0.1:50108 nvidia@svr.gddg08.top
```

打开 `http://127.0.0.1:50110/outdoor`。

1. 检查模式下用遥控录制道路；完成后检查楼梯区段，确认连接并保存地图。
2. 加载地图，核对地图向东/向北、实际位置和机头方向。新融合模式需要有效 RTK 和连续静止 5 秒。
3. 停止检查模式，再启动运动模式并重新加载地图。
4. 操作员通过遥控设置 state=17、usage_mode=1 和可用导航步态。
5. 预览短平地路线，再执行；确认取消和到点停车后，最后验证楼梯。

到点容差为 3 米。楼梯及前后 1 米缓冲区暂停局部避障，保留急停和定位/反馈保护；
平地恢复避障。不要把目标放在楼梯边缘，也不要把楼梯与绕障叠加作为首次测试。

## 数据和排查

地图与原始采集在 `data/outdoor_mapping`，服务日志在 `run`，Robonix 进程日志和
状态在 `rbnx-boot`。Scene 标注在 `framework/robonix/system/scene/rbnx-build/data`。
更新和清理构建前保留这些数据。

只看状态可读取 `http://127.0.0.1:19980/status`（定位）、
`http://127.0.0.1:19985/api/status`（导航），或查看 `/outdoor/chassis_status`。
使用 `rbnx shutdown`，不要按通用 ROS/Python 进程名批量杀进程。

旧目录的数据和私密配置不自动搬运。首次迁移时在停止旧部署后，把需要的地图、
原始记录、Scene 标注和私密配置复制到上述新位置，再核对权限；旧入口已删除。

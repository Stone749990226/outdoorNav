# 验证和现场复验

本轮只验证软件和隔离测试。实机启动、动态定位精度、真实运动与楼梯效果由操作员后续验收。

## 离线检查

本轮结果：26 项通过，1 项 DDS 测试因缺少 ROS/drdds 依赖跳过；
底盘、雷达两个原语通过 `rbnx build` 和接口生成。未启动真实硬件。

开发机使用工作区的 uv 虚拟环境，在 outdoorNav 根目录执行：

```bash
uv pip install --python .venv/bin/python -e robonix/pylib/robonix-api \
  -r robot-deep_robotics-lynx_s10/outdoor/requirements.txt
# Robonix 的源码配置应指向当前工作区 robonix。
rbnx codegen -p service-navigation-outdoor-rbnx --mcp --python "$PWD/.venv/bin/python"
.venv/bin/python -m pytest robot-deep_robotics-lynx_s10/outdoor/tests/ \
  robot-deep_robotics-lynx_s10/primitives/primitive-lynx_chassis/tests/
```

已准备依赖的机器人上，也可以在整机仓库的 `outdoor` 目录中执行纯软件测试：

```bash
source scripts/environment.sh
.venv/bin/python -m pytest tests/ ../primitives/primitive-lynx_chassis/tests/
```

配置测试检查两种模式的运动隔离、依赖一致、定位与地图绑定，以及准备脚本不会覆盖
本地依赖修改。底盘测试覆盖反馈过期、非法步态、指令超时和速度限制。
Scene 测试使用真实网页后端、地图逻辑和生成的 gRPC，控制端使用内存任务记录，不接硬件。
DDS 测试需要 ROS/drdds，仅在本机隔离 domain 229/230 通信，缺少依赖时跳过。

## 现场复验

1. 检查模式启动、停止，确认没有本体 `/NAV_CMD` 和 `/GAIT` 发布者由本部署创建。
2. 核对 RTK 数据、100 Hz IMU、雷达点云、实际步态和 usage_mode 反馈。
3. 录制、保存和重新加载新地图，确认原点、位置与航向一致。
4. 完成磁航向质量、安装方向和传感器偏移核对，再填写真实的确认参数。
5. 人工监督下执行短平地路线，检查取消、定位失效、控制指令中断和到点停车。
6. 最后验证楼梯切步态：先停稳、等待真实反馈、通过后恢复平地避障。

## 定位状态怎么读

RTK 解状态是接收机的固定/浮点/不可用；初始化阶段与解状态分别显示。
纯惯导时长只计算本次失锁段，恢复可用 RTK 后清零。距上次固定解从每条新接受的
固定解重新计时。页面关闭后 Scene 仍记录状态到 `run/scene_requests/localization.jsonl`。

持续输出位置不代表允许新任务，也不代表已经达到现场运动精度。静止初始化不能代替标定。

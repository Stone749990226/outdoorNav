# 验证记录

日期：2026-10-15。运行环境：WSL2 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 6.18；数据处理使用 conda `torch`。

## 自动回归

- Python 43 项核心回归通过，覆盖轨迹处理、道路约束、任务控制、进程身份及保留的感知算法。
- 重新处理 `1.ubx`、`2.ubx` 后，路网几何、共同原点、质量标签和处理报告与基线一致。
- 前端 16 项单元测试、10 项 Chromium 浏览器测试通过，包含离线巡航、导出、选点、封路、在线目标提交/替换/取消、版本不匹配及静止/移动障碍物的添加与删除。
- 两个 ROS 包及前端构建通过；统一入口通过传感器、定位、TF、Nav2 lifecycle、API 和网页健康检查。

## 真实运动检查

已通过短程、完整环线、完整支路、掉头、取消、替换、静态绕行，以及行驶中插入静止和移动障碍物。估计到达容差保持 0.25 米；确认停车后再次检查距离，完成时位姿保存在任务快照中。

| 场景 | 估计到达误差 | 独立真值误差 | 最小中心至障碍边界距离 |
| --- | --- | --- | --- |
| 环线返回起点 | 0.247 米 | 0.168 米 | 1.119 米 |
| 支路 | 0.184 米 | 0.308 米 | 1.492 米 |
| 途中插入静止障碍 | 0.153 米 | 0.081 米 | 0.684 米 |
| 途中插入移动障碍 | 0.220 米 | 0.142 米 | 0.159 米 |
| 静态绕行 | 0.209 米 | 0.167 米 | 0.295 米 |

上述场景道路越界采样数均为 0，末半秒真值位移为 0。机器人半径为 0.105 米；中心净空须扣除此半径才是车体间隙。移动绕障后末两米距参考线最远为 0.167 米。估计误差和真值误差分别报告，不将定位估计当作真值。

完全堵路时报告 `No traversable path inside the selected road corridor`，失败后确认停车，越界采样数为 0。场景版本不匹配返回 HTTP 409，不改变当前任务；暂停仿真后任务失败，恢复时钟后确认停车。

时钟回退后任务明确失败并要求重启，独立真值停车位移为 0。回退会使部分 Gazebo 传感器和里程计暂时失效，此时系统不宣称已从里程计确认停车，而是禁止新任务并关闭本轮；真值仅用于验收。硬件与软件渲染均已实际启动通过。

行驶中终止本轮深度点云处理节点后，系统检测到断流、任务失败并停车，真值停车位移为 0。验收涉及的 5 轮运行均已退出，确认无本轮残留进程；版本、源码指纹及实际配置哈希见 `artifacts/validation/runs.json`。

证据位于 `artifacts/validation/`。完整物理仿真已在 WSL2 验证；原生 Ubuntu 的环境适配已有核心回归，完整仿真仍需在该平台实际验收。未取得的结果不视为通过。

深度负障碍检测算法保留，平地校园默认不启用。合成检测覆盖平地、无回波和 0.30 米落差；坡面参照仍保留原默认值，不把未验证的实验配置用于正式导航。

## 在实际机器复验

```bash
./nav doctor
./nav up
```

另一个终端执行：

```bash
./nav validate --scenario short
./nav validate --scenario cancel
./nav validate --scenario replace
./nav validate --scenario turnaround
./nav validate --scenario detour
./nav validate --scenario injected-static
./nav validate --scenario moving
./nav validate --scenario blocked
./nav validate --scenario ring
./nav validate --scenario branch
./nav validate --scenario version-mismatch
./nav validate --scenario pause
./nav down
```

故障验收会中止任务；下面两项分别在新启动的仿真中执行，会触发本轮关闭：

```bash
./nav validate --scenario rewind --timeout 35
# 重新 ./nav up 后再执行下一项
./nav validate --scenario sensor-loss --timeout 35
```

逐项执行，不要并发下发验收任务。`ring` 会沿两条环线弧行驶完整一圈；长任务可能需要十几分钟。每项报告写入当前运行目录，包含任务状态、独立真值停车误差、道路越界采样数和障碍净空。`--timeout` 设置每个任务的墙钟验收期限。

源码检查与离线回归：

```bash
conda run -n torch python -m unittest discover -s tests
npm --prefix web test
npm --prefix web run test:e2e
```

存在仓库 uv venv 时，第一条改用 `uv run python -m unittest discover -s tests`。浏览器测试需要安装 Playwright Chromium，或通过 `PLAYWRIGHT_EXECUTABLE_PATH` 指定浏览器。

原始采集文件及历史数值证据位于 `artifacts/baseline/`。本轮精简后的验收证据位于 `artifacts/validation/`；完整日志与轨迹留在 `artifacts/runs/`，不会随 Git 提交。

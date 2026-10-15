# 验证记录

日期：2026-10-15。运行环境：WSL2 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 6.18；数据处理使用 conda `torch`。

## 自动回归

- Python 41 项数据、几何、路由、任务、运行管理、深度检测和运动预测回归通过。覆盖异常点、缺测、跨午夜、转角、闭环、封路、过期任务结果和进程身份。
- 重新处理 `1.ubx`、`2.ubx` 后，路网几何、共同原点、质量标签和处理报告与基线一致。
- 前端 16 项单元测试、10 项 Chromium 浏览器测试通过，包含离线巡航、导出、选点、封路、在线目标提交/替换/取消、版本不匹配及静止/移动障碍物的添加与删除。
- 两个 ROS 包及前端构建通过；统一入口通过传感器、定位、TF、Nav2 lifecycle、API 和网页健康检查。

## 真实运动检查

已验证短程运动与停车、取消、替换、立即取消、静态障碍绕行和完全堵路失败。短程验收的一次估计终点误差为 0.152 米，独立真值误差为 0.213 米，末半秒真值位移为 0，越出允许道路的采样数为 0。估计到达容差仍为 0.25 米。

支路完整导航、掉头、取消和任务替换已有通过报告。途中插入静止障碍物后绕行通过：估计到达误差 0.091 米，真值误差 0.166 米，最小车体中心至障碍边界距离 0.212 米；机器人半径为 0.105 米。

长距离验证发现并修复了规划失败次数累加与贴道路边界的问题。移动障碍物验证曾发现碰撞，随后加入雷达运动预测、路径占据时立即更新及车身回波过滤；独立移动避障测试已通过：估计到达误差 0.093 米，真值误差 0.120 米，最小中心至障碍边界距离 0.636 米，末两米到参考线的最大距离 0.497 米。连续验收仍需覆盖完整环线和持续运行负载；未通过报告不计作通过。

完整环线、支路、掉头及原生 Ubuntu 的最终验收状态以`artifacts/validation/` 对应的验收 JSON 为准，未取得的结果不视为通过。原生平台的环境适配已有单元验证，完整物理仿真仍需在该平台运行。

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
./nav down
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

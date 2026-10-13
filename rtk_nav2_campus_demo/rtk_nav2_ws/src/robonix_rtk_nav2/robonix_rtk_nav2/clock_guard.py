#!/usr/bin/env python3
"""仿真时钟守卫：决定"数据时间戳"能不能用来判断新鲜度。

**这是从上一轮（视觉版）的失败里学来的**：节点刚起来、Gazebo 的 `/clock` 还没被处理时，
节点时钟读数是 0.0。此时若有一帧数据到达，把 0.0 记成"数据时间"，
等 `/clock` 到达、时间跳到当前值（例如 21 s）之后，`now - last` 就变成 ~21 s，
被误判成"数据过期二十多秒"而停车——**实测在 RTK 版上原样复现了一次**：
路线跟随起步 1 秒就以 `costmap_stale（代价地图的状态与危险距离都已 20.84 s 没有更新）` 中止，
而代价地图其实一直在 3.5 Hz 正常发布（独立监测 100 秒零停顿）。

修法与视觉版一致：
  - 时钟未就绪（读数 0）时 `stamp()` 返回 None，回调**不记**时间戳；
  - 控制回路检测到时钟跳变（含 0 → 真实时间的首次就绪）时，清空所有新鲜度基准与计时起点。
"""


class ClockGuard:
    """守卫仿真时钟的可用性。用法见模块文档。"""

    def __init__(self, jump_threshold=2.0):
        # 控制回路 10 Hz，相邻 tick 正常间隔 0.1 s；超过 jump_threshold 视为时钟跳变
        self.jump_threshold = float(jump_threshold)
        self.ready = False
        self._last_tick = None
        self.jumps = 0

    def update(self, now):
        """每个控制 tick 调一次；返回 True 表示检测到需要重置基准的时钟跳变。"""
        reset = False
        if self._last_tick is not None:
            delta = now - self._last_tick
            if delta < 0.0 or delta > self.jump_threshold:
                reset = True
                self.jumps += 1
                self.ready = False
        self._last_tick = now
        if not self.ready and now > 0.0:
            self.ready = True
        return reset

    def stamp(self, now):
        """时钟就绪时返回可用于新鲜度判断的时间戳，否则 None。"""
        if self.ready and now > 0.0:
            return now
        return None

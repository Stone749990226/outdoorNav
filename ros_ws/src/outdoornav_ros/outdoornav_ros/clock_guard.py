#!/usr/bin/env python3
"""Simulation clock startup and reset guard."""


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

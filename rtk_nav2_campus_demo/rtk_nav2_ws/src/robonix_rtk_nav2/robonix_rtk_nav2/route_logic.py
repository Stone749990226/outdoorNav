#!/usr/bin/env python3
"""路线跟随的纯逻辑（不依赖 ROS，可单测）。

与视觉版巡游的差别只有一个，但是本质的：**这里的位姿是绝对位姿**。
路线点是 map 坐标（相对 RTK 基准点的东/北，单位米），不是"从出生点算起"的
里程计坐标。所以：
  - 不存在"里程计漂了路线就跟着歪"的问题；
  - 路线可以在出发前就用真实坐标写死，验收时能直接拿场地几何核对。
"""

import math


def wrap_angle(angle):
    """把角度规整到 (-pi, pi]。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_velocity(angle_error, distance, goal_tolerance,
                     max_linear, max_angular, angular_gain, linear_gain,
                     align_threshold=0.35):
    """朝未对准就先原地转，对准后按比例给线速度，线速度上限受限。

    与视觉版同一套控制律，只是输入位姿来自绝对定位。
    """
    if abs(angle_error) > align_threshold:
        linear = 0.0
    else:
        linear = min(max_linear, linear_gain * distance)
        # 转向偏差越大走得越慢，避免大角度时画大弧
        linear *= max(0.0, 1.0 - abs(angle_error) / align_threshold)

    angular = max(-max_angular, min(max_angular, angular_gain * angle_error))

    if distance <= goal_tolerance:
        linear = 0.0
    return linear, angular


def cross_track_error(position, route, current_index):
    """当前位置到**当前航段**的横向偏差（米，带符号：左正右负）。

    用法：验收要的是"路线跟随的横向跟踪误差"，这个量必须在**确定的几何**上算——
    route 是写死的路线折线，position 是某一时刻的位置（离线评估时用真值）。
    取当前航段而不是整条折线，是因为整条折线在拐角处会把"离另一段很近"也算成零误差。
    """
    if len(route) < 2:
        return 0.0
    i = max(0, min(current_index, len(route) - 2))
    ax, ay = route[i]
    bx, by = route[i + 1]
    px, py = position[0], position[1]

    dx, dy = bx - ax, by - ay
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-9:
        return math.hypot(px - ax, py - ay)

    # 点到直线的有符号横向距离（叉积 / 段长）
    return ((px - ax) * dy - (py - ay) * dx) / seg_len


def progress_along_route(position, route, current_index):
    """在**当前航段**上的投影进度（0..1），用于说明当前在第几段。"""
    if len(route) < 2:
        return 0.0
    i = max(0, min(current_index, len(route) - 2))
    ax, ay = route[i]
    bx, by = route[i + 1]
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom < 1e-9:
        return 1.0
    t = ((position[0] - ax) * dx + (position[1] - ay) * dy) / denom
    return max(0.0, min(1.0, t))

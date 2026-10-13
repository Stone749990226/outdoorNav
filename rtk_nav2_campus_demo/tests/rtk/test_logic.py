"""RTK 版纯逻辑测试（不需要 ROS 运行环境）。

运行：
    PYTHONPATH=rtk_ws/src/robonix_rtk_demo python3 -m pytest tests/rtk/test_logic.py -v

覆盖三类容易悄悄出错、又直接影响验收结论的地方：
  1. 深度反投影与坐标系变换（错一个符号，代价地图就会整体镜像或前后颠倒）
  2. 代价地图的分层与膨胀（地面必须算自由，否则图上全黑）
  3. 安全判定（前方危险距离只认前方的障碍，不能把身后的算进来）
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, os.pardir,
                                'rtk_ws', 'src', 'robonix_rtk_demo'))

from robonix_rtk_demo.costmap_logic import (  # noqa: E402
    LETHAL_COST, R_OPTICAL_TO_LINK, RollingGrid, backproject, forward_danger,
    inflate, mark_cells, optical_to_map)
from robonix_rtk_demo.route_logic import (  # noqa: E402
    compute_velocity, cross_track_error, progress_along_route, wrap_angle)


# ---------------------------------------------------------------- 反投影与坐标系

def test_optical_to_link_rotation_matches_ros_convention():
    """光学系 +z(前) -> 机体 +x；+x(右) -> -y；+y(下) -> -z。"""
    forward = np.array([[0.0, 0.0, 1.0]])
    right = np.array([[1.0, 0.0, 0.0]])
    down = np.array([[0.0, 1.0, 0.0]])
    assert np.allclose(forward @ R_OPTICAL_TO_LINK.T, [[1.0, 0.0, 0.0]])
    assert np.allclose(right @ R_OPTICAL_TO_LINK.T, [[0.0, -1.0, 0.0]])
    assert np.allclose(down @ R_OPTICAL_TO_LINK.T, [[0.0, 0.0, -1.0]])


def test_backproject_center_pixel_is_on_optical_axis():
    """图像正中央的像素应当落在光轴上（x=y=0，z=深度）。"""
    depth = np.full((4, 4), 2.0, dtype=np.float32)
    cx, cy = 1.5, 1.5
    points, _ = backproject(depth, fx=100.0, fy=100.0, cx=cx, cy=cy)
    assert len(points) == 16
    center = points[[np.argmin(np.abs(points[:, 0]) + np.abs(points[:, 1]))]]
    assert abs(center[0][0]) < 1e-6 and abs(center[0][1]) < 1e-6
    assert abs(center[0][2] - 2.0) < 1e-6


def test_backproject_filters_invalid_and_out_of_range():
    depth = np.array([[np.nan, np.inf, 0.05, 9.0, 3.0]], dtype=np.float32)
    points, _ = backproject(depth, fx=100.0, fy=100.0, cx=2.0, cy=0.0,
                            min_range=0.1, max_range=8.0)
    assert len(points) == 1, '只有 3.0 这一个像素应当有效'
    assert abs(points[0][2] - 3.0) < 1e-6


def test_optical_to_map_applies_yaw_and_translation():
    """机器人位于 (1,2) 朝向 90°（+y 方向）时，正前方 1 m 的点应落在 (1,3)。"""
    points = np.array([[0.0, 0.0, 1.0]])          # 光学系：正前方 1 m
    out = optical_to_map(points, camera_xyz=(0.0, 0.0, 0.0),
                         robot_xyz_yaw=(1.0, 2.0, math.pi / 2))
    assert np.allclose(out[0][:2], [1.0, 3.0], atol=1e-9)


def test_optical_to_map_includes_camera_offset():
    """相机装在机体前方 0.055 m、高 0.19 m，正前方 1 m 的点要带上这个偏移。"""
    points = np.array([[0.0, 0.0, 1.0]])
    out = optical_to_map(points, camera_xyz=(0.055, 0.0, 0.19),
                         robot_xyz_yaw=(0.0, 0.0, 0.0))
    assert np.allclose(out[0], [1.055, 0.0, 0.19], atol=1e-9)


# ---------------------------------------------------------------- 栅格与代价

def test_mark_cells_separates_floor_from_obstacle():
    grid = RollingGrid(size_m=4.0, resolution=0.1)   # 40x40
    grid.recenter(0.0, 0.0)
    # 一个地面点（z=0）与一个障碍点（z=0.5），位置相同
    floor = np.array([[0.0, 1.0, 0.0]])
    obstacle = np.array([[0.0, 1.0, 0.5]])
    occ_f, obs_f = mark_cells(floor, grid)
    occ_o, obs_o = mark_cells(obstacle, grid)
    row, col = grid.world_to_cell(np.array([0.0]), np.array([1.0]))
    assert obs_f[row[0], col[0]] and not occ_f[row[0], col[0]], '地面点应为自由'
    assert obs_o[row[0], col[0]] and occ_o[row[0], col[0]], '障碍点应为占据'


def test_mark_cells_ignores_points_outside_window():
    grid = RollingGrid(size_m=2.0, resolution=0.1)
    grid.recenter(0.0, 0.0)
    far = np.array([[100.0, 100.0, 0.5]])
    occ, obs = mark_cells(far, grid)
    assert not occ.any() and not obs.any()


def test_inflate_decays_with_distance_and_is_bounded():
    occupied = np.zeros((21, 21), dtype=bool)
    occupied[10, 10] = True
    cost = inflate(occupied, radius_cells=3)
    assert cost[10, 10] == LETHAL_COST
    assert cost[10, 11] > cost[10, 12] > cost[10, 13] > 0, '代价应随距离衰减'
    assert cost[10, 14] == 0, '超出膨胀半径应为 0'
    assert cost.max() == LETHAL_COST


def test_forward_danger_only_sees_ahead():
    grid = RollingGrid(size_m=4.0, resolution=0.1)
    grid.recenter(0.0, 0.0)
    cost = np.zeros((grid.cells, grid.cells), dtype=np.int16)
    # 机器人位于 (0,0) 朝向 +x：把障碍放在正前方 1.0 m 处
    ahead = grid.world_to_cell(np.array([1.0]), np.array([0.0]))
    cost[ahead[0][0], ahead[1][0]] = LETHAL_COST
    d = forward_danger(cost, grid, (0.0, 0.0, 0.0))
    assert d is not None and 0.9 <= d <= 1.1

    # 同样的障碍放到身后，前方就不该报危险
    cost[:] = 0
    behind = grid.world_to_cell(np.array([-1.0]), np.array([0.0]))
    cost[behind[0][0], behind[1][0]] = LETHAL_COST
    assert forward_danger(cost, grid, (0.0, 0.0, 0.0)) is None


def test_forward_danger_respects_max_range():
    grid = RollingGrid(size_m=8.0, resolution=0.05)
    grid.recenter(0.0, 0.0)
    cost = np.zeros((grid.cells, grid.cells), dtype=np.int16)
    far = grid.world_to_cell(np.array([5.0]), np.array([0.0]))
    cost[far[0][0], far[1][0]] = LETHAL_COST
    assert forward_danger(cost, grid, (0.0, 0.0, 0.0), max_range=3.0) is None
    assert forward_danger(cost, grid, (0.0, 0.0, 0.0), max_range=6.0) is not None


def test_forward_danger_ignores_weak_inflation():
    """只有代价 >= 阈值（默认 50，即膨胀内圈）才算危险，外圈不算。"""
    grid = RollingGrid(size_m=4.0, resolution=0.1)
    grid.recenter(0.0, 0.0)
    cost = np.zeros((grid.cells, grid.cells), dtype=np.int16)
    cell = grid.world_to_cell(np.array([1.0]), np.array([0.0]))
    cost[cell[0][0], cell[1][0]] = 20
    assert forward_danger(cost, grid, (0.0, 0.0, 0.0), lethal_threshold=50) is None
    assert forward_danger(cost, grid, (0.0, 0.0, 0.0), lethal_threshold=10) is not None


# ---------------------------------------------------------------- 控制与跟踪误差

def test_velocity_turns_in_place_when_misaligned():
    linear, angular = compute_velocity(math.radians(60), 5.0, 0.15,
                                       0.16, 0.6, 1.4, 0.6)
    assert linear == 0.0 and angular > 0


def test_velocity_respects_limits_and_stops_at_goal():
    linear, angular = compute_velocity(0.0, 100.0, 0.15, 0.16, 0.6, 1.4, 0.6)
    assert abs(linear) <= 0.16 and abs(angular) <= 0.6
    linear, _ = compute_velocity(0.0, 0.15, 0.15, 0.16, 0.6, 1.4, 0.6)
    assert linear == 0.0


def test_cross_track_error_sign_and_magnitude():
    route = [(0.0, 0.0), (10.0, 0.0)]      # 沿 +x 的直线
    assert abs(cross_track_error((5.0, 0.0), route, 0)) < 1e-9
    assert cross_track_error((5.0, 1.0), route, 0) == pytest.approx(1.0)
    assert cross_track_error((5.0, -1.0), route, 0) == pytest.approx(-1.0)
    # 只按**当前航段**算：第二段在 y=10 上，点 (5,1) 相对它的偏差完全不同
    route2 = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]
    assert cross_track_error((9.0, 5.0), route2, 1) == pytest.approx(1.0)


def test_progress_along_route_clamps():
    route = [(0.0, 0.0), (10.0, 0.0)]
    assert progress_along_route((-5.0, 0.0), route, 0) == 0.0
    assert progress_along_route((5.0, 0.0), route, 0) == pytest.approx(0.5)
    assert progress_along_route((50.0, 0.0), route, 0) == 1.0


def test_wrap_angle():
    assert wrap_angle(0.0) == 0.0
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9
    assert abs(wrap_angle(-3 * math.pi) - math.pi) < 1e-9


# ---------------------------------------------------------------- 位姿合成（关键回归）
#
# 这一条是踩过坑的：absolute_localizer 里 map→odom 的合成公式写错时，
# TF 链 map→base 与定位节点自己发布的 /odometry/global 会差好几米，
# 而"定位精度"的日志看起来完全正常（因为它看的是 /odometry/global）。
# 下面直接验证"合成回去必须等于原值"。

def compose(t_parent_child, t_child_grandchild):
    """T_A_C = T_A_B ∘ T_B_C（二维）。"""
    import math
    y1, (x1, y1t) = t_parent_child
    y2, (x2, y2t) = t_child_grandchild
    c, s = math.cos(y1), math.sin(y1)
    return (y1 + y2, (c * x2 - s * y2t + x1, s * x2 + c * y2t + y1t))


def invert(t):
    import math
    yaw, (x, y) = t
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (-yaw, (-(c * x - s * y), -(s * x + c * y)))


def test_map_to_odom_composition_round_trip():
    """给定 map→base 与 odom→base，算出的 map→odom 复合回去必须等于原值。"""
    import math
    for myaw, mx, my in ((1.049, 4.416, 0.399), (0.0, 1.0, -1.0), (-2.5, 0.3, 0.9)):
        for oyaw, ox, oy in ((1.049, 1.768, 3.066), (0.0, 0.0, 0.0), (0.7, -0.4, 2.2)):
            map_to_base = (myaw, (mx, my))
            odom_to_base = (oyaw, (ox, oy))
            # 被测公式（与 absolute_localizer 里一致）
            odom_yaw = myaw - oyaw
            c, s = math.cos(odom_yaw), math.sin(odom_yaw)
            map_to_odom = (odom_yaw, (mx - (c * ox - s * oy), my - (s * ox + c * oy)))
            got = compose(map_to_odom, odom_to_base)
            assert abs(got[0] - map_to_base[0]) < 1e-9
            assert abs(got[1][0] - map_to_base[1][0]) < 1e-9
            assert abs(got[1][1] - map_to_base[1][1]) < 1e-9


def test_wrong_composition_would_be_off():
    """反证：第一版那种写法（用 odom→base 的 yaw 转）在同一组数上会差出米级。"""
    import math
    myaw, mx, my = 1.049, 4.416, 0.399
    oyaw, ox, oy = 1.049, 1.768, 3.066
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = mx - ox, my - oy
    bad = (myaw - oyaw, (c * dx + s * dy, -s * dx + c * dy))
    got = compose(bad, (oyaw, (ox, oy)))
    err = math.hypot(got[1][0] - mx, got[1][1] - my)
    assert err > 1.0, '错误写法应当复现出米级偏差（实测 3.6 m），实际 %.3f' % err


# ---------------------------------------------------------------- 时钟守卫（回归）
#
# 这一条同样是从失败里学来的：节点刚起来、/clock 还没处理时，时钟读数是 0.0。
# 若此时把 0.0 记成"数据时间"，等时钟跳到 21 s 之后，就会算出"已过期 20.84 s"
# 而停车——而代价地图其实一直在正常发布（实测：路线跟随起步 1 秒即中止）。

def test_clock_guard_refuses_stamp_before_ready():
    from robonix_rtk_demo.clock_guard import ClockGuard
    g = ClockGuard(jump_threshold=2.0)
    g.update(0.0)
    assert g.stamp(0.0) is None       # 时钟未就绪：不能给出时间戳


def test_clock_guard_detects_first_ready_jump():
    from robonix_rtk_demo.clock_guard import ClockGuard
    g = ClockGuard(jump_threshold=2.0)
    g.update(0.0)
    assert g.update(20.84) is True    # 0 -> 真实时间必须判为跳变
    assert g.stamp(20.84) == 20.84
    assert g.jumps == 1


def test_without_guard_that_gap_would_look_like_stale():
    """反证：不做守卫时算出来的"过期时长"就是仿真时间总量（实测 20.84 s）。"""
    bogus_last = 0.0
    now = 20.84
    assert (now - bogus_last) > 3.0   # danger_timeout = 3.0 s，必然误判为过期


def test_clock_guard_resets_on_backward_jump():
    from robonix_rtk_demo.clock_guard import ClockGuard
    g = ClockGuard(jump_threshold=2.0)
    g.update(100.0)
    assert g.update(3.0) is True
    assert g.jumps == 1


# ---------------------------------------------------------------- 全局累积图（第二阶段）
#
# 四条语义都要钉住：走过的填上、没去的保持未知、障碍移走要能清除、清空只能沿实测射线。

def test_global_grid_marks_free_and_occupied():
    from robonix_rtk_demo.costmap_logic import GlobalGrid
    g = GlobalGrid(size_m=10.0, resolution=0.1)
    g.mark_points(np.array([[1.0, 1.0, 0.3]]))       # 障碍带
    g.mark_points(np.array([[2.0, 2.0, 0.0]]))       # 地面点
    occ = g.to_occupancy()
    r1, c1 = g.world_to_cell(np.array([1.0]), np.array([1.0]))
    r2, c2 = g.world_to_cell(np.array([2.0]), np.array([2.0]))
    assert occ[r1[0], c1[0]] == 100, '障碍带内的点应标为占据'
    assert occ[r2[0], c2[0]] == 0, '地面点应标为空闲'


def test_global_grid_keeps_unvisited_unknown():
    from robonix_rtk_demo.costmap_logic import GlobalGrid
    g = GlobalGrid(size_m=10.0, resolution=0.1)
    g.mark_points(np.array([[0.0, 0.0, 0.0]]))
    occ = g.to_occupancy()
    assert occ[0, 0] != -1, '见过的格子不该还是未知'
    # 对角线另一端没去过
    n = g.cells - 1
    assert occ[n, n] == -1, '没去过的地方必须仍是未知'


def test_global_grid_clears_obstacle_after_it_moves_away():
    from robonix_rtk_demo.costmap_logic import GlobalGrid
    g = GlobalGrid(size_m=10.0, resolution=0.1)
    g.mark_points(np.array([[1.0, 1.0, 0.3]]))       # 先有障碍
    r, c = g.world_to_cell(np.array([1.0]), np.array([1.0]))
    assert g.to_occupancy()[r[0], c[0]] == 100
    g.mark_points(np.array([[1.0, 1.0, 0.0]]))       # 障碍移走后再看到地面
    assert g.to_occupancy()[r[0], c[0]] == 0, '障碍移走后再次经过应被清除，而不是永久记住'


def test_global_grid_scan_clearing_stops_at_measured_range():
    """激光只清"射线到实测距离为止"——超量程的射线整条跳过，不擦墙后的信息。"""
    from robonix_rtk_demo.costmap_logic import GlobalGrid
    g = GlobalGrid(size_m=10.0, resolution=0.1)
    # 一条朝前的射线，量到 1.0 m；另一条方向无回波（inf）
    ranges = [1.0, float('inf')]
    g.mark_scan_free(ranges, angle_min=0.0, angle_increment=math.pi,
                     range_min=0.02, range_max=2.0,
                     robot_pose=(0.0, 0.0, 0.0))
    occ = g.to_occupancy()
    near = g.world_to_cell(np.array([0.5]), np.array([0.0]))     # 射线上（1 m 以内）
    beyond = g.world_to_cell(np.array([1.6]), np.array([0.0]))   # 实测距离之外
    assert occ[near[0][0], near[1][0]] == 0, '射线扫过的一段应标为空闲'
    assert occ[beyond[0][0], beyond[1][0]] == -1, '实测距离之外不得被清除'


def test_global_grid_clear_path_marks_trajectory_free():
    from robonix_rtk_demo.costmap_logic import GlobalGrid
    g = GlobalGrid(size_m=10.0, resolution=0.1)
    g.mark_points(np.array([[1.0, 0.0, 0.3]]))
    g.clear_path(0.0, 0.0, 2.0, 0.0)
    occ = g.to_occupancy()
    r, c = g.world_to_cell(np.array([1.0]), np.array([0.0]))
    assert occ[r[0], c[0]] == 0, '驶过的轨迹应把沿途清成空闲'

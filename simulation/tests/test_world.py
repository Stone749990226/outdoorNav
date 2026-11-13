import math

import pytest

from simulation.world import World, coordinates, radii, wrap


def test_commands_move_truth_and_expire_without_more_commands():
    w = World({"x": 0, "y": 0, "yaw_deg": 90})
    for i in range(100):
        now = i * 0.01
        w.command(0.4, 0, now)
        w.tick(0.01, now)
    assert abs(w.x) < 1e-8 and w.y > 0.2
    for i in range(100, 250):
        w.tick(0.01, i * 0.01)
    assert w.v == 0
    y = w.y
    w.tick(0.01, 3)
    assert w.y == y


def test_minus_sensor_y_is_forward_and_acceleration_includes_turning():
    w = World({"x": 0, "y": 0, "yaw_deg": -170})
    w.v, w.w, w.acceleration = 0.4, 0.3, 0.2
    sensor_yaw, acceleration, gyro = w.imu()
    assert wrap(sensor_yaw - math.pi / 2) == pytest.approx(w.yaw)
    # Rotate sensor acceleration by body_from_imu = Rz(+90°).
    assert (-acceleration[1], acceleration[0], acceleration[2]) == pytest.approx((0.2, 0.12, 9.80665))
    assert gyro == (0.0, 0.0, 0.3)


def test_standing_mode_and_nonfinite_commands_do_not_move():
    w = World({"x": 0, "y": 0, "yaw_deg": 0})
    w.state = 0
    w.command(0.4, 0.4, 0)
    w.tick(0.1, 0.1)
    assert w.v == 0 and w.w == 0
    w.state = 17
    w.command(float("nan"), 0.2, 0.2)
    w.tick(0.1, 0.3)
    assert w.v == 0 and w.w == 0


def test_gnss_positions_use_same_east_north_axes_as_map():
    origin = {"latitude": 30.2240679, "longitude": 120.2598139}
    lat, lon = coordinates(12, -7, origin)
    east, north = radii(origin["latitude"])
    assert math.radians(lon - origin["longitude"]) * east == pytest.approx(12, abs=1e-8)
    assert math.radians(lat - origin["latitude"]) * north == pytest.approx(-7, abs=1e-8)

"""Planar GNSS/odometry alignment with explicit calibration and reset latch."""

from collections import deque
import math

from .coordinates import enu
from .gnss import fix_grade
from .config import DEFAULTS


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


class GeoAlignment:
    def __init__(self, anchor, config):
        self.anchor, self.config = anchor, {**DEFAULTS, **config}
        self.history = deque(maxlen=300)
        self.rotation = None
        self.translation = None
        self.fault = None
        self.stable_since = None
        self.grade = None
        self.fix_at = -math.inf
        self.last_tow = None
        self.fix = None

    def odom(self, x, y, yaw, stamp, received):
        if not all(math.isfinite(v) for v in (x, y, yaw, stamp)):
            self.fault = "invalid_odometry"
            return False
        if self.history:
            old = self.history[-1]
            dt = stamp - old[3]
            if (
                dt <= 0
                or dt > self.config["sensor_timeout_s"]
                or math.hypot(x - old[0], y - old[1]) > 0.25 + 2 * dt
                or abs(wrap(yaw - old[2])) > 0.25 + 2 * dt
            ):
                self.fault = "odometry_reset_or_gap; recalibrate and restart"
                return False
        if self.rotation is None:
            self.rotation = wrap(self.config["initial_heading_rad"] - yaw)
        self.history.append((x, y, yaw, stamp, received))
        return True

    def update_fix(self, fix, received):
        grade = fix_grade(fix, self.config)
        if grade is None:
            self.grade = None
            self.stable_since = None
        self.fix = dict(fix)
        if fix["tow_ms"] == self.last_tow:
            return
        if self.last_tow is not None and fix["tow_ms"] < self.last_tow:
            # iTOW wraps once per GPS week. Other reversals indicate replay/reset.
            if not (self.last_tow >= 604799000 and fix["tow_ms"] < 1000):
                self.fault = "gnss_epoch_reversed; inspect receiver and restart"
                self.stable_since = None
                return
        self.last_tow = fix["tow_ms"]
        previous = self.fix_at
        self.fix_at = received
        grade = fix_grade(fix, self.config)
        if grade != self.grade:
            self.stable_since = None
        self.grade = grade
        if grade is None or not self.history:
            self.stable_since = None
            return
        sample = min(self.history, key=lambda p: abs(p[4] - received))
        if abs(sample[4] - received) > 0.2:
            self.stable_since = None
            return
        x, y, yaw = sample[:3]
        angle = self.rotation
        c, s = math.cos(angle), math.sin(angle)
        heading = yaw + angle
        ax, ay = self.config["antenna_xy_m"]
        gx, gy = enu(fix["latitude"], fix["longitude"], self.anchor)
        # Surveyed map correction is independent of the odom-frame rotation.
        gx += self.config["map_offset_xy_m"][0]
        gy += self.config["map_offset_xy_m"][1]
        target = (
            gx - math.cos(heading) * ax + math.sin(heading) * ay - (c * x - s * y),
            gy - math.sin(heading) * ax - math.cos(heading) * ay - (s * x + c * y),
        )
        limit = 0.75 if grade == 'fixed' else max(0.75, 2 * fix['hacc_m'])
        if self.translation and math.dist(target, self.translation) > limit:
            self.fault = "gnss_position_jump; inspect alignment and restart"
            self.stable_since = None
            return
        if self.translation is None:
            self.translation = target
        else:
            self.translation = tuple(
                a + 0.15 * (b - a) for a, b in zip(self.translation, target)
            )
        if self.stable_since is None or received - previous > self.config["sensor_timeout_s"]:
            self.stable_since = received

    def ready(self, now):
        return bool(
            self.config["calibrated"]
            and not self.fault
            and self.history
            and 0 <= now - self.history[-1][4] < self.config['sensor_timeout_s']
            and 0 <= now - self.fix_at < self.config['sensor_timeout_s']
            and self.stable_since is not None
            and now - self.stable_since >= self.config['gnss_stable_s']
        )

    def pose(self):
        if not self.history or self.translation is None:
            return None
        x, y, yaw = self.history[-1][:3]
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        return {
            "x": c * x - s * y + self.translation[0],
            "y": s * x + c * y + self.translation[1],
            "yaw": wrap(yaw + self.rotation),
        }

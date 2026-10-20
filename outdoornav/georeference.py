"""Passive ENU-to-existing-map calibration. Never publishes or replaces TF."""

from collections import deque
import math

from .coordinates import enu
from .gnss import usable_fix


class MapAlignment:
    def __init__(self, anchor, antenna_xy=(0.0, 0.0), min_baseline=5.0):
        self.anchor = anchor
        self.antenna_xy = tuple(antenna_xy)
        self.min_baseline = min_baseline
        self.samples = deque(maxlen=600)
        self.transform = None
        self.fault = None
        self.last_epoch = None
        self.fix = None
        self.fix_at = -math.inf
        self.fixed_since = None
        self.generation = None
        self.residual_m = None

    def generation_changed(self, generation):
        if generation != self.generation:
            self.samples.clear()
            self.transform = None
            self.fixed_since = None
            self.fault = None
            self.last_epoch = None
            self.generation = generation

    def update(self, fix, pose, now):
        self.fix = dict(fix)
        epoch = fix['tow_ms']
        if epoch == self.last_epoch:
            return
        if self.last_epoch is not None and epoch < self.last_epoch:
            if not (self.last_epoch >= 604799000 and epoch < 1000):
                self.fault = 'GNSS epoch reversed; restart alignment'
                return
        self.last_epoch = epoch
        previous = self.fix_at
        self.fix_at = now
        if not usable_fix(fix) or pose is None or not 0 <= pose['age_s'] < 0.2:
            self.fixed_since = None
            return
        if self.fixed_since is None or now - previous > 1:
            self.fixed_since = now
        gx, gy = enu(fix['latitude'], fix['longitude'], self.anchor)
        # Compare the antenna, not the base position, in both frames.
        yaw = pose['yaw']
        ax, ay = self.antenna_xy
        mx = pose['x'] + math.cos(yaw) * ax - math.sin(yaw) * ay
        my = pose['y'] + math.sin(yaw) * ax + math.cos(yaw) * ay
        if not all(math.isfinite(v) for v in (gx, gy, mx, my, yaw)):
            self.fault = 'Nonfinite map/GNSS observation'
            return
        if self.transform:
            predicted = self.to_map({'x': gx, 'y': gy})
            if math.hypot(predicted['x'] - mx, predicted['y'] - my) > 0.75:
                self.fault = 'GNSS/map disagreement or map reset; recalibrate'
            return
        self.samples.append((gx, gy, mx, my))
        if len(self.samples) < 20:
            return
        origin = self.samples[0]
        baseline = max(math.hypot(p[0] - origin[0], p[1] - origin[1]) for p in self.samples)
        if baseline < self.min_baseline:
            return
        means = [sum(p[i] for p in self.samples) / len(self.samples) for i in range(4)]
        dot = cross = energy = 0.0
        for x, y, a, b in self.samples:
            x, y, a, b = x - means[0], y - means[1], a - means[2], b - means[3]
            dot += x * a + y * b
            cross += x * b - y * a
            energy += x * x + y * y
        if energy < 1e-6 or math.hypot(dot, cross) < 1e-6:
            return
        angle = math.atan2(cross, dot)
        c, s = math.cos(angle), math.sin(angle)
        tx = means[2] - c * means[0] + s * means[1]
        ty = means[3] - s * means[0] - c * means[1]
        errors = [math.hypot(c*x-s*y+tx-a, s*x+c*y+ty-b) for x,y,a,b in self.samples]
        self.residual_m = max(errors)
        if self.residual_m > 0.25:
            self.fault = 'Calibration residual exceeds 0.25 m; inspect synchronization/map'
            return
        self.transform = (angle, tx, ty)

    def ready(self, now):
        return bool(self.transform and not self.fault and self.fixed_since is not None
                    and now - self.fixed_since >= 5 and now - self.fix_at < 1)

    def to_map(self, point):
        if self.transform is None or self.fault:
            raise ValueError('ENU/map alignment is not valid')
        angle, tx, ty = self.transform
        c, s = math.cos(angle), math.sin(angle)
        result = {'x': c * point['x'] - s * point['y'] + tx,
                  'y': s * point['x'] + c * point['y'] + ty}
        if 'yaw' in point:
            result['yaw'] = math.atan2(math.sin(point['yaw'] + angle), math.cos(point['yaw'] + angle))
        return result

    def to_enu(self, point):
        if self.transform is None:
            return None
        angle, tx, ty = self.transform
        c, s = math.cos(angle), math.sin(angle)
        x, y = point['x'] - tx, point['y'] - ty
        return {**point, 'x': c*x+s*y, 'y': -s*x+c*y,
                'yaw': math.atan2(math.sin(point['yaw']-angle), math.cos(point['yaw']-angle))}

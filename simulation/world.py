"""Plan-independent planar motion; all mock sensors observe this same state."""

import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def radii(latitude):
    denominator = math.sqrt(1 - 6.69437999014e-3 * math.sin(math.radians(latitude)) ** 2)
    return (6378137 / denominator * math.cos(math.radians(latitude)),
            6378137 * (1 - 6.69437999014e-3) / denominator ** 3)


def coordinates(x, y, origin):
    east, north = radii(origin["latitude"])
    return (origin["latitude"] + math.degrees(y / north),
            origin["longitude"] + math.degrees(x / east))


class World:
    def __init__(self, start, max_speed=0.4):
        self.start = dict(start)
        self.max_speed = max_speed
        self.reset()

    def reset(self):
        self.x, self.y = float(self.start["x"]), float(self.start["y"])
        self.yaw = math.radians(self.start["yaw_deg"])
        self.v = self.w = self.acceleration = 0.0
        self.target = (0.0, 0.0)
        self.command_at = -math.inf
        self.state, self.gait, self.usage_mode = 17, 0x3002, 1
        self.distance = 0.0
        self.command_count = 0
        self.gait_history = [self.gait]

    def command(self, v, w, now):
        if not all(math.isfinite(a) for a in (v, w)):
            self.target = (0.0, 0.0)
            return
        self.target = (max(0.0, min(self.max_speed, v)), max(-0.4, min(0.4, w)))
        self.command_at = now
        self.command_count += 1

    def tick(self, dt, now):
        target = self.target
        if now - self.command_at > 0.35 or self.state != 17 or self.usage_mode != 1:
            target = (0.0, 0.0)
        old_v = self.v
        self.v += max(-0.6 * dt, min(0.6 * dt, target[0] - self.v))
        self.w += max(-0.8 * dt, min(0.8 * dt, target[1] - self.w))
        mid_yaw = self.yaw + self.w * dt / 2
        speed = (self.v + old_v) / 2
        self.x += speed * math.cos(mid_yaw) * dt
        self.y += speed * math.sin(mid_yaw) * dt
        self.yaw = wrap(self.yaw + self.w * dt)
        self.acceleration = (self.v - old_v) / dt
        self.distance += abs(speed) * dt

    def imu(self, heading_bias_deg=0.0):
        # body_from_imu = Rz(+90°): sensor -Y is the robot's forward axis.
        sensor_yaw = wrap(self.yaw + math.pi / 2 + math.radians(heading_bias_deg))
        acceleration = (self.v * self.w, -self.acceleration, 9.80665)
        return sensor_yaw, acceleration, (0.0, 0.0, self.w)

    def snapshot(self):
        return {"x": self.x, "y": self.y, "yaw_deg": math.degrees(self.yaw),
                "linear_mps": self.v, "angular_rps": self.w,
                "state": self.state, "gait": self.gait, "usage_mode": self.usage_mode,
                "distance_m": self.distance, "command_count": self.command_count,
                "gait_history": list(self.gait_history)}

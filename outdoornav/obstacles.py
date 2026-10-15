"""Validated scripted obstacles; all positions are map meters and time is simulated."""

import math
import re

from .routing import validate_point


def obstacle_spec(payload):
    if not isinstance(payload, dict):
        raise ValueError("Obstacle must be an object")
    name = payload.get("name", "")
    if not isinstance(name, str) or not re.fullmatch(r"test_[A-Za-z0-9_]{1,48}", name):
        raise ValueError(
            "Obstacle name must start with test_ and use letters, numbers or underscores"
        )
    kind = payload.get("kind", "static")
    if kind not in ("static", "moving"):
        raise ValueError("Obstacle kind must be static or moving")
    start = validate_point(payload.get("start"))
    end = validate_point(payload.get("end")) if kind == "moving" else start.copy()
    sizes = {}
    for key, default in (("width", 0.6), ("depth", 0.6), ("height", 0.8)):
        value = payload.get(key, default)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0.1 <= value <= 15
        ):
            raise ValueError(f"Obstacle {key} must be within 0.1..15 meters")
        sizes[key] = float(value)
    speed = payload.get("speed_mps", 0.3)
    if (
        isinstance(speed, bool)
        or not isinstance(speed, (int, float))
        or not math.isfinite(speed)
        or not 0.05 <= speed <= 1
    ):
        raise ValueError("Moving speed must be within 0.05..1 m/s")
    if kind == "moving" and math.dist(tuple(start.values()), tuple(end.values())) < 0.2:
        raise ValueError(
            "Moving obstacle needs two positions at least 0.2 meters apart"
        )
    if kind == "moving" and math.dist(tuple(start.values()), tuple(end.values())) > 30:
        raise ValueError("Moving obstacle travel must be at most 30 meters")
    return dict(
        name=name, kind=kind, start=start, end=end, speed_mps=float(speed), **sizes
    )


def position_at(spec, elapsed):
    """Move continuously back and forth; pause naturally when simulation time pauses."""
    a, b = spec["start"], spec["end"]
    distance = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    if spec["kind"] == "static" or not distance:
        return a.copy()
    phase = (max(0, elapsed) * spec["speed_mps"] / distance) % 2
    fraction = phase if phase <= 1 else 2 - phase
    return {key: a[key] + fraction * (b[key] - a[key]) for key in ("x", "y")}


def box_sdf(spec):
    size = f"{spec['width']} {spec['depth']} {spec['height']}"
    # A kinematic test fixture: Gazebo updates its real visual and collision geometry.
    # Navigation receives no privileged obstacle coordinates; it uses lidar/depth.
    return f'''<sdf version="1.9"><model name="{spec["name"]}"><static>true</static>
<link name="box"><pose>0 0 {spec["height"] / 2} 0 0 0</pose>
<collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
<visual name="visual"><geometry><box><size>{size}</size></box></geometry>
<material><ambient>1 .35 .08 1</ambient><diffuse>1 .35 .08 1</diffuse></material></visual>
</link></model></sdf>'''

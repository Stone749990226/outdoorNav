"""Version-bound human annotations shared by HTTP and Robonix."""

import json
import math
from pathlib import Path

from .routing import validate_point


class Places:
    def __init__(self, path, bundle):
        self.data = json.loads(Path(path).read_text())
        if self.data["route_version"] != bundle["route_version"]:
            raise ValueError("Place annotations belong to another road version")
        self.items = self.data["places"]
        ids = [p["id"] for p in self.items]
        if len(ids) != len(set(ids)):
            raise ValueError("Duplicate place IDs")
        for place in self.items:
            validate_point(place)
            if (
                type(place["enabled"]) is not bool
                or not isinstance(place["name"], str)
                or not place["name"].strip()
            ):
                raise ValueError("Places need a name and a boolean enabled flag")
            if not math.isfinite(place["yaw"]):
                raise ValueError("Place yaw must be finite")

    def resolve(self, name):
        found = [
            p for p in self.items if name in (p["id"], p["name"], *p.get("aliases", []))
        ]
        if len(found) != 1:
            raise ValueError(
                "Place not found or ambiguous; query places and use its ID"
            )
        return dict(found[0])

    def snapshot(self):
        return self.data

"""Shared probe IO and metric geometry. OccupancyGrid and costmap codes differ."""

import hashlib
import json
import math
from pathlib import Path
import re

OCCUPANCY_LETHAL = frozenset({99, 100})
COSTMAP_LETHAL = 254


def seq_of(path):
    match = re.search(r"_(\d+)\.", Path(path).name)
    if not match:
        raise ValueError(f"Probe file has no sequence number: {path}")
    return int(match[1])


def densify(points, step=0.05):
    if step <= 0:
        raise ValueError("Spacing must be positive")
    result = []
    for a, b in zip(points, points[1:]):
        count = max(1, int(math.dist(a, b) / step))
        result.extend(
            tuple(x + (y - x) * i / count for x, y in zip(a, b)) for i in range(count)
        )
    if points:
        result.append(tuple(points[-1]))
    return result


def sha(path):
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def probe_files(directory, max_sequence=None):
    paths = sorted(Path(directory).glob("tick_*.json"), key=seq_of)
    paths = [
        p
        for p in paths
        if (max_sequence is None or seq_of(p) <= max_sequence)
        and (p.parent / f"map_{seq_of(p):04d}.bin").is_file()
    ]
    if not paths:
        raise ValueError("No paired probe ticks and maps")
    return paths


def load_grid(directory):
    paths = sorted(Path(directory).glob("map_*.json"), key=seq_of)
    paths = [p for p in paths if p.with_suffix(".bin").is_file()]
    if not paths:
        return None, None
    path = paths[-1]
    info, data = json.loads(path.read_text()), path.with_suffix(".bin").read_bytes()
    if len(data) != info["width"] * info["height"]:
        raise ValueError("Grid data length does not match metadata")
    return info, data

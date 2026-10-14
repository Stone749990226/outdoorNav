"""The shared, versioned scene contract consumed by ROS and the web build."""

import hashlib
import json
import math
from pathlib import Path

SCHEMA_VERSION = 1
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUNDLE = ROOT / "scenes/north_campus/generated/bundle.json"


def load_bundle(path=DEFAULT_BUNDLE, verify=True):
    path = Path(path)
    data = json.loads(path.read_text())
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Scene schema mismatch; regenerate with ./nav scene build")
    if data.get("frame_id") != "map" or data.get("units") != "meters":
        raise ValueError("Scene must use map meters")
    if verify:
        for relative, digest in data["files"].items():
            resource = (path.parent / relative).resolve()
            if not resource.is_relative_to(path.parent.resolve()):
                raise ValueError("Scene resource escapes bundle directory")
            if hashlib.sha256(resource.read_bytes()).hexdigest() != digest:
                raise ValueError(
                    f"Scene resource changed: {relative}; regenerate the bundle"
                )
    return data


def masks(path=DEFAULT_BUNDLE):
    import numpy as np

    with np.load(Path(path).parent / "masks.npz", allow_pickle=False) as arrays:
        return arrays["free"].copy()


def world_cell(grid, x, y):
    col = math.floor((x - grid["origin"][0]) / grid["resolution"])
    row = math.floor((y - grid["origin"][1]) / grid["resolution"])
    return row, col


def is_free(free, grid, point):
    row, col = world_cell(grid, point["x"], point["y"])
    return (
        0 <= row < free.shape[0] and 0 <= col < free.shape[1] and bool(free[row, col])
    )


def route_mask(bundle, free, route, actual_start):
    """Rasterize the chosen reference path with each measured road's assumed width.

    Outside is keepout=100. Static free-space is intersected last, so neither
    sensor clearing nor an off-center route can remove a permanent obstacle.
    """
    import numpy as np

    grid = bundle["grid"]
    res, (ox, oy) = grid["resolution"], grid["origin"]
    allowed = np.zeros_like(free)
    widths = bundle["road_widths_m"]
    pts, edges = list(route["reference_path"]), list(route["segment_roads"])
    if len(pts) == 1:
        pts.append(pts[0])
        edges.append(route["start_road_id"])
    if (
        math.dist((actual_start["x"], actual_start["y"]), (pts[0]["x"], pts[0]["y"]))
        > 1e-6
    ):
        pts.insert(0, actual_start)
        edges.insert(0, edges[0])
    height, width = free.shape
    for a, b, road in zip(pts, pts[1:], edges):
        radius = widths[road] / 2 - res * math.sqrt(2) / 2
        c0 = max(0, math.floor((min(a["x"], b["x"]) - radius - ox) / res))
        c1 = min(width, math.ceil((max(a["x"], b["x"]) + radius - ox) / res))
        r0 = max(0, math.floor((min(a["y"], b["y"]) - radius - oy) / res))
        r1 = min(height, math.ceil((max(a["y"], b["y"]) + radius - oy) / res))
        yy, xx = np.ogrid[r0:r1, c0:c1]
        x, y = ox + (xx + 0.5) * res, oy + (yy + 0.5) * res
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        norm = dx * dx + dy * dy
        t = np.clip(((x - a["x"]) * dx + (y - a["y"]) * dy) / norm, 0, 1) if norm else 0
        allowed[r0:r1, c0:c1] |= (x - a["x"] - t * dx) ** 2 + (
            y - a["y"] - t * dy
        ) ** 2 <= radius**2
    allowed &= free
    if not is_free(allowed, grid, actual_start) or not is_free(
        allowed, grid, route["goal"]
    ):
        raise ValueError(
            "Robot or destination is outside the selected traversable road corridor"
        )
    return np.where(allowed, 0, 100).astype(np.int8)


def write_bundle(output, config, network, scene):
    import numpy as np

    output = Path(output)
    # Generator writes a PGM with a fixed three-line header. ROS grid rows run northward.
    with (output / "campus.pgm").open("rb") as stream:
        stream.readline()
        cols, rows = map(int, stream.readline().split())
        stream.readline()
        pixels = np.frombuffer(stream.read(), dtype=np.uint8).reshape(rows, cols)
    free = np.flipud(pixels == 254).copy()
    np.savez_compressed(output / "masks.npz", free=free)
    survey_dir = output.parent / "survey"
    traces = {
        key: json.loads((survey_dir / f"{key}_tracks.json").read_text())
        for key in ("raw", "filtered")
    }
    report = json.loads((survey_dir / "report.json").read_text())
    files = {
        str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(output.rglob("*"))
        if p.is_file()
        and p.suffix in (".pgm", ".yaml", ".sdf", ".dae", ".config", ".npz")
    }
    data = {
        "schema_version": SCHEMA_VERSION,
        "layout_sha256": hashlib.sha256(
            (output.parent / "layout.json").read_bytes()
        ).hexdigest(),
        "scene_id": config["name"],
        "scene_version": scene["version"],
        "route_version": network["version"],
        "frame_id": "map",
        "units": "meters",
        "origin_wgs84": network["origin_wgs84"],
        "network": network,
        "survey": traces,
        "survey_report": report,
        "geometry": scene,
        "road_widths_m": {
            r["id"]: config["road_widths"][r["id"]] for r in network["roads"]
        },
        "grid": {
            "resolution": config["resolution"],
            "origin": scene["bounds"][:2],
            "width": cols,
            "height": rows,
        },
        "spawn": scene["default_spawn"],
        "files": files,
    }
    (output / "bundle.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n"
    )
    return data

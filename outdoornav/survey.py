#!/usr/bin/env python3
"""Reconstruct a measured loop and a confirmed branch in a shared metric frame."""

from collections import Counter
import hashlib
import html
import json
import math
from pathlib import Path
from statistics import median

from .coordinates import enu
from .receiver import read_log

ROOT = Path(__file__).resolve().parents[1]

PARAMETERS = {
    "hampel_window": 11,
    "hampel_min_threshold_m": 0.5,
    "hampel_sigma": 3,
    "max_speed_m_s": 3,
    "max_gap_s": 2,
    "stationary_duration_s": 10,
    "stationary_radius_m": 0.5,
    "smoothing_weights": [1, 4, 6, 4, 1],
    "max_smoothing_displacement_m": 0.3,
    "turn_degrees": 25,
    "resample_spacing_m": 0.5,
    "preserve_resampled_turns": True,
    "simplification_tolerance_m": 0.15,
    "closure_limit_m": 2,
    "junction_limit_m": 5,
}
DEFAULT_OUTPUT = ROOT / "scenes/north_campus/survey"


def distance(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def center(points):
    return {axis: median(p[axis] for p in points) for axis in ("x", "y")}


def utc_seconds(utc):
    return int(utc[:2]) * 3600 + int(utc[2:4]) * 60 + float(utc[4:])


def timed_fixes(fixes):
    result, rejected = [], []
    day, previous = 0, None
    for index, fix in enumerate(fixes):
        seconds = utc_seconds(fix["utc"]) + day
        if previous is not None and seconds < previous - 43200:
            day += 86400
            seconds += 86400
        if previous is not None and seconds <= previous:
            rejected.append(
                {
                    "index": index,
                    "utc": fix["utc"],
                    "reason": "duplicate_or_out_of_order_time",
                }
            )
            continue
        result.append({**fix, "index": index, "time_s": seconds})
        previous = seconds
    return result, rejected


def collapse_stops(points):
    result, stops = [], []
    i = 0
    while i < len(points):
        j = i + 1
        while (
            j < len(points)
            and points[j]["time_s"] - points[i]["time_s"]
            < PARAMETERS["stationary_duration_s"]
        ):
            j += 1
        window = points[i : j + 1]
        if (
            j < len(points)
            and max(distance(p, center(window)) for p in window)
            <= PARAMETERS["stationary_radius_m"]
        ):
            anchor = center(window)
            j += 1
            while (
                j < len(points)
                and distance(points[j], anchor) <= PARAMETERS["stationary_radius_m"]
            ):
                j += 1
            group = points[i:j]
            representative = {**group[-1], **center(group), "stationary": True}
            result.append(representative)
            stops.append(
                {
                    "from_utc": group[0]["utc"],
                    "to_utc": group[-1]["utc"],
                    "count": len(group),
                    "position": center(group),
                }
            )
            i = j
        else:
            result.append(points[i])
            i += 1
    return result, stops


def smooth(points):
    result = []
    for i, point in enumerate(points):
        if i < 2 or i >= len(points) - 2 or point.get("stationary"):
            result.append(dict(point))
            continue
        a, b = points[i - 2], points[i + 2]
        u = (point["x"] - a["x"], point["y"] - a["y"])
        v = (b["x"] - point["x"], b["y"] - point["y"])
        lengths = math.hypot(*u) * math.hypot(*v)
        angle = (
            math.degrees(
                math.acos(max(-1, min(1, (u[0] * v[0] + u[1] * v[1]) / lengths)))
            )
            if lengths
            else 0
        )
        if angle >= PARAMETERS["turn_degrees"]:
            result.append(dict(point))
            continue
        candidate = {
            axis: sum(
                weight * points[i + j - 2][axis]
                for j, weight in enumerate(PARAMETERS["smoothing_weights"])
            )
            / 16
            for axis in ("x", "y")
        }
        shift = distance(candidate, point)
        ratio = (
            min(1, PARAMETERS["max_smoothing_displacement_m"] / shift) if shift else 1
        )
        result.append(
            {
                **point,
                **{
                    axis: point[axis] + ratio * (candidate[axis] - point[axis])
                    for axis in ("x", "y")
                },
            }
        )
    return result


def filter_track(points):
    rejected, usable = [], []
    half_window = PARAMETERS["hampel_window"] // 2
    max_speed = PARAMETERS["max_speed_m_s"]
    for i, point in enumerate(points):
        if point["quality"] not in (2, 4, 5):
            rejected.append(
                {
                    "index": point["index"],
                    "utc": point["utc"],
                    "reason": "unusable_position_quality",
                }
            )
            continue
        window = points[max(0, i - half_window) : i + half_window + 1]
        mid = center(window)
        residuals = [distance(p, mid) for p in window]
        mad = median(abs(r - median(residuals)) for r in residuals)
        threshold = max(
            PARAMETERS["hampel_min_threshold_m"],
            PARAMETERS["hampel_sigma"] * 1.4826 * mad,
        )
        # The bypass check prevents a real corner or a persistent shift being erased as a spike.
        spike = False
        if 0 < i < len(points) - 1:
            before, after = points[i - 1], points[i + 1]
            incoming = distance(before, point) / (point["time_s"] - before["time_s"])
            outgoing = distance(point, after) / (after["time_s"] - point["time_s"])
            bypass = distance(before, after) / (after["time_s"] - before["time_s"])
            spike = (
                distance(point, mid) > threshold
                and incoming > max_speed
                and outgoing > max_speed
                and bypass <= max_speed
            )
        if spike:
            rejected.append(
                {
                    "index": point["index"],
                    "utc": point["utc"],
                    "reason": "isolated_hampel_jump",
                }
            )
        else:
            usable.append(point)
    gaps = []
    unavailable = {p["index"] for p in points if p["quality"] not in (2, 4, 5)}

    def interrupted(a, b):
        dt = b["time_s"] - a["time_s"]
        return (
            dt > PARAMETERS["max_gap_s"]
            or distance(a, b) / dt > max_speed
            or any(a["index"] < i < b["index"] for i in unavailable)
        )

    for a, b in zip(usable, usable[1:]):
        dt = b["time_s"] - a["time_s"]
        if interrupted(a, b):
            gaps.append(
                {
                    "from_utc": a["utc"],
                    "to_utc": b["utc"],
                    "duration_s": dt,
                    "distance_m": distance(a, b),
                    "reason": "gap_or_discontinuity",
                }
            )
    chunks, chunk = [], []
    for point in usable:
        if chunk:
            dt = point["time_s"] - chunk[-1]["time_s"]
            if interrupted(chunk[-1], point):
                chunks.append(chunk)
                chunk = []
        chunk.append(point)
    if chunk:
        chunks.append(chunk)
    filtered, stops = [], []
    for chunk in chunks:
        collapsed, stationary = collapse_stops(chunk)
        stops.extend(stationary)
        filtered.append(smooth(collapsed))
    original = {p["index"]: p for p in usable}
    return filtered, {
        "rejected": rejected,
        "gaps": gaps,
        "stationary_segments": stops,
        "max_nonstationary_filter_shift_m": max(
            (
                distance(p, original[p["index"]])
                for line in filtered
                for p in line
                if not p.get("stationary")
            ),
            default=0,
        ),
        "raw_polyline_length_m": sum(
            distance(a, b) for a, b in zip(points, points[1:])
        ),
        "filtered_point_count": sum(map(len, filtered)),
    }


def projection(point, points):
    best = None
    offset = 0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        norm = dx * dx + dy * dy
        t = (
            max(
                0,
                min(
                    1, ((point["x"] - a["x"]) * dx + (point["y"] - a["y"]) * dy) / norm
                ),
            )
            if norm
            else 0
        )
        foot = {"x": a["x"] + t * dx, "y": a["y"] + t * dy}
        candidate = {
            "point": foot,
            "distance_m": distance(point, foot),
            "segment": i,
            "offset_m": offset + t * math.sqrt(norm),
        }
        if best is None or candidate["distance_m"] < best["distance_m"]:
            best = candidate
        offset += math.sqrt(norm)
    if best is None:
        raise ValueError("A polyline requires at least two points")
    return best


def resample(points, spacing=None):
    spacing = PARAMETERS["resample_spacing_m"] if spacing is None else spacing
    result = [{axis: points[0][axis] for axis in ("x", "y")}]
    next_offset, offset = spacing, 0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = distance(a, b)
        while length and next_offset < offset + length:
            t = (next_offset - offset) / length
            result.append(
                {axis: a[axis] + t * (b[axis] - a[axis]) for axis in ("x", "y")}
            )
            next_offset += spacing
        offset += length
        if PARAMETERS["preserve_resampled_turns"] and i + 2 < len(points):
            c = points[i + 2]
            u = (b["x"] - a["x"], b["y"] - a["y"])
            v = (c["x"] - b["x"], c["y"] - b["y"])
            norm = math.hypot(*u) * math.hypot(*v)
            angle = (
                math.degrees(
                    math.acos(max(-1, min(1, (u[0] * v[0] + u[1] * v[1]) / norm)))
                )
                if norm
                else 0
            )
            if angle >= PARAMETERS["turn_degrees"] and distance(result[-1], b) > 1e-9:
                result.append({axis: b[axis] for axis in ("x", "y")})
    end = {axis: points[-1][axis] for axis in ("x", "y")}
    if distance(result[-1], end) > 1e-9:
        result.append(end)
    return result


def simplify(points, tolerance=None):
    tolerance = (
        PARAMETERS["simplification_tolerance_m"] if tolerance is None else tolerance
    )
    if len(points) <= 2:
        return points
    ranked = [
        (projection(p, [points[0], points[-1]])["distance_m"], i)
        for i, p in enumerate(points[1:-1], 1)
    ]
    deviation, index = max(ranked)
    if deviation <= tolerance:
        return [points[0], points[-1]]
    return simplify(points[: index + 1], tolerance)[:-1] + simplify(
        points[index:], tolerance
    )


def reconstruct(loop, branch):
    end = loop[-1]
    early = [
        p
        for p in loop
        if p["time_s"] <= loop[0]["time_s"] + 0.2 * (end["time_s"] - loop[0]["time_s"])
    ]
    seam = projection(end, early)
    if seam["distance_m"] > PARAMETERS["closure_limit_m"]:
        raise ValueError(f"Loop closure exceeds 2 m: {seam['distance_m']:.2f} m")
    ring = [seam["point"], *loop[seam["segment"] + 1 : -1], seam["point"]]
    junction = projection(branch[-1], ring)
    if junction["distance_m"] > PARAMETERS["junction_limit_m"]:
        raise ValueError(
            f"Confirmed junction correction exceeds 5 m: {junction['distance_m']:.2f} m"
        )
    i, node = junction["segment"], junction["point"]
    arcs = [ring[: i + 1] + [node], [node] + ring[i + 1 :]]
    roads = [
        {
            "id": "loop-a",
            "from": "S",
            "to": "J",
            "name": "北区环线 A",
            "source_log": "1.ubx",
            "points": arcs[0],
        },
        {
            "id": "loop-b",
            "from": "J",
            "to": "S",
            "name": "北区环线 B",
            "source_log": "1.ubx",
            "points": arcs[1],
        },
        {
            "id": "branch",
            "from": "P",
            "to": "J",
            "name": "中央斜向道路",
            "source_log": "2.ubx",
            "points": branch + [node],
        },
    ]
    for road in roads:
        road["points"] = simplify(resample(road["points"]))
        road["length_m"] = sum(
            distance(a, b) for a, b in zip(road["points"], road["points"][1:])
        )
    nodes = [
        {"id": "S", "name": "环线起点", **seam["point"]},
        {"id": "J", "name": "公共路口", **node},
        {
            "id": "P",
            "name": "支路端点",
            **{axis: branch[0][axis] for axis in ("x", "y")},
        },
    ]
    return (
        nodes,
        roads,
        {
            "closure": seam,
            "junction": {**junction, "confirmed_by_user": True},
            "loop_trim_before_utc": loop[seam["segment"] + 1]["utc"],
            "closure_adjustment": "local_seam_only",
            "junction_adjustment": "confirmed_terminal_connection",
        },
    )


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def build(
    loop_path=ROOT / "1.ubx",
    branch_path=ROOT / "2.ubx",
    output=DEFAULT_OUTPUT,
    origin_path=None,
):
    parsed, summaries, source_hashes = {}, {}, {}
    for label, path in (("loop", loop_path), ("branch", branch_path)):
        data = path.read_bytes()
        nmea, bad, fixes, pvt = read_log(data)
        parsed[label], rejected = timed_fixes(fixes)
        source_hashes[label] = hashlib.sha256(data).hexdigest()
        summaries[label] = {
            "source_log": path.name,
            "sha256": source_hashes[label],
            "valid_nmea": len(nmea),
            "bad_checksums": bad,
            "gga_count": len(fixes),
            "quality_counts": dict(Counter(str(f["quality"]) for f in fixes)),
            "nav_pvt_count": len(pvt),
            "carrier_counts": dict(Counter(str(p["carrier"]) for p in pvt)),
            "receiver_hacc_median_m": median(p["hacc_m"] for p in pvt) if pvt else None,
            "time_rejected": rejected,
        }
    last = parsed["loop"][-1]
    tail = [p for p in parsed["loop"] if p["time_s"] >= last["time_s"] - 10]
    if len(tail) < 2 or tail[-1]["time_s"] - tail[0]["time_s"] < 9.5:
        raise ValueError(
            "At least ten seconds of stable endpoint observations are required"
        )
    candidate = {
        axis: median(p[axis] for p in tail) for axis in ("latitude", "longitude")
    }
    if (
        max(math.hypot(*enu(p["latitude"], p["longitude"], candidate)) for p in tail)
        > 0.5
    ):
        raise ValueError("Loop endpoint is not stable within 0.5 m")
    origin_file = Path(origin_path) if origin_path else output / "origin.json"
    origin = json.loads(origin_file.read_text()) if origin_file.exists() else candidate
    raw_tracks, filtered_tracks = [], []
    lines = {}
    for label, path in (("loop", loop_path), ("branch", branch_path)):
        points = [
            {**p, **dict(zip(("x", "y"), enu(p["latitude"], p["longitude"], origin)))}
            for p in parsed[label]
        ]
        chunks, report = filter_track(points)
        summaries[label].update(report)
        raw_tracks.append({"source_log": path.name, "points": points})
        filtered_tracks.append({"source_log": path.name, "segments": chunks})
        if len(chunks) != 1 or len(chunks[0]) < 2:
            raise ValueError(
                f"{path.name} contains missing or discontinuous segments; no road will bridge them"
            )
        lines[label] = chunks[0]
    nodes, roads, joins = reconstruct(lines["loop"], lines["branch"])
    for road in roads:
        road["source_log"] = (
            branch_path.name if road["id"] == "branch" else loop_path.name
        )
    signature = {
        "sources": source_hashes,
        "parameters": PARAMETERS,
        "origin": origin,
        "algorithm_version": 1,
    }
    version = (
        "survey-"
        + hashlib.sha256(json.dumps(signature, sort_keys=True).encode()).hexdigest()[
            :12
        ]
    )
    metric = {
        "version": version,
        "frame_id": "map",
        "units": "meters",
        "axes": "X east, Y north",
        "origin_wgs84": origin,
        "model": "centerline_graph",
        "nodes": nodes,
        "roads": roads,
        "parameters": PARAMETERS,
        "quality_status": "survey_dgps_unverified",
        "executable_on_robot": False,
    }
    report = {
        "version": version,
        "parameters": PARAMETERS,
        "logs": summaries,
        "joins": joins,
        "quality_status": "survey_dgps_unverified",
        "absolute_accuracy_verified": False,
        "simulation_map": "uniform_free_space_for_controller_testing_only",
    }
    all_points = [p for r in roads for p in r["points"]]
    min_x = math.floor(min(p["x"] for p in all_points) - 10)
    min_y = math.floor(min(p["y"] for p in all_points) - 10)
    max_x = math.ceil(max(p["x"] for p in all_points) + 10)
    max_y = math.ceil(max(p["y"] for p in all_points) + 10)
    scale = 0.125

    def drawing(p):
        return {"x": (p["x"] - min_x) / scale, "y": (max_y - p["y"]) / scale}

    for name, value in (
        ("origin", origin),
        ("raw_tracks", raw_tracks),
        ("filtered_tracks", filtered_tracks),
        ("road_network", metric),
        ("report", report),
    ):
        write_json(output / (name + ".json"), value)
    svg_paths = "\n".join(
        f'<polyline points="{" ".join(str(drawing(p)["x"]) + "," + str(drawing(p)["y"]) for p in r["points"])}" fill="none" stroke="{"#167a63" if r["id"] != "branch" else "#b26123"}" stroke-width="4"><title>{html.escape(r["name"])}</title></polyline>'
        for r in roads
    )
    width, height = (max_x - min_x) / scale, (max_y - min_y) / scale
    (output / "map.svg").write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="#f7faf9"/><text x="24" y="30" font-family="sans-serif" font-size="18">North building measured network | X east, Y north | DGPS</text>{svg_paths}<path d="M24 {height - 35}h{10 / scale}" stroke="#26352f" stroke-width="2"/><text x="24" y="{height - 45}" font-family="sans-serif">10 m</text><text x="{width - 40}" y="45" font-family="sans-serif">N ↑</text></svg>\n'
    )
    return metric, report

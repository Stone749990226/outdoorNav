#!/usr/bin/env python3
"""Prepare a provisional campus alignment and a simulation-only Nav2 map."""

import argparse
import json
import math
from pathlib import Path
import re
import struct

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "北侧路口到东侧楼前.ubx"
CAMPUS = ROOT / "outdoor-patrol-demo/src/campus.json"
OUTPUT = ROOT / "outdoor-patrol-demo/src/calibration.json"
SIM_MAP = ROOT / "ros2_bridge/maps"
NMEA = re.compile(rb"\$([A-Z]{2}[A-Z]{3}),([^\r\n]*?)\*([0-9A-Fa-f]{2})\r\n")


def nmea_records(data):
    valid = []
    invalid = 0
    for match in NMEA.finditer(data):
        body = match.group(1) + b"," + match.group(2)
        checksum = 0
        for byte in body:
            checksum ^= byte
        if checksum != int(match.group(3), 16):
            invalid += 1
            continue
        valid.append((match.group(1).decode(), match.group(2).decode().split(",")))
    return valid, invalid


def ubx_records(data):
    messages = []
    offset = 0
    while offset + 8 <= len(data):
        if data[offset : offset + 2] != b"\xb5\x62":
            offset += 1
            continue
        msg_class, msg_id, size = struct.unpack_from("<BBH", data, offset + 2)
        end = offset + 6 + size
        if end + 2 > len(data):
            offset += 1
            continue
        first = second = 0
        for byte in data[offset + 2 : end]:
            first = (first + byte) & 255
            second = (second + first) & 255
        if data[end : end + 2] != bytes((first, second)):
            offset += 1
            continue
        messages.append((msg_class, msg_id, data[offset + 6 : end]))
        offset = end + 2
    return messages


def degrees(value, hemisphere):
    digits = 2 if hemisphere in "NS" else 3
    result = int(value[:digits]) + float(value[digits:]) / 60
    return -result if hemisphere in "SW" else result


def read_log(data):
    nmea, bad = nmea_records(data)
    gga = []
    for name, fields in nmea:
        if not name.endswith("GGA") or len(fields) < 14:
            continue
        if not fields[1] or not fields[3]:
            continue
        gga.append(
            {
                "utc": fields[0],
                "latitude": degrees(fields[1], fields[2]),
                "longitude": degrees(fields[3], fields[4]),
                "quality": int(fields[5]),
                "satellites": int(fields[6]),
                "hdop": float(fields[7]),
            }
        )
    pvt = []
    for msg_class, msg_id, payload in ubx_records(data):
        if (msg_class, msg_id) == (1, 7) and len(payload) >= 92:
            pvt.append(
                {
                    "carrier": (payload[21] >> 6) & 3,
                    "differential": bool(payload[21] & 2),
                    "hacc_m": struct.unpack_from("<I", payload, 40)[0] / 1000,
                }
            )
    if len(gga) < 2:
        raise ValueError("At least two valid GGA positions are required")
    return nmea, bad, gga, pvt


def local_radii(latitude):
    a = 6378137.0
    eccentricity_sq = 6.69437999014e-3
    sin_lat = math.sin(math.radians(latitude))
    denominator = math.sqrt(1 - eccentricity_sq * sin_lat * sin_lat)
    east_radius = a / denominator * math.cos(math.radians(latitude))
    north_radius = a * (1 - eccentricity_sq) / denominator**3
    return east_radius, north_radius


def enu(latitude, longitude, anchor):
    east_radius, north_radius = local_radii(anchor["latitude"])
    return (
        math.radians(longitude - anchor["longitude"]) * east_radius,
        math.radians(latitude - anchor["latitude"]) * north_radius,
    )


def calibration_from_points(start, end, north, east):
    dx = east["x"] - north["x"]
    dy = north["y"] - east["y"]
    measured_east, measured_north = enu(end["latitude"], end["longitude"], start)
    scale = math.hypot(measured_east, measured_north) / math.hypot(dx, dy)
    rotation = math.atan2(measured_north, measured_east) - math.atan2(dy, dx)
    return {
        "version": "provisional-2026-09-28-01",
        "status": "provisional",
        "robot_executable": False,
        "frame_id": "map",
        "map_origin_svg": north,
        "origin_wgs84": {"latitude": start["latitude"], "longitude": start["longitude"]},
        "end_wgs84": {"latitude": end["latitude"], "longitude": end["longitude"]},
        "meters_per_unit": scale,
        "rotation_rad": rotation,
        "anchor_quality": {"start": start["quality"], "end": end["quality"]},
        "anchor_utc": {"start": start["utc"], "end": end["utc"]},
        "anchor_match": "nearby_landmarks_not_surveyed_control_points",
        "source_log": LOG.name,
        "warning": "Simulation only: endpoint associations and map geometry are not field verified.",
    }


def map_point(point, alignment):
    origin = alignment["map_origin_svg"]
    dx = point["x"] - origin["x"]
    dy = origin["y"] - point["y"]
    rotation = alignment["rotation_rad"]
    scale = alignment["meters_per_unit"]
    return (
        scale * (math.cos(rotation) * dx - math.sin(rotation) * dy),
        scale * (math.sin(rotation) * dx + math.cos(rotation) * dy),
    )


def segment_distance(point, start, end):
    dx, dy = end["x"] - start["x"], end["y"] - start["y"]
    norm = dx * dx + dy * dy
    t = max(0, min(1, ((point["x"] - start["x"]) * dx + (point["y"] - start["y"]) * dy) / norm)) if norm else 0
    return math.hypot(point["x"] - start["x"] - t * dx, point["y"] - start["y"] - t * dy)


def inside_polygon(point, polygon):
    inside = False
    j = len(polygon) - 1
    for i, vertex in enumerate(polygon):
        previous = polygon[j]
        if (vertex["y"] > point["y"]) != (previous["y"] > point["y"]):
            cross = (previous["x"] - vertex["x"]) * (point["y"] - vertex["y"]) / (previous["y"] - vertex["y"]) + vertex["x"]
            if point["x"] < cross:
                inside = not inside
        j = i
    return inside


def write_sim_map(campus, alignment, destination):
    destination.mkdir(parents=True, exist_ok=True)
    step = campus["navigation"]["grid_resolution_units"]
    resolution = alignment["meters_per_unit"] * step
    corners = [
        map_point({"x": x, "y": y}, alignment)
        for x in (0, campus["width"])
        for y in (0, campus["height"])
    ]
    min_x = min(p[0] for p in corners)
    min_y = min(p[1] for p in corners)
    max_x = max(p[0] for p in corners)
    max_y = max(p[1] for p in corners)
    width = math.ceil((max_x - min_x) / resolution)
    height = math.ceil((max_y - min_y) / resolution)
    road_radius = campus["navigation"]["road_width_units"] / 2
    segments = [
        (start, end)
        for road in campus["roads"]
        for start, end in zip(road["points"], road["points"][1:])
    ]
    pixels = bytearray()
    for row in range(height):
        for column in range(width):
            map_x = min_x + (column + 0.5) * resolution
            map_y = min_y + (height - row - 0.5) * resolution
            angle = alignment["rotation_rad"]
            scale = alignment["meters_per_unit"]
            point = {
                "x": alignment["map_origin_svg"]["x"] + (math.cos(angle) * map_x + math.sin(angle) * map_y) / scale,
                "y": alignment["map_origin_svg"]["y"] + (math.sin(angle) * map_x - math.cos(angle) * map_y) / scale,
            }
            if not (0 <= point["x"] < campus["width"] and 0 <= point["y"] < campus["height"]):
                pixels.append(205)
                continue
            if any(inside_polygon(point, region["points"]) for region in campus["regions"]):
                pixels.append(0 if any(inside_polygon(point, r["points"]) and r["kind"] == "building" for r in campus["regions"]) else 205)
            elif any(segment_distance(point, a, b) <= road_radius for a, b in segments):
                pixels.append(254)
            else:
                pixels.append(205)
    image = destination / "campus_sim.pgm"
    image.write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    (destination / "campus_sim.yaml").write_text(
        "image: campus_sim.pgm\n"
        f"resolution: {resolution:.12f}\n"
        f"origin: [{min_x:.9f}, {min_y:.9f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )


def prepare(log_path=LOG, campus_path=CAMPUS):
    nmea, bad, fixes, pvt = read_log(log_path.read_bytes())
    campus = json.loads(campus_path.read_text())
    if campus.get("navigation", {}).get("model") == "centerline_graph":
        raise ValueError("Measured network installed: use tools/build_survey_map.py; legacy alignment cannot replace it")
    north = next(node for node in campus["nodes"] if node["id"] == "N")
    east = next(node for node in campus["nodes"] if node["id"] == "E")
    alignment = calibration_from_points(fixes[0], fixes[-1], north, east)
    OUTPUT.write_text(json.dumps(alignment, ensure_ascii=False, indent=2) + "\n")
    write_sim_map(campus, alignment, SIM_MAP)
    (SIM_MAP / "calibration.json").write_text(json.dumps(alignment, ensure_ascii=False, indent=2) + "\n")
    jumps = []
    for before, after in zip(fixes, fixes[1:]):
        distance = math.hypot(*enu(after["latitude"], after["longitude"], before))
        if distance > 2:
            jumps.append({"from_utc": before["utc"], "to_utc": after["utc"], "distance_m": round(distance, 2)})
    trace = {
        "source_log": log_path.name,
        "valid_nmea": len(nmea),
        "invalid_nmea": bad,
        "quality_counts": {
            str(q): sum(f["quality"] == q for f in fixes)
            + sum(name.endswith("GGA") and len(fields) > 5 and fields[5] == str(q) and not fields[1] for name, fields in nmea)
            for q in (0, 1, 2, 4, 5)
        },
        "nav_pvt_count": len(pvt),
        "jumps_over_2m": jumps,
        "fixed_trace": [f for f in fixes if f["quality"] == 4],
    }
    (SIM_MAP / "survey_summary.json").write_text(json.dumps(trace, ensure_ascii=False, indent=2) + "\n")
    (ROOT / "outdoor-patrol-demo/src/fixed_trace.json").write_text(
        json.dumps(trace["fixed_trace"], ensure_ascii=False, separators=(",", ":")) + "\n"
    )
    return trace, alignment


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=LOG)
    arguments = parser.parse_args()
    summary, calibration = prepare(arguments.log)
    print(json.dumps({key: value for key, value in summary.items() if key != "fixed_trace"}, ensure_ascii=False, indent=2))
    print(f"scale={calibration['meters_per_unit']:.9f} m/unit rotation={math.degrees(calibration['rotation_rad']):.4f} deg")

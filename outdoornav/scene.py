#!/usr/bin/env python3
"""Generate Gazebo geometry and a conservative Nav2 map from one metric scene.

Survey inputs are read-only. Geometry checks cover continuous lines, not just
vertices. No satellite artwork or building shadows are interpreted as obstacles.
"""

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import shapely
from shapely.geometry import LineString, Point, Polygon, box, mapping
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "scenes/north_campus"


def read(path):
    return json.loads(path.read_text())


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")


def survey_lines():
    source = HERE / "survey"
    raw = read(source / "raw_tracks.json")
    filtered = read(source / "filtered_tracks.json")
    network = read(source / "road_network.json")
    lines = {}
    # Explicitly split at missing/invalid observations: never invent a crossing.
    for track in raw:
        segments, current = [], []
        for p in track["points"]:
            if p.get("quality", 0) <= 0 or (
                current and p["time_s"] - current[-1]["time_s"] > 2
            ):
                if len(current) > 1:
                    segments.append(current)
                current = []
            if p.get("quality", 0) > 0:
                current.append(p)
        if len(current) > 1:
            segments.append(current)
        for i, segment in enumerate(segments):
            lines[f"raw/{track['source_log']}/{i}"] = LineString(
                [(p["x"], p["y"]) for p in segment]
            )
    for track in filtered:
        for i, segment in enumerate(track["segments"]):
            lines[f"filtered/{track['source_log']}/{i}"] = LineString(
                [(p["x"], p["y"]) for p in segment]
            )
    for road in network["roads"]:
        lines[f"network/{road['id']}"] = LineString(
            [(p["x"], p["y"]) for p in road["points"]]
        )
    return network, lines


def geometry(config):
    network, lines = survey_lines()
    trace = unary_union(list(lines.values()))
    # Polygonal buffers slightly overestimate the exact circular clearance test.
    protected = trace.buffer(
        config["robot_radius"] + config["trace_margin"] + 0.01, quad_segs=16
    )
    surfaces = [
        line.buffer(
            config["road_widths"].get(name.split("/")[-1], 4.0) / 2, quad_segs=12
        )
        for name, line in lines.items()
        if name.startswith("network/")
    ]
    # Include every measured approach/stop envelope, even if trimmed from road_network.
    surfaces += [
        line.buffer(config["road_widths"]["raw_approach"] / 2, quad_segs=12)
        for name, line in lines.items()
        if not name.startswith("network/")
    ]
    roads = unary_union(surfaces)
    plaza = Polygon(config["plaza"])
    obstacles, adjustments = [], []
    for spec in config["buildings"] + config["obstacles"]:
        footprint = box(*spec["bounds"])
        # Keep rectangular buildings by shrinking toward their centre, deterministically.
        original = footprint
        while footprint.intersects(protected):
            footprint = footprint.buffer(-0.25, join_style=2)
            if footprint.is_empty:
                raise ValueError(f"No safe placement for {spec['name']}")
        obstacles.append({**spec, "footprint": footprint})
        if not footprint.equals(original):
            adjustments.append(
                {
                    "name": spec["name"],
                    "original_bounds": spec["bounds"],
                    "bounds": list(footprint.bounds),
                }
            )
    # Trees outside paved space; trunk collision is the map obstacle, canopy is visual.
    paved = roads.union(plaza)
    for i, (x, y) in enumerate(
        [(105, y) for y in range(-84, 5, 8)]
        + [(x, 15) for x in range(12, 100, 9)]
        + [(x, y) for x, y in [(0, -65), (5, -72), (12, -61), (17, -81), (23, -85)]]
    ):
        trunk = Point(x, y).buffer(0.3, quad_segs=16)
        if trunk.intersects(protected) or trunk.intersects(paved):
            continue
        obstacles.append(
            {
                "name": f"tree_{i}",
                "footprint": trunk,
                "height": 2.6,
                "color": "0.32 0.20 0.11 1",
                "tree": True,
            }
        )
    blocked = unary_union([o["footprint"] for o in obstacles])
    free = paved.difference(blocked)
    # Exact distance guards the small error of polygonal circular buffers.
    if trace.distance(free.boundary) < config["robot_radius"] + config["trace_margin"]:
        raise ValueError("Continuous measured trajectory clearance is insufficient")
    if not free.covers(protected) or blocked.intersects(protected):
        raise ValueError("Measured trajectory protection corridor is obstructed")
    return network, lines, roads, plaza, free, obstacles, adjustments


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def shape(link, name, kind, size, pose, color, collision=True):
    for tag in ["visual", "collision"] if collision else ["visual"]:
        obj = element(link, tag, name=name)
        element(obj, "pose", " ".join(map(str, pose)))
        geo = element(element(obj, "geometry"), kind)
        if kind == "box":
            element(geo, "size", " ".join(map(str, size)))
        elif kind == "cylinder":
            element(geo, "radius", size[0])
            element(geo, "length", size[1])
        elif kind == "sphere":
            element(geo, "radius", size[0])
        if tag == "visual":
            material = element(obj, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)


def mesh(path, polygon):
    triangles = shapely.constrained_delaunay_triangles(polygon)
    # GEOS triangulation uses clockwise rings; Ogre needs upward-facing CCW faces.
    coordinates = [
        list(shapely.orient_polygons(t).exterior.coords)[:3] for t in triangles.geoms
    ]
    vertices = [f"{x:.6f} {y:.6f} 0" for triangle in coordinates for x, y in triangle]
    count = len(vertices)
    path.write_text(f'''<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="surface"><mesh>
<source id="positions"><float_array id="array" count="{count * 3}">{" ".join(vertices)}</float_array>
<technique_common><accessor source="#array" count="{count}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="normals"><float_array id="normal-array" count="3">0 0 1</float_array><technique_common><accessor source="#normal-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="{len(coordinates)}"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><p>{" ".join(f"{i} 0" for i in range(count))}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="node"><instance_geometry url="#surface"/></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>''')


def write_world(output, config, roads, plaza, obstacles, bounds):
    model_dir = output / "models/campus_scene"
    (model_dir / "meshes").mkdir(parents=True, exist_ok=True)
    root = ET.Element("sdf", version="1.6")
    model = element(root, "model", name="campus_scene")
    element(model, "static", "true")
    link = element(model, "link", name="site")
    xmin, ymin, xmax, ymax = bounds
    shape(
        link,
        "ground",
        "box",
        [xmax - xmin, ymax - ymin, 0.2],
        [(xmin + xmax) / 2, (ymin + ymax) / 2, -0.071, 0, 0, 0],
        "0.29 0.40 0.25 1",
    )
    for name, poly, z, color in [
        ("roads", roads.difference(plaza), 0.03, "0.25 0.29 0.31 1"),
        ("plaza", plaza, 0.03, "0.72 0.70 0.62 1"),
    ]:
        mesh(model_dir / f"meshes/{name}.dae", poly)
        visual = element(link, "visual", name=name)
        element(visual, "pose", f"0 0 {z} 0 0 0")
        element(
            element(element(visual, "geometry"), "mesh"),
            "uri",
            f"model://campus_scene/meshes/{name}.dae",
        )
        material = element(visual, "material")
        element(material, "ambient", color)
        element(material, "diffuse", color)
        element(visual, "cast_shadows", "false")
    for obj in obstacles:
        x0, y0, x1, y1 = obj["footprint"].bounds
        x, y, height = (x0 + x1) / 2, (y0 + y1) / 2, obj["height"]
        if obj.get("tree"):
            shape(
                link,
                obj["name"],
                "cylinder",
                [0.3, height],
                [x, y, height / 2, 0, 0, 0],
                obj["color"],
            )
            shape(
                link,
                obj["name"] + "_canopy",
                "sphere",
                [1.75],
                [x, y, 3.8, 0, 0, 0],
                "0.18 0.36 0.19 1",
                False,
            )
        else:
            shape(
                link,
                obj["name"],
                "box",
                [x1 - x0, y1 - y0, height],
                [x, y, height / 2, 0, 0, 0],
                obj["color"],
            )
            if "tower" in obj["name"] or "podium" in obj["name"]:
                # Thin coloured bands are decorative; no collision beyond the footprint.
                for floor in range(1, round(height / 3)):
                    z = floor * 3
                    for side, yy in [("south", y0 - 0.06), ("north", y1 + 0.06)]:
                        shape(
                            link,
                            f"{obj['name']}_{side}_{floor}",
                            "box",
                            [x1 - x0 - 0.5, 0.02, 1.8],
                            [x, yy, z, 0, 0, 0],
                            "0.17 0.31 0.39 1",
                            False,
                        )
                    for side, xx in [("west", x0 - 0.06), ("east", x1 + 0.06)]:
                        shape(
                            link,
                            f"{obj['name']}_{side}_{floor}",
                            "box",
                            [0.02, y1 - y0 - 0.5, 1.8],
                            [xx, y, z, 0, 0, 0],
                            "0.17 0.31 0.39 1",
                            False,
                        )
                shape(
                    link,
                    obj["name"] + "_roof",
                    "box",
                    [x1 - x0 - 0.2, y1 - y0 - 0.2, 0.1],
                    [x, y, height + 0.05, 0, 0, 0],
                    "0.72 0.75 0.73 1",
                    False,
                )
            elif "planter" in obj["name"]:
                shape(
                    link,
                    obj["name"] + "_plants",
                    "box",
                    [x1 - x0 - 0.25, y1 - y0 - 0.25, 0.15],
                    [x, y, height + 0.075, 0, 0, 0],
                    "0.21 0.40 0.19 1",
                    False,
                )
    ET.indent(root)
    ET.ElementTree(root).write(
        model_dir / "model.sdf", encoding="unicode", xml_declaration=True
    )
    (model_dir / "model.config").write_text(
        '<model><name>North campus</name><version>1.0</version><sdf version="1.6">model.sdf</sdf><description>Survey-constrained simulation campus</description></model>\n'
    )
    root = ET.Element("sdf", version="1.6")
    world = element(root, "world", name="north_campus")
    element(world, "gravity", "0 0 -9.81")
    physics = element(world, "physics", name="ode", type="ode")
    element(physics, "max_step_size", "0.002")
    element(physics, "real_time_update_rate", "500")
    scene = element(world, "scene")
    element(scene, "ambient", "0.65 0.65 0.65 1")
    element(scene, "background", "0.73 0.83 0.90 1")
    element(scene, "shadows", "false")
    light = element(world, "light", name="sun", type="directional")
    element(light, "pose", "0 0 150 0 0 0")
    element(light, "diffuse", "0.85 0.85 0.80 1")
    element(light, "specular", "0.15 0.15 0.15 1")
    element(light, "direction", "-0.4 0.2 -1")
    element(element(world, "include"), "uri", "model://campus_scene")
    gui = element(world, "gui", fullscreen="false")
    camera = element(gui, "camera", name="user_camera")
    element(camera, "pose", "210 -255 195 0 0.60 2.22")
    element(camera, "view_controller", "orbit")
    element(world, "plugin", name="gazebo_ros_state", filename="libgazebo_ros_state.so")
    ET.indent(root)
    # Fortress owns physics and sensors through world systems; there is no Classic runtime.
    world.remove(world.find("gui"))
    world.remove(world.find("plugin"))
    physics.set("type", "dartsim")
    spherical = element(world, "spherical_coordinates")
    for name, value in [
        ("surface_model", "EARTH_WGS84"),
        ("world_frame_orientation", "ENU"),
        ("latitude_deg", survey_lines()[0]["origin_wgs84"]["latitude"]),
        ("longitude_deg", survey_lines()[0]["origin_wgs84"]["longitude"]),
        ("elevation", 0),
    ]:
        element(spherical, name, value)
    for suffix, system in [
        ("physics", "Physics"),
        ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"),
        ("sensors", "Sensors"),
        ("imu", "Imu"),
        ("navsat", "NavSat"),
    ]:
        plugin = element(
            world,
            "plugin",
            filename=f"ignition-gazebo-{suffix}-system",
            name=f"ignition::gazebo::systems::{system}",
        )
        if suffix == "sensors":
            element(plugin, "render_engine", "ogre2")
    ET.indent(root)
    ET.ElementTree(root).write(
        output / "world.sdf", encoding="unicode", xml_declaration=True
    )


def raster(free, bounds, resolution):
    xmin, ymin, xmax, ymax = bounds
    cols, rows = round((xmax - xmin) / resolution), round((ymax - ymin) / resolution)
    # Erode by cell circumradius so every pixel marked free is wholly traversable.
    safe = free.buffer(-math.sqrt(2) * resolution / 2)
    values = np.zeros((rows, cols), dtype=np.uint8)
    for row in range(0, rows, 128):
        yy, xx = np.meshgrid(
            ymax - (np.arange(row, min(row + 128, rows)) + 0.5) * resolution,
            xmin + (np.arange(cols) + 0.5) * resolution,
            indexing="ij",
        )
        values[row : row + 128] = np.where(shapely.contains_xy(safe, xx, yy), 254, 0)
    return values


def preview(output, free, roads, plaza, obstacles, lines, bounds):
    xmin, ymin, xmax, ymax = bounds
    scale = 9
    image = Image.new(
        "RGB", (round((xmax - xmin) * scale), round((ymax - ymin) * scale)), "#4a6640"
    )
    draw = ImageDraw.Draw(image)

    def pixels(points):
        return [((x - xmin) * scale, (ymax - y) * scale) for x, y in points]

    def fill(geom, color):
        for poly in geom.geoms if hasattr(geom, "geoms") else [geom]:
            draw.polygon(pixels(poly.exterior.coords), fill=color)
            for hole in poly.interiors:
                draw.polygon(pixels(hole.coords), fill="#4a6640")

    fill(roads, "#414a4f")
    fill(plaza, "#b8b39e")
    for obj in obstacles:
        fill(obj["footprint"], "#708b9a" if obj["height"] > 10 else "#315b31")
    for name, line in lines.items():
        if name.startswith("raw/"):
            draw.line(pixels(line.coords), fill="#efcf52", width=2)
    draw.text(
        (18, 18), "NORTH CAMPUS | measured traces in yellow | +Y North", fill="white"
    )
    draw.text(
        (18, 36), "Simulation geometry / assumed widths and buildings", fill="white"
    )
    image.save(output / "site_plan.png")


def build(output, config_path=HERE / "layout.json"):
    config = read(config_path)
    network, lines, roads, plaza, free, obstacles, adjustments = geometry(config)
    extent = unary_union([roads, plaza, *[o["footprint"] for o in obstacles]]).bounds
    bounds = [
        math.floor(extent[0] - 12),
        math.floor(extent[1] - 12),
        math.ceil(extent[2] + 12),
        math.ceil(extent[3] + 12),
    ]
    output.mkdir(parents=True, exist_ok=True)
    image = raster(free, bounds, config["resolution"])
    rows, cols = image.shape
    (output / "campus.pgm").write_bytes(
        f"P5\n{cols} {rows}\n255\n".encode() + image.tobytes()
    )
    (output / "campus.yaml").write_text(
        f"image: campus.pgm\nresolution: {config['resolution']}\norigin: [{bounds[0]}, {bounds[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n"
    )
    write_world(output, config, roads, plaza, obstacles, bounds)
    preview(output, free, roads, plaza, obstacles, lines, bounds)
    traces = [
        {"name": name, "points": list(line.coords)} for name, line in lines.items()
    ]
    write_json(output / "traces.json", traces)
    for task in config["tasks"]:
        for key in ["start", "goal"] if task.get("expect_success", True) else ["start"]:
            if not free.covers(
                Point(task[key][:2]).buffer(config["robot_radius"] + 0.1)
            ):
                raise ValueError(
                    f"Task {task['name']} {key} is not safely traversable: {task[key]}"
                )
    signature = hashlib.sha256(
        (json.dumps(config, sort_keys=True) + network["version"] + free.wkt).encode()
    ).hexdigest()[:12]
    write_json(
        output / "scene_geometry.json",
        {
            "version": f"campus-{signature}",
            "survey_version": network["version"],
            "origin_wgs84": network["origin_wgs84"],
            "frame_id": "map",
            "bounds": bounds,
            "resolution": config["resolution"],
            "robot_radius": config["robot_radius"],
            "free": mapping(free),
            "plaza": mapping(plaza),
            "obstacles": [
                {k: (mapping(v) if k == "footprint" else v) for k, v in obj.items()}
                for obj in obstacles
            ],
            "tasks": config["tasks"],
            "default_spawn": [
                network["nodes"][0]["x"],
                network["nodes"][0]["y"],
                -1.24,
            ],
        },
    )
    report = {
        "survey_version": network["version"],
        "scene_version": f"campus-{signature}",
        "required_clearance_m": 0.85,
        "map_resolution_m": config["resolution"],
        "free_area_m2": free.area,
        "adjusted_obstacles": adjustments,
        "assumptions": "Flat simulation; building shapes/heights and widths are illustrative; ideal localization.",
        "traces": {
            name: {
                "length_m": line.length,
                "coverage_fraction": line.intersection(free).length / line.length,
                "min_boundary_clearance_m": line.distance(free.boundary),
                "protected_area_covered": free.covers(line.buffer(0.86)),
            }
            for name, line in lines.items()
        },
    }
    write_json(output / "validation.json", report)
    print(
        json.dumps(
            {
                "output": str(output),
                "version": signature,
                "free_area_m2": round(free.area, 2),
                "min_clearance_m": min(
                    v["min_boundary_clearance_m"] for v in report["traces"].values()
                ),
            },
            indent=2,
        )
    )
    from .bundle import write_bundle

    write_bundle(output, config, network, read(output / "scene_geometry.json"))
    return report

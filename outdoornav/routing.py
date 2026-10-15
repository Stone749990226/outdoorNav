"""Shortest paths on measured polylines, including endpoints inside an edge."""

import heapq
import math

from .survey import distance, projection


def validate_point(value):
    if not isinstance(value, dict) or not all(
        isinstance(value.get(k), (int, float))
        and not isinstance(value[k], bool)
        and math.isfinite(value[k])
        for k in ("x", "y")
    ):
        raise ValueError("A map point requires finite x and y in meters")
    return {k: float(value[k]) for k in ("x", "y")}


def snap(network, point, blocked=()):
    point = validate_point(point)
    candidates = [
        {**projection(point, r["points"]), "road_id": r["id"]}
        for r in network["roads"]
        if r["id"] not in blocked
    ]
    if not candidates:
        raise ValueError("No open measured road")
    return min(candidates, key=lambda s: s["distance_m"])


def plan_route(network, start, goal, blocked=()):
    unknown = set(blocked) - {r["id"] for r in network["roads"]}
    if unknown:
        raise ValueError("Unknown road IDs: " + ", ".join(sorted(unknown)))
    # Never silently project a target from a closed road onto a different road.
    a, b = snap(network, start), snap(network, goal)

    def open_endpoint(original, point):
        if original["road_id"] not in blocked:
            return original
        alternative = snap(network, point, blocked)
        if distance(original["point"], alternative["point"]) > 1e-6:
            raise ValueError("An endpoint lies on a closed road")
        return alternative

    a, b = open_endpoint(a, start), open_endpoint(b, goal)
    key = lambda p: (round(p["x"], 9), round(p["y"], 9))
    graph, points = {}, {}
    for road in network["roads"]:
        if road["id"] in blocked:
            continue
        ordered, offset = [], 0.0
        for i, p in enumerate(road["points"]):
            if i:
                offset += distance(road["points"][i - 1], p)
            ordered.append((offset, p))
        ordered.extend(
            (s["offset_m"], s["point"]) for s in (a, b) if s["road_id"] == road["id"]
        )
        ordered.sort(key=lambda v: v[0])
        for (_, p), (_, q) in zip(ordered, ordered[1:]):
            x, y = key(p), key(q)
            points[x], points[y] = p, q
            graph.setdefault(x, [])
            graph.setdefault(y, [])
            if x != y:
                length = distance(p, q)
                graph[x].append((y, length, road["id"]))
                graph[y].append((x, length, road["id"]))
    begin, end = key(a["point"]), key(b["point"])
    queue, costs, previous = [(0.0, begin)], {begin: 0.0}, {}
    while queue:
        cost, current = heapq.heappop(queue)
        if cost > costs.get(current, math.inf):
            continue
        if current == end:
            path, edges = [points.get(end, b["point"])], []
            while current != begin:
                current, edge = previous[current]
                path.append(points[current])
                edges.append(edge)
            path.reverse()
            edges.reverse()
            return {
                "start": a["point"],
                "goal": b["point"],
                "start_road_id": a["road_id"],
                "reference_path": path,
                "segment_roads": edges,
                "road_ids": list(dict.fromkeys(edges)),
                "length_m": costs[end],
                "blocked_road_ids": sorted(set(blocked)),
            }
        for nxt, length, edge in graph.get(current, []):
            candidate = cost + length
            if candidate < costs.get(nxt, math.inf):
                costs[nxt] = candidate
                previous[nxt] = (current, edge)
                heapq.heappush(queue, (candidate, nxt))
    raise ValueError("Destination is disconnected from the measured route")

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

    if a["road_id"] in blocked:
        # Arrival accepts a small neighborhood, not an exact graph vertex. A robot
        # already inside a shared junction can enter an open outgoing road.
        closed = next(road for road in network["roads"] if road["id"] == a["road_id"])
        junctions = [
            node
            for node in network["nodes"]
            if node["id"] in (closed["from"], closed["to"])
            and distance(start, node) <= 0.5
            and any(
                road["id"] not in blocked and node["id"] in (road["from"], road["to"])
                for road in network["roads"]
            )
        ]
        a = (
            snap(
                network, min(junctions, key=lambda node: distance(start, node)), blocked
            )
            if junctions
            else open_endpoint(a, start)
        )
    b = open_endpoint(b, goal)
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


class RouteGuidance:
    """Ordered, short planning legs; road geometry is guidance, never map data.

    Simplify survey noise before subdividing long edges. Progress only visits the
    next leg, never the globally nearest road (which can be a later parallel arm
    of a U). A rolling window has several legs, so RPP drives through the via
    points rather than treating every road vertex as a stopping goal.
    """

    def __init__(self, reference, start, goal):
        points = []
        for point in [start, *reference, goal]:
            point = validate_point(point)
            if not points or distance(points[-1], point) > 1e-6:
                points.append(point)
        # Iterative Ramer-Douglas-Peucker, retaining bends to within 5 cm.
        keep = {0, len(points) - 1}
        pending = [(0, len(points) - 1)]
        while pending:
            a, b = pending.pop()
            if b - a < 2:
                continue
            errors = [
                projection(points[i], [points[a], points[b]])["distance_m"]
                for i in range(a + 1, b)
            ]
            offset = max(range(len(errors)), key=errors.__getitem__)
            if errors[offset] > 0.05:
                middle = a + 1 + offset
                keep.add(middle)
                pending.extend(((a, middle), (middle, b)))
        corners = [points[i] for i in sorted(keep)]
        self.points, self.bends = [corners[0]], set()
        for a, b in zip(corners, corners[1:]):
            steps = max(1, math.ceil(distance(a, b) / 2.0))
            self.points.extend(
                {key: a[key] + (b[key] - a[key]) * i / steps for key in ("x", "y")}
                for i in range(1, steps + 1)
            )
            self.points[-1] = dict(b)
            self.bends.add(len(self.points) - 1)
        self.index = min(1, len(self.points) - 1)

    @property
    def final_leg(self):
        return self.index == len(self.points) - 1

    def window(self, position):
        """Return an ordered ~6 m horizon and its identity for preemption.

        Advance before the controller approaches its temporary endpoint. Crossing
        a station's end-plane also permits an obstacle detour up to 1.5 m away.
        Neither test can skip ahead to a spatially adjacent, later road segment.
        """
        while not self.final_leg:
            a, b = self.points[self.index - 1 : self.index + 1]
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            px, py = position["x"] - b["x"], position["y"] - b["y"]
            crossed = px * dx + py * dy >= 0
            lateral = abs(px * dy - py * dx) / math.hypot(dx, dy)
            if distance(position, b) > 0.6 and not (crossed and lateral <= 1.5):
                break
            self.index += 1
        end, ahead = self.index, distance(position, self.points[self.index])
        # Bound per-replan action calls even for extremely tightly spaced bends.
        while end + 1 < len(self.points) and ahead < 6.0 and end - self.index < 11:
            ahead += distance(self.points[end], self.points[end + 1])
            end += 1
        return tuple(range(self.index, end + 1))

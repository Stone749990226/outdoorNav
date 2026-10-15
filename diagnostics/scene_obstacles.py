"""Scene obstacle footprint distances."""

import json
import math


def _poly(o):
    fp = o.get("footprint") or {}
    c = fp.get("coordinates")
    if not c:
        return None
    while (
        isinstance(c, list)
        and c
        and isinstance(c[0], list)
        and c[0]
        and isinstance(c[0][0], list)
    ):
        c = c[0]
    if not isinstance(c, list) or len(c) < 3:
        return None
    return [
        (float(p[0]), float(p[1])) for p in c if isinstance(p, list) and len(p) >= 2
    ]


def load_obstacles(path, skip_names=("ground",)):
    G = json.load(open(path, encoding="utf-8"))
    obs = G["obstacles"] if isinstance(G, dict) and "obstacles" in G else G
    out = []
    for o in obs:
        n = o.get("name") or o.get("n") or "?"
        if n in skip_names:
            continue
        poly = _poly(o)
        if poly:
            cx = sum((p[0] for p in poly)) / len(poly)
            cy = sum((p[1] for p in poly)) / len(poly)
            rmax = max((math.hypot(p[0] - cx, p[1] - cy) for p in poly))
        elif "bounds" in o:
            x0, y0, x1, y1 = o["bounds"]
            cx, cy = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
            rmax = math.hypot((x1 - x0) / 2.0, (y1 - y0) / 2.0)
            poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        else:
            continue
        out.append((n, cx, cy, rmax, poly))
    return out


def _pt_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            xin = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < xin:
                inside = not inside
    return inside


def _seg_d(px, py, x0, y0, x1, y1):
    dx, dy = (x1 - x0, y1 - y0)
    L = dx * dx + dy * dy
    if L <= 1e-12:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def dist_to_obstacle(x, y, obs):
    best = 1000000000.0
    for _n, cx, cy, rmax, poly in obs:
        if math.hypot(x - cx, y - cy) - rmax > best:
            continue
        if poly is None:
            d = max(0.0, math.hypot(x - cx, y - cy))
        elif _pt_in_poly(x, y, poly):
            d = 0.0
        else:
            d = min(
                (
                    _seg_d(
                        x,
                        y,
                        poly[i][0],
                        poly[i][1],
                        poly[(i + 1) % len(poly)][0],
                        poly[(i + 1) % len(poly)][1],
                    )
                    for i in range(len(poly))
                )
            )
        best = min(best, d)
    return best

"""Describe persistent occupied cells relative to scene geometry."""

from outdoornav.diagnostics import probe_files, seq_of, densify
import argparse
import collections
import json
import math
import os
import random
import re
import sys
from .scene_obstacles import load_obstacles, dist_to_obstacle


def quant(vals, res=None):
    vals = sorted((v for v in vals if v is not None and v < 100000000.0))
    if not vals:
        return None
    q = lambda p: vals[min(len(vals) - 1, int(len(vals) * p))]
    return {
        "n": len(vals),
        "min": round(vals[0], 3),
        "p10": round(q(0.1), 3),
        "median": round(q(0.5), 3),
        "p90": round(q(0.9), 3),
        "max": round(vals[-1], 3),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("probe_dir")
    ap.add_argument("--max-seq", type=int, required=True)
    ap.add_argument("--geo", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--unmatched", default=None)
    ap.add_argument("--radii", default="0.105,0.50")
    a = ap.parse_args()
    R = [float(x) for x in a.radii.split(",")]
    obs = load_obstacles(a.geo)
    trees = [(cx, cy, rmax) for n, cx, cy, rmax, _p in obs if "tree" in n]
    ticks = probe_files(a.probe_dir, a.max_seq)
    meta = json.load(open(ticks[-1], encoding="utf-8"))["map"]
    W, res = (meta["width"], meta["resolution"])
    ox, oy = (meta["origin"]["x"], meta["origin"]["y"])
    H = None
    maps = {
        seq_of(t): os.path.join(a.probe_dir, "map_%04d.bin" % seq_of(t)) for t in ticks
    }
    b0 = open(maps[seq_of(ticks[0])], "rb").read()
    b1 = open(maps[seq_of(ticks[-1])], "rb").read()
    H = len(b1) // W
    L = lambda v: 99 <= v <= 100
    added = {k for k in range(len(b1)) if L(b1[k]) and (not L(b0[k]))}
    pts = [
        tuple(json.load(open(t, encoding="utf-8"))["robot_xy_map"])
        for t in ticks
        if json.load(open(t, encoding="utf-8")).get("robot_xy_map")
    ]
    dense = densify(pts, 0.05)

    def xy(k):
        return (ox + (k % W + 0.5) * res, oy + (k // W + 0.5) * res)

    def dtraj(k):
        x, y = xy(k)
        return min((math.hypot(x - px, y - py) for px, py in dense))

    LETH = re.compile(b"[\\x63\\x64]")
    sets = {
        s: {m.start() for m in LETH.finditer(open(maps[s], "rb").read())} for s in maps
    }
    first_mark = {}
    prev = None
    for t in ticks:
        s = seq_of(t)
        cur = sets[s]
        if prev is not None:
            for k in cur - prev:
                first_mark.setdefault(k, s)
        prev = cur
    pos_at = {}
    for t in ticks:
        r = json.load(open(t, encoding="utf-8")).get("robot_xy_map")
        if r:
            pos_at[seq_of(t)] = tuple(r)
    seq_sorted = sorted(pos_at)
    out = {
        "probe_dir": os.path.abspath(a.probe_dir),
        "geo": os.path.abspath(a.geo),
        "radii_m": R,
        "n_added": len(added),
        "n_obstacles": len(obs),
        "n_trees": len(trees),
        "convention": ' "粘住" = 格心到加密轨迹(0.05 m)任一点 ≤ R ∧ 末拍致命 ∧ 起始拍不致命 ； 净空 = 格心到最近障碍 footprint 的距离 （树用多边形 ）； "车在哪" 用该格**第一次被标上**那一拍的 `robot_xy_map` ',
    }
    unmatched = set()
    if a.unmatched and os.path.isfile(a.unmatched):
        unmatched = set(json.load(open(a.unmatched, encoding="utf-8"))["cells"])
    matched_echo = added - unmatched if unmatched else set()
    rnd = random.Random(20261011)
    free = [k for k in range(len(b1)) if b1[k] == 0]
    fs = rnd.sample(free, min(len(free), 8000))
    fs_d = {k: dtraj(k) for k in fs}
    for r in R:
        swept = set()
        for px, py in dense:
            i0, i1 = (int((px - r - ox) / res), int((px + r - ox) / res))
            j0, j1 = (int((py - r - oy) / res), int((py + r - oy) / res))
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    if 0 <= i < W and 0 <= j < H:
                        k = j * W + i
                        x, y = xy(k)
                        if math.hypot(x - px, y - py) <= r:
                            swept.add(k)
        sticky = swept & added
        not_sticky = added - sticky
        bins = collections.Counter((int(dtraj(k) / 0.25) for k in sticky))
        pool = collections.defaultdict(list)
        for k in fs:
            pool[int(fs_d[k] / 0.25)].append(k)
        ctrl = []
        for b, need in bins.items():
            p = pool.get(b) or []
            if p:
                ctrl += [p[rnd.randrange(len(p))] for _ in range(need)]

        def prof(cells):
            cells = list(cells)
            if not cells:
                return {"n": 0}
            clear = [dist_to_obstacle(*xy(k), obs) for k in cells]
            dt = (
                [
                    min(
                        (
                            math.hypot(*[xy(k)[0] - tx, xy(k)[1] - ty])
                            for tx, ty, _ in trees
                        )
                    )
                    for k in cells
                ]
                if trees
                else []
            )
            dpath = [dtraj(k) for k in cells]
            behind, at_car = ([], [])
            for k in cells:
                s = first_mark.get(k)
                if s is None:
                    continue
                p = pos_at.get(s)
                if p is None:
                    continue
                at_car.append(math.hypot(xy(k)[0] - p[0], xy(k)[1] - p[1]))
                idx = seq_sorted.index(s)
                rest = sum(
                    (
                        math.dist(pos_at[seq_sorted[i]], pos_at[seq_sorted[i + 1]])
                        for i in range(idx, len(seq_sorted) - 1)
                    )
                )
                behind.append(rest)
            cs = set(cells)
            seen = set()
            comps = []
            for k in cs:
                if k in seen:
                    continue
                q = [k]
                seen.add(k)
                n = 0
                while q:
                    x = q.pop()
                    n += 1
                    gx, gy = (x % W, x // W)
                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nx, ny = (gx + dx, gy + dy)
                        if 0 <= nx < W and 0 <= ny < H:
                            k2 = ny * W + nx
                            if k2 in cs and k2 not in seen:
                                seen.add(k2)
                                q.append(k2)
                comps.append(n)
            comps.sort(reverse=True)
            return {
                "n": len(cells),
                "clearance_to_obstacle_m": quant(clear),
                "dist_to_nearest_tree_m": quant(dt),
                "frac_within_0.5m_of_tree": sum((1 for d in dt if d <= 0.5)) / len(dt)
                if dt
                else None,
                "dist_to_traj_m": quant(dpath),
                "dist_to_car_when_marked_m": quant(at_car),
                "path_remaining_when_marked_m": quant(behind),
                "n_components": len(comps),
                "largest_component": comps[0] if comps else 0,
                "frac_in_largest": comps[0] / len(cells) if comps else None,
                "components_top": comps[:6],
            }

        key = "R=%.3f" % r
        out[key] = {
            "n_swept": len(swept),
            "n_sticky": len(sticky),
            "n_not_sticky": len(not_sticky),
            "sticky": prof(sticky),
            "control_not_sticky": prof(not_sticky),
            "control_matched_echo": prof(matched_echo),
            "control_free_same_distance": prof(ctrl),
        }
    json.dump(out, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    for r in R:
        v = out["R=%.3f" % r]
        print(
            "  === R=%.3f m ⇒ 扫过 %d ｜  粘住 %d ｜ 未粘住 %d"
            % (r, v["n_swept"], v["n_sticky"], v["n_not_sticky"])
        )
        for name in (
            "sticky",
            "control_not_sticky",
            "control_matched_echo",
            "control_free_same_distance",
        ):
            p = v[name]
            if not p.get("n"):
                print("     %-28s （空）" % name)
                continue
            print(
                "     %-28s n=%4d ｜ 净空中位 %.3f m ｜ 到树 %.3f ｜ 到轨迹 %.3f ｜ 标上时车距 %.3f ｜ 连通块 %d（最大占 %.0f%%）"
                % (
                    name,
                    p["n"],
                    (p["clearance_to_obstacle_m"] or {}).get("median", -1),
                    (p["dist_to_nearest_tree_m"] or {}).get("median", -1),
                    (p["dist_to_traj_m"] or {}).get("median", -1),
                    (p["dist_to_car_when_marked_m"] or {}).get("median", -1),
                    p["n_components"],
                    100 * (p["frac_in_largest"] or 0),
                )
            )
    print("   已落盘 %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

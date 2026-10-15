"""Compare residual cell distances to unknown-space inflation."""

from outdoornav.diagnostics import load_grid
import argparse
import collections
import csv
import json
import math
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("probe_dir")
    ap.add_argument("residue_csv")
    ap.add_argument("--X", type=float, default=0.1)
    ap.add_argument("--geo", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    info, blob = load_grid(a.probe_dir)
    if info is None or blob is None:
        print(' 读不到图 ⇒  记成"未取到" ')
        return 2
    W, H, res = (info["width"], info["height"], info["resolution"])
    ox, oy = (info["origin"]["x"], info["origin"]["y"])
    print("   用**末拍**那张图 ：%d×%d @%.3f m " % (W, H, res))
    INF = 10**9
    dist = [INF] * (W * H)
    dq = collections.deque()
    for i, b in enumerate(blob):
        if b >= 128:
            dist[i] = 0
            dq.append(i)
    while dq:
        i = dq.popleft()
        x, y = (i % W, i // W)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = (x + dx, y + dy)
            if 0 <= nx < W and 0 <= ny < H and (dist[ny * W + nx] > dist[i] + 1):
                dist[ny * W + nx] = dist[i] + 1
                dq.append(ny * W + nx)
    print('   已算完"到最近未知格的距离场" （多源 BFS ，单位=格 ）')

    def d_unknown(wx, wy):
        gx, gy = (int((wx - ox) / res), int((wy - oy) / res))
        if not (0 <= gx < W and 0 <= gy < H):
            return None
        return dist[gy * W + gx] * res

    G = json.load(open(a.geo, encoding="utf-8"))
    cols = []
    for obstacle in G["obstacles"]:
        points = obstacle["footprint"]["coordinates"][0]
        x0, x1 = min(p[0] for p in points), max(p[0] for p in points)
        y0, y1 = min(p[1] for p in points), max(p[1] for p in points)
        cols.append(
            {
                "x": (x0 + x1) / 2,
                "y": (y0 + y1) / 2,
                "hx": (x1 - x0) / 2,
                "hy": (y1 - y0) / 2,
                "k": "cylinder" if obstacle.get("tree") else "box",
            }
        )
    rows = list(csv.DictReader(open(a.residue_csv, encoding="utf-8")))

    def is_pair(v):
        return (
            isinstance(v, list)
            and len(v) == 2
            and all((isinstance(t, (int, float)) for t in v))
        )

    def rings_of(g):
        out = []

        def walk(c):
            if is_pair(c):
                return
            if isinstance(c, list) and c and is_pair(c[0]):
                out.append(c)
                return
            for x in c:
                walk(x)

        walk(g)
        return out

    edges = []
    for geom in (G["free"]["coordinates"], G["plaza"]["coordinates"]):
        for ring in rings_of(geom):
            for i in range(len(ring) - 1):
                edges.append((ring[i][0], ring[i][1], ring[i + 1][0], ring[i + 1][1]))

    def d_seg(px, py, e):
        x0, y0, x1, y1 = e
        dx, dy = (x1 - x0, y1 - y0)
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
        return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))

    def d_mesh(px, py):
        return min((d_seg(px, py, e) for e in edges), default=1000000000.0)

    def d_col(px, py):
        best = 1000000000.0
        for c in cols:
            d = (
                math.hypot(
                    max(0, abs(px - c["x"]) - c["hx"]),
                    max(0, abs(py - c["y"]) - c["hy"]),
                )
                if c["k"] == "box"
                else max(0.0, math.hypot(px - c["x"], py - c["y"]) - c["hx"])
            )
            if d < best:
                best = d
        return best

    unexp = []
    for r in rows:
        px, py = (float(r["wx"]), float(r["wy"]))
        if d_mesh(px, py) > a.X and d_col(px, py) > a.X:
            unexp.append((px, py))
    du = sorted((v for v in (d_unknown(px, py) for px, py in unexp) if v is not None))
    if not du:
        from pathlib import Path

        destination = Path(a.out)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps({"applicable": False, "reason": "No unexplained residual cells"})
        )
        return 0
    print()
    print("   a. 那 **%d** 格到**最近未知格**的距离 ：" % len(du))
    print(
        "     min %.2f ｜ p25 %.2f ｜  **中位 %.2f** ｜ p75 %.2f ｜ p90 %.2f ｜ max %.2f m"
        % (
            du[0],
            du[int(len(du) * 0.25)],
            du[len(du) // 2],
            du[int(len(du) * 0.75)],
            du[int(len(du) * 0.9)],
            du[-1],
        )
    )
    for X in (0.0, 0.1, 0.25, 0.5, 1.0):
        k = sum((1 for v in du if v <= X + 1e-09))
        print(
            "        ≤ %.2f m（ 0.25 = `inflation_radius` ）：%5d（%5.1f%%）"
            % (X, k, 100.0 * k / max(1, len(du)))
        )
    occ = {int(r["j"]) * W + int(r["i"]) for r in rows if r.get("i") and r.get("j")}
    ctrl = []
    for k in range(W * H):
        if blob[k] != 0:
            continue
        if k in occ:
            continue
        gx, gy = (k % W, k // W)
        ctrl.append((ox + (gx + 0.5) * res, oy + (gy + 0.5) * res))
        if len(ctrl) >= 20000:
            break
    dc = sorted((v for v in (d_unknown(px, py) for px, py in ctrl) if v is not None))
    if not dc:
        raise ValueError("No free control cells available")
    print()
    print("   b. **对照**： 随机自由格 **%d** 个  到最近未知格的距离 ：" % len(dc))
    print(
        "     min %.2f ｜ p25 %.2f ｜  **中位 %.2f** ｜ p75 %.2f ｜ p90 %.2f ｜ max %.2f m"
        % (
            dc[0],
            dc[int(len(dc) * 0.25)],
            dc[len(dc) // 2],
            dc[int(len(dc) * 0.75)],
            dc[int(len(dc) * 0.9)],
            dc[-1],
        )
    )
    for X in (0.25, 0.5, 1.0):
        k = sum((1 for v in dc if v <= X + 1e-09))
        print("        ≤ %.2f m：%5d（%5.1f%%）" % (X, k, 100.0 * k / max(1, len(dc))))
    ka = sum((1 for v in du if v <= 0.25 + 1e-09)) / float(max(1, len(du)))
    kb = sum((1 for v in dc if v <= 0.25 + 1e-09)) / float(max(1, len(dc)))
    print()
    print(
        "   判读（ ≤0.25 m 那档 ）： 残留格 **%.1f%%** ｜  对照自由格 **%.1f%%**"
        % (100.0 * ka, 100.0 * kb)
    )
    if ka >= 0.7 and ka > kb * 2:
        print('     ⇒  **支持"就是它"** （ 残留格紧贴未知区、而对照不紧贴 ）')
    elif ka > kb * 2:
        print("     ⇒  **部分支持** （ 残留明显更贴未知区，但比例不足 70%% ）")
    else:
        print('     ⇒  **不支持** （ 两批贴得差不多 ⇒  不是"紧邻未知"这个特征 ）')
    out = {
        "a_residue_d_unknown_m": {
            "n": len(du),
            "min": du[0],
            "p25": du[int(len(du) * 0.25)],
            "median": du[len(du) // 2],
            "p75": du[int(len(du) * 0.75)],
            "p90": du[int(len(du) * 0.9)],
            "max": du[-1],
            "frac_le_0.25": ka,
        },
        "b_control_d_unknown_m": {
            "n": len(dc),
            "median": dc[len(dc) // 2],
            "frac_le_0.25": kb,
        },
        "inflation_radius_m": 0.25,
        "supported": bool(ka > kb * 2),
    }
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    json.dump(out, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    print("   已落盘 %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

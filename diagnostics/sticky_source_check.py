"""Attribute first obstacle marks to lidar/depth observations."""

from outdoornav.diagnostics import probe_files, seq_of, densify
import argparse
import json
import math
import os
import re
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("probe_dir")
    ap.add_argument("--max-seq", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--radii", default="0.105,0.50")
    ap.add_argument("--near", type=float, default=0.15)
    a = ap.parse_args()
    R = [float(x) for x in a.radii.split(",")]
    P = a.probe_dir
    ticks = probe_files(P, a.max_seq)
    meta = json.load(open(ticks[-1], encoding="utf-8"))["map"]
    W, res = (meta["width"], meta["resolution"])
    ox, oy = (meta["origin"]["x"], meta["origin"]["y"])
    maps = {seq_of(t): os.path.join(P, "map_%04d.bin" % seq_of(t)) for t in ticks}
    b0 = open(maps[seq_of(ticks[0])], "rb").read()
    b1 = open(maps[seq_of(ticks[-1])], "rb").read()
    L = lambda v: 99 <= v <= 100
    H = len(b1) // W
    added = {k for k in range(len(b1)) if L(b1[k]) and (not L(b0[k]))}
    pts = [
        tuple(json.load(open(t, encoding="utf-8"))["robot_xy_map"])
        for t in ticks
        if json.load(open(t, encoding="utf-8")).get("robot_xy_map")
    ]
    dense = densify(pts, 0.05)
    xy = lambda k: (ox + (k % W + 0.5) * res, oy + (k // W + 0.5) * res)
    LETH = re.compile(b"[\\x63\\x64]")
    sets = {
        s: {m.start() for m in LETH.finditer(open(maps[s], "rb").read())} for s in maps
    }
    first_mark, prev = ({}, None)
    for t in ticks:
        s = seq_of(t)
        cur = sets[s]
        if prev is not None:
            for k in cur - prev:
                first_mark.setdefault(k, s)
        prev = cur
    idx = {}
    for cloud in ("scan", "cam"):
        idx[cloud] = {}
        for t in ticks:
            s = seq_of(t)
            f = os.path.join(P, "%s_%04d.xyz" % (cloud, s))
            if not os.path.isfile(f):
                continue
            d = json.load(open(t, encoding="utf-8"))
            M = ((d.get(cloud) or {}).get("tf") or {}).get("M")
            org = (M[0][3], M[1][3], M[2][3]) if M else None
            m = {}
            for line in open(f, encoding="utf-8"):
                v = line.split()
                if len(v) != 3:
                    continue
                x, y, z = map(float, v)
                gi, gj = (int((x - ox) / res), int((y - oy) / res))
                for di, dj in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
                    k = (gj + dj) * W + (gi + di)
                    if 0 <= k < len(b1):
                        dd = math.hypot(*[xy(k)[0] - x, xy(k)[1] - y])
                        if k not in m or dd < m[k][0]:
                            m[k] = (dd, math.dist((x, y, z), org) if org else None, z)
            idx[cloud][s] = m
    out = {
        "probe_dir": os.path.abspath(P),
        "max_seq": a.max_seq,
        "near_m": a.near,
        "n_added": len(added),
        "radii_m": R,
        "convention": ' 取"第一次被标上"那一拍 （ 不取末尾 ）； "有点" = 该拍到格心 ≤ %.2f m ； 另给"截至那一拍为止有没有过点" '
        % a.near,
    }
    for r in R:
        s = set()
        for px, py in dense:
            i0, i1 = (int((px - r - ox) / res), int((px + r - ox) / res))
            j0, j1 = (int((py - r - oy) / res), int((py + r - oy) / res))
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    if 0 <= i < W and 0 <= j < H:
                        k = j * W + i
                        if math.hypot(xy(k)[0] - px, xy(k)[1] - py) <= r:
                            s.add(k)
        sticky = sorted(s & added)
        res_stat = {"n_sticky": len(sticky)}
        seqs = [seq_of(t) for t in ticks]
        for cloud in ("scan", "cam"):
            hit = hit_ever = 0
            ranges, heights = ([], [])
            for k in sticky:
                fs = first_mark.get(k)
                if fs is None:
                    continue
                m = idx[cloud].get(fs) or {}
                if k in m and m[k][0] <= a.near:
                    hit += 1
                    if m[k][1] is not None:
                        ranges.append(m[k][1])
                    heights.append(m[k][2])
                else:
                    ever = False
                    for s2 in seqs:
                        if s2 > fs:
                            break
                        mm = idx[cloud].get(s2) or {}
                        if k in mm and mm[k][0] <= a.near:
                            ever = True
                            break
                    if ever:
                        hit_ever += 1

            def q(v, p):
                v = sorted(v)
                return round(v[min(len(v) - 1, int(len(v) * p))], 3) if v else None

            res_stat[cloud] = {
                "n_checked": len(sticky),
                "n_hit_at_mark_tick": hit,
                "frac_hit_at_mark_tick": hit / max(1, len(sticky)),
                "n_hit_on_some_earlier_tick_only": hit_ever,
                "range_m": {
                    "median": q(ranges, 0.5),
                    "p10": q(ranges, 0.1),
                    "p90": q(ranges, 0.9),
                },
                "z_m": {
                    "median": q(heights, 0.5),
                    "p10": q(heights, 0.1),
                    "p90": q(heights, 0.9),
                },
            }
        out["R=%.3f" % r] = res_stat
    json.dump(out, open(a.out, "w", encoding="utf-8"), ensure_ascii=False, indent=2)
    for r in R:
        v = out["R=%.3f" % r]
        print("  === R=%.3f m ⇒ 粘住 %d 格" % (r, v["n_sticky"]))
        for cloud in ("scan", "cam"):
            c = v[cloud]
            print(
                "     %-5s 标上那一拍就有点 %d/%d（%.1f%%）｜ 只有更早的拍有点（那一拍没有）%d ｜ 测距中位 %s ｜ z 中位 %s"
                % (
                    cloud,
                    c["n_hit_at_mark_tick"],
                    c["n_checked"],
                    100 * c["frac_hit_at_mark_tick"],
                    c["n_hit_on_some_earlier_tick_only"],
                    c["range_m"]["median"],
                    c["z_m"]["median"],
                )
            )
    print("   已落盘 %s" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

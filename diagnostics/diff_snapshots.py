"""Compare two reproducible scene or probe snapshots."""

import argparse
import json
import sys
import numpy as np


def load(path):
    z = np.load(path, allow_pickle=False)
    return {
        "data": z["data"],
        "resolution": float(z["resolution"]),
        "origin_x": float(z["origin_x"]),
        "origin_y": float(z["origin_y"]),
        "frame": str(z["frame"]),
    }


def classify(v):
    if v < 0:
        return "unknown"
    if v >= 100:
        return "lethal"
    if v >= 50:
        return "inflated(>=50)"
    if v > 0:
        return "inflated(1..49)"
    return "free"


def cell_of(snap, x, y):
    col = int((x - snap["origin_x"]) / snap["resolution"])
    row = int((y - snap["origin_y"]) / snap["resolution"])
    if (
        row < 0
        or col < 0
        or row >= snap["data"].shape[0]
        or (col >= snap["data"].shape[1])
    ):
        return None
    return (row, col)


def patch(snap, x, y, radius):
    rc = cell_of(snap, x, y)
    if rc is None:
        return None
    row, col = rc
    r0, r1 = (max(0, row - radius), min(snap["data"].shape[0], row + radius + 1))
    c0, c1 = (max(0, col - radius), min(snap["data"].shape[1], col + radius + 1))
    return snap["data"][r0:r1, c0:c1]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("a")
    parser.add_argument("b")
    parser.add_argument(
        "--at", nargs=2, type=float, action="append", default=[], metavar=("X", "Y")
    )
    parser.add_argument("--radius-cells", type=int, default=6)
    parser.add_argument("--json", default="")
    args = parser.parse_args(argv)
    a, b = (load(args.a), load(args.b))
    out = {
        "a": args.a,
        "b": args.b,
        "frame": [a["frame"], b["frame"]],
        "shape": [list(a["data"].shape), list(b["data"].shape)],
        "resolution": [a["resolution"], b["resolution"]],
        "origin": [[a["origin_x"], a["origin_y"]], [b["origin_x"], b["origin_y"]]],
    }
    if a["data"].shape != b["data"].shape:
        out["error"] = "两张快照尺寸不同，不能逐格比"
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 1
    da, db = (a["data"], b["data"])
    la, lb = (da >= 100, db >= 100)
    out["global"] = {
        "lethal_a": int(la.sum()),
        "lethal_b": int(lb.sum()),
        "lethal_new_in_b": int((lb & ~la).sum()),
        "lethal_cleared_in_b": int((la & ~lb).sum()),
        "lethal_same": int((la & lb).sum()),
        "free_a": int((da == 0).sum()),
        "free_b": int((db == 0).sum()),
        "unknown_a": int((da < 0).sum()),
        "unknown_b": int((db < 0).sum()),
        "note": "lethal = 代价 >= 100（障碍表面命中的格子）。lethal_cleared_in_b = 从 A 到 B 被清掉的致命格数；障碍移走时它应当 > 0，且要落到障碍所在的那几格上",
    }
    if args.at:
        rows = []
        for x, y in args.at:
            rc = cell_of(a, x, y)
            rec = {"at": [x, y], "cell": list(rc) if rc else None}
            if rc is None:
                rec["error"] = "落在图外"
            else:
                va, vb = (int(da[rc[0], rc[1]]), int(db[rc[0], rc[1]]))
                rec.update(
                    {
                        "a_value": va,
                        "a_class": classify(va),
                        "b_value": vb,
                        "b_class": classify(vb),
                    }
                )
                pa, pb = (
                    patch(a, x, y, args.radius_cells),
                    patch(b, x, y, args.radius_cells),
                )
                rec["patch_lethal"] = [int((pa >= 100).sum()), int((pb >= 100).sum())]
                rec["patch_free"] = [int((pa == 0).sum()), int((pb == 0).sum())]
                rec["patch_radius_cells"] = args.radius_cells
                rec["cleared"] = bool(va >= 100 and vb < 100)
            rows.append(rec)
        out["points"] = rows
    print(json.dumps(out, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(out, handle, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())

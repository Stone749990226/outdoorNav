#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 `cell_series.py` 的 CSV 汇成**"两张图 × 标记/清空"**的判据表（放箱前/有箱/移箱后）。

为什么要有它：判据④要求"**该源是否被消费**"必须写成"**每张图各一格**"，
**不许写成"局部图正常"** ⇒ 需要一张把 `g_raw`/`g_inf`/`l_raw`/`l_inf` 四行并排列出的表，
而不是散在四个 CSV 里让人自己对齐。

用法（容器内）：
  cell_summary.py --run-dir /work/artifacts/rtk-nav2/<run-id> [--prefix r2_] [--out <txt>]
"""
import argparse
import csv
import os
import sys

TAGS = ["g_raw", "g_inf", "l_raw", "l_inf"]


def load(path):
    if not os.path.exists(path):
        return None
    return list(csv.DictReader(open(path)))


def vals(rows, tag, col):
    out = []
    for r in rows:
        if r["tag"] != tag:
            continue
        v = r.get(col, "")
        out.append(None if v in ("", None) else int(v))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--prefix", default="r2_")
    ap.add_argument("--rounds", default="A,B",
                    help="要汇总的轮次（逗号分隔），另有 rear 车尾箱")
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    lines = []
    lines.append("=== 每张图 × 每格：有箱 / 移箱后 的取值集合（**两张图分列**）===")
    lines.append("判据：箱格应「有箱时被标记（非 0）→ 移箱后回到 free(0)」；"
                 "对照格应「两态都不变且非 0」（= 图整体还在工作）")
    for L in a.rounds.split(","):
        L = L.strip()
        if not L:
            continue
        w = load(os.path.join(a.run_dir, "%s%s_with.csv" % (a.prefix, L)))
        o = load(os.path.join(a.run_dir, "%s%s_without.csv" % (a.prefix, L)))
        if w is None or o is None:
            lines.append("\n######### %s #########  ** 缺 CSV（with=%s without=%s）**"
                         % (L, w is not None, o is not None))
            continue
        lines.append("\n######### 轮次 %s #########" % L)
        for col, what in (("box", "箱格"), ("ctrl", "对照格")):
            lines.append("  -- %s --" % what)
            for tag in TAGS:
                vw, vo = vals(w, tag, col), vals(o, tag, col)
                sw = sorted(set(v for v in vw if v is not None))
                so = sorted(set(v for v in vo if v is not None))
                nw = sum(1 for v in vw if v is None)
                no = sum(1 for v in vo if v is None)
                lines.append("     %-6s 有箱: %3d 帧 取值=%-22s%s | 移箱后: %3d 帧 取值=%-22s%s"
                             % (tag, len(vw), sw, ("（%d 帧取不到值）" % nw) if nw else "",
                                len(vo), so, ("（%d 帧取不到值）" % no) if no else ""))
    # 车尾箱（源消费判据）：有 before/with/after 三段
    for what, suf in (("放箱前", "before"), ("有箱", "with"), ("移箱后", "after")):
        d = load(os.path.join(a.run_dir, "%srear_%s.csv" % (a.prefix, suf)))
        if d is None:
            continue
        lines.append("\n######### 车尾箱（相机视场之外）· %s #########" % what)
        for tag in TAGS:
            v = vals(d, tag, "rear")
            s = sorted(set(x for x in v if x is not None))
            n = sum(1 for x in v if x is None)
            lines.append("     %-6s %3d 帧 取值=%-22s%s"
                         % (tag, len(v), s, ("（%d 帧取不到值）" % n) if n else ""))
    txt = "\n".join(lines) + "\n"
    if a.out:
        open(a.out, "w").write(txt)
    sys.stdout.write(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())

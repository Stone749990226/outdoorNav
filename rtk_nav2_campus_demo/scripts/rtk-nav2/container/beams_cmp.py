#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三态**逐束**对比：放箱前 / 放箱后 / 移箱后，**同一批 beam 索引**的回波列表。

判据（协调者口径，也是 C1 的更正值）：
  · **哪几束 `inf` ↔ 有限值双向可逆** ⇒ 那几束才是箱子（放箱 `inf`→有限、移箱 有限→`inf`）；
  · **在"放箱前"和"移箱后"两次都在某个有限值上** ⇒ 那是**别的真实障碍**（与箱子无关）；
  · **一律不取最小/极值**（窗口里可能同时有别的障碍，"取最小"会把结论带偏 ✗）。

用法：
  beams_cmp.py --before a.json --with b.json --after c.json [--half 30]
"""

import argparse
import json
import math
import sys


def load(p):
    with open(p) as f:
        return json.load(f)


def per_index(doc):
    """每束在**该态全部帧**上的取值集合。返回 (list_of_sets, values_of_first_frame)。"""
    frames = doc.get("frames") or []
    if not frames:
        return None, None
    n = frames[0]["n"]
    sets = [set() for _ in range(n)]
    for fr in frames:
        for i, v in enumerate(fr["ranges"][:n]):
            sets[i].add(v)
    return sets, frames[0]["ranges"]


def fin(setv):
    return sorted(v for v in setv if v != "inf")


def state(setv):
    f = fin(setv)
    if not f:
        return "inf"
    if len(set(f)) == 1:
        return f[0]
    return "有限(%d值)" % len(set(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True)
    ap.add_argument("--with", dest="w", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--half", type=float, default=30.0, help="以箱方位为中心的半窗（度）")
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    B, W, A = load(a.before), load(a.w), load(a.after)
    if B.get("error") or W.get("error") or A.get("error"):
        print("** 有一段没采到 scan：before=%s with=%s after=%s **"
              % (B.get("error"), W.get("error"), A.get("error")))
        return 1

    sb, _ = per_index(B)
    sw, _ = per_index(W)
    sa, _ = per_index(A)
    n = min(len(sb), len(sw), len(sa))
    inc = B["angle_increment_deg"]
    ang0 = B["angle_min_deg"]

    exp = W.get("expect") or B.get("expect") or {}
    idx0 = exp.get("idx")
    print("=== 三态逐束对比（同一批 beam 索引；**不取最小/极值**）===")
    print("scan 帧=%s  波束=%d  angle_min=%.2f°  inc=%.4f°  每态帧数 before=%d with=%d after=%d"
          % (B["frame_id"], B["n_beams"], ang0, inc,
             len(B["frames"]), len(W["frames"]), len(A["frames"])))
    if idx0 is not None:
        print("箱期望方位 %+.2f°、距离 %.3f m ⇒ 索引 %d（水平张角约 %.1f° = 约 %.0f 束）"
              % (exp["bearing_deg"], exp["dist_m"], idx0,
                 math.degrees(2 * math.atan(0.2 / exp["dist_m"])),
                 math.degrees(2 * math.atan(0.2 / exp["dist_m"])) / inc))
    print("三态的 header 帧 id 都应为 %s：%s/%s/%s"
          % (B["frame_id"], B["frame_id"], W["frame_id"], A["frame_id"]))

    lo, hi = 0, n - 1
    if idx0 is not None:
        k = int(round(a.half / inc))
        lo, hi = max(0, idx0 - k), min(n - 1, idx0 + k)

    reversible, real_obs, other = [], [], []
    rows = []
    for i in range(lo, hi + 1):
        bs, ws, as_ = state(sb[i]), state(sw[i]), state(sa[i])
        kind = ""
        if bs == "inf" and ws != "inf" and as_ == "inf":
            kind = "**箱子**（inf→有限→inf，双向可逆）"
            reversible.append(i)
        elif bs != "inf" and as_ != "inf":
            kind = "别的真实障碍（放箱前/移箱后都是有限值）"
            real_obs.append(i)
        elif bs == "inf" and ws == "inf" and as_ == "inf":
            kind = "三态都 inf"
        else:
            kind = "**其它形态（要单独看）**"
            other.append(i)
        rows.append((i, ang0 + i * inc, bs, ws, as_, kind))

    print("\n%-4s %-9s %-14s %-14s %-14s %s"
          % ("idx", "方位(°)", "放箱前", "放箱后", "移箱后", "判定"))
    for i, br, bs, ws, as_, kind in rows:
        print("%-4d %+9.2f %-14s %-14s %-14s %s" % (i, br, bs, ws, as_, kind))

    print("\n---- 汇总（**列的是束，不是极值**）----")
    print("① 双向可逆 ⇒ 箱子：%d 束  %s" % (len(reversible), reversible))
    print("   对应方位：%s" % ", ".join("%+.2f°" % (ang0 + i * inc) for i in reversible))
    print("② 放箱前/移箱后都是有限值 ⇒ 别的真实障碍：%d 束  %s" % (len(real_obs), real_obs))
    for i in real_obs:
        print("   idx %d（%+.2f°）：放箱前=%s  放箱后=%s  移箱后=%s"
              % (i, ang0 + i * inc, state(sb[i]), state(sw[i]), state(sa[i])))
    print("③ 其它形态（既不可逆、也不是「两次都有」）的束：%s" % (other or "（无）"))
    for i in other:
        print("   idx %d（%+.2f°）：放箱前=%s  放箱后=%s  移箱后=%s"
              % (i, ang0 + i * inc, state(sb[i]), state(sw[i]), state(sa[i])))

    # 全数组（不只窗口）统计：箱子那几束是否**唯一**发生了变化
    all_fin_w = [i for i in range(n) if state(sw[i]) != "inf"]
    all_fin_b = [i for i in range(n) if state(sb[i]) != "inf"]
    all_fin_a = [i for i in range(n) if state(sa[i]) != "inf"]
    print("\n---- 全数组 %d 束（窗口之外也看，避免「只看窗口」漏判）----" % n)
    print("放箱前有限值束：%d 束 %s" % (len(all_fin_b), all_fin_b[:40]))
    print("放箱后有限值束：%d 束 %s" % (len(all_fin_w), all_fin_w[:40]))
    print("移箱后有限值束：%d 束 %s" % (len(all_fin_a), all_fin_a[:40]))
    only_with = sorted(set(all_fin_w) - set(all_fin_b) - set(all_fin_a))
    print("**只在「放箱后」有有限值（= 箱子独有的束）：%d 束 %s**" % (len(only_with), only_with))
    print("放箱前就没有、移箱后仍在（= 箱子没被清掉 / 新长出来的）：%d 束 %s"
          % (len(sorted(set(all_fin_a) - set(all_fin_b))), sorted(set(all_fin_a) - set(all_fin_b))))

    if a.out:
        json.dump({"reversible_box_beams": reversible, "real_obstacle_beams": real_obs,
                   "other_beams": other, "only_in_with": only_with,
                   "finite_before": all_fin_b, "finite_with": all_fin_w, "finite_after": all_fin_a,
                   "window": [lo, hi], "rows": [
                       {"idx": i, "bearing_deg": round(br, 3), "before": bs, "with": ws,
                        "after": as_, "verdict": kind} for i, br, bs, ws, as_, kind in rows]},
                  open(a.out, "w"), ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())

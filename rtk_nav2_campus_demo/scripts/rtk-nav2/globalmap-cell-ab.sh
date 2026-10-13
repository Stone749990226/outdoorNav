#!/usr/bin/env bash
# 全局图"吃不吃新障碍"的**格级** A/B 复核（2026-09-22）。
# 为什么要做第二遍：第一遍的"移箱"那步**输出为空**，箱可能没被删掉
#   ⇒ 后两态其实是同一态，A/B 不成立 ✗（本项目对"动作到底成没成"有专门的账，见 §9 家族）。
# 判据（§8.8 自己写的）：**整图哈希冻不冻是弱判据，要看"格"** ——
#   `cell_series.py --track box=X,Y` 给出**箱格 + 对照格**在两张图上的值序列。
set +u
P="$HOME/projects/robonix"; C=robonix-rtk-nav2-demo; D=/usr/bin/docker
R="$P/artifacts/rtk-nav2/odomframe-1"; RC=/work/artifacts/rtk-nav2/odomframe-1
F="$R/freeze2"; mkdir -p "$F"
NAS='source /opt/ros/humble/setup.bash && source /work/rtk_nav2_ws/install/setup.bash'
CS="python3 /work/scripts/rtk-nav2/container/cell_series.py"
BOX="$(cat "$F/box_xy.txt" 2>/dev/null | grep -oE '\-?[0-9.]+,\-?[0-9.]+' | head -1)"
echo "箱位（沿用上一遍选的点）：${BOX:-**没读到**}"
[ -z "$BOX" ] && exit 1

# ★★ 2026-10-10（reviewer ①′）：`models()` 原来**直连** `$D exec`（无分区）⇒ **分区盲** ✗
#   ⇒ 下面 0)/1)/3)/5) 四处报的"运行时箱个数"全是**默认分区**的数 ✗
#   ⇒ 最坏情形：本轮真放进了箱，这里报 0 ⇒ "A/B 不成立"被误读成"脚本坏了" ✗。
#   ★ 修法：本文件**不 source lib.sh**（自己定义 P/C/D ✓）⇒ 直接读盘上那枚 run-id 锚 ✓
#     （与 `spawn_obstacle.sh` 读的是**同一个文件** ✓ ⇒ 两处口径不会分叉 ✓）。
_GZP=""
_ROUND="$(cat "$P/artifacts/rtk-nav2/.current-run" 2>/dev/null | tr -d '\r\n')"
if [ -n "$_ROUND" ]; then _GZP="rtknav2_${_ROUND}"; fi
models() {
    if [ -n "$_GZP" ]; then
        $D exec -e GZ_PARTITION="$_GZP" -e IGN_PARTITION="$_GZP" $C bash -lc "ign model --list 2>/dev/null | grep -c 'test_obstacle' || true"
    else
        $D exec $C bash -lc "ign model --list 2>/dev/null | grep -c 'test_obstacle' || true"
    fi
}
echo "###### 0) 当前世界里"运行时箱"的个数：$(models)（0 = 没有）"

series() {  # series <名字>
    local n="$1"
    $D exec -d $C bash -lc "$NAS && cd /work/rtk_nav2_ws && timeout 120 $CS --track box=$BOX --seconds 12 --label $n --out $RC/freeze2/cs_$n.csv --json $RC/freeze2/cs_$n.json > $RC/logs/cs_$n.log 2>&1"
    for _ in $(seq 1 60); do [ -f "$F/cs_$n.json" ] && break; sleep 2; done
    if [ ! -f "$F/cs_$n.json" ]; then
        # ★★ 失败要**当场停**，不能带着"缺一态"的 A/B 继续跑完（2026-09-22 实测踩到：
        #    参数名照抄了姊妹工具（`--out-csv` vs 本工具的 `--out`）⇒ 工具 argparse 当场死，
        #    而脚本继续放箱/移箱跑完整轮，产出一个**不成立的 A/B** ✗）
        echo "  [$n] **没落盘 ⇒ 本 A/B 不成立，当场停**（下面 3 行是容器内日志，直接给死因）"
        tail -3 "$R/logs/cs_$n.log"
        exit 1
    fi
    echo "  [$n] 落盘 ✓"
}

echo "###### 1) 先确保"没有箱"：清掉所有运行时箱"
$D exec $C bash -lc "$NAS && bash /work/scripts/rtk-nav2/container/spawn_obstacle.sh remove" 2>&1 | tail -2
sleep 6; echo "  清完个数：$(models)"
echo "###### 2) 无箱 12 s"
series without

echo "###### 3) 放箱 $(printf '%s' "$BOX")"
$D exec $C bash -lc "$NAS && bash /work/scripts/rtk-nav2/container/spawn_obstacle.sh spawn ${BOX%,*} ${BOX#*,} 0.4 1.0" 2>&1 | tail -2
sleep 14; echo "  放完个数：$(models)"
echo "###### 4) 有箱 12 s"
series with

echo "###### 5) 移箱"
$D exec $C bash -lc "$NAS && bash /work/scripts/rtk-nav2/container/spawn_obstacle.sh remove" 2>&1 | tail -2
sleep 12; echo "  移完个数：$(models)"
echo "###### 6) 移箱后 12 s"
series removed

echo "###### 7) 逐格对比（**四张图分列**；判据：箱格"无箱→有箱→移后"应能来回）"
python3 - "$F" "$BOX" <<'PY'
import csv, json, os, sys
F, BOX = sys.argv[1], sys.argv[2]
def load(n):
    p = os.path.join(F, "cs_%s.json" % n)
    return json.load(open(p)) if os.path.isfile(p) else None
A, B, Cc = load("without"), load("with"), load("removed")
for tag in ("g_raw", "g_inf", "l_raw", "l_inf"):
    row = []
    for nm, j in (("无箱", A), ("有箱", B), ("移后", Cc)):
        if j is None: row.append("%s=无产物" % nm); continue
        t = (j.get("tags") or {}).get(tag) or {}
        s = t.get("box") or t.get("series") or t
        # cell_series 的 JSON 结构按工具实际输出取（下面兜底打印键名，便于人工核）
        vals = None
        if isinstance(s, dict):
            for k in ("values", "series", "vals", "box"):
                if k in s and isinstance(s[k], list): vals = s[k]; break
        row.append("%s=%s" % (nm, ("%d 个值，%s→%s" % (len(vals), vals[0], vals[-1])) if vals else "结构未识别:" + ",".join(list(s)[:6])))
    print("  %-6s %s" % (tag, " | ".join(row)))
print("  （键名没对上时**不猜**：直接看 cs_*.json 的原文）")
PY
echo "###### 8) 顺带确认输入还活着（两个传感器话题的发布者计数）"
$D exec $C bash -lc "$NAS && for t in /rtk/scan_points /rtk/camera/points /rtk/scan; do echo -n \"\$t: \"; timeout 20 ros2 topic info \$t 2>/dev/null | tr '\n' ' '; echo; done"

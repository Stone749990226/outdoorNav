#!/usr/bin/env bash
# R2 长窗口复验驱动（run-id nav10-r3）—— 一轮回答三件事：
#   ① **本轮自测拐点**（"同一内容持续不变"的起点）——用整图哈希序列，不套别人的拐点；
#   ② **至少一档读数在"拐点之后 ≥1 分钟"**（上一轮判据③没满足的就是这项）；
#   ③ **代价格**：`clearing:false` 之后"无回波 ⇒ 判空闲"还剩多少 ——
#      采两个"开阔、无回波"的格：
#        c2m = 相机 raytrace 量程内（≤6 m）的格 ⇒ R2 下**应仍 free**（相机在清）
#        c7m = 相机量程外、激光量程内的格（~7 m）⇒ R2 下**很可能变 unknown**（激光不再清）
#      这一条决定 R2 能不能当"修"对外讲。
# 另：逐束三态 A/B（放/移箱）、对照格、锚点、勾选表。
#
# ★ 硬步骤（上一轮 B 的教训）：**两轮测定都跑完才允许停栈** —— 本脚本最后写
#   `MEASUREMENTS_DONE` 标记；没有这个标记就不许 stop（写进脚本，不靠人记）。
set +u

R=/work/artifacts/rtk-nav2/nav10-r3
source /opt/ros/humble/setup.bash
source /work/rtk_nav2_ws/install/setup.bash
SP=/work/scripts/rtk-nav2/container/spawn_obstacle.sh
CS=/work/scripts/rtk-nav2/container/cell_series.py
FP=/work/scripts/rtk-nav2/container/follow_probe.py
BOX="4.57,-3.32"          # 相机视场 -20°、2 m、无遮挡（前几轮验证过）
T0="$(cat ${R}/.host_start_wall 2>/dev/null || echo 0)"
rel() { echo "起栈后 +$(( $(date +%s) - T0 )) s（墙钟）"; }

rm -f "${R}/MEASUREMENTS_DONE"

echo "###### ① 拐点序列 + 代价格（同一支采样同时拿） $(rel)"
# 240 s ≈ 覆盖"起栈 → 拐点 → 拐点后 1 分钟以上"
timeout 320 python3 "${CS}" --track "c2m=0.68,-2.61;c7m=-4.31,-2.64" \
    --seconds 240 --label "onset+代价格" \
    --out "${R}/onset_cost.csv" --json "${R}/onset_cost.json" > "${R}/onset_log.txt" 2>&1
echo "--- 代价格（末段；c2m 应在相机量程内、c7m 在激光量程内）$(rel) ---"
grep -E "小结|^  (g_|l_)" "${R}/onset_log.txt" | tail -14

for L in A B; do
    echo
    echo "###### ③-${L} 逐束三态 A/B $(rel)"
    echo "-- 放箱前逐束 --"
    timeout 90 python3 "${FP}" --scan-check "${BOX}" 2>&1 | tail -2
    timeout 60 bash "${SP}" spawn "${BOX%%,*}" "${BOX##*,}" 0.4 1.0 2>&1 | tail -1
    echo "-- 放箱后逐束（哪几束 inf→有限 且 可逆 ⇒ 那几束是箱子）--"
    timeout 90 python3 "${FP}" --scan-check "${BOX}" 2>&1 | tail -2
    timeout 120 python3 "${CS}" --track "box=${BOX}" --auto-control --seconds 12 \
        --label "R3-${L}-有箱" --out "${R}/r3_${L}_with.csv" --json "${R}/r3_${L}_with.json" \
        2>&1 | grep -E "对照格|小结|^  "
    timeout 30 bash "${SP}" remove 2>&1 | tail -1
    echo "-- 移箱后逐束（应逐束变回 inf）--"
    timeout 90 python3 "${FP}" --scan-check "${BOX}" 2>&1 | tail -2
    timeout 120 python3 "${CS}" --track "box=${BOX}" --seconds 10 \
        --label "R3-${L}-移箱后" --out "${R}/r3_${L}_without.csv" --json "${R}/r3_${L}_without.json" \
        2>&1 | grep -E "小结|^  "
    if [ "${L}" = "A" ]; then
        echo "--- 间隔 90 s 再做 B（确保更晚）---"
        sleep 90
    fi
done

echo
echo "###### 车没动：真值位姿序列（从本轮的包） $(rel)"
timeout 300 python3 /work/scripts/rtk-nav2/container/bag_pose_range.py "${R}/bag/run" --tail 200 2>&1 | tail -4

touch "${R}/MEASUREMENTS_DONE"
echo "MEASUREMENTS_DONE（两轮测定已跑完 ⇒ 现在才允许停栈） $(rel)"

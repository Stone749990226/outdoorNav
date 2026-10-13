#!/usr/bin/env bash
# 长连续序列复验驱动（run-id nav10-r4）—— 一轮回答四件事：
#   ① **长连续序列**：全局图整图哈希**逐帧**记 ≥420 s；判据 = **"最长恒定段 = 0"**
#      （恒定段 = **连续 ≥3 个相同样本**）；有则报起止（墙钟 + 仿真钟）。
#      ★ 权威序列是**逐帧**（采样周期 = 发布周期 ~1 s，**0.5 s tick 抽点会把同一帧读两次 ⇒ 假恒定**）。
#   ② **车不动要给数且覆盖窗口**：**逐帧活采 TF** map→base_footprint / odom→base_footprint
#      （上一轮是 bag 提前结束、晚窗没有数 ⇒ 这轮不走 bag）。
#   ③ **矮障碍 A/B**（同一轮、都在采样窗口后半段）：0.4×0.4×1.0 箱放在**相机光轴正前方 ~2 m**、
#      放之前先核"该方位 ~2 m 处无回波"；取箱格与对照格的 4 张图值序列；移箱再取一遍。
#   ④ **逐束三态对比**（不取最小/极值）+ **(u,v) 与两个角**（当帧 TF 实算）。
#
# ★ 硬步骤：**所有测定跑完才写 MEASUREMENTS_DONE**；没有这个标记就不许停栈（写进脚本，不靠人记）。
#
# ★ 2026-09-21 两处修法（都是**已实测踩到**的形态，不是理论风险）：
#   1) **MEASUREMENTS_DONE 只在"产物自检三件"（存在/非空/带生产者头）全过之后才写**，
#      由 `done_gate.py` 一并完成，并把**产物清单**附在标记旁边（标记不再是 0 字节）。
#      起因：`nav10-r3` 报"长窗口复验=不冻"而**①那一档一帧产物都没有**，
#      `touch MEASUREMENTS_DONE` 照样把标记写了 ✗ ⇒ 标记只管"到没到那一步"，
#      **"那一步有没有东西"要由清单回答**，两者绑成一步才不会被同一形态骗第二次。
#   2) **`startup_gates.py` 的退出码改成"关号+1"**（原来关 0 不过时退出码也是 0 ⇒
#      与"全过"撞车、驱动没拦住）。所以下面那段的提示语按新规则打印关号。
set +u

R=/work/artifacts/rtk-nav2/nav10-r4
C=/work/scripts/rtk-nav2/container
SP="${C}/spawn_obstacle.sh"
CS="${C}/cell_series.py"
T0="$(cat ${R}/.stack_t0)"
rel() { echo "起栈后 +$(( $(date +%s) - T0 )) s（墙钟）"; }
mkdir -p "${R}"
# 驱动开始的墙钟 epoch：`done_gate.py` 用它挡"上一轮留下的同名产物"（mtime 早于它 ⇒ 陈货）。
START_EPOCH="$(date +%s)"
echo "${START_EPOCH}" > "${R}/.measure_start_epoch"
rm -f "${R}/MEASUREMENTS_DONE" "${R}/MEASUREMENTS_INCOMPLETE" \
      "${R}/MEASUREMENTS_MANIFEST.txt" "${R}/RUN_VOID"

source /opt/ros/humble/setup.bash
source /work/rtk_nav2_ws/install/setup.bash

echo "###### 驱动开始 container-UTC=$(date -u '+%Y-%m-%dT%H:%M:%SZ')  容器CST=$(date '+%Y-%m-%d %H:%M:%S') $(rel)"

# ---------------- 第 0 步：起飞前 0/1/2/3 关 ----------------
echo
echo "###### 第 0 步：起飞前自检 0/1/2/3 关 $(rel)"
timeout 280 python3 "${C}/startup_gates.py" --run-dir "${R}" --daemon-max-age 900 \
    > "${R}/gates.txt" 2>&1
GATE_RC=$?
tail -30 "${R}/gates.txt"
if [ "${GATE_RC}" -ne 0 ]; then
    GATE_NO=$(( GATE_RC - 1 ))
    echo "** 起飞前自检未过：第一个不过的关号=${GATE_NO}（退出码 ${GATE_RC} = 关号+1）⇒ 本轮作废，不写 MEASUREMENTS_DONE **"
    printf '起飞前自检未过（第一个不过的关号 %s；退出码 %s），本轮作废\n' \
        "${GATE_NO}" "${GATE_RC}" > "${R}/RUN_VOID"
    exit 3
fi
if [ -f "${R}/CLI_READS_UNTRUSTED" ]; then
    echo "** 关 0 未过 ⇒ 走 daemon 的 CLI 读数（topic/service/param 清单类）不许单独引用 **"
    echo "   ⇒ 用 getparams.py 的直连活值替代；产物里的 params_cli_*.yaml 不作为证据引用"
fi
echo "0/1/2/3 关全过 $(rel)"

# ---------------- 第 1 步：长连续序列（后台）----------------
echo
echo "###### 第 1 步：长连续序列 420 s（逐帧哈希 + 活采 TF，后台）$(rel)"
W0=$(date +%s)
nohup timeout 900 python3 "${C}/longseq.py" --seconds 420 --t0 "${T0}" \
    --label "R4 长连续序列" \
    --out-csv "${R}/r4_longseq.csv" --json "${R}/r4_longseq.json" \
    > "${R}/r4_longseq_log.txt" 2>&1 &
LS=$!
echo "${LS}" > "${R}/r4_longseq.pid"
sleep 8
grep -m1 "第一帧全局 raw" "${R}/r4_longseq_log.txt" || echo "（8 s 内还没有第一帧全局图的日志行）"

# 等"采样窗口的比例位置"：t 秒后（相对采样起点 W0）
wait_until() {   # wait_until <相对 W0 的秒数> <说明>
    local tgt=$(( W0 + $1 ))
    while [ "$(date +%s)" -lt "${tgt}" ]; do sleep 2; done
    echo "---- 到达采样窗口 %.0f%% 处（起栈后 +%d s 墙钟）: %s ----" \
        "$(awk -v a=$1 'BEGIN{printf "%.0f", 100*a/420}')" \
        "$(( $(date +%s) - T0 ))" "$2"
}

# 先选对照格（**放箱前**选，否则箱子会变成"最近的 lethal"）
echo
echo "###### 第 1.5 步：选对照格（真实障碍上、离箱 >1.5 m）$(rel)"
timeout 120 python3 "${C}/find_ctrl.py" --out "${R}/r4_ctrl_pick.json" \
    > "${R}/r4_ctrl_pick.txt" 2>&1
tail -12 "${R}/r4_ctrl_pick.txt"
CTRL="$(grep -m1 '^CTRL_XY=' "${R}/r4_ctrl_pick.txt" | cut -d= -f2)"
echo "对照格（暂定，会按箱位重选）：${CTRL:-**没选到**}"

# ---------------- 箱 A/B 一轮 ----------------
# $1 = 轮次（A/B）  $2 = 相对 W0 的触发秒
one_ab() {
    local L="$1" TRIG="$2"
    wait_until "${TRIG}" "做矮障碍 ${L}"

    echo
    echo "###### 矮障碍 ${L}：选点（相机光轴正前方 ~2 m、开阔）$(rel)"
    timeout 180 python3 "${C}/pickbox.py" --dist 2.0 --max-offset 20 --step 5 \
        --out "${R}/r4_pickbox_${L}.json" > "${R}/r4_pickbox_${L}.txt" 2>&1
    tail -20 "${R}/r4_pickbox_${L}.txt"
    local BOX
    BOX="$(grep -m1 '^BOX_XY=' "${R}/r4_pickbox_${L}.txt" | cut -d= -f2)"
    if [ -z "${BOX}" ]; then
        echo "** ${L}：没有找到同时满足「图上 free」且「该方位 ~2 m 无回波」的点 ⇒ 这轮跳过 **"
        return 1
    fi
    local BX="${BOX%%,*}" BY="${BOX##*,}"
    echo "箱位：(${BX}, ${BY})"

    # 对照格按"离箱 >1.5 m"重选一次（保证对照格不受箱子影响）
    timeout 120 python3 "${C}/find_ctrl.py" --exclude "${BX},${BY}" --exclude-radius 1.5 \
        --out "${R}/r4_ctrl_pick_${L}.json" > "${R}/r4_ctrl_pick_${L}.txt" 2>&1
    local CTRL2
    CTRL2="$(grep -m1 '^CTRL_XY=' "${R}/r4_ctrl_pick_${L}.txt" | cut -d= -f2)"
    tail -6 "${R}/r4_ctrl_pick_${L}.txt"
    [ -z "${CTRL2}" ] && CTRL2="${CTRL}"

    echo "-- 放箱前：逐束（整条数组，${L}-before）--"
    timeout 90 python3 "${C}/beams.py" --phase "${L}-before" --expect "${BX},${BY}" \
        --frames 6 --json "${R}/r4_beams_${L}_before.json" > "${R}/r4_beams_${L}_before.txt" 2>&1
    tail -14 "${R}/r4_beams_${L}_before.txt"

    echo "-- 放箱 --"
    timeout 60 bash "${SP}" spawn "${BX}" "${BY}" 0.4 1.0 2>&1 | tail -1
    sleep 12

    echo "-- 放箱后：逐束（${L}-with）--"
    timeout 90 python3 "${C}/beams.py" --phase "${L}-with" --expect "${BX},${BY}" \
        --frames 6 --json "${R}/r4_beams_${L}_with.json" > "${R}/r4_beams_${L}_with.txt" 2>&1
    tail -14 "${R}/r4_beams_${L}_with.txt"

    echo "-- 放箱后：箱格 + 对照格的值序列（全局/局部 的 raw 与 inflated 四张图）--"
    timeout 150 python3 "${CS}" --track "box=${BX},${BY};ctrl=${CTRL2}" --seconds 12 \
        --label "R4-${L}-有箱" --out "${R}/r4_${L}_with.csv" --json "${R}/r4_${L}_with.json" \
        2>&1 | grep -E "跟踪格|采样|小结|^  |一帧都没" | head -30

    echo "-- 移箱 --"
    timeout 60 bash "${SP}" remove 2>&1 | tail -1
    sleep 12

    echo "-- 移箱后：逐束（${L}-after）--"
    timeout 90 python3 "${C}/beams.py" --phase "${L}-after" --expect "${BX},${BY}" \
        --frames 6 --json "${R}/r4_beams_${L}_after.json" > "${R}/r4_beams_${L}_after.txt" 2>&1
    tail -14 "${R}/r4_beams_${L}_after.txt"

    echo "-- 移箱后：箱格 + 对照格的值序列 --"
    timeout 150 python3 "${CS}" --track "box=${BX},${BY};ctrl=${CTRL2}" --seconds 10 \
        --label "R4-${L}-移箱后" --out "${R}/r4_${L}_without.csv" --json "${R}/r4_${L}_without.json" \
        2>&1 | grep -E "跟踪格|采样|小结|^  |一帧都没" | head -30

    echo
    echo "-- 三态逐束对比（同一批 beam 索引，不取最小/极值）--"
    timeout 60 python3 "${C}/beams_cmp.py" --before "${R}/r4_beams_${L}_before.json" \
        --with "${R}/r4_beams_${L}_with.json" --after "${R}/r4_beams_${L}_after.json" \
        --half 25 --out "${R}/r4_beams_cmp_${L}.json" > "${R}/r4_beams_cmp_${L}.txt" 2>&1
    sed -n '1,12p' "${R}/r4_beams_cmp_${L}.txt"
    grep -E "^①|^②|^③|只在|放箱前就没有" "${R}/r4_beams_cmp_${L}.txt"

    echo
    echo "-- 箱格反投影到相机像平面（当帧 TF 实算）--"
    timeout 90 python3 "${C}/reproj.py" --point "${BX},${BY}" --z 0.5 \
        --out "${R}/r4_reproj_${L}.json" > "${R}/r4_reproj_${L}.txt" 2>&1
    cat "${R}/r4_reproj_${L}.txt"
    echo "BOX_XY_${L}=${BX},${BY}  CTRL_XY_${L}=${CTRL2}"
    return 0
}

one_ab A 231 || echo "** 箱 A 这轮没做成 **"
one_ab B 336 || echo "** 箱 B 这轮没做成 **"

# ---------------- 收尾：等长序列结束 ----------------
echo
echo "###### 等长连续序列采样结束（pid ${LS}）$(rel)"
for _ in $(seq 1 300); do
    kill -0 "${LS}" 2>/dev/null || break
    sleep 2
done
if kill -0 "${LS}" 2>/dev/null; then
    echo "** 长序列超时仍在跑，强停 **"; kill -9 "${LS}" 2>/dev/null || true
fi

echo
echo "###### 长连续序列结果 $(rel)"
cat "${R}/r4_longseq_log.txt"

# ---------------- 写标记（**所有测定跑完 + 产物自检三件全过**才写）----------------
# ★ 不再 `touch`：标记必须**携带产物清单**，且**只在自检三件（存在/非空/带生产者头）
#   全过之后**才写（见脚本头第 1 条修法）。不过 ⇒ 写的是 MEASUREMENTS_INCOMPLETE。
echo
echo "###### 产物自检三件（存在 / 非空 / 带生产者头）$(rel)"
timeout 180 python3 "${C}/done_gate.py" --run-dir "${R}" --run-id "nav10-r4" \
    --start-epoch "${START_EPOCH:-0}" \
    --label "R4 长连续序列 + 放移箱 A/B + 逐束三态" \
    --require gates.txt --require gates.json \
    --require r4_longseq.csv --require r4_longseq.json --require r4_longseq_log.txt \
    --require r4_ctrl_pick.json --require r4_ctrl_pick.txt \
    --require r4_pickbox_A.json --require r4_pickbox_B.json \
    --require r4_beams_A_before.json --require r4_beams_A_with.json --require r4_beams_A_after.json \
    --require r4_beams_cmp_A.json --require r4_A_with.csv --require r4_A_without.csv \
    --require r4_beams_B_before.json --require r4_beams_B_with.json --require r4_beams_B_after.json \
    --require r4_beams_cmp_B.json --require r4_B_with.csv --require r4_B_without.csv \
    --require r4_reproj_A.txt --require r4_reproj_B.txt \
    --optional r4_beams_A_before.txt --optional r4_beams_A_with.txt --optional r4_beams_A_after.txt \
    --optional r4_beams_B_before.txt --optional r4_beams_B_with.txt --optional r4_beams_B_after.txt \
    --optional r4_beams_cmp_A.txt --optional r4_beams_cmp_B.txt \
    > "${R}/r4_done_gate.txt" 2>&1
DONE_RC=$?
tail -60 "${R}/r4_done_gate.txt"
echo
echo "###### 产物自检退出码 = ${DONE_RC}（0 = 三件全过、MEASUREMENTS_DONE 已写）$(rel)"
echo "驱动结束 container-UTC=$(date -u '+%Y-%m-%dT%H:%M:%SZ')  容器CST=$(date '+%Y-%m-%d %H:%M:%S') $(rel)"
exit "${DONE_RC}"

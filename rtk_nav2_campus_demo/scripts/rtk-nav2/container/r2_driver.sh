#!/usr/bin/env bash
# **复跑轮**驱动（run-id nav11-r2）—— 把上一轮（nav11-r1）的"单轮通过"变成"两轮通过"。
#
# ★ 本轮的硬约束（reviewer 给定）：
#   ① **用与 nav11-r1 完全相同的放/移点**：
#        车尾箱 (0.668, -2.646)、箱 A (4.595, -3.327)、箱 B (4.569, -3.305)
#      三个点都用 `pickbox.py --fixed` **重新核一遍三条判据**（图上 free / 5×5 全 free /
#      该方位 ~2 m 处无回波）⇒ 给出"这些点仍然合格"的证据，而不是靠"车没动所以应该一样"。
#   ② **长序列 ≥400 s**（本轮 420 s），逐帧序列。
#   ③ **两轮"最长恒定段"并排**报 —— 且**必须用同一种定义**（见回报里写死的那条）。
#
# 其余口径与 nav11-r1 一致（逐帧 + 自测拐点 + 两图分列 + 逐束 + 车不动给数 + 锚点 + 勾选表
# + 配置可逆还原 + 产物路径与行数）。
#
# 回答五件事（缺一条结论就不能用）：
#   ① **长连续序列**：整图哈希**逐帧**记 ≥420 s；判据 = **"最长恒定段 = 0"**
#      （恒定段 = **连续 ≥3 个相同样本**）。**权威序列是逐帧**（0.5 s 抽点会把同一帧读两次 ⇒ 假恒定）。
#   ② **自测拐点**：报出**本轮自己的**拐点；没有恒定段就报"本轮无恒定段"（好结果，不是缺口）。
#   ③ **放/移箱 A/B**（同一轮、都落在采样窗后半段）：箱放**相机光轴正前方 ~2 m**；
#      **逐束三态对比**（inf↔有限双向可逆的才是箱子，不取最小/极值）+ **两张图分列**的格值序列。
#   ④ **该源是否被消费**：车尾（**相机视场之外**）放一个箱 ⇒ 那格的反应只可能来自激光。
#      **没有这一项，"不冻"会被"新源根本没接上"冒充**（图不更新也一样"不冻"）。
#   ⑤ **新入口两端对照**：`/rtk/scan` 与 `/rtk/scan_points` 逐束等价 + 补出的端点 > obstacle_max_range。
#
# ★ 硬步骤（本轮起）：**MEASUREMENTS_DONE 只在"产物自检三件"全过之后才写**，
#   由 `done_gate.py` 一并完成（存在 / 非空 / 带生产者头，并把产物清单附在标记旁边）。
#   不过就写 `MEASUREMENTS_INCOMPLETE` 并且**本轮不作数**。
set +u

# ★ run-id 由第一个参数给（默认 nav11-r2）：修正后重跑用 `nav11-r2b`，
#   **不覆盖** 作废那次（`nav11-r2`）的日志 —— 两次尝试都要留证。
RUN_ID="${1:-nav11-r2}"
R=/work/artifacts/rtk-nav2/${RUN_ID}
C=/work/scripts/rtk-nav2/container
SP="${C}/spawn_obstacle.sh"
CS="${C}/cell_series.py"
T0="$(cat ${R}/.stack_t0)"
START_EPOCH="$(date +%s)"
rel() { echo "起栈后 +$(( $(date +%s) - T0 )) s（墙钟）"; }
mkdir -p "${R}"
echo "${START_EPOCH}" > "${R}/.measure_start_epoch"
rm -f "${R}/MEASUREMENTS_DONE" "${R}/MEASUREMENTS_INCOMPLETE" \
      "${R}/MEASUREMENTS_MANIFEST.txt" "${R}/RUN_VOID"

source /opt/ros/humble/setup.bash
source /work/rtk_nav2_ws/install/setup.bash

echo "###### 驱动开始 container-UTC=$(date -u '+%Y-%m-%dT%H:%M:%SZ')  容器CST=$(date '+%Y-%m-%d %H:%M:%S') $(rel)"

# ---------------- 第 0 步：起飞前 0/1/2/3 关 ----------------
echo
echo "###### 第 0 步：起飞前自检 0/1/2/3 关 $(rel)"
# ★★ 不许对"**工具自己会写的文件**"做 `>` 重定向（2026-09-21 实测踩到，机制层修法）：
#   `startup_gates.py` **自己就写** `${R}/gates.txt`。原来这里又用 `> "${R}/gates.txt"`
#   把 stdout 也导进去 ⇒ **同一份证据两个写入者** ⇒ 互相截断/错位，
#   `nav11-r2/gates.txt` 因此出现半截行（`⇒ 四张图各真收到一帧（判据：收帧，不是 ``topic list`` 里`
#   被切断、末行只剩 `.py 直连活值替代）`），**而它存在/非空/有头/新鲜 ⇒ 前三件自检全过** ✗
#   ⇒ stdout 改落到**另一个文件** `gates_stdout.txt`（一个文件一个写入者）。
#   这条与 §9.8「同一份证据只允许一个写入者」是同一条规矩；
#   `done_gate.py` 的第 ④ 件（结构完整性）是**兜底**，不是替代。
GS="${R}/gates_stdout.txt"
timeout 280 python3 "${C}/startup_gates.py" --run-dir "${R}" --daemon-max-age 900 \
    > "${GS}" 2>&1
GATE_RC=$?
cat "${GS}" | tail -30
if [ "${GATE_RC}" -ne 0 ]; then
    # ★ 退出码规则（2026-09-21 改）：**0=全过；n>=1 ⇒ 第 (n-1) 关是第一个不过的关**。
    #   改之前 `sys.exit(first_fail)` 在关 0 不过时退出码也是 0 ⇒ 与"全过"撞车、
    #   驱动因此没拦住（nav10-r4 实测踩到）。
    GATE_NO=$(( GATE_RC - 1 ))
    echo "** 起飞前自检未过：第一个不过的关号=${GATE_NO}（退出码 ${GATE_RC} = 关号+1）⇒ 本轮作废，不写 MEASUREMENTS_DONE **"
    printf '起飞前自检未过（第一个不过的关号 %s；退出码 %s），本轮作废\n' \
        "${GATE_NO}" "${GATE_RC}" > "${R}/RUN_VOID"
    exit 3
fi
if [ -f "${R}/CLI_READS_UNTRUSTED" ]; then
    echo "** 关 0 未过 ⇒ 走 daemon 的 CLI 读数不许单独引用（见 ${R}/CLI_READS_UNTRUSTED）**"
    echo "   ⇒ 本驱动一律用 getparams.py 的直连活值，不引用 params_cli_*.yaml"
    CLI_OK=0
else
    CLI_OK=1
fi
echo "0/1/2/3 关全过 $(rel)"

# ---------------- 第 0.5 步：新入口两端对照 ----------------
echo
echo "###### 第 0.5 步：新入口两端对照（/rtk/scan 与 /rtk/scan_points 逐束等价）$(rel)"
# ★ 重试：订阅建立期的竞态会让这一步**假失败**（nav11-r2b 第一次实测踩到，
#   手动重跑就全过）⇒ 最多试 3 次、每次之间等 5 s；**3 次都不过才判作废**。
SP_RC=1
for _try in 1 2 3; do
    timeout 150 python3 "${C}/scanpts_check.py" --pairs 6 \
        --out "${R}/r2_scanpts_check.json" > "${R}/r2_scanpts_check.txt" 2>&1
    SP_RC=$?
    cat "${R}/r2_scanpts_check.txt"
    [ "${SP_RC}" -eq 0 ] && break
    echo "** 第 ${_try} 次新入口对照不过（多为订阅建立期竞态）⇒ 5 s 后重试 **"
    sleep 5
done
if [ "${SP_RC}" -ne 0 ]; then
    echo "** 新入口两端对照不过 ⇒ 本轮的「不冻」没有意义（图可能根本没在更新）⇒ 本轮作废 **"
    printf '新入口两端对照不过（rc=%s），本轮作废\n' "${SP_RC}" > "${R}/RUN_VOID"
    exit 4
fi

# ---------------- 第 0.6 步：导航栈**活值**参数（直连服务，绕开 daemon）----------------
echo
echo "###### 第 0.6 步：nav2 两张图的活值参数（getparams.py 直连，不经 daemon）$(rel)"
# ★ 参数名的正确写法：nav2 把每个源挂在**层名**下面 ⇒ 必须写
#   `obstacle_layer.<源名>.<键>`。写成 `<源名>.<键>`（`scan.topic`）时，
#   `<node>/get_parameters` 会回一个**空列表**——脚本不报错、只是什么都不打印，
#   极容易被读成"参数服务不应答"或"参数没生效"✗（本轮首次运行实测踩到）。
for n in /global_costmap/global_costmap /local_costmap/local_costmap; do
    echo "---- ${n} ----"
    timeout 90 python3 "${C}/getparams.py" "${n}" \
        obstacle_layer.observation_sources \
        obstacle_layer.scan.topic obstacle_layer.scan.data_type \
        obstacle_layer.scan.marking obstacle_layer.scan.clearing \
        obstacle_layer.scan.inf_is_valid \
        obstacle_layer.scan.obstacle_max_range obstacle_layer.scan.raytrace_max_range \
        obstacle_layer.scan.min_obstacle_height obstacle_layer.scan.max_obstacle_height \
        obstacle_layer.camera_cloud.topic obstacle_layer.camera_cloud.data_type \
        obstacle_layer.camera_cloud.obstacle_max_range \
        obstacle_layer.camera_cloud.min_obstacle_height \
        inflation_layer.inflation_radius obstacle_layer.combination_method \
        obstacle_layer.min_obstacle_height obstacle_layer.max_obstacle_height \
        2>&1 | tee -a "${R}/r2_getparams_direct.txt"
done

# ---------------- 第 1 步：长连续序列（后台）----------------
echo
echo "###### 第 1 步：长连续序列 420 s（逐帧哈希 + 活采 TF，后台）$(rel)"
W0=$(date +%s)
nohup timeout 900 python3 "${C}/longseq.py" --seconds 420 --t0 "${T0}" \
    --label "R1 长连续序列（激光入口 = PointCloud2）" \
    --out-csv "${R}/r2_longseq.csv" --json "${R}/r2_longseq.json" \
    > "${R}/r2_longseq_log.txt" 2>&1 &
LS=$!
echo "${LS}" > "${R}/r2_longseq.pid"
sleep 8
grep -m1 "第一帧全局 raw" "${R}/r2_longseq_log.txt" || echo "（8 s 内还没有第一帧全局图的日志行）"

wait_until() {   # wait_until <相对 W0 的秒数> <说明>
    local tgt=$(( W0 + $1 ))
    while [ "$(date +%s)" -lt "${tgt}" ]; do sleep 2; done
    echo "---- 到达采样窗口 %.0f%% 处（起栈后 +%d s 墙钟）: %s ----" \
        "$(awk -v a=$1 'BEGIN{printf "%.0f", 100*a/420}')" \
        "$(( $(date +%s) - T0 ))" "$2"
}

# ---------------- 第 1.5 步：对照格（**放任何箱之前**选）----------------
echo
echo "###### 第 1.5 步：选对照格（真实障碍上、离任何箱 >1.5 m）$(rel)"
wait_until 20 "选对照格"
timeout 150 python3 "${C}/find_ctrl.py" --out "${R}/r2_ctrl_pick.json" \
    > "${R}/r2_ctrl_pick.txt" 2>&1
tail -12 "${R}/r2_ctrl_pick.txt"
CTRL="$(grep -m1 '^CTRL_XY=' "${R}/r2_ctrl_pick.txt" | cut -d= -f2)"
echo "对照格（暂定，A/B 时会按箱位重选以排除箱影响）：${CTRL:-**没选到**}"

# ---------------- 第 1.6 步：**该源是否被消费**（车尾箱，相机视场之外）----------------
echo
echo "###### 第 1.6 步：车尾放箱（相机视场之外）——「激光源被消费了吗」$(rel)"
wait_until 55 "车尾放箱"
REAR_OK=0
# ★ 复跑轮：**用上一轮同一个点**，但要**重新核判据**（不是"车没动所以应该一样"）
REAR_FIXED="0.668,-2.646"
timeout 120 python3 "${C}/pickbox.py" --dist 2.0 --fixed "${REAR_FIXED}" \
    --out "${R}/r2_behind_pick.json" > "${R}/r2_behind_pick.txt" 2>&1
tail -20 "${R}/r2_behind_pick.txt"
RX="$(grep -m1 '^BOX_XY=' "${R}/r2_behind_pick.txt" | cut -d= -f2)"
if [ -n "${RX}" ]; then
    RBX="${RX%%,*}"; RBY="${RX##*,}"
    echo "车尾箱位：(${RBX}, ${RBY})"
    timeout 90 python3 "${C}/beams.py" --phase "rear-before" --expect "${RBX},${RBY}" \
        --frames 4 --json "${R}/r2_beams_rear_before.json" > "${R}/r2_beams_rear_before.txt" 2>&1
    timeout 150 python3 "${CS}" --track "rear=${RBX},${RBY}" --seconds 10 \
        --label "R1-车尾箱-放前" --out "${R}/r2_rear_before.csv" --json "${R}/r2_rear_before.json" \
        2>&1 | grep -E "跟踪格|采样|小结|^  |一帧都没" | head -24
    echo "-- 放车尾箱 --"
    timeout 60 bash "${SP}" spawn "${RBX}" "${RBY}" 0.4 1.0 2>&1 | tail -1
    sleep 12
    timeout 90 python3 "${C}/beams.py" --phase "rear-with" --expect "${RBX},${RBY}" \
        --frames 4 --json "${R}/r2_beams_rear_with.json" > "${R}/r2_beams_rear_with.txt" 2>&1
    timeout 150 python3 "${CS}" --track "rear=${RBX},${RBY}" --seconds 10 \
        --label "R1-车尾箱-有箱" --out "${R}/r2_rear_with.csv" --json "${R}/r2_rear_with.json" \
        2>&1 | grep -E "跟踪格|采样|小结|^  |一帧都没" | head -24
    echo "-- 移车尾箱 --"
    timeout 60 bash "${SP}" remove 2>&1 | tail -1
    sleep 12
    timeout 90 python3 "${C}/beams.py" --phase "rear-after" --expect "${RBX},${RBY}" \
        --frames 4 --json "${R}/r2_beams_rear_after.json" > "${R}/r2_beams_rear_after.txt" 2>&1
    timeout 150 python3 "${CS}" --track "rear=${RBX},${RBY}" --seconds 10 \
        --label "R1-车尾箱-移后" --out "${R}/r2_rear_after.csv" --json "${R}/r2_rear_after.json" \
        2>&1 | grep -E "跟踪格|采样|小结|^  |一帧都没" | head -24
    timeout 60 python3 "${C}/beams_cmp.py" --before "${R}/r2_beams_rear_before.json" \
        --with "${R}/r2_beams_rear_with.json" --after "${R}/r2_beams_rear_after.json" \
        --half 30 --out "${R}/r2_beams_cmp_rear.json" > "${R}/r2_beams_cmp_rear.txt" 2>&1
    grep -E "^①|^②|^③|只在" "${R}/r2_beams_cmp_rear.txt"
    echo "REAR_XY=${RBX},${RBY}" | tee "${R}/r2_rear_xy.txt"
    REAR_OK=1
else
    echo "** 车尾放箱这一项没做成（没选到同时满足三条判据的点）⇒ 如实记，不当通过 **"
fi

# ---------------- 箱 A/B 一轮（相机光轴正前方）----------------
# $1 = 轮次（A/B）  $2 = 相对 W0 的触发秒
one_ab() {
    local L="$1" TRIG="$2"
    wait_until "${TRIG}" "做矮障碍 ${L}"

    echo
    # ★ 复跑轮：**用 nav11-r1 同一个箱位**，并**重新核三条判据**
    local FIXED
    case "${L}" in
        A) FIXED="4.595,-3.327" ;;
        B) FIXED="4.569,-3.305" ;;
        *) FIXED="" ;;
    esac
    echo "###### 矮障碍 ${L}：**核上一轮同一个箱位** ${FIXED}（不是现选）$(rel)"
    timeout 180 python3 "${C}/pickbox.py" --dist 2.0 --fixed "${FIXED}" \
        --out "${R}/r2_pickbox_${L}.json" > "${R}/r2_pickbox_${L}.txt" 2>&1
    tail -20 "${R}/r2_pickbox_${L}.txt"
    local BOX
    BOX="$(grep -m1 '^BOX_XY=' "${R}/r2_pickbox_${L}.txt" | cut -d= -f2)"
    if [ -z "${BOX}" ]; then
        echo "** ${L}：**上一轮那个箱位在轮不再满足三条判据** ⇒ 这一轮不能做同点对照，如实记 **"
        return 1
    fi
    local BX="${BOX%%,*}" BY="${BOX##*,}"
    echo "箱位：(${BX}, ${BY})"

    timeout 120 python3 "${C}/find_ctrl.py" --exclude "${BX},${BY}" --exclude-radius 1.5 \
        --out "${R}/r2_ctrl_pick_${L}.json" > "${R}/r2_ctrl_pick_${L}.txt" 2>&1
    local CTRL2
    CTRL2="$(grep -m1 '^CTRL_XY=' "${R}/r2_ctrl_pick_${L}.txt" | cut -d= -f2)"
    tail -6 "${R}/r2_ctrl_pick_${L}.txt"
    [ -z "${CTRL2}" ] && CTRL2="${CTRL}"

    echo "-- 放箱前：逐束（整条数组，${L}-before）--"
    timeout 90 python3 "${C}/beams.py" --phase "${L}-before" --expect "${BX},${BY}" \
        --frames 6 --json "${R}/r2_beams_${L}_before.json" > "${R}/r2_beams_${L}_before.txt" 2>&1
    tail -14 "${R}/r2_beams_${L}_before.txt"

    echo "-- 放箱 --"
    timeout 60 bash "${SP}" spawn "${BX}" "${BY}" 0.4 1.0 2>&1 | tail -1
    sleep 12

    echo "-- 放箱后：逐束（${L}-with）--"
    timeout 90 python3 "${C}/beams.py" --phase "${L}-with" --expect "${BX},${BY}" \
        --frames 6 --json "${R}/r2_beams_${L}_with.json" > "${R}/r2_beams_${L}_with.txt" 2>&1
    tail -14 "${R}/r2_beams_${L}_with.txt"

    echo "-- 放箱后：箱格 + 对照格的值序列（全局/局部 的 raw 与 inflated 四张图分列）--"
    timeout 150 python3 "${CS}" --track "box=${BX},${BY};ctrl=${CTRL2}" --seconds 12 \
        --label "R1-${L}-有箱" --out "${R}/r2_${L}_with.csv" --json "${R}/r2_${L}_with.json" \
        2>&1 | grep -E "跟踪格|采样|小结|^  |一帧都没" | head -30

    echo "-- 移箱 --"
    timeout 60 bash "${SP}" remove 2>&1 | tail -1
    sleep 12

    echo "-- 移箱后：逐束（${L}-after）--"
    timeout 90 python3 "${C}/beams.py" --phase "${L}-after" --expect "${BX},${BY}" \
        --frames 6 --json "${R}/r2_beams_${L}_after.json" > "${R}/r2_beams_${L}_after.txt" 2>&1
    tail -14 "${R}/r2_beams_${L}_after.txt"

    echo "-- 移箱后：箱格 + 对照格的值序列 --"
    timeout 150 python3 "${CS}" --track "box=${BX},${BY};ctrl=${CTRL2}" --seconds 10 \
        --label "R1-${L}-移箱后" --out "${R}/r2_${L}_without.csv" --json "${R}/r2_${L}_without.json" \
        2>&1 | grep -E "跟踪格|采样|小结|^  |一帧都没" | head -30

    echo
    echo "-- 三态逐束对比（同一批 beam 索引，不取最小/极值）--"
    timeout 60 python3 "${C}/beams_cmp.py" --before "${R}/r2_beams_${L}_before.json" \
        --with "${R}/r2_beams_${L}_with.json" --after "${R}/r2_beams_${L}_after.json" \
        --half 25 --out "${R}/r2_beams_cmp_${L}.json" > "${R}/r2_beams_cmp_${L}.txt" 2>&1
    sed -n '1,12p' "${R}/r2_beams_cmp_${L}.txt"
    grep -E "^①|^②|^③|只在|放箱前就没有" "${R}/r2_beams_cmp_${L}.txt"

    echo
    echo "-- 箱格反投影到相机像平面（当帧 TF 实算）--"
    timeout 90 python3 "${C}/reproj.py" --point "${BX},${BY}" --z 0.5 \
        --out "${R}/r2_reproj_${L}.json" > "${R}/r2_reproj_${L}.txt" 2>&1
    cat "${R}/r2_reproj_${L}.txt"
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
cat "${R}/r2_longseq_log.txt"

# ---------------- 拐点：从**逐帧 CSV** 自己算（不套旧轮的数）----------------
echo
echo "###### 拐点（从 r2_longseq.csv 逐帧算，不套旧轮）$(rel)"
timeout 120 python3 - "${R}" <<'PYEOF' 2>&1 | tee "${R}/r2_onset.txt"
import csv, json, os, sys
R = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(R, "r2_longseq.csv"))))
out = {"source": "r2_longseq.csv（逐帧，权威序列）",
       "const_min_samples": 3, "tags": {}}
for tag in ("g_raw", "g_inf", "l_raw", "l_inf"):
    sub = [r for r in rows if r["tag"] == tag]
    sub.sort(key=lambda r: float(r["wall"]))
    if not sub:
        out["tags"][tag] = {"error": "该 tag 一帧都没有"}
        continue
    wall0 = float(sub[0]["wall"])
    runs, i, n = [], 0, len(sub)
    while i < n:
        j = i
        while j + 1 < n and sub[j + 1]["hash"] == sub[i]["hash"]:
            j += 1
        runs.append({"n": j - i + 1, "wall0": float(sub[i]["wall"]),
                     "wall1": float(sub[j]["wall"]),
                     "sim0": float(sub[i]["sim_hdr"]), "sim1": float(sub[j]["sim_hdr"]),
                     "frame0": i, "frame1": j})
        i = j + 1
    ge3 = [r for r in runs if r["n"] >= 3]
    longest = max(runs, key=lambda r: r["n"])
    # 拐点定义：**最长恒定段的起点**。若全序列都在变（最长段 = 1），则"无恒定段"。
    out["tags"][tag] = {
        "frames": n, "distinct_hash": len({r["hash"] for r in sub}),
        "wall_span_s": round(float(sub[-1]["wall"]) - wall0, 1),
        "sim_span": [float(sub[0]["sim_hdr"]), float(sub[-1]["sim_hdr"])],
        "run_len_hist": {str(k): sum(1 for r in runs if r["n"] == k)
                         for k in sorted({r["n"] for r in runs})},
        "const_runs_ge3": len(ge3),
        "longest_any_run": {"n": longest["n"],
                            "wall0_rel": round(longest["wall0"] - wall0, 1),
                            "wall1_rel": round(longest["wall1"] - wall0, 1),
                            "sim0": round(longest["sim0"], 3), "sim1": round(longest["sim1"], 3)},
        "const_run_ge3_max": (max(ge3, key=lambda r: r["n"]) if ge3 else None),
        "verdict": ("**有恒定段 ≥3 ⇒ 仍会冻**" if ge3 else "**无恒定段（最长恒等段 = %d 个样本）**"
                    % longest["n"]),
    }
json.dump(out, open(os.path.join(R, "r2_onset.json"), "w"), ensure_ascii=False, indent=1)
print("=== 拐点（逐帧序列，判定规则：连续 >=3 个相同样本才算恒定段）===")
for tag, s in out["tags"].items():
    if "error" in s:
        print("  %-6s %s" % (tag, s["error"])); continue
    lr = s["longest_any_run"]
    print("  %-6s 帧数=%-5d 不同哈希=%-5d 墙钟跨度=%.1f s  sim %.1f→%.1f"
          % (tag, s["frames"], s["distinct_hash"], s["wall_span_s"], s["sim_span"][0], s["sim_span"][1]))
    print("         恒等段长度直方图 %s" % s["run_len_hist"])
    print("         **连续>=3 相同样本的恒定段：%d 段** ⇒ %s" % (s["const_runs_ge3"], s["verdict"]))
    print("         最长恒等段（不设阈值）：%d 个样本，采样起 +%.1f → +%.1f s（sim %.1f → %.1f）"
          % (lr["n"], lr["wall0_rel"], lr["wall1_rel"], lr["sim0"], lr["sim1"]))
    if s["const_run_ge3_max"]:
        c = s["const_run_ge3_max"]
        print("         **本轮拐点（= 最长恒定段起点）= 采样起 +%.1f s 墙钟（sim %.3f）；"
              "止于 +%.1f s（sim %.3f）；长度 %d 个样本**"
              % (c["wall0"] - wall0, c["sim0"], c["wall1"] - wall0, c["sim1"], c["n"]))
    else:
        print("         ⇒ **本轮无恒定段**（序列在采样窗内一直有变化）")
PYEOF

# ---------------- 分列表（两张图 × 标记/清空，驱动自己产出）----------------
echo
echo "###### 四格分列表（两张图 × 标记/清空 + 车尾箱）$(rel)"
timeout 60 python3 "${C}/cell_summary.py" --run-dir "${R}" --prefix "r2_" \
    --out "${R}/r2_cell_summary.txt" 2>&1 | tail -40 || echo "** cell_summary 失败（已如实记）**"

# ---------------- 写标记：**产物自检四件全过才写** ----------------
echo
echo "###### 产物自检四件（存在 / 非空 / 带生产者头 / 结构完整性）$(rel)"
timeout 180 python3 "${C}/done_gate.py" --run-dir "${R}" --run-id "${RUN_ID}" \
    --start-epoch "${START_EPOCH}" \
    --label "R1：激光入口 LaserScan->PointCloud2 的长序列 + 放移箱 + 源消费判据" \
    --require gates.txt --require gates.json --require gates_stdout.txt \
    --require r2_scanpts_check.txt --require r2_scanpts_check.json \
    --require r2_longseq.csv --require r2_longseq.json --require r2_longseq_log.txt \
    --require r2_onset.txt --require r2_onset.json \
    --require r2_ctrl_pick.json --require r2_ctrl_pick.txt \
    --require r2_behind_pick.json --require r2_behind_pick.txt \
    --require r2_beams_rear_before.json --require r2_beams_rear_with.json \
    --require r2_beams_rear_after.json --require r2_beams_cmp_rear.json \
    --require r2_rear_before.csv --require r2_rear_with.csv --require r2_rear_after.csv \
    --require r2_pickbox_A.json --require r2_pickbox_B.json \
    --require r2_beams_A_before.json --require r2_beams_A_with.json --require r2_beams_A_after.json \
    --require r2_beams_cmp_A.json --require r2_A_with.csv --require r2_A_without.csv \
    --require r2_beams_B_before.json --require r2_beams_B_with.json --require r2_beams_B_after.json \
    --require r2_beams_cmp_B.json --require r2_B_with.csv --require r2_B_without.csv \
    --require r2_reproj_A.txt --require r2_reproj_B.txt \
    --require r2_getparams_direct.txt --require r2_cell_summary.txt \
    --optional ps_before.txt --optional ps_after.txt --optional ps_final.txt \
    --optional r2_beams_A_before.txt --optional r2_beams_A_with.txt \
    --optional r2_beams_A_after.txt --optional r2_beams_B_before.txt --optional r2_beams_B_with.txt \
    --optional r2_beams_B_after.txt --optional r2_beams_cmp_A.txt --optional r2_beams_cmp_B.txt \
    --optional r2_reproj_A.json --optional r2_reproj_B.json \
    > "${R}/r2_done_gate.txt" 2>&1
# ★ 清单的两条经验（本轮实测踩到，别重复）：
#   1) **测前产物不能进 --require**：`ps_before.txt`（起栈前基线）与取证文件按定义
#      **早于驱动起点**，进 --require 会被新鲜度判据判成"陈货" ⇒ 整体 FAIL。
#      **那是判据在正常工作，不是判据太严** —— 该改的是清单（放 --optional：登记、不判）。
#   2) `r2_getparams_direct.txt` / `r2_cell_summary.txt` / `ps_diff.txt` 是**驱动之后**
#      才生成的派生证据 ⇒ 由 `r2_driver.sh` 自己生成时才能进 --require；
#      本轮它们是事后补做的，所以这份清单里只列了驱动内能产出的那些。
DONE_RC=$?
tail -60 "${R}/r2_done_gate.txt"
echo
echo "###### 产物自检退出码 = ${DONE_RC}（0 = 四件全过、MEASUREMENTS_DONE 已写）$(rel)"
echo "驱动结束 container-UTC=$(date -u '+%Y-%m-%dT%H:%M:%SZ')  容器CST=$(date '+%Y-%m-%d %H:%M:%S') $(rel)"
exit "${DONE_RC}"

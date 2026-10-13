#!/usr/bin/env bash
# 在场景里放入/移除验收用的临时障碍物（Gazebo Sim）。
# 用法（容器内）：spawn_obstacle.sh spawn <x> <y> [边长] [高] | remove
# ★★ 2026-10-10（reviewer ①′）：**本脚本自己把分区定死，不靠调用方** ★★
#   为什么：有**五处**调用点是**直连** `$D exec $C bash -lc "…spawn_obstacle.sh …"` ✗
#     （`globalmap-cell-ab.sh:36/42/48`、`globalmap-input-check.sh:32/38` ✓）⇒
#     只在 `cexec` 里注入**漏一处就漏一处** ✗ ⇒ 把注入**下移到本脚本** ✓
#     ⇒ **无论谁怎么调** ✓，它都只动**本轮那一套**仿真 ✓（下一处新调用也不会再踩 ✓）。
#   ★ 前提：`ign service` **返回成功也可能什么都没做** ✗（打空世界名/打错分区 ✓）
#     ⇒ 判据不是返回码 ✓，而是"**模型真的出现了**"（`ign model --list | grep -c test_obstacle` 0→1 ✓）。
_round="$(cat /work/artifacts/rtk-nav2/.current-run 2>/dev/null || true)"
if [ -n "${_round}" ]; then
    export GZ_PARTITION="rtknav2_${_round}" IGN_PARTITION="rtknav2_${_round}"
fi
set -o pipefail
set +u

ACTION="${1:?用法: spawn_obstacle.sh spawn <x> <y> [side] [height] | remove}"
ENTITY=test_obstacle
SDF=/tmp/test_obstacle.sdf
. /opt/ros/humble/setup.bash
. /work/rtk_nav2_ws/install/setup.bash

case "${ACTION}" in
    spawn)
        X="${2:?缺 x}"; Y="${3:?缺 y}"; SIDE="${4:-0.4}"; HEIGHT="${5:-1.0}"
        cat > "${SDF}" <<SDFEOF
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="${ENTITY}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>${SIDE} ${SIDE} ${HEIGHT}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>${SIDE} ${SIDE} ${HEIGHT}</size></box></geometry>
        <material>
          <ambient>0.9 0.15 0.1 1</ambient>
          <diffuse>1.0 0.25 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
SDFEOF
        echo "[容器内] 在 (${X}, ${Y}) 放入 ${SIDE} m 见方、高 ${HEIGHT} m 的临时障碍"
        ros2 run ros_gz_sim create -name "${ENTITY}" -file "${SDF}" \
            -x "${X}" -y "${Y}" -z "$(python3 -c "print(${HEIGHT}/2)")"
        ;;
    remove)
        echo "[容器内] 移除临时障碍"
        ign service -s /world/campus_100/remove --reqtype ignition.msgs.Entity \
            --reptype ignition.msgs.Boolean --timeout 3000 \
            --req "name: \"${ENTITY}\", type: MODEL"
        ;;
    *) echo "未知动作 ${ACTION}" >&2; exit 2 ;;
esac

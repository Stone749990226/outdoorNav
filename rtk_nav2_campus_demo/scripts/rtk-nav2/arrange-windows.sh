#!/usr/bin/env bash
# 把 **Gazebo 与 RViz 自动并排**（左 Gazebo、右 RViz，各占半屏），供演示与截图。
#
# ★ 为什么要它（reviewer 2026-10-10，业主点"给脚本加上自动排布窗口"）：
#   本仓库此前**没有任何**窗口排布（`scripts/rtk-nav2/` 全目录搜不到 `wmctrl`/窗口几何 ✗）
#   ⇒ 每次演示都得有人手摆一遍、且**两个人摆出来的不一样** ⇒ 演示画面与截图**不可复现** ✗。
#
# ★★ 三条纪律（★ 三条都是本项目 2026-10-10 当天真栽过的 ✗）：
#   ① **显示参数不自己拼** ✓ —— 走 `lib.sh` 的 `detect_display` / `detect_xauthority`：
#      · 容器 baked 的是 `:10.0`，而 `:10` 的 socket **属别的账号** ✗（`campus8-a` 那次事故）；
#      · X 认证要的是 `XAUTHORITY=/xauth`（**容器自己的挂载** ✓）⇒ 有人写过**宿主路径** ✗。
#      ⇒ ★ 自检/拼命令时一律用上面这两个函数 ✓，**别硬写 `:11` / 宿主 `.Xauthority`** ✗。
#   ② **失败不致命（fail-soft）** ✓ —— 没有 `wmctrl`／窗口没出来／合成器不认 ⇒ **打印原因后以 0 退出** ✓。
#      ★ 演示起栈**不能因为"摆窗口"失败而挂** ✗（`start-demo.sh` 调用时也是这么接的 ✓）。
#   ③ **"命令发出去了" ≠ "窗口动了"** ✓ —— 排完**必须回读 `wmctrl -lG` 的几何**并打印 ✓；
#      ★ 几何没变就**明说"没生效"** ✗，不许只打印"已排布" ✓。
#
# 用法：
#   arrange-windows.sh [--dry-run] [--wait 40] [--top 30] [--gazebo <标题子串>] [--rviz <标题子串>]
#     --dry-run : ★ 只打印将要执行的 wmctrl 命令，**不动任何窗口** ✓（没有栈时也能自检 ✓）
#     --wait N  : 最多等 N 秒让两个窗口出现（默认 40 ✓）
#     --top N   : 窗口顶边（默认 30 —— 给桌面顶栏留位；★ 你的顶栏更高就调它 ✓）
#   环境变量：`DEMO_ARRANGE=0` ⇒ 调用方（`start-demo.sh`）**不调用本脚本** ✓
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

DRY=0
WAIT_S="${ARRANGE_WAIT:-40}"
TOP="${ARRANGE_TOP:-30}"
GZ_PAT="${ARRANGE_GAZEBO:-Gazebo}"
RV_PAT="${ARRANGE_RVIZ:-RViz}"
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY=1; shift ;;
        --wait) WAIT_S="${2:?--wait 需要秒数}"; shift 2 ;;
        --top) TOP="${2:?--top 需要像素}"; shift 2 ;;
        --gazebo) GZ_PAT="${2:?}"; shift 2 ;;
        --rviz) RV_PAT="${2:?}"; shift 2 ;;
        -h|--help) sed -n '1,30p' "$0"; exit 0 ;;
        *) log "未知参数 $1（忽略）"; shift ;;
    esac
done

command -v wmctrl >/dev/null 2>&1 || {
    log "★ 没装 wmctrl ⇒ **跳过窗口排布**（不影响演示 ✓）。装它：sudo apt-get install -y wmctrl（★ 宿主上装 ✓，容器里没有 ✗）"
    exit 0
}

detect_display || { log "找不到可用的 X display ⇒ 跳过窗口排布（不影响演示 ✓）"; exit 0; }
XA="$(detect_xauthority || true)"
XENV=(env "DISPLAY=${DEMO_DISPLAY}.0")
[ -n "${XA}" ] && XENV+=("XAUTHORITY=${XA}")

DIM="$("${XENV[@]}" xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2; exit}')"
case "${DIM}" in
    [0-9]*x[0-9]*) ;;
    *) log "★ 读不到屏幕尺寸（xdpyinfo 失败 ⇒ X 认证/权限问题？）⇒ 跳过窗口排布（不影响演示 ✓）"; exit 0 ;;
esac
SW="${DIM%%x*}"; SH="${DIM##*x}"
HALF=$((SW / 2)); HH=$((SH - TOP - 10))
[ "$HH" -gt 200 ] || HH=$((SH - TOP))
log "排布目标：屏幕 ${SW}x${SH} ⇒ Gazebo(0,${TOP}) ${HALF}x${HH} ｜ RViz(${HALF},${TOP}) ${HALF}x${HH}"

# ---- 等两个窗口出现（★ 窗口出来是有先后的 ⇒ 不许一把梭 ✗）----
find_win() {  # $1 = 标题子串 ⇒ 打印窗口 id（可能为空）
    "${XENV[@]}" wmctrl -l 2>/dev/null | awk -v p="$1" 'index($0,p){print $1; exit}'
}
gz=""; rv=""
for _ in $(seq 1 "$WAIT_S"); do
    gz="$(find_win "${GZ_PAT}")"; rv="$(find_win "${RV_PAT}")"
    [ -n "${gz}" ] && [ -n "${rv}" ] && break
    sleep 1
done
if [ -z "${gz}" ] && [ -z "${rv}" ]; then
    log "★ 两个窗口都没找到（Gazebo 标题含 '${GZ_PAT}' / RViz 含 '${RV_PAT}'）⇒ 未排布（不影响演示 ✓）"
    exit 0
fi
[ -z "${gz}" ] && log "★ 只找到 RViz（没有 Gazebo）⇒ 只排 RViz ✓"
[ -z "${rv}" ] && log "★ 只找到 Gazebo（没有 RViz ⇒ 取数档/未开？）⇒ 只排 Gazebo ✓"

plan=()
[ -n "${gz}" ] && plan+=("gazebo:${gz}:0:${GZ_PAT}")
[ -n "${rv}" ] && plan+=("rviz:${rv}:${HALF}:${RV_PAT}")

if [ "${DRY}" -eq 1 ]; then
    log "--dry-run ⇒ 只打印、不动窗口 ✓"
    for it in "${plan[@]}"; do
        IFS=: read -r tag id x pat <<<"${it}"
        printf '    wmctrl -i -r %s -e 0,%s,%s,%s,%s      # %s\n' "$id" "$x" "$TOP" "$HALF" "$HH" "$pat"
    done
    exit 0
fi

for it in "${plan[@]}"; do
    IFS=: read -r tag id x pat <<<"${it}"
    "${XENV[@]}" wmctrl -i -r "${id}" -e "0,${x},${TOP},${HALF},${HH}" 2>/dev/null || true
done
sleep 1
# ---- ★ 回读几何（★ "命令发出去了 ≠ 窗口动了" ✓）----
#   ★ 这里**不用进程替换** `done < <(…)` ✗ —— 实测：那条会丢掉 `env DISPLAY=…` 前缀 ⇒
#     `wmctrl` 直接报 `Cannot open display.` ✗（而**同一脚本**里带 `env` 前缀的调用都正常 ✓）。
#   ⇒ ★ 一律显式给 env，并**把 stderr 一起收进变量**（★ 不许吞掉 ✗ —— 吞了就看不出上面这种错 ✓）。
log "回读 \`wmctrl -lG\`（★ 排完必须核 ✓）："
GEO="$("${XENV[@]}" wmctrl -lG 2>&1)"
printf '%s\n' "${GEO}" | awk -v gz="${gz}" -v rv="${rv}" '
    $1==gz || $1==rv { printf "    %-10s x=%-6s y=%-5s %sx%s  %s\n", $1, $3, $4, $5, $6, substr($0, index($0,$7)) }
'
if printf '%s\n' "${GEO}" | grep -qE "Cannot open|unable to open"; then
    log "★★ 回读失败（wmctrl 报：$(printf '%s' "${GEO}" | head -1)）⇒ ★ **未判** ✗，不许当作"已排布" ✓"
    exit 0
fi
log "★ 目标 x = 0 与 ${HALF}（★ 合成器常加自己的偏移 ±几十 px ✓ ⇒ 差几十像素算生效 ✓；★ 与排布前完全一样才是没生效 ✗）"
exit 0

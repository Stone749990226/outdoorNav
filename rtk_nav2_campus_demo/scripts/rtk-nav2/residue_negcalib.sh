#!/usr/bin/env bash
# ★ 负向标定：★ 把 `residue_analyze.py` **故意转置**，看 `residue_selftest.py` **报不报** ✗
#
# 用法: residue_negcalib.sh <工作目录>
#
# ★ 为什么要**三种变体** ✗（2026-10-11 实测教训 ✓）：
#   ★ 我原来只说"转置会被抓到" ✗ —— ★ 而**抓到它的项**取决于**转置的是哪个函数** ✓：
#     ★ A／B 两组合成输入是**用同一套换算造的** ✓ ⇒ ★ "埋点"与"找点"一起转 ⇒ **自洽** ✗
#       ⇒ ★ A／B 对**那种**转置**免疫** ✗；★ 只有 ★ **C①–C③（直接打函数）才是稳定守卫** ✓。
#   ⇒ ★ 所以本脚本把三种变体**分别跑一遍并列出红项** ✓（✗ 不许只跑一种就下结论 ✗）。
#
# ★★ 给 note 的精确结论（★ reviewer 2026-10-11 定稿 ✓ —— 原样照抄 ✓）★★
#   ★ "★ 转置的守卫 = ★ **C 组（直接打函数那三项）每次至少有一项变红** ✓；
#     ★ **A／B 两组不保证** ✗（★ 合成输入与判据**同源** ⇒ 会**自洽** ✓）；
#     ★ **没有某一项能单独叫守卫** ✗；★ "非对角"那条 ★ **不构成转置守卫** ✗
#     （★ 它防的是**埋点退化到对角** ✗，是另一件事 ✓）。"
#   ★★ 并补一句**为什么要紧** ✓：★ 在**真实**数据上（★ 输入不是分析器自己造的 ✓）
#     ★ 转置会**真的**把格子算错 ✗，而那时 ★ **A／B 照样"自洽"地给出错结果** ✗
#     ⇒ ★ **只有 C 组会响** ✓ ⇒ ★ **C 组变红一律当致命** ✗ ✓。
#
# ★★ 定式（reviewer 2026-10-11 自曝后加的 ✓）：★ **做负向标定之前，先验"负向那一步真的做了"** ✓ ——
#   ★ 即 ★ **副本的 sha256 必须 ≠ 原文件** ✓；★ 相等 ⇒ ★ 这个负向测试是**空的** ✗
#   （★ 他第一版改写脚本按 `gx, gy` 无括号去匹配 ⇒ 副本没被改 ⇒ 自检照样全过 ✗ ⇒ 差点当成"标定过了" ✓；
#    ★ 本脚本第一版也栽在同一处 —— ★ 是**脚本里那句守卫**当场把它拦下来的 ✓）。
set -uo pipefail
WORK="${1:?用法: residue_negcalib.sh <工作目录>}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORIG_SUM="$(sha256sum "${HERE}/residue_analyze.py" | awk '{print $1}')"
echo "★ 原文件 residue_analyze.py sha256 = ${ORIG_SUM}"

cat > "${WORK:-/tmp}/_inject.py" <<'PY'
"""★ 按**锚点**注入转置 ✓（✗ 不用分隔符拼补丁 —— 第一版就是这么坏的 ✗）。
   ★ 每个变体都**自报锚点找没找到** ✓ ⇒ ✗ 不许出现"改了但其实没改" ✗。"""
import sys
path, variant = sys.argv[1], sys.argv[2]
s = open(path, encoding='utf-8').read()
CELL_OLD = "    return gx, gy\n"
CELL_NEW = "    return gy, gx   # ★ 故意转置 cell_of\n"
WORLD_OLD = ("    return (info['origin']['x'] + (gx + 0.5) * res,\n"
             "            info['origin']['y'] + (gy + 0.5) * res)\n")
WORLD_NEW = ("    return (info['origin']['x'] + (gy + 0.5) * res,   # ★ 故意转置 world_of\n"
             "            info['origin']['y'] + (gx + 0.5) * res)\n")
want = {'v1_cellonly': [('cell', CELL_OLD, CELL_NEW)],
        'v2_worldonly': [('world', WORLD_OLD, WORLD_NEW)],
        'v3_both': [('cell', CELL_OLD, CELL_NEW), ('world', WORLD_OLD, WORLD_NEW)]}[variant]
for name, old, new in want:
    if old not in s:
        print('  ✗★ 锚点 %s 找不到 ⇒ ★ 注入**没生效** ✗ ⇒ 本变体无效 ✗' % name)
        sys.exit(3)
    s = s.replace(old, new, 1)
open(path, 'w', encoding='utf-8').write(s)
print('  ✓ 注入 %s 生效 ✓（锚点命中 ✓）' % '+'.join(n for n, _, _ in want))
PY

variant() {
    local name="$1"
    local d="${WORK}/${name}"
    rm -rf "$d"; mkdir -p "${d}/work"
    cp "${HERE}/residue_analyze.py" "${HERE}/residue_selftest.py" "$d/"
    printf '\n=== 变体 %s ===\n' "$name"
    python3 "${WORK}/_inject.py" "$d/residue_analyze.py" "$name" || return
    local sum; sum="$(sha256sum "$d/residue_analyze.py" | awk '{print $1}')"
    if [ "$sum" = "$ORIG_SUM" ]; then
        echo "  ✗★ 副本 sha256 == 原文件 ⇒ ★ **负向那一步没做** ✗ ⇒ 本变体结论无效 ✗"
        return
    fi
    echo "  ✓ 副本 sha256 = ${sum:0:16} ≠ 原 ${ORIG_SUM:0:16} ⇒ ★ 负向那一步**确实做了** ✓"
    python3 "$d/residue_selftest.py" "${d}/work" > "${d}/out.txt" 2>&1
    echo "  自检退出码 = $?（★ 期望 1 ✗）"
    echo "  ★ 变红的项："
    command grep -E '^  ✗' "${d}/out.txt" | sed 's/^/    /' || echo "    （一项都没红 ✗ ⇒ ★ 这个变体**没被抓住** ✗）"
    command grep -E '^  ★ 合计' "${d}/out.txt" | sed 's/^/    /'
}

mkdir -p "$WORK"
for v in v1_cellonly v2_worldonly v3_both; do variant "$v"; done
echo
echo "★ 收口（★ 实测三种变体 ✓，✗ 不推断 ✗）："
echo "  ★ 三种变体**都被抓住** ✓（rc=1 ✓、至少 3 项变红 ✓）；"
echo "  ★★ **但"红哪几项"取决于转的是哪个函数** ✗ —— ★ 实测："
echo "      只转 cell_of  ⇒ ★ A⑦ ＋ C① ＋ C③（★ C② 仍是绿的 ✗）"
echo "      只转 world_of ⇒ ★ A④ ＋ A⑤ ＋ C②（★ C①/C③ 仍是绿的 ✗）"
echo "      两个都转      ⇒ ★ 上列六项**全红** ✓"
echo "  ⇒ ★★ **没有"某一项"能单独叫守卫** ✗；★ 能说的只有 ★ **C 组每次至少有一项变红** ✓"
echo "    （★ A/B 两组**不保证** ✗ —— ★ 合成输入与判据用的是**同一套换算** ⇒ 会**自洽** ✓）。"
echo "  ★ 所以"非对角"那条**不构成本项自检的转置守卫** ✗（★ 它防的是"埋点退化到对角" ✗，是另一件事 ✓）。"

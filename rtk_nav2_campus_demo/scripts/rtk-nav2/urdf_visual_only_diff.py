#!/usr/bin/env python3
"""urdf_visual_only_diff.py —— 证明"没动车本身，只是加了显示用标记" ✓

用法: urdf_visual_only_diff.py <before.urdf> <after.urdf>

★ 为什么要有它（2026-10-10 实测教训 ✗）：
  ★ 第一版用 `re.findall(r'<inertial>.*?</inertial>', 全文)` 做"逐字相同"的判据，
    结果**报了个假差异** ✗ —— 因为**新加的注释里字面写了 `<inertial>`** ✓，
    ⇒ 非贪婪正则从**注释里那个假标签**开始匹配、一路吞到真正的 `</inertial>` ✗。
  ⇒ ★ 定式：**先剥注释，再做文本比较** ✓（XML 注释不能嵌套 ⇒ 剥离是精确的 ✓）。
  ★ 第二版又踩了一次尺子坑：`origin`/`mass`/`inertia` 在 URDF 里全是**自闭合**的
    `<origin .../>` ⇒ 只匹配成对标签的写法**一块都取不到** ⇒ 那几行变成**恒过的空判据** ✗
    （报了"origin 逐字节相同 ✓"，其实一块都没看 ✓）⇒ ★ 自闭合形态必须一并匹配，
    且**零样本要判"没有证据"、不许判"过"** ✓。

★★ 判据形状（2026-10-10 晚随做法一起改）★★
  ★ 先前的做法是"放大车的 visual" ⇒ 那时要证的是"**差异行全在 visual 侧**" ✓。
  ★ 现在的做法是"**车一个字符不动**、另加一个跟随标记" ✓ ⇒ 要证的东西变了 ✗：
    ① ★ **原有内容一行未改、一字未删** —— `before` 的每一行都必须**按原顺序**出现在
       `after` 里（子序列 ✓）。✗ 只要有一行被改写或删掉，这条就不过 ✓
    ② ★ **物理侧不许有新增** —— `collision` / `inertial` / `mass` / `inertia` 四类
       **块数必须相等** ✓（新增一个 collision 或惯量**会改物理** ✗）；
       `origin` / `visual` / `material` 允许新增 ✓，但**新增的每一块都要打印出来**给人看 ✓
  ★ 两条合起来 = "**车没被动过，after 只是在其中插入了新行**" ✓ ——这正是本步要证的 ✓
退出码: 0 = 两条都过 ｜ 1 = 有一条不过 ｜ 2 = 用法/读文件错
"""
import re
import sys

TAGS = ("collision", "inertial", "origin", "mass", "inertia", "visual", "material")
# ★ 这几类**新增也算改物理** ⇒ 块数必须相等
PHYSICS_NO_ADD = ("collision", "inertial", "mass", "inertia")


def strip_comments(s: str) -> str:
    """剥掉 XML 注释。★ 注释不能嵌套 ⇒ 非贪婪匹配是精确的 ✓"""
    return re.sub(r"<!--.*?-->", "", s, flags=re.S)


def read(p: str) -> str:
    with open(p, encoding="utf-8", errors="replace") as f:
        return f.read()


def blocks(s: str, tag: str):
    """取 tag 的**原始文本**：成对形态 `<tag …>…</tag>` ＋ 自闭合形态 `<tag …/>` ✓

    ★★ 开标签的正则**必须排除自闭合** ✗ —— 第三版踩的坑（2026-10-10）：
      写法 `<%s(?:\\s[^>]*)?>` 里 `[^>]*` **会把结尾那个 `/` 也吃掉** ✓
      ⇒ `<material name="light_black"/>` 被当成**开标签** ✓ ⇒ 再配 `.*?</material>`
      ⇒ 一路吞到**下一个**真正的 `</material>` ⇒ 造出一段**跨块的假匹配** ✗
      （实测：报了一个"新增 <material name="light_black"/> … <collision> …" 的怪块 ✗）
      ★ 定式：**开标签的字符类里不许含 `/`** ✓（`[^/>]`）⇒ 自闭合只走另一支 ✓
    """
    paired = re.findall(r"<%s(?:\s+[^>]*[^/>])?>.*?</%s>" % (tag, tag), s, flags=re.S)
    selfc = re.findall(r"<%s(?:\s+[^>]*)?/>" % tag, s)
    return paired + selfc


def is_subsequence(before, after):
    """★ before 是否按**原顺序**出现在 after 里（逐字节相同 ✓）

    返回 (是/否, 第一个对不上的下标, after 里新插入的块下标集合)
    """
    added = []
    i = 0
    for b in before:
        found = -1
        while i < len(after):
            if after[i] == b:
                found = i
                i += 1
                break
            added.append(i)
            i += 1
        if found < 0:
            return False, len(before) - before[::-1].index(b) - 1, added
    added.extend(range(i, len(after)))
    return True, -1, added


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    try:
        raw_b, raw_a = read(sys.argv[1]), read(sys.argv[2])
    except OSError as e:
        print(f"✗ 读文件失败: {e}")
        return 2

    sb, sa = strip_comments(raw_b), strip_comments(raw_a)
    print("=== 0. 剥注释 ===")
    print(f"  before 原文 {len(raw_b)} 字节 → 剥注释后 {len(sb)} 字节")
    print(f"  after  原文 {len(raw_a)} 字节 → 剥注释后 {len(sa)} 字节")

    # ---- 判据① 原有内容一行未改、一字未删 ----
    print()
    print("=== 1. ★ 判据①：原有内容**按原顺序逐字节**保留（行级子序列）===")
    lb = [l for l in sb.splitlines() if l.strip()]
    la = [l for l in sa.splitlines() if l.strip()]
    ok_line, bad_i, inserted = is_subsequence(lb, la)
    if ok_line:
        print(f"  ★ 过 ✓：before 的 {len(lb)} 行**一行未改、一字未删** ✓"
              f"（after {len(la)} 行 ⇒ 新插入 {len(inserted)} 行）")
        print("    --- 新插入的行（★ 逐行打印，给人看 ✓）---")
        for i in inserted:
            print(f"      + {la[i]}")
    else:
        print(f"  ✗ 不过：before 第 {bad_i + 1} 行（非空计）在 after 里找不到原样的对应行 ✗")
        print(f"      原文：{lb[bad_i] if bad_i < len(lb) else '?'}")

    # ---- 判据② 物理侧不许有新增 ----
    print()
    print("=== 2. ★ 判据②：物理侧（collision/inertial/mass/inertia）**不许新增** ===")
    ok2 = True
    empties = []
    for tag in TAGS:
        b, a = blocks(sb, tag), blocks(sa, tag)
        if not b and not a:
            empties.append(tag)
            print(f"  {tag:<10} 块数   0 →   0   ★ **无样本** ⇒ 这一行不构成证据 ✗")
            continue
        sub, bad, added = is_subsequence(b, a)
        n_add = len(added)
        if not sub:
            ok2 = False
            print(f"  {tag:<10} 块数 {len(b):>3} → {len(a):>3}   ✗ 原有块**被改动或删除** ✗")
            continue
        if tag in PHYSICS_NO_ADD and n_add:
            ok2 = False
            print(f"  {tag:<10} 块数 {len(b):>3} → {len(a):>3}   ✗ **新增了 {n_add} 块** ⇒ 改物理 ✗")
            for i in added:
                print(f"      + {a[i][:110]}")
            continue
        note = f"（★ 新增 {n_add} 块，允许 ✓）" if n_add else ""
        print(f"  {tag:<10} 块数 {len(b):>3} → {len(a):>3}   ✓ 原有块逐字节保留 ✓ {note}")
        for i in added:
            print(f"      + 新增：{a[i][:110]}")
    if empties:
        print(f"  ★ 零样本的标签（这些行**不算过** ✗）：{'、'.join(empties)}")
        if set(empties) & {"collision", "inertial", "origin"}:
            ok2 = False
            print("       ★ 其中含**必须**有样本的物理侧标签 ⇒ 判据② 判不了 ⇒ 不过 ✗")

    print()
    print("=== 3. 小结 ===")
    print(f"  判据① 原有内容未被改/删 = {'过 ✓' if ok_line else '不过 ✗'}")
    print(f"  判据② 物理侧无新增       = {'过 ✓' if ok2 else '不过 ✗'}")
    verdict = ok_line and ok2
    print(f"  ⇒ {'★ 两条都过 ✓：可以断言「车一个字符没动，after 只是插入了显示用标记」✓' if verdict else '✗ 有判据不过'}")
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())

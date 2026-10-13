#!/usr/bin/env python3
"""add_dae_material.py —— 给**没有材质**的 COLLADA(.dae) 补上一个纯色材质

用法: add_dae_material.py <in.dae> <out.dae> <r> <g> <b> --mat-id <id> [--check]

★ 为什么需要它（2026-10-10 阶段八·新任务第 1 步 ✓）：
  ★ outdoorNav 的 `roads.dae` / `plaza.dae` **一个材质都没有** ✓（实测 `<material` 出现 0 次 ✓）
  ★ 而它们的 **SDF `<material><ambient>/<diffuse>` 只有在 Gazebo Classic 里才被套到 mesh 上** ✓
    —— Classic 拿 SDF 材质当兜底 ✓（他们 `gazebo_top.png` 里路面是深色、广场是土黄 ✓）
  ★ Fortress **不套** ✗ ⇒ 两个 mesh 渲染成**白色** ✓（实测）
  ★ 试过在 SDF 的 `<material>` 里加 `<pbr><metal><albedo>` ⇒ **仍然白色** ✗
    （实测：`model.sdf` 加了 pbr、sha256 换成 `9401e1060e64339c` ✓ 且确实被加载 ✓，
     ★ 而俯视图里路面/广场**照样白** ✗ ⇒ 此路不通 ✓）
  ⇒ ★ 兜底：**把材质写进 DAE 自己** ✓ —— 这是 COLLADA 的正路，
    ★ Classic 与 Fortress **都认** ✓（mesh 自带材质时两边行为一致 ✓）。

★ 补四样（COLLADA 1.4 的元素顺序：effects → geometries → materials → visual_scenes ✓）：
  ① `<library_effects>`   插在 `<library_geometries>` **之前** ✓
  ② `<library_materials>` 插在 `</library_geometries>` **之后** ✓
  ③ `<triangles>` 上加 `material="<id>"` 属性 ✓
     （★ 不重写 `<p>` 索引表 ✓ —— 没有 MATERIAL 输入时，Assimp 用 `material` 属性这个符号 ✓，
      ✗ 重写 758 个三角形的交织索引表只会引入新错误 ✗）
  ④ `<instance_geometry>` 里补 `<bind_material>` ✓（`symbol` 与 ③ 的符号**同名** ✓ 才配得上 ✓）

★ 用法约束：**只输出到新文件** ✓（✗ 不原地改 ✗）；`--check` 只做检查不改 ✓。
  已含 `library_effects` 的**拒绝处理** ✓（✗ 防重复注入 ✗）。
"""
import re
import sys


def build(color, mid):
    r, g, b = color
    c = '%.6f %.6f %.6f 1' % (r, g, b)
    fx = (
        '<library_effects><effect id="%s_fx"><profile_COMMON><technique sid="common">'
        '<lambert>'
        '<emission><color>0 0 0 1</color></emission>'
        '<ambient><color>%s</color></ambient>'
        '<diffuse><color>%s</color></diffuse>'
        '</lambert>'
        '</technique></profile_COMMON></effect></library_effects>' % (mid, c, c)
    )
    mat = (
        '<library_materials><material id="%s" name="%s">'
        '<instance_effect url="#%s_fx"/></material></library_materials>' % (mid, mid, mid)
    )
    bind = (
        '<bind_material><technique_common><instance_material symbol="%s" target="#%s">'
        '</instance_material></technique_common></bind_material>' % (mid, mid)
    )
    return fx, mat, bind


def main():
    a = sys.argv[1:]
    if len(a) < 5:
        print(__doc__)
        return 2
    src, dst = a[0], a[1]
    color = tuple(float(v) for v in a[2:5])
    mid = a[a.index('--mat-id') + 1] if '--mat-id' in a else 'surface_mat'

    s = open(src, encoding='utf-8').read()
    print('输入: %s（%d B）' % (src, len(s.encode())))
    if '<library_effects' in s:
        print('✗ 已经有 <library_effects> ⇒ 拒绝重复注入 ✗')
        return 1
    if '<material' in s:
        print('✗ 已经有 <material ⇒ 拒绝（本工具只处理"零材质"的 mesh）✗')
        return 1

    fx, mat, bind = build(color, mid)

    # ① effects 在 geometries 之前
    anchor = '<library_geometries>'
    if s.count(anchor) != 1:
        print('✗ 锚点 %r 出现 %d 次（应为 1）' % (anchor, s.count(anchor)))
        return 1
    s = s.replace(anchor, fx + anchor, 1)

    # ② materials 在 geometries 之后、visual_scenes 之前
    anchor = '</library_geometries>'
    if s.count(anchor) != 1:
        print('✗ 锚点 %r 出现 %d 次（应为 1）' % (anchor, s.count(anchor)))
        return 1
    s = s.replace(anchor, anchor + mat, 1)

    # ③ <triangles ...> 补 material 符号
    m = re.search(r'<triangles\b([^>]*)>', s)
    if not m or ' material=' in m.group(1):
        print('✗ 找不到唯一的 <triangles> 或无 material 属性')
        return 1
    s = s[:m.start()] + '<triangles%s material="%s">' % (m.group(1), mid) + s[m.end():]

    # ④ instance_geometry 补 bind_material（自闭合与非自闭合两种写法都支持 ✓）
    for pat in (r'<instance_geometry url="#([^"]+)"/>',
                r'<instance_geometry url="#([^"]+)"></instance_geometry>'):
        m = re.search(pat, s)
        if m:
            s = s[:m.start()] + '<instance_geometry url="#%s">%s</instance_geometry>' \
                % (m.group(1), bind) + s[m.end():]
            break
    else:
        print('✗ 找不到 <instance_geometry …/>')
        return 1

    # ★ 回读校验 ✓（写之前先验，✗ 不"写完就当成了" ✗）
    for need in (fx, mat, bind, 'material="%s"' % mid):
        if need not in s:
            print('✗ 校验失败：补进去的片段回读不到 ⇒ %s' % need[:60])
            return 1
    import xml.etree.ElementTree as ET
    try:
        ET.fromstring(s)
    except ET.ParseError as e:
        print('✗ 生成的 XML 解析不过：%s' % e)
        return 1

    if '--check' in a:
        print('★ --check：只验不改 ✓（没写 %s）' % dst)
        return 0
    open(dst, 'w', encoding='utf-8').write(s)
    print('★ 写好了 %s（%d B）｜ 材质 id=%s 颜色=%s'
          % (dst, len(s.encode()), mid, ' '.join('%.2f' % v for v in color)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

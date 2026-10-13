#!/usr/bin/env python3
"""add_dae_normals.py —— 给**没有法线**的、纯平（z 恒定）的 COLLADA 补上平法线

用法: add_dae_normals.py <in.dae> <out.dae> [--z 1.0] [--check]

★ 为什么（2026-10-10 阶段八·新任务第 1 步 ✓）：
  ★ outdoorNav 的 `roads.dae` / `plaza.dae` **只有 POSITION** ✓（实测：无 NORMAL 源 ✓）
  ★ 而它们在 Fortress 里渲染成 **纯白 255,255,255 且不受光** ✗
    —— ★ 已排掉三条路（都实测过 ✓）：
      ① SDF `<material><ambient>/<diffuse>`（原样）⇒ 白 ✗
      ② SDF `<material><pbr><metal><albedo>`        ⇒ 白 ✗
      ③ DAE 自带 `<library_effects>/<library_materials>/<bind_material>` ⇒ 仍白 ✗
      ④ ★ 连"把 SDF 的 `<material>` 整块删掉、只留 DAE 材质" ⇒ 仍白 ✗
    ⇒ ★ 剩下最可疑的一条：**mesh 没有法线** ⇒ 渲染器点不着灯 ⇒ 退回"不受光的白" ✗
  ⇒ 本工具给它们补上平法线（★ 两个 mesh 都在 z=0 平面上 ⇒ 法线恒为 ±Z ✓，
    `+Z` 朝上 ✓ 与地面一致 ✓）。

★ 补三样：
  ① `<source id="normals">`（每顶点一个 `0 0 <z>` ✓）插在 `<vertices>` **之前** ✓
  ② `<vertices>` 里加 `<input semantic="NORMAL" source="#normals"/>` ✓
     （★ 有了它 `<triangles>` **不用**再加法线输入 ✓ —— 这正是常见导出器的写法 ✓）
  ③ 不做其它改动 ✓

★ 与 `add_dae_material.py` 一样：**只输出到新文件** ✓，已含法线的**拒绝** ✓，`--check` 只验不改 ✓。
"""
import re
import sys
import xml.etree.ElementTree as ET


def main():
    a = sys.argv[1:]
    if len(a) < 2:
        print(__doc__)
        return 2
    src, dst = a[0], a[1]
    z = float(a[a.index('--z') + 1]) if '--z' in a else 1.0

    s = open(src, encoding='utf-8').read()
    print('输入: %s（%d B）' % (src, len(s.encode())))
    if 'semantic="NORMAL"' in s:
        print('✗ 已经有 NORMAL ⇒ 拒绝重复注入 ✗')
        return 1

    m = re.search(r'<float_array[^>]*id="[^"]*"[^>]*count="(\d+)"', s)
    if not m:
        print('✗ 找不到位置 float_array')
        return 1
    cnt = int(m.group(1))
    if cnt % 3:
        print('✗ count=%d 不是 3 的倍数' % cnt)
        return 1
    n = cnt // 3
    print('  顶点数 = %d' % n)

    arr = ' '.join(['0 0 %g' % z] * n)
    src_block = (
        '<source id="normals"><float_array id="normals-array" count="%d">%s</float_array>'
        '<technique_common><accessor source="#normals-array" count="%d" stride="3">'
        '<param name="X" type="float"/><param name="Y" type="float"/>'
        '<param name="Z" type="float"/></accessor></technique_common></source>'
        % (cnt, arr, n)
    )

    anchor = '<vertices'
    if s.count(anchor) != 1:
        print('✗ 锚点 %r 出现 %d 次（应为 1）' % (anchor, s.count(anchor)))
        return 1
    s = s.replace(anchor, src_block + anchor, 1)

    m = re.search(r'(<vertices\b[^>]*>)(<input[^>]*semantic="POSITION"[^>]*/>)', s)
    if not m:
        print('✗ 找不到 <vertices> 里的 POSITION 输入')
        return 1
    s = s[:m.end(2)] + '<input semantic="NORMAL" source="#normals"/>' + s[m.end(2):]

    if 'semantic="NORMAL"' not in s or src_block not in s:
        print('✗ 校验失败：补进去的片段回读不到')
        return 1
    try:
        ET.fromstring(s)
    except ET.ParseError as e:
        print('✗ 生成的 XML 解析不过：%s' % e)
        return 1

    if '--check' in a:
        print('★ --check：只验不改 ✓（没写 %s）' % dst)
        return 0
    open(dst, 'w', encoding='utf-8').write(s)
    print('★ 写好了 %s（%d B）｜ 平法线 0 0 %g × %d' % (dst, len(s.encode()), z, n))
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""world_collider_inventory.py —— ★ 一个世界的**碰撞体清册**（名字／位姿／形状／尺寸／高度 ✓）

用法: world_collider_inventory.py <sdf路径> --out <json> [--root-pose x,y,z]

★★ 为什么（reviewer 2026-10-11 (b)② ✓）★★
  ★ 要回答"**他们的场景里多出来的是什么**" ⇒ 先得把两个世界的**碰撞体**逐项列出来 ✓
  （★ 只看 `<collision>` ✓ —— ★ `<visual>` 不参与物理、也扫不到 ✓，本项目已记过 ✓）。
★ 口径（写死 ✓）：
  · 位姿**逐级合成**：model pose ＋ link pose ＋ collision pose ✓（SDF 的语义 ✓）
  · 尺寸：box → (sx,sy,sz) ✓；cylinder → (2r,2r,length) ✓；sphere → (2r,2r,2r) ✓；mesh → ✗ **标成 unknown** ✓
    （★ mesh 的尺寸**不许猜** ✗ —— 本项目的"元数据一律实测" ✓）
  · 输出**每个碰撞体的世界 AABB**（x0,y0,z0,x1,y1,z1 ✓）＋ **底面高度 z0** 与 **高度 dz** ✓
  · 再按 (形状, 尺寸档, 高度档) 归一个**类** ✓，便于两台机器逐类对差 ✓
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET


def f3(s, default=(0.0, 0.0, 0.0)):
    if not s:
        # ★ 坑（2026-10-11 实测 ✗）：默认值只有 3 个数 ⇒ 后面按 6 个用会 IndexError ✗
        v = list(default)
    else:
        v = [float(x) for x in s.split()]
    while len(v) < 6:
        v.append(0.0)
    return v


def rot_apply(p, rpy):
    """★ 只处理绕 z 的旋转 ＋ 平移（本场景里的位姿 rpy 多为 0 或绕 z ✓）——
    ★ 若 rpy 含绕 x/y 的旋转 ⇒ 标记 `rotated_xy=true` ✓（✗ 不假装 AABB 精确 ✗）。"""
    import math
    x, y, z = p
    r, pp, yy = rpy[0], rpy[1], rpy[2]
    c, s = math.cos(yy), math.sin(yy)
    return x * c - y * s, x * s + y * c, z, (abs(r) > 1e-9 or abs(pp) > 1e-9)


def walk(el, pose_acc, out, path):
    """★ 递归走 model/link/collision ✓（也吃 `<include>` 之外的普通结构 ✓）"""
    tag = el.tag
    if tag in ('model', 'link', 'collision'):
        p = f3(el.findtext('pose'))
        acc = [pose_acc[i] + p[i] for i in range(3)] + [pose_acc[i + 3] + p[i + 3] for i in range(3)]
        nm = el.get('name') or '?'
        path2 = path + '/' + nm
        if tag == 'collision':
            g = el.find('geometry')
            shape, size, unknown = None, None, False
            if g is not None:
                for ch in g:
                    if ch.tag == 'box':
                        size = [float(v) for v in (ch.findtext('size') or '0 0 0').split()]
                        shape = 'box'
                    elif ch.tag == 'cylinder':
                        r = float(ch.findtext('radius') or 0)
                        l = float(ch.findtext('length') or 0)
                        size = [2 * r, 2 * r, l]; shape = 'cylinder'
                    elif ch.tag == 'sphere':
                        r = float(ch.findtext('radius') or 0)
                        size = [2 * r, 2 * r, 2 * r]; shape = 'sphere'
                    elif ch.tag == 'mesh':
                        shape, unknown = 'mesh', True
                        size = None
            cx, cy, cz, rot = rot_apply(acc[:3], acc[3:])
            rec = {'path': path2, 'shape': shape, 'size': size, 'unknown_size': unknown,
                   'center': [round(cx, 4), round(cy, 4), round(cz, 4)], 'rotated_xy': rot}
            if size:
                sx, sy, sz = size
                rec['aabb'] = [round(cx - sx / 2, 4), round(cy - sy / 2, 4), round(cz - sz / 2, 4),
                               round(cx + sx / 2, 4), round(cy + sy / 2, 4), round(cz + sz / 2, 4)]
                rec['z0'] = round(cz - sz / 2, 4)
                rec['dz'] = round(sz, 4)
                rec['footprint_m2'] = round(sx * sy, 4)
                # ★ 归类（供两台机器逐类对差 ✓）
                if sz >= 5.0:
                    cls = 'building'
                elif sz <= 0.6 and sx <= 0.6 and sy <= 0.6:
                    cls = 'thin-low (树/杆/柱)'
                elif sz <= 1.2 and max(sx, sy) <= 3.0:
                    cls = 'low-furniture (长椅/花坛)'
                elif rec['z0'] < -0.05:
                    cls = 'ground-slab'
                else:
                    cls = 'other'
                rec['class'] = cls
            out.append(rec)
        for ch in el:
            walk(ch, acc, out, path2)
    elif tag in ('world', 'sdf'):
        for ch in el:
            walk(ch, [0.0] * 6, out, '')
    else:
        # ★ 兜底：别的标签（`<include>` 等）**不下钻** ✗ 但也要把子节点走一遍 ✓
        for ch in el:
            walk(ch, pose_acc, out, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sdf')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    root = ET.parse(a.sdf).getroot()
    out = []
    walk(root, [0.0] * 6, out, '')
    # 汇总
    byc = {}
    for r in out:
        k = r.get('class') or ('unknown-size' if r['unknown_size'] else '?')
        byc.setdefault(k, []).append(r)
    summary = {k: {'n': len(v),
                   'examples': [{'path': x['path'], 'size': x.get('size'), 'z0': x.get('z0'),
                                 'dz': x.get('dz')} for x in v[:4]]}
               for k, v in sorted(byc.items(), key=lambda kv: -len(kv[1]))}
    doc = {'sdf': os.path.abspath(a.sdf), 'n_collisions': len(out),
           'n_with_known_size': sum(1 for r in out if not r['unknown_size']),
           'classes': summary, 'collisions': out,
           'convention': ('★ 只取 `<collision>` ✓（visual 不参与物理 ✗）；位姿逐级合成 ✓；'
                          '尺寸只对 box/cylinder/sphere 给 ✗ mesh 一律 unknown（✗ 不猜 ✓）')}
    json.dump(doc, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ %s ⇒ 碰撞体 %d 个（已知尺寸 %d）' % (os.path.basename(a.sdf), len(out), doc['n_with_known_size']))
    for k, v in summary.items():
        print('     %-26s %3d 个' % (k, v['n']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

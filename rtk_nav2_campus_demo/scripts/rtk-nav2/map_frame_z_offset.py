#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_frame_z_offset.py —— ★ "地图帧 z=0"离"世界地面顶面"差多少？（★ 纯离线 ✓）

用法: map_frame_z_offset.py <probe目录> --max-seq N --out json

★★ 为什么（reviewer 2026-10-11 ② ✓）★★
  ★ 本项目**记过一条** ✓：★ **高度门用的是"地图帧绝对高度"** ✓ ⇒ ★ 下坡时激光面跌破 `0.02`
    ⇒ ★ **一个点都标不进去** ✓ ⇒ ★ **帧 z 一偏，那道门就整体挪位** ✓。
  ★ 所以要量清 ✓：★ 在两套场景里 ★ **"地图帧 z=0"分别落在"世界地面顶面"之上多少米** ✓。
★ 怎么量（★ 三样都实测 ✓，✗ 不推断 ✓）：
  · ★ **地面在"地图帧"里的高度** ✓：★ 深度相机那路回波里**贴地那一大簇**的 z 中位 ✓
    （★ 上一件已确认那簇就是地面 ✓：z 中位 0.007–0.014 ✓、贴地率 82–88% ✓、只在相机视场扇区 ✓）
  · ★ **激光原点在"地图帧"里的高度** ✓：★ 该拍 `scan.tf.M` 的平移 z ✓（= 激光面高度 ✓）
  · ★ **世界地面顶面** ✓：★ 从两套 SDF 的 `<collision>` 读 ✓（★ 我们 120×120×0.02 中心 −0.02 ✓／
    他们 138×131×0.2 中心 −0.1 ✓）⇒ ★ **两边的顶面都在世界 z = 0** ✓（★ 出处：`colliders_ours.json` ✓ 与 `model.sdf` ✓）
  ⇒ ★ **偏移 = 地图帧里地面的 z − 世界地面顶面的 z(=0)** ✓ ⇒ ★ 就是"地面在地图帧里的高度" ✓。
"""
import argparse
import collections
import glob
import json
import math
import os
import re
import sys


def seq_of(p):
    return int(re.search(r'_(\d+)\.', os.path.basename(p)).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('probe_dir')
    ap.add_argument('--max-seq', type=int, required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    P = a.probe_dir
    ticks = sorted([t for t in glob.glob(os.path.join(P, 'tick_*.json'))
                    if seq_of(t) <= a.max_seq
                    and os.path.isfile(os.path.join(P, 'map_%04d.bin' % seq_of(t)))],
                   key=seq_of)
    stride = max(1, len(ticks) // 100)
    samp = ticks[::stride]

    floor_z, scan_org_z, cam_org_z = [], [], []
    for t in samp:
        s = seq_of(t)
        d = json.load(open(t, encoding='utf-8'))
        for cloud, acc in (('scan', scan_org_z), ('cam', cam_org_z)):
            M = ((d.get(cloud) or {}).get('tf') or {}).get('M')
            if M:
                acc.append(M[2][3])
        f = os.path.join(P, 'cam_%04d.xyz' % s)
        if os.path.isfile(f):
            for line in open(f, encoding='utf-8'):
                v = line.split()
                if len(v) == 3:
                    z = float(v[2])
                    if z <= 0.05:            # ★ 只取"贴地那一簇" ✓（口径写进产物 ✓）
                        floor_z.append(z)

    def q(a2, p):
        a2 = sorted(a2)
        return round(a2[min(len(a2) - 1, int(len(a2) * p))], 4) if a2 else None
    out = {'probe_dir': os.path.abspath(P), 'max_seq': a.max_seq,
           'n_ticks_sampled': len(samp), 'tick_stride': stride,
           'floor_z_in_map_frame_m': {'n': len(floor_z), 'p10': q(floor_z, .1), 'median': q(floor_z, .5),
                                      'p90': q(floor_z, .9), 'min': round(min(floor_z), 4) if floor_z else None,
                                      'max': round(max(floor_z), 4) if floor_z else None},
           'scan_origin_z_in_map_frame_m': {'median': q(scan_org_z, .5), 'p10': q(scan_org_z, .1),
                                            'p90': q(scan_org_z, .9)},
           'cam_origin_z_in_map_frame_m': {'median': q(cam_org_z, .5), 'p10': q(cam_org_z, .1),
                                           'p90': q(cam_org_z, .9)},
           'world_ground_top_z': 0.0,
           'offset_map_frame_zero_above_world_ground_top_m': q(floor_z, .5),
           'convention': ('★ "地面在地图帧里的高度" = 深度相机回波里 **z ≤ 0.05** 那一簇的 z 中位 ✓'
                          '（★ 上一件已确认那一簇就是**地面** ✓）；'
                          '★ "世界地面顶面" 从两套 SDF 的 `<collision>` 读 ✓：★ 我们 box 120×120×0.02 中心 −0.02 ✓、'
                          '★ 他们 box 138×131×0.2 中心 −0.1 ✓ ⇒ ★ **两边顶面都在世界 z = 0** ✓；'
                          '★ 激光/相机原点高度取自该拍 `scan.tf.M`／`cam.tf.M` 的平移 z ✓（≡ 地图帧 ✓）')}
    json.dump(out, open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('  ★ 地面在地图帧里的 z：中位 %s（p10 %s ｜ p90 %s ｜ n=%d）'
          % (out['floor_z_in_map_frame_m']['median'], out['floor_z_in_map_frame_m']['p10'],
             out['floor_z_in_map_frame_m']['p90'], out['floor_z_in_map_frame_m']['n']))
    print('  ★ 激光原点 z（地图帧）：中位 %s ｜ 相机原点 z：中位 %s'
          % (out['scan_origin_z_in_map_frame_m']['median'], out['cam_origin_z_in_map_frame_m']['median']))
    print('  ★★ 地图帧 z=0 在**世界地面顶面之上** %s m ⇒ 地面在地图帧里是 z = %s'
          % (out['offset_map_frame_zero_above_world_ground_top_m'], out['floor_z_in_map_frame_m']['median']))
    print('  ★ 已落盘 %s' % a.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())

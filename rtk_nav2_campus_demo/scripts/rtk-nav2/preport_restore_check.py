#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""preport_restore_check.py —— ★ 「接入前」复原的两头判据 ＋ contemporaneous 交叉核 ✓

用法: preport_restore_check.py <主工作区树> <复原副本树> <输出 json>

★★ 判据（reviewer 2026-10-11 指定 ✓，★ 都要实测 ✓）★★
  (i)   ★ datum 与 world 的球坐标**逐字段**比 ✓（★ 两份文件各读一次 ✓）
  (ii)  ★ 航线**全部点落在栅格内** ✓（★ 逐点算格号 ✓）
  (iii) ★ 与主工作区的差异 = **只有那两处** ✓（★ 逐文件 sha256 清单逐行比 ✓）
  (iii-b) ★ contemporaneous 交叉核 ✓：`docs/campus-8-delivery.md` §3 那 13 项
          —— ★ 拿**复原后的** campus_nav2.yaml 去对 ✓；★ 对不上就**报出来** ✗

★ 判词规矩 ✓：★ 逐字段比要**把两边都打出来** ✓（✗ 不写"一致/不一致"这种没有数的话 ✗）；
  ★ "取不到"与"缺失"分开 ✓。
"""
import hashlib
import json
import os
import re
import sys

import yaml

# ★ 航线 10 点：取自 cut1 **实跑日志** stream_runner.log 的 map=(x, y) ✓（✗ 不是推算 ✗）
ROUTE_MAP = [
    ('start', 2.687, -30.026), ('wp0', -3.923, -26.033), ('wp1', -10.932, -22.040),
    ('wp2', -17.945, -15.546), ('wp3', -23.957, -9.551), ('wp4', -33.472, -3.060),
    ('wp5', -32.480, 3.445), ('wp6', -34.991, 9.946), ('wp7', -37.501, 16.446),
    ('goal', -40.014, 24.948),
]

# ★ docs/campus-8-delivery.md §3 那 13 项的**底本值** ✓（★ 逐字抄自文档表 ✓）
DOC13_BASE = {
    'global_costmap.global_costmap.ros__parameters.resolution': 0.05,
    'global_costmap.global_costmap.ros__parameters.width': 40,
    'global_costmap.global_costmap.ros__parameters.height': 40,
    'global_costmap.global_costmap.ros__parameters.origin_x': -20.0,
    'global_costmap.global_costmap.ros__parameters.origin_y': -20.0,
    'global_costmap.global_costmap.ros__parameters.plugins': ['obstacle_layer', 'inflation_layer'],
    'global_costmap.global_costmap.ros__parameters.inflation_layer.inflate_unknown': False,
    'controller_server.ros__parameters.FollowPath.desired_linear_vel': 0.16,
}
DOC13_CAMPUS = {
    'global_costmap.global_costmap.ros__parameters.resolution': 0.10,
    'global_costmap.global_costmap.ros__parameters.width': 120,
    'global_costmap.global_costmap.ros__parameters.height': 120,
    'global_costmap.global_costmap.ros__parameters.origin_x': -60.0,
    'global_costmap.global_costmap.ros__parameters.origin_y': -60.0,
    'global_costmap.global_costmap.ros__parameters.plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
    'global_costmap.global_costmap.ros__parameters.inflation_layer.inflate_unknown': True,
    'controller_server.ros__parameters.FollowPath.desired_linear_vel': 0.5,
}
DOC13_STATIC5 = [
    'global_costmap.global_costmap.ros__parameters.static_layer.plugin',
    'global_costmap.global_costmap.ros__parameters.static_layer.enabled',
    'global_costmap.global_costmap.ros__parameters.static_layer.map_topic',
    'global_costmap.global_costmap.ros__parameters.static_layer.map_subscribe_transient_local',
    'global_costmap.global_costmap.ros__parameters.static_layer.subscribe_to_updates',
]


def flat(d, p=''):
    out = {}
    for k, v in (d or {}).items():
        kk = '%s.%s' % (p, k) if p else str(k)
        if isinstance(v, dict):
            out.update(flat(v, kk))
        else:
            out[kk] = v
    return out


def manifest(root):
    m = {}
    for dp, dns, fns in os.walk(root):
        dns[:] = [d for d in dns if d != '__pycache__']
        for fn in fns:
            if fn.endswith('.pyc'):
                continue
            p = os.path.join(dp, fn)
            rel = './' + os.path.relpath(p, root)
            m[rel] = hashlib.sha256(open(p, 'rb').read()).hexdigest()
    return m


def read_datum(p):
    raw = open(p, encoding='utf-8').read()
    m = re.search(r'^\s*datum:\s*\[([^\]]*)\]', raw, re.M)
    if not m:
        return None, raw
    vals = [x.strip() for x in m.group(1).split(',')]
    return vals, raw


def read_spherical(world):
    raw = open(world, encoding='utf-8').read()
    blk = re.search(r'<spherical_coordinates>(.*?)</spherical_coordinates>', raw, re.S)
    if not blk:
        return None
    out = {}
    for tag in ('latitude_deg', 'longitude_deg', 'elevation'):
        mm = re.search(r'<%s>([^<]*)</%s>' % (tag, tag), blk.group(1))
        out[tag] = mm.group(1).strip() if mm else None
    return out


def main():
    main_tree, copy_tree, out_path = sys.argv[1:4]
    res = {'main_tree': os.path.abspath(main_tree), 'copy_tree': os.path.abspath(copy_tree)}

    # ---- (i) datum vs world 球坐标，逐字段 ----
    datum, raw_datum = read_datum(os.path.join(copy_tree, 'config/navsat.yaml'))
    world = os.path.join(copy_tree, 'worlds/campus_100.sdf')
    sph = read_spherical(world)
    res['criteria_i_datum_vs_world'] = {
        'datum_file': os.path.join(copy_tree, 'config/navsat.yaml'),
        'datum_raw': ('datum: [%s]' % ', '.join(datum)) if datum else None,
        'world_file': world,
        'world_spherical': sph,
        'field_by_field': [
            {'field': 'latitude', 'datum': datum[0] if datum else None,
             'world_latitude_deg': sph.get('latitude_deg') if sph else None,
             'equal_as_text': bool(datum and sph and datum[0] == sph['latitude_deg'])},
            {'field': 'longitude', 'datum': datum[1] if datum else None,
             'world_longitude_deg': sph.get('longitude_deg') if sph else None,
             'equal_as_text': bool(datum and sph and datum[1] == sph['longitude_deg'])},
            {'field': '第三分量', 'datum': datum[2] if datum and len(datum) > 2 else None,
             'world_elevation': sph.get('elevation') if sph else None,
             'equal_as_text': bool(datum and sph and len(datum) > 2 and datum[2] == sph['elevation']),
             'note': ('★ 两个**不同的量**：navsat_transform 的 datum 第三分量与世界文件的 elevation ✗ '
                      '（navsat.yaml 里 zero_altitude: true ⇒ 高度被置零）✓。'
                      '★ 实测：接入前那份 navsat.yaml **原件**写的就是 0.0 '
                      '（rtk_nav2_ws_campus9-cut0/…/navsat.yaml:27 ✓，cut0 的树就是接入前那份），'
                      '而 docs/campus-8-delivery.md:261 写的是 30.2276 / 120.27 / **4.0** ✗ '
                      '⇒ 两处记录**分歧** ✓，本复原按**原件**（0.0）✓，分歧原样报出 ✗'),
             },
        ],
    }

    # ---- (ii) 航线点是否都在栅格内 ----
    camp = yaml.safe_load(open(os.path.join(copy_tree, 'config/campus_nav2.yaml'), encoding='utf-8'))
    gc = camp['global_costmap']['global_costmap']['ros__parameters']
    # ★ width/height 的单位是**米** ✗ 不是格 ✓（配置注释与实测都如此：120 m @0.10 = 1200 格 ✓）
    #   ⇒ ★ 格数是 width/resolution ✓ —— ★ 第一次就是栽在这里（拿 120 当格数 ⇒ 十个点全"出界" ✗）
    W_m, H_m = float(gc['width']), float(gc['height'])
    res_x = float(gc['resolution'])
    ox, oy = float(gc['origin_x']), float(gc['origin_y'])
    W, H = int(round(W_m / res_x)), int(round(H_m / res_x))
    rows = []
    for name, x, y in ROUTE_MAP:
        ix = int((x - ox) // res_x)
        iy = int((y - oy) // res_x)
        rows.append({'name': name, 'x': x, 'y': y, 'ix': ix, 'iy': iy,
                     'in_grid': bool(0 <= ix < W and 0 <= iy < H)})
    res['criteria_ii_route_in_grid'] = {
        'grid': {'width_m': W_m, 'height_m': H_m, 'resolution': res_x,
                 'origin_x': ox, 'origin_y': oy,
                 'width_cells': W, 'height_cells': H,
                 'unit_note': '★ width/height 的单位是米（120 m @0.10 ⇒ 1200 格）—— 拿 120 当格数会把全部点误判成出界 ✗'},
        'route_source': ('★ 10 个 map 坐标取自 cut1 **实跑日志** stream_runner.log 的 map=(x, y) ✓'
                         '（✗ 不是推算 ✓）'),
        'points': rows,
        'n_in_grid': sum(1 for r in rows if r['in_grid']),
        'n_total': len(rows),
    }

    # ---- (iii) 与主工作区逐文件比 ----
    m0, m1 = manifest(main_tree), manifest(copy_tree)
    only_main = sorted(set(m0) - set(m1))
    only_copy = sorted(set(m1) - set(m0))
    changed = sorted(k for k in set(m0) & set(m1) if m0[k] != m1[k])
    res['criteria_iii_diff_vs_main'] = {
        'n_files_main': len(m0), 'n_files_copy': len(m1),
        'changed': [{'path': k, 'main_sha256_16': m0[k][:16], 'copy_sha256_16': m1[k][:16]}
                    for k in changed],
        'only_in_main': only_main, 'only_in_copy': only_copy,
        'verdict': ('仅有这 %d 个文件不同 ✓' % len(changed)) if (len(changed) == 2 and not only_main and not only_copy)
                   else ('★ 不是"只有两处" ✗：changed=%d / only_main=%d / only_copy=%d'
                         % (len(changed), len(only_main), len(only_copy))),
        'manifest_sha256_copy': None,
    }
    # ★ 清单哈希（与 cut_provision.json 记的那份同算法 ✓）
    lines = []
    for rel in sorted(m1):
        lines.append('%s  %s\n' % (m1[rel], rel))
    blob = ''.join(lines).encode('utf-8')
    res['criteria_iii_diff_vs_main']['manifest_sha256_copy'] = hashlib.sha256(blob).hexdigest()

    # ---- (iii-b) contemporaneous 13 项交叉核 ----
    base = flat(yaml.safe_load(open(os.path.join(copy_tree, 'config/nav2_params.yaml'), encoding='utf-8')))
    now = flat(yaml.safe_load(open(os.path.join(copy_tree, 'config/campus_nav2.yaml'), encoding='utf-8')))
    items = []
    for k, want in DOC13_CAMPUS.items():
        got = now.get(k, '<无>')
        bl = base.get(k, '<无>')
        items.append({'key': k, 'doc_campus': want, 'restored': got, 'baseline_doc_says': DOC13_BASE.get(k),
                      'matches_doc_campus': got == want,
                      'baseline_matches_doc': bl == DOC13_BASE.get(k)})
    for k in DOC13_STATIC5:
        items.append({'key': k, 'doc_campus': '（5 键，文档记为新增）', 'restored': now.get(k, '<无>'),
                      'baseline_doc_says': '<无>',
                      'matches_doc_campus': k in now, 'baseline_matches_doc': k not in base})
    res['criteria_iii_b_contemporaneous_13'] = {
        'doc': 'docs/campus-8-delivery.md §3',
        'items': items,
        'n_match': sum(1 for i in items if i['matches_doc_campus']),
        'n_total': len(items),
        'mismatch': [i['key'] for i in items if not i['matches_doc_campus']],
    }

    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print('wrote %s' % out_path)

    # ---- 打印 ----
    print('\n★ (i) datum vs world 逐字段：')
    for r in res['criteria_i_datum_vs_world']['field_by_field']:
        print('   %-8s datum=%-28s world=%-12s 一致=%s'
              % (r['field'], r.get('datum'), r.get('world_latitude_deg') or r.get('world_longitude_deg')
                 or r.get('world_elevation'), r['equal_as_text']))
    print('\n★ (ii) 航线点：%d/%d 在图内' % (res['criteria_ii_route_in_grid']['n_in_grid'],
                                          res['criteria_ii_route_in_grid']['n_total']))
    bad = [r for r in rows if not r['in_grid']]
    print('   出界点：%s' % (bad if bad else '无 ✓'))
    print('\n★ (iii) 与主工作区：%s' % res['criteria_iii_diff_vs_main']['verdict'])
    for c in res['criteria_iii_diff_vs_main']['changed']:
        print('   %s  %s → %s' % (c['path'], c['main_sha256_16'], c['copy_sha256_16']))
    print('\n★ (iii-b) 13 项交叉核：对得上 %d/%d' % (res['criteria_iii_b_contemporaneous_13']['n_match'],
                                                  res['criteria_iii_b_contemporaneous_13']['n_total']))
    for i in items:
        flag = '✓' if i['matches_doc_campus'] else '✗'
        print('   %s %-64s 文档=%-46s 复原=%s' % (flag, i['key'], str(i['doc_campus'])[:46], str(i['restored'])[:34]))


if __name__ == '__main__':
    main()

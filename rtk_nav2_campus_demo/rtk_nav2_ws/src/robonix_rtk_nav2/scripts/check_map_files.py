#!/usr/bin/env python3
"""**纯文件**读回 nav2 map_saver 存下来的 pgm+yaml，并与实时快照的元数据/计数对比（判据 5）。

为什么要"纯文件"：判据是「干净进程读回后尺寸/原点/分辨率一致」——如果读回这一步
又去连 ROS，验的就是"话题还在不在"，不是"存下来的文件对不对"。这个脚本**不 import rclpy**，
只按 map_server 的规范解析 yaml + P5 pgm，所以它跑在什么环境里都一样。

判据（三条一起才算通过）：
  1. 分辨率、像素尺寸、原点 与实时快照一致（`--live-json` 来自 `nav2_probe.py stats`）；
  2. 三分类计数（占据/空闲/未知）与实时快照一致；
  3. 能说明编码：给出像素值直方图，并说明分类用的阈值来自 yaml 自己
     （nav2 map_saver 默认 occupied_thresh 0.65 / free_thresh 0.196，未知存成 205）。

用法：
  check_map_files.py --yaml <x>.yaml [--live-json <stats.json>] [--json 结果.json] [--png 可视化.png]
"""

import argparse
import json
import os
import sys

import numpy as np


def read_pgm(path):
    """读二进制 P5 pgm（map_saver 输出的就是 P5）。返回 (array, maxval)。"""
    with open(path, 'rb') as handle:
        raw = handle.read()
    # 头部：magic、宽、高、maxval，各项之间可能有注释行
    tokens = []
    i = 0
    while len(tokens) < 4:
        # 跳过空白
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while i < len(raw) and raw[i:i + 1] != b'\n':
                i += 1
            continue
        start = i
        while i < len(raw) and not raw[i:i + 1].isspace():
            i += 1
        tokens.append(raw[start:i])
    if tokens[0] != b'P5':
        raise ValueError('不是二进制 P5 pgm：%r' % tokens[0])
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1                      # 头部与数据之间恰好一个空白字符
    dtype = np.uint8 if maxval < 256 else '>u2'
    data = np.frombuffer(raw, dtype=dtype, count=width * height, offset=i)
    return data.reshape(height, width), maxval


def read_yaml(path):
    """只解析 map_server yaml 里我们需要的几个键（不引入 yaml 依赖之外的东西）。"""
    try:
        import yaml
        with open(path, encoding='utf-8') as handle:
            return yaml.safe_load(handle)
    except ImportError:
        out = {}
        with open(path, encoding='utf-8') as handle:
            for line in handle:
                line = line.split('#')[0].strip()
                if not line or ':' not in line:
                    continue
                key, _, value = line.partition(':')
                value = value.strip()
                try:
                    value = float(value)
                except ValueError:
                    pass
                out[key.strip()] = value
        return out


def main(argv=None):
    parser = argparse.ArgumentParser(description='纯文件读回 pgm+yaml 并与实时快照对比')
    parser.add_argument('--yaml', required=True)
    parser.add_argument('--live-json', default='')
    parser.add_argument('--json', default='')
    parser.add_argument('--png', default='')
    args = parser.parse_args(argv)

    meta = read_yaml(args.yaml)
    image = meta.get('image')
    if not image:
        print('** yaml 里没有 image 键 **', file=sys.stderr)
        return 1
    pgm_path = image if os.path.isabs(image) else os.path.join(
        os.path.dirname(os.path.abspath(args.yaml)), image)
    if not os.path.exists(pgm_path):
        print('** 找不到 %s **' % pgm_path, file=sys.stderr)
        return 1

    data, maxval = read_pgm(pgm_path)
    negate = int(meta.get('negate', 0))
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))

    # **方向别搞反**：negate=0 时 occ = (255 - pixel)/255。
    # 这条用 nav2 自己的输出反推验证过：它把占据存成 0、空闲存成 254、未知存成 205，
    # 配上它写进 yaml 的 occupied_thresh 0.65 / free_thresh 0.25，
    # 只有 occ=(255-p)/255 才能让这三类各自落回自己的档（1.0 / 0.004 / 0.196）。
    # 写成 p/255 的话 205 会算成 0.80 > 0.65 —— 整张图会被判成"几乎全是障碍"，
    # 而"计数一致"这条判据会以看起来很像真问题的形式失败（第一次就是这么错的）。
    occ_prob = (255.0 - data) / 255.0 if not negate else data / 255.0
    occupied = occ_prob > occ_thresh
    free = occ_prob < free_thresh
    unknown = ~occupied & ~free

    values, freqs = np.unique(data, return_counts=True)
    hist = {int(v): int(c) for v, c in zip(values, freqs)}

    result = {
        'yaml': os.path.abspath(args.yaml),
        'pgm': pgm_path,
        'yaml_meta': {k: meta.get(k) for k in
                      ('image', 'resolution', 'origin', 'negate', 'occupied_thresh',
                       'free_thresh', 'mode')},
        'pgm_shape': [int(data.shape[1]), int(data.shape[0])],   # (宽, 高)
        'pgm_maxval': int(maxval),
        'pixel_histogram': hist,
        'counts': {'occupied': int(occupied.sum()), 'free': int(free.sum()),
                   'unknown': int(unknown.sum()),
                   'cells': int(data.size)},
        'note': '分类用 yaml 自己的阈值：occ_prob = (255-p)/255 或 p/255（negate），'
                '> occupied_thresh 记占据、< free_thresh 记空闲、之间记未知。'
                'nav2 map_saver 默认把未知写成 205、空闲 254、占据 0',
    }

    checks = []
    if args.live_json:
        with open(args.live_json, encoding='utf-8') as handle:
            live = json.load(handle)
        lm = live.get('global', live).get('meta', {})
        lc = live.get('global', live).get('counts', {})
        # 实时快照用的是栅格坐标；pgm 是 (row=y, col=x)，y 需要翻转（image 顶行对应 max y）
        ok_res = abs(float(meta.get('resolution', -1)) - float(lm.get('resolution', -2))) < 1e-9
        ok_size = (data.shape[1] == lm.get('width') and data.shape[0] == lm.get('height'))
        origin = meta.get('origin') or [None, None, None]
        ok_origin = (abs(float(origin[0]) - float(lm.get('origin_x', 1e9))) < 1e-6
                     and abs(float(origin[1]) - float(lm.get('origin_y', 1e9))) < 1e-6)
        # 计数对比：y 轴翻转不影响计数，但**未知/空闲/占据的分类必须一致**
        ok_counts = (result['counts']['occupied'] == lc.get('occupied')
                     and result['counts']['free'] == lc.get('free')
                     and result['counts']['unknown'] == lc.get('unknown'))
        result['vs_live'] = {
            'live_json': args.live_json,
            'live_meta': lm, 'live_counts': lc,
            'resolution_match': ok_res, 'size_match': ok_size,
            'origin_match': ok_origin, 'counts_match': ok_counts,
            'live_occupied_vs_file': [lc.get('occupied'), result['counts']['occupied']],
            'live_free_vs_file': [lc.get('free'), result['counts']['free']],
            'live_unknown_vs_file': [lc.get('unknown'), result['counts']['unknown']],
        }
        checks = [ok_res, ok_size, ok_origin, ok_counts]

    result['all_checks_pass'] = all(checks) if checks else None
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
    if args.png:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            plt.figure(figsize=(8, 8))
            plt.imshow(data, cmap='gray', origin='upper')
            plt.title('saved global costmap (pgm)')
            plt.colorbar(fraction=0.046)
            plt.savefig(args.png, dpi=110, bbox_inches='tight')
            print('已写可视化：%s' % args.png)
        except Exception as exc:                       # 画图只是方便看，不是判据
            print('画 PNG 失败（不影响判据）：%s' % exc, file=sys.stderr)

    return 0 if result['all_checks_pass'] in (True, None) else 1


if __name__ == '__main__':
    raise SystemExit(main())

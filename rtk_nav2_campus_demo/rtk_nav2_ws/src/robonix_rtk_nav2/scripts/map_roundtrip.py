#!/usr/bin/env python3
"""**nav2 自己存、nav2 自己读**：存读一致性的权威版本（判据 5）。

为什么不能只靠自己写的 pgm 解析器：那验的是"我按我的理解解出来的对不对"。
真正要回答的是"**这张图存下来、下次当静态图读回来，还是不是同一张图**"
（brief §7 明确说这件事将来要一起想清楚）。所以这个脚本把 nav2 的 `map_server`
真的起起来，把 `map_saver` 存的 yaml 读回去，再拿读回来的 `/map` 与
① 存下来的文件、② 实时快照 三方对照。

做法：直接调 `map_server` 的 lifecycle 服务（`ChangeState`）而不是 `ros2 lifecycle set`
——CLI 走 daemon，这台机器上不可靠（第二阶段已经吃过一次）。

用法：
  map_roundtrip.py --yaml <x>.yaml [--live-json <stats.json>] [--json 结果.json]
                   [--node-name map_roundtrip_server] [--timeout 60]
"""

import argparse
import copy
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_map_files import read_pgm, read_yaml  # noqa: E402

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def counts_from_grid(msg):
    data = np.asarray(msg.data, dtype=np.int16)
    return {'occupied': int((data >= 50).sum()),
            'free': int((data == 0).sum()),
            'unknown': int((data < 0).sum()),
            'cells': int(data.size)}


def main(argv=None):
    parser = argparse.ArgumentParser(description='nav2 map_server 读回 nav2 map_saver 存的图')
    parser.add_argument('--yaml', required=True)
    parser.add_argument('--live-json', default='')
    parser.add_argument('--json', default='')
    parser.add_argument('--node-name', default='map_roundtrip_server')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args(argv)

    meta = read_yaml(args.yaml)
    yaml_dir = os.path.dirname(os.path.abspath(args.yaml))
    image = meta.get('image', '')
    pgm = image if os.path.isabs(image) else os.path.join(yaml_dir, image)
    file_data, _ = read_pgm(pgm)

    rclpy.init()
    node = Node('map_roundtrip_check')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])

    # 起一个独立命名的 map_server，避免与将来可能存在的同名节点冲突
    # **必须新开进程组**：`ros2 run` 只是个包装，真正的节点是它的子进程，
    # 对包装进程 terminate 不会杀掉节点——实测留下过三个还活着的 map_server，
    # 它们各自 publish 一个 latched `/map`，把后面做的 /map 实验全部污染成"读到别人的图"。
    # 所以这里 start_new_session=True，收尾时整组 killpg。
    proc = subprocess.Popen(
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
         '-r', '__node:=%s' % args.node_name,
         '-p', 'yaml_filename:=%s' % os.path.abspath(args.yaml),
         '-p', 'use_sim_time:=false'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    out_lines = []

    # **日志必须在后台线程里收，绝不能在主线程 readline()**。
    # 踩到的坑：主线程 `proc.stdout.readline()` 在管道没数据时会**永久阻塞**，
    # 而 Python 的 `timeout` 之所以没救回来，是因为 PEP 475 让被信号打断的系统调用
    # 自动重启——SIGTERM 递到了，但 Python 层的处理器要等 `readline` 返回才跑，
    # 于是 `timeout 120` 眼睁睁看着进程跑了 7 分钟。
    # 现在改成守护线程收日志，主线程永不读管道。
    import threading

    def _reader():
        try:
            for line in proc.stdout:
                out_lines.append(line.rstrip())
                del out_lines[:-40]
        except Exception:
            pass

    threading.Thread(target=_reader, daemon=True).start()
    # **注意**：把 `pump_log()` 换成这个后台线程时，必须同时删掉原来的两处调用点——
    # 我漏删过一次，`pump_log()` 就变成未定义名，整个回环每次都以
    # `NameError: name 'pump_log' is not defined`（line 142）崩掉，
    # 而产物里只留一个 traceback、**没有 map_roundtrip.json**。
    # 现象是"判据 5 在最终配置下没有产物"，很容易被误读成"回环不一致"或"这一步没跑"。

    # 兜底看门狗：到点直接把**整个进程组**带走，保证 map_server 不会变成孤儿
    # （孤儿会 publish 一个 latched `/map`，把后面所有 /map 相关的实验污染成"读到别人的图"）。
    def _watchdog(limit):
        def _fire():
            time.sleep(limit)
            print('** map_roundtrip 看门狗超时（%.0f s），强制退出 **' % limit, file=sys.stderr)
            try:
                os.killpg(os.getpgid(0), signal.SIGKILL)
            except Exception:
                pass
            os._exit(3)
        threading.Thread(target=_fire, daemon=True).start()

    _watchdog(args.timeout + 45)

    change = node.create_client(ChangeState, '/%s/change_state' % args.node_name)
    get_state = node.create_client(GetState, '/%s/get_state' % args.node_name)
    received = {}
    node.create_subscription(OccupancyGrid, '/map',
                             lambda m: received.setdefault('map', m), LATCHED)

    result = {'yaml': os.path.abspath(args.yaml), 'node': args.node_name}
    try:
        if not change.wait_for_service(timeout_sec=args.timeout):
            result['error'] = 'change_state 服务不可用（map_server 没起来？）'
            result['server_log'] = out_lines[-15:]
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 1

        for label, transition in (('configure', Transition.TRANSITION_CONFIGURE),
                                  ('activate', Transition.TRANSITION_ACTIVATE)):
            req = ChangeState.Request()
            req.transition.id = transition
            fut = change.call_async(req)
            end = time.time() + 20
            while not fut.done() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            ok = fut.done() and fut.result() is not None and fut.result().success
            result['%s_ok' % label] = bool(ok)
            if not ok:
                result['error'] = '%s 失败' % label
                result['server_log'] = out_lines[-15:]
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 1

        end = time.time() + 20
        while 'map' not in received and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'map' not in received:
            result['error'] = '起起来了但没收到 /map'
            result['server_log'] = out_lines[-15:]
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 1

        msg = received['map']
        loaded = counts_from_grid(msg)
        result['loaded'] = {
            'meta': {'resolution': msg.info.resolution, 'width': msg.info.width,
                     'height': msg.info.height,
                     'origin_x': msg.info.origin.position.x,
                     'origin_y': msg.info.origin.position.y,
                     'frame': msg.header.frame_id},
            'counts': loaded,
        }

        # 文件按 yaml 阈值解码后的分类（与 check_map_files.py 同一套规则）
        negate = int(meta.get('negate', 0))
        occ_p = (255.0 - file_data) / 255.0 if not negate else file_data / 255.0
        f_occ = int((occ_p > float(meta.get('occupied_thresh', 0.65))).sum())
        f_free = int((occ_p < float(meta.get('free_thresh', 0.196))).sum())
        file_counts = {'occupied': f_occ, 'free': f_free,
                       'unknown': int(file_data.size) - f_occ - f_free,
                       'cells': int(file_data.size)}
        result['file_decoded'] = file_counts

        ok_geo = (abs(float(meta.get('resolution', -1)) - msg.info.resolution) < 1e-9
                  and int(msg.info.width) == file_data.shape[1]
                  and int(msg.info.height) == file_data.shape[0])
        origin = meta.get('origin') or [0.0, 0.0, 0.0]
        ok_origin = (abs(float(origin[0]) - msg.info.origin.position.x) < 1e-6
                     and abs(float(origin[1]) - msg.info.origin.position.y) < 1e-6)
        ok_counts = (loaded['occupied'] == file_counts['occupied']
                     and loaded['free'] == file_counts['free']
                     and loaded['unknown'] == file_counts['unknown'])
        result['roundtrip'] = {
            'geometry_match': bool(ok_geo),
            'origin_match': bool(ok_origin),
            'counts_match': bool(ok_counts),
            'occupied': [file_counts['occupied'], loaded['occupied']],
            'free': [file_counts['free'], loaded['free']],
            'unknown': [file_counts['unknown'], loaded['unknown']],
        }
        if args.live_json:
            with open(args.live_json, encoding='utf-8') as fh:
                live = json.load(fh)
            lc = live.get('global', live).get('counts', {})
            result['vs_live'] = {
                'live_counts': lc,
                'loaded_vs_live': {k: [lc.get(k), loaded[k]]
                                   for k in ('occupied', 'free', 'unknown')},
                'counts_match': all(lc.get(k) == loaded[k]
                                    for k in ('occupied', 'free', 'unknown')),
            }
        result['note'] = ('file_decoded = 按 yaml 自己的阈值解码 pgm 得到的分类；'
                          'loaded = nav2 map_server 读回后发布的 /map 的分类。'
                          '两者一致才说明"存下来的文件与 nav2 的理解一致"')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if args.json:
            with open(args.json, 'w', encoding='utf-8') as fh:
                json.dump(result, fh, ensure_ascii=False, indent=2)
        return 0 if (ok_geo and ok_origin and ok_counts) else 1
    finally:
        try:
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_ACTIVE_SHUTDOWN
            fut = change.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        except Exception:
            pass
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            proc.terminate()
        try:
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                proc.kill()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

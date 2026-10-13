#!/usr/bin/env python3
"""**用实验定死 nav2 的 pgm 读取约定**：造一张已知像素值的图，让 nav2 的 map_server 读它。

为什么要做这个实验：判据 5（存读一致）在"未知格怎么存、怎么读回来"上会给出一个
很关键的结论，而这件事**只看文档和直觉会搞错**——我在这一步就先后错过两次
（一次把 negate 方向写反、一次以为 free_thresh 会决定未知档）。与其继续推，
不如造一张 8 个像素的小图，把"像素值 -> 占据/空闲/未知"的映射**测出来**。

这张图每个像素值都是刻意选的：
    0    典型"占据"编码（map_saver 存占据就用它）
    64   落在 free_thresh..occupied_thresh 之间的低段
    128  正中
    205  典型"未知"编码（map_saver 存未知用它）
    254  典型"空闲"编码

用法：pgm_convention_probe.py [--free 0.196] [--occ 0.65] [--negate 0] [--json 结果.json]
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

PIXELS = [0, 64, 128, 205, 254]


def classify(value):
    if value < 0:
        return 'unknown(-1)'
    if value >= 50:
        return 'occupied(>=50)'
    if value > 0:
        return 'inflated(1..49)'
    return 'free(0)'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--free', type=float, default=0.196)
    parser.add_argument('--occ', type=float, default=0.65)
    parser.add_argument('--negate', type=int, default=0)
    parser.add_argument('--json', default='')
    parser.add_argument('--node-name', default='pgm_convention_server')
    args = parser.parse_args(argv)

    work = tempfile.mkdtemp(prefix='pgmconv-')
    pgm = os.path.join(work, 'probe.pgm')
    yaml_path = os.path.join(work, 'probe.yaml')
    data = np.array([PIXELS], dtype=np.uint8)
    with open(pgm, 'wb') as fh:
        fh.write(b'P5\n%d 1\n255\n' % len(PIXELS))
        fh.write(data.tobytes())
    with open(yaml_path, 'w', encoding='utf-8') as fh:
        fh.write('image: %s\nmode: trinary\nresolution: 0.05\n'
                 'origin: [0.0, 0.0, 0.0]\nnegate: %d\n'
                 'occupied_thresh: %s\nfree_thresh: %s\n'
                 % (os.path.basename(pgm), args.negate, args.occ, args.free))

    rclpy.init()
    node = Node('pgm_convention_check')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=False)])
    received = {}
    node.create_subscription(OccupancyGrid, '/map',
                             lambda m: received.setdefault('map', m), LATCHED)
    # **必须新开进程组**：`ros2 run` 只是个包装，真正的节点是它的子进程，
    # 对包装进程 terminate 不会杀掉节点——实测留下过三个还活着的 map_server，
    # 它们各自 publish 一个 latched `/map`，把后面做的 /map 实验全部污染成"读到别人的图"。
    # 所以这里 start_new_session=True，收尾时整组 killpg。
    proc = subprocess.Popen(
        ['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
         '-r', '__node:=%s' % args.node_name,
         '-p', 'yaml_filename:=%s' % yaml_path,
         '-p', 'use_sim_time:=false'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    change = node.create_client(ChangeState, '/%s/change_state' % args.node_name)

    result = {'pixels': PIXELS, 'yaml': {'negate': args.negate, 'occupied_thresh': args.occ,
                                         'free_thresh': args.free}}
    try:
        if not change.wait_for_service(timeout_sec=40):
            result['error'] = 'map_server 没起来（change_state 服务不可用）'
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 1
        for label, tid in (('configure', Transition.TRANSITION_CONFIGURE),
                           ('activate', Transition.TRANSITION_ACTIVATE)):
            req = ChangeState.Request()
            req.transition.id = tid
            fut = change.call_async(req)
            end = time.time() + 20
            while not fut.done() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not (fut.done() and fut.result() and fut.result().success):
                result['error'] = '%s 失败' % label
                print(json.dumps(result, ensure_ascii=False, indent=2))
                return 1
        end = time.time() + 20
        while 'map' not in received and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'map' not in received:
            result['error'] = '没收到 /map'
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 1
        msg = received['map']
        row = np.asarray(msg.data, dtype=np.int16)[:len(PIXELS)]
        result['loaded_values'] = [int(v) for v in row]
        result['mapping'] = [
            {'pixel': int(p), 'nav2_value': int(v), 'class': classify(int(v))}
            for p, v in zip(PIXELS, row)]
        result['note'] = ('这张图只有一行 8 像素，每个像素值都是刻意选的。'
                          'mapping 就是 nav2 map_server 的读取约定——'
                          '205（map_saver 存未知用的值）读回来是什么，决定了'
                          '"存下来的图下次读回时未知区域还算不算未知"')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if args.json:
            with open(args.json, 'w', encoding='utf-8') as fh:
                json.dump(result, fh, ensure_ascii=False, indent=2)
        return 0
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
    sys.exit(main())

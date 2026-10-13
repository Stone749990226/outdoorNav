#!/usr/bin/env python3
"""★ 代价格读数 → **JSON**（reviewer 2026-10-09 要求：**"我核过"必须可复算** ✓）。

**为什么单列一个工具** ✗：先前那次的读数只落在聊天里 ✓ ⇒ reviewer **核不了**（栈已停、无法重取 ✗）
⇒ 他只能说"我信" ✗ —— 这正是我们那条"**可被复算 > 被相信**" ✓ ⇒ 把它做成**落盘 JSON** 的工具 ✓。

三样内容（都是 reviewer 点名要的 ✓）：
· **持久化有效性** ✓：`x=0.15` 的 `y=3.80/4.20/4.35/4.50/5.00/6.00` **六格**（封边块内部，车远时应当**已是致命** ✓）；
· **两条横向剖面** ✓：封边块 `y=5.00` / 落差主边 `y=0.50` ⇒ 看**有没有充气梯度** ✓
  （layer 在 inflation **之前** ⇒ 有带 ✓；keepout filter 在**之后** ⇒ 平的 ✗ —— 这一条是 `as` 那次 `plan_min_dist=0.0` 的机理 ✓）；
· **局部图同一批格** ✓ + **对齐核对** ✓：局部图是**滚动窗口** ⇒ 墙必须落在**车旁边** ✓（拿"**车正下方**那格"做对照 ✓，防"图里有、位置错" ✗）。

用法：`costmap_probe.py --json out.json [--seconds 30]`（**需要 ROS** ✓）
"""
import argparse
import json
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
               reliability=ReliabilityPolicy.RELIABLE)
TOPICS = ('/rtk/static_map', '/global_costmap/costmap', '/local_costmap/costmap')
# ★ 2026-10-09 加（reviewer 要求 ✓）：**报"合并节点有没有走回退"** ✗ ——
#   回退 = `/rtk/static_map` 当时不可用 ⇒ 节点**原样转发检测器掩膜** ⇒ **那批实心块又不进图** ✗（= 回到 ④ 之前 ✓）
#   ⇒ 若本轮 ④ 又不过，必须先能分清"**修法不够**"还是"**回退被触发**" ✓。
#   判别很便宜 ✓：**`mask_merged` 的格数** —— 带静态图 ≈ **3605** ✓、回退 ≈ **701** ✓（差一个数量级 ✓）。
MERGE_TOPICS = ('/rtk/drop/mask_merged', '/rtk/drop/mask_map')
Y_SIX = (3.80, 4.20, 4.35, 4.50, 5.00, 6.00)      # reviewer 点名的六格
PROFILE_X = (-0.60, -0.40, -0.30, -0.20, -0.10, 0.0, 0.10, 0.20, 0.30, 0.40, 0.60)


def sampler(msg):
    if msg is None:
        return None
    i = msg.info
    g = np.asarray(msg.data, dtype=np.int16).reshape(i.height, i.width)

    def q(x, y):
        cx = int((x - i.origin.position.x) / i.resolution)
        cy = int((y - i.origin.position.y) / i.resolution)
        return int(g[cy, cx]) if (0 <= cx < i.width and 0 <= cy < i.height) else None
    return q, {'res': i.resolution, 'w': i.width, 'h': i.height,
               'origin': [i.origin.position.x, i.origin.position.y]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', required=True)
    ap.add_argument('--seconds', type=float, default=30.0)
    a = ap.parse_args()

    rclpy.init()
    n = Node('costmap_probe')
    d = {t: None for t in TOPICS}
    for t in TOPICS + MERGE_TOPICS:
        n.create_subscription(OccupancyGrid, t,
                              (lambda tt: (lambda m: d.__setitem__(tt, m)))(t), Q)
    end = time.time() + a.seconds
    while time.time() < end and any(v is None for v in d.values()):
        rclpy.spin_once(n, timeout_sec=0.2)

    # 车身位姿（对齐核对用 ✓）：直接问 TF，不靠外部传参 ✓
    # ★★ 修：这两个 import **必须在模块顶层** ✗ ——
    #   我第一版写成"在 `main()` 里 `import rclpy.time`" ⇒ **Python 把 `rclpy` 当成 `main()` 的局部名** ✗
    #   ⇒ 上面那句 `rclpy.init()` 变成"引用未赋值的局部变量" ⇒ 首跑就
    #     `UnboundLocalError: local variable 'rclpy' referenced before assignment` ✗✗
    #   ⇒ 与 `name = name` 那类**作用域**坑同族 ✓；`py_compile`/`bash -n` **都查不出** ✓
    #     （**又一次"编译过 ≠ 跑得通"** ✓ —— 这是我今天第四次栽在"只在运行时暴露"上 ✗）
    pose = {}
    try:
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf, n)
        t0 = time.time()
        while time.time() - t0 < 8:
            rclpy.spin_once(n, timeout_sec=0.2)
            try:
                tr = buf.lookup_transform('map', 'base_footprint', Time())
                pose = {'x': round(tr.transform.translation.x, 3),
                        'y': round(tr.transform.translation.y, 3),
                        'z': round(tr.transform.translation.z, 3)}
                break
            except Exception:
                pass
    except Exception as e:                                   # pragma: no cover
        pose = {'error': str(e)}

    out = {'pose_base_footprint': pose}
    # ★ 回退判别 ✓：两个掩膜的格数（≈3605 = 正常 ✓；≈701 = **回退被触发** ✗）
    for t in MERGE_TOPICS:
        g_ = None if d[t] is None else np.asarray(d[t].data, dtype=np.int16)
        out['cells_' + t] = None if g_ is None else int((g_ >= 100).sum())
    # ★★ 2026-10-09 改（reviewer：**别比绝对常数** ✗）：`3605` 只是那一次 probe 的**点值** ✗ ——
    #   本轮检测器掩膜若本来就标得少（驻留短了等）⇒ 并集也会低于 3605 ⇒ **可能误判成"回退被触发"** ✗。
    #   ⇒ **改成比两个计数的关系** ✓（不吃绝对量级 ✓、自动适配每一轮的掩膜大小 ✓）：
    #     `merged > mask_map`（差 ≈ 静态块那部分 ✓）⇒ **并集生效** ✓；
    #     `merged ≈ mask_map`（差在容差内 ✓）⇒ **回退被触发** ✗；
    #     `mask_map` 缺失 ⇒ **第三种状态** ✓（**不二值化** ✓ —— "没收到"不等于"没生效" ✓）。
    cm = out.get('cells_/rtk/drop/mask_merged')
    co = out.get('cells_/rtk/drop/mask_map')
    tol = 20          # 容差：静态块那部分是**几千格**量级 ✓ ⇒ 20 格足以区分 ✓
    if cm is None:
        out['merge_note'] = '**没收到 `mask_merged`** ✗ ⇒ 合并节点没发 ⇒ ④ 本轮未生效'
    elif co is None:
        out['merge_note'] = '没收到 `mask_map`（检测器那份）⇒ **第三种状态**，无法比对 ✓ —— 不判"回退" ✗'
    elif cm > co + tol:
        out['merge_note'] = ('并集**生效** ✓：merged=%d > mask_map=%d（多 %d 格 ≈ 静态块那部分 ✓）'
                             % (cm, co, cm - co))
    elif abs(cm - co) <= tol:
        out['merge_note'] = ('★ **回退被触发** ✗：merged=%d ≈ mask_map=%d（差 %d ≤ 容差 %d）⇒ '
                             '`/rtk/static_map` 当时不可用 ⇒ **实心块没进图**（= 回到 ④ 之前 ✗）'
                             % (cm, co, abs(cm - co), tol))
    else:
        out['merge_note'] = ('异常：merged=%d **小于** mask_map=%d ✗ ⇒ 合并不该减格 ⇒ 值得查 ✓' % (cm, co))
    for t in TOPICS:
        s = sampler(d[t])
        if s is None:
            out[t] = {'received': False, 'note': '**没收到** ⇒ 该图/该层没起来 ✗（这本身就是一种结果 ✓）'}
            continue
        q, meta = s
        rec = {'received': True, **meta,
               'six_cells_x0.15': {('y=%.2f' % y): q(0.15, y) for y in Y_SIX},
               'profile_seal_block_y5.00': {('x=%+.2f' % x): q(x, 5.00) for x in PROFILE_X},
               'profile_drop_mainedge_y0.50': {('x=%+.2f' % x): q(x, 0.50) for x in PROFILE_X},
               # 对齐核对（只在局部图那段真正用得着 ✓）：车旁那格 vs 车正下方那格 ✓
               'near_vehicle': ({'beside_x-0.70': q(pose['x'] - 0.70, pose['y']),
                                 'under': q(pose['x'], pose['y'])}
                                if 'x' in pose else None)}
        out[t] = rec
    n.destroy_node()
    rclpy.shutdown()

    json.dump(out, open(a.json, 'w'), indent=1, ensure_ascii=False)
    g = out.get('/global_costmap/costmap', {})
    print('static_map     收到=%s' % out['/rtk/static_map'].get('received'))
    print('global_costmap 收到=%s  六格=%s' % (g.get('received'), g.get('six_cells_x0.15')))
    print('local_costmap  收到=%s  六格=%s' % (out['/local_costmap/costmap'].get('received'),
                                              out['/local_costmap/costmap'].get('six_cells_x0.15')))
    print('车旁那格 / 车正下方 = %s' % (out['/local_costmap/costmap'].get('near_vehicle'),))
    print('→ 已落 %s' % a.json)


if __name__ == '__main__':
    main()

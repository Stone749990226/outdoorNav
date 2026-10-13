#!/usr/bin/env python3
"""就绪探测：确认"定位链 + 相机/点云 + nav2 两张图 + 危险距离"都真的在跑，再宣布启动成功。

为什么不用 ros2 CLI：`ros2 topic echo` 会走 CLI daemon，在负载高的时候它自己会掉线，
把"还没就绪"误报成"永远不就绪"。这里直接用 rclpy 探测。

判定标准（**全部**满足才算就绪）：
  1. TF 里存在 map→base_footprint（定位链通了）
  2. 收到过 /rtk/camera/depth_image（相机在出图）
  3. 收到过 /rtk/camera/points（第五阶段起由自写的 `rtk_depth_points` 出点云）
  3b. 收到过 /rtk/scan（第五阶段新增的低装激光雷达；它是低矮障碍的唯一来源）
  4. /rtk/costmap_status **连续 N 秒**都是 ok，**且** /rtk/costmap_danger 还在按周期来
  5. nav2 的**两张图**都在按周期发布：/global_costmap/costmap、/local_costmap/costmap
  6. ★ 2026-10-09（阶段八）**仿真器活着**：`ign topic -l` 里能看到 `/world/` 开头的话题。
     为什么单列这一条（一次真实事故）：容器 baked 的 DISPLAY 是**别人的**（`:10` 属 czn、
     `:11` 属 xjy）⇒ Gazebo GUI 起不来 ⇒ **整个 `ign gazebo` 进程死掉**（连 server 一起）。
     而那一轮 `ros2 node list` **有 29 个节点**、话题都在、`/rtk/static_map` 也有发布者
     ⇒ 前面五条里"两张图在发"甚至会短暂成立（代价图会发一张全未知的图）⇒ 现象长得像
     "定位链上游没起来"，**完全不像"仿真器死了"** ✗。
     ⇒ **"栈起来了"与"仿真器活着"是两件事**，只有这一条能把两者一眼分开 ✓。
     （nav2 原生命名：Costmap2DROS 的相对名 "costmap" 解析到节点命名空间，
      也就是 /global_costmap/costmap 与 /local_costmap/costmap）

第 4 条为什么要"连续"而不是"出现过一次"：代价地图刚起来时会经历 wait/no_pose 的瞬态，
只判一次会在瞬态边缘放行，紧接着启动的路线跟随就会撞上 `costmap_stale` 而中止（第一阶段实测过）。

第 4 条为什么还要求"危险距离仍在按周期来"：latched 的状态字符串**可能比发布者活得久**
——节点崩了以后，旧的那条 ok 还会被后加入的订阅者收到（第二阶段实测踩过：
节点崩了、就绪判定却通过了，因为只看那条 latched 的 ok）。所以"状态 ok"必须配上
"它还在发东西"才算数。

第 5 条是第三阶段新增的：nav2 的两张图是分两个进程发布的（planner_server / 自写的
costmap 宿主），任一张没起来，"建图与规划"这条主线就是残的，不该宣布启动成功。

## ★ 超时那行怎么写（2026-09-21 修，别退回旧写法）

原来超时只印一个 `satisfied` 字典，值来自 `fresh(key)`（阈值见下面的 `FRESH = 3.0`）。于是
**`global_map: False` 会被读成"从来没有收到过全局图"** ✗ —— 它真实的含义是
"**超时那一刻，最近 FRESH 秒内没收到**"。两者是完全不同的故障：
  · **从未收到** ⇒ 话题/发布者/接线的问题（要去查上游）；
  · **最近没来** ⇒ 曾经来过、那一刻停了（要去查它为什么卡住/掉线）。
看同一个 `False` 分不出这两种，就会把排查方向带偏。

⇒ 现在**每个键记两样**：`seen_count`（累计收到多少次）与 `last_age_s`（最近一次距今多久），
  超时行按"**次数 + 最近间隔 + 阈值**"逐键打印，并直接给出判定：
      `从未收到`（count == 0） / `最近没来`（count > 0 且 age ≥ 阈值） / `在来`
  ⇒ **"从未收到"与"最近没来"当场可分** ✓

**这与 reviewer 那条"活节点方式要给 settle 时间"同族**：**别把"读一次的结果"当结论** ——
一次读到的 `False` 既可能是"没有"，也可能是"这一瞬没来"。

用法：wait_ready.py [超时秒数，默认 300] [状态需连续的秒数，默认 3.0]
"""

import os
import subprocess
import sys
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Float32, String

# 话题新鲜度上限（秒）。两张 nav2 图按 2 Hz / 5 Hz 发布，给到 3 s 容忍偶发抖动。
FRESH = 3.0


def judge_seen(count, last, now, fresh=FRESH):
    """把 (次数, 最近时刻) 判成一句话。**抽成函数是为了能单测三个分支** ——
    "最近没来"这一支在没起栈时造不出来，靠单测来证它写对了。

    返回 (短判定, 完整说明)。三分支：
      · count == 0                ⇒ `从未收到`
      · count > 0 且 age ≥ fresh  ⇒ `最近没来`（附带次数与间隔）
      · 否则                      ⇒ `在来`
    """
    if count == 0 or last is None:
        return '从未收到', '**从未收到** ⇒ 查话题/发布者/接线（上游）'
    age = now - last
    if age >= fresh:
        return '最近没来', ('**最近没来**（来过 %d 次、最近一次在 %.2f s 前 ≥ 阈值 %.1f s）'
                            '⇒ 查它为什么停了' % (count, age, fresh))
    return '在来', '在来'


def _has_publisher(node, topic, wait_s=12.0):
    """等一小会儿看这个话题有没有发布者（用来判断某个可选组件是否启用）。"""
    end = time.time() + wait_s
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.count_publishers(topic) > 0:
            return True
    return node.count_publishers(topic) > 0


def main(argv=None):
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    stable_needed = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    # ★★ 2026-10-10（reviewer §4）：**探针开机自报自己的分区** ✓ ★★
    #   为什么：判别"注入到没到探针"的**直接**测量是读它自己的 `/proc/<pid>/environ` ✗，
    #   而那要**赶在 300 s 窗口里抓** ✗（今天这条规矩已经栽过两次 ✓）⇒ 复算方只能采信转述 ✗。
    #   ⇒ 探针**开机就把 PID + 两个分区变量打出来** ✓（进 `readiness.log` ✓）
    #     ⇒ 事后拿产物即可判 ✓：★ 打印的若等于本轮分区 ⇒ 注入到了 ✓；
    #       若为 `None`/容器默认 ⇒ ★ **注入没到探针** ✗ ⇒ 就是 §4 那个洞 ✓。
    print('[probe-env] pid=%d GZ_PARTITION=%r IGN_PARTITION=%r'
          % (os.getpid(), os.environ.get('GZ_PARTITION'),
             os.environ.get('IGN_PARTITION')), flush=True)
    # ★★ 判 §4 那个洞要**同刻三件一起取**（reviewer 2026-10-10）★★
    #   ① 上面那行的分区   ⇒ 判"注入到没到探针" ✓
    #   ② ★ 本行：本分区仿真进程集 ⇒ 判"**取数那一刻它活着**" ✓（活性证据 ✓）
    #   ③ 下面就绪检查里的 `/world/` 计数（三态 ✓）⇒ 判分区名/可见性 ✓
    #   ★ 只有三件**同刻**，"0" 才有含义 ✓：
    #     ①=容器默认 `rtknav2` ⇒ 注入的错 ✗；②=空 ⇒ **未判** ✗（仿真根本没起 ✓）；③ 空 ⇒ **未判** ✗。
    #   ⇒ ★ 由**探针自己**把三件都记进产物 ✓ ⇒ 复算方**事后可判** ✓，
    #     不必抢那 300 s 窗口 ✗（今天这条规矩已栽过两次 ✓）。
    _simset = []
    try:
        for _p in sorted(os.listdir('/proc')):
            if not _p.isdigit():
                continue
            try:
                with open('/proc/%s/cmdline' % _p, 'rb') as _f:
                    _cmd = _f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
                if 'ign gazebo' not in _cmd:
                    continue
                # `/proc/<pid>/stat` 第 22 字段 = starttime（jiffies，全精度）
                with open('/proc/%s/stat' % _p) as _f:
                    _st = _f.read().split()[21]
                _part = '-'
                try:
                    with open('/proc/%s/environ' % _p, 'rb') as _f:
                        for _ln in _f.read().replace(b'\0', b'\n').decode(
                                'utf-8', 'replace').splitlines():
                            if _ln.startswith('GZ_PARTITION='):
                                _part = _ln.split('=', 1)[1]
                                break
                except OSError:
                    _part = '?'
                _simset.append('%s:%s:%s' % (_p, _st, _part))
            except OSError:
                continue
    except OSError:
        pass
    print('[probe-env] sim_procs=%d set=%s'
          % (len(_simset), ' '.join(_simset)), flush=True)

    rclpy.init()
    node = Node('rtk_nav2_wait_ready')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    # ★ 每个键记两样：**累计次数**与**最近一次的时刻** ⇒ 超时行才能把
    #   "从未收到"（count == 0）与"最近没来"（count > 0 但 age ≥ FRESH）分开。
    stats = {}

    def mark(key):
        def cb(_msg):
            st = stats.setdefault(key, {"count": 0, "last": None})
            st["count"] += 1
            st["last"] = time.time()
        return cb

    # 深度图同理：ros_gz_bridge 出来的图像也是 BEST_EFFORT。
    node.create_subscription(Image, '/rtk/camera/depth_image', mark('depth'),
                             qos_profile_sensor_data)
    # **点云必须用 sensor-data QoS（BEST_EFFORT）订阅**：`depth_image_proc` 的
    # 发布端是 BEST_EFFORT（传感器数据 QoS），用默认的 RELIABLE 订阅端会收不到任何消息，
    # 只在日志里留一行 "offering incompatible QoS ... Last incompatible policy: RELIABILITY"
    # ——实测踩过（就绪判定一直等点云，看起来像"点云没在出"）。
    node.create_subscription(PointCloud2, '/rtk/camera/points', mark('points'),
                             qos_profile_sensor_data)
    # **第五阶段新增：低装激光雷达**。它是"探低矮障碍"的唯一来源（相机源阈值 0.30），
    # 所以它也进就绪判据：激光不在，本次运行就不该被宣布为"就绪"。
    # 同样是 sensor-data QoS（Gazebo 桥接出来的 LaserScan 是 BEST_EFFORT）。
    node.create_subscription(LaserScan, '/rtk/scan', mark('scan'),
                             qos_profile_sensor_data)
    # **`danger` 是第三步的组件、第四步默认不启动**（brief 第四步裁定 2：它退役）。
    # 所以这两条就绪条件要能自动让开：先看 `/rtk/costmap_danger` 有没有发布者，
    # 没有就认为"本步没启用 danger"，跳过"状态连续 ok + 危险距离在流"这两条。
    # 不做这个判断的话，第四步起栈会**永远不就绪**（脚本重试 3 次后失败），
    # 而现象看起来像"栈起不来"，其实只是就绪探测还在等一个已经退役的节点。
    require_danger = _has_publisher(node, '/rtk/costmap_danger')
    if not require_danger:
        print('注意：/rtk/costmap_danger 没有发布者 → 按"本步未启用 danger"处理，跳过这两条就绪条件')
    node.create_subscription(Float32, '/rtk/costmap_danger', mark('danger'), 10)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                             mark('global_map'), 5)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                             mark('local_map'), 5)

    latched = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    ok_since = {'t': None}

    def on_status(msg):
        if msg.data == 'ok':
            if ok_since['t'] is None:
                ok_since['t'] = time.time()
        else:
            ok_since['t'] = None

    node.create_subscription(String, '/rtk/costmap_status', on_status, latched)

    def fresh(key):
        st = stats.get(key)
        return st is not None and st["last"] is not None and (time.time() - st["last"]) < FRESH

    def sim_alive():
        """★ 仿真器活着：`ign topic -l` 里有 `/world/` 开头的话题（= 世界真的加载了）。

        ★★ 判读口径（reviewer 2026-10-10 干跑实测，**照读即可** ✓）★★
          本函数上方那行 `[probe-env] …` 自报的就是这个子进程的分区 ✓：
            · 报 `rtknav2`            ✗ ⇒ **注入没到探针** ⇒ 本条判 False 是**探针的错**，不是仿真没了 ✓
            · 报 `rtknav2_<本轮run-id>` ✓ ⇒ 注入到位 ✓，此时判 False 才真是"仿真不在" ✓
          （`rtknav2` 是容器默认值 ✓，不注入时就是它 ✓ —— 别把"没注入"读成"仿真死了" ✗）

        ★★ 下面那个 `ign topic -l` 的计数必须**三态**读（reviewer 2026-10-10）★★
          同一个 `0` 有三种成因 ✗，只有一种对"仿真不在"有意义 ✓：
            · 列表**非空**且有 `/world/…`  ✓ ⇒ 注入/分区名都对 ⇒ 报 False 才是**真·仿真不在** ✓
            · 列表**非空**但无 `/world/`   ✗ ⇒ **分区名对这个 `ign` 版本不对**（或世界真没加载）
            · ★ 列表**整个为空**            ⇒ **`undetermined`** ✗ —— 多半是**客户端连不上 daemon**
              （gz 没起 / transport 不通 ✓）⇒ **绝不许读成"没有 /world"** ✗✗
          ⇒ ★ 口径：**"列表非空"是前提** ✓；空列表 ⇒ 本判据**不成立也不否证** ✓。
          ⇒ ★ 实测教训：`campus8-n1` 死栈上拿到的那个 `0` **就是第三种** ✗ ⇒ 什么都证明不了 ✓。
          ⇒ ✓ 同刻对照量：那套仿真的 `/proc/<pid>/environ` 里的 `GZ_PARTITION`（`docker exec` 读 ✓）——
            **两者一起看才能定性** ✓。

        用 `/world/` 话题而不是 `ign model --list`：后者要求"世界里有模型"，没有模型的世界
        （空场地）会**假报**"仿真器死了" ✗。`/world/<名>/...` 是 server 起来就有的 ✓。
        """
        try:
            out = subprocess.run(['ign', 'topic', '-l'], capture_output=True,
                                 text=True, timeout=15).stdout
        except Exception:                                      # noqa: BLE001
            return False, 'ign 调用失败'
        lines = [ln.strip() for ln in out.splitlines()]
        world_topics = [ln for ln in lines if ln.startswith('/world/')]
        return bool(world_topics), ('%d 个 /world/ 话题' % len(world_topics)
                                    if world_topics else '**一个 /world/ 话题都没有**')

    t0 = time.time()
    satisfied = {}
    # TF 不是"收消息"、是"查询"，所以单独记成功/失败次数。
    tf_stat = {"ok": 0, "fail": 0, "last_ok": None}
    sim_stat = [False, '']
    sim_ticks = 0
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        # `ign topic -l` 要起子进程 ⇒ 每 20 拍查一次，别把 CPU 喂给它
        sim_ticks += 1
        if sim_ticks % 20 == 1:
            sim_stat[0], sim_stat[1] = sim_alive()
        try:
            tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            satisfied['tf'] = True
            tf_stat['ok'] += 1
            tf_stat['last_ok'] = time.time()
        except Exception:
            satisfied['tf'] = False
            tf_stat['fail'] += 1
        satisfied['depth'] = fresh('depth')
        satisfied['points'] = fresh('points')
        satisfied['scan'] = fresh('scan')
        satisfied['danger'] = fresh('danger') if require_danger else True
        satisfied['danger_status_ok'] = ((ok_since['t'] is not None
                                          and (time.time() - ok_since['t']) >= stable_needed)
                                         if require_danger else True)
        satisfied['global_map'] = fresh('global_map')
        satisfied['local_map'] = fresh('local_map')
        satisfied['sim'] = sim_stat[0]
        if all(satisfied.values()):
            print('就绪（%.1f s）：仿真器活着（%s）；map→base_footprint 可查；深度图、点云与'
                  '/rtk/scan 在来；危险距离状态连续 %.1f s 为 ok；nav2 全局图与局部图都在发布'
                  % (time.time() - t0, sim_stat[1], stable_needed))
            node.destroy_node()
            rclpy.shutdown()
            return 0
        time.sleep(0.2)

    # ★★ 超时行：**不许只印一个 satisfied 字典**（那会把"最近没来"读成"从未收到"）★★
    #   逐键印"次数 + 最近间隔 + 阈值 + 判定"，两种故障当场可分。
    now = time.time()
    print('** 超时未就绪 **（%.1f s 内全部条件未同时满足）' % timeout, file=sys.stderr)
    print('   阈值：话题新鲜度 FRESH=%.1f s；/rtk/costmap_status 需连续 %.1f s 为 ok；'
          'danger %s'
          % (FRESH, stable_needed, '本轮启用' if require_danger else '本轮未启用（已跳过）'),
          file=sys.stderr)
    print('   %-22s %8s %14s  %s' % ('键', '收到次数', '最近间隔(s)', '判定'), file=sys.stderr)
    # ★ 仿真器那一项单独一行（它不是"收消息"，是起子进程查的）
    print('   %-22s %8s %14s  %s' % ('sim(仿真器)', '—', '—',
          ('在（%s）' % sim_stat[1]) if sim_stat[0]
          else '**不在**：%s ⇒ 查 Gazebo 为什么没起来（先看 DISPLAY/xdpyinfo，'
               '见 lib.sh 的 ensure_display；`node list` 有一堆节点**不代表**仿真器活着）'
               % (sim_stat[1] or '未查到')), file=sys.stderr)
    for k in ('depth', 'points', 'scan', 'danger', 'global_map', 'local_map'):
        if k == 'danger' and not require_danger:
            print('   %-22s %8s %14s  %s' % (k, '—', '—', '本轮未启用（不作判据）'),
                  file=sys.stderr)
            continue
        st = stats.get(k) or {"count": 0, "last": None}
        cnt = st["count"]
        age = '—' if st["last"] is None else '%.2f' % (now - st["last"])
        _short, verdict = judge_seen(cnt, st["last"], now, FRESH)
        print('   %-22s %8d %14s  %s' % (k, cnt, age, verdict), file=sys.stderr)
    # TF 单独一行（查询类）
    tf_age = ('—' if tf_stat['last_ok'] is None
              else '%.2f' % (now - tf_stat['last_ok']))
    _tf_short, tf_v = judge_seen(tf_stat['ok'], tf_stat['last_ok'], now, FRESH)
    # ★ 只替换**内层词**，不要再加一层 `**` —— 否则会印成 `****从未查到****`
    #   （第一版就是这么写错的，实测印出来了）。
    tf_v = tf_v.replace('从未收到', '从未查到').replace('最近没来', '最近查不到')
    print('   %-22s %8s %14s  %s（成功 %d / 失败 %d）'
          % ('tf map→base_footprint', tf_stat['ok'], tf_age, tf_v,
             tf_stat['ok'], tf_stat['fail']), file=sys.stderr)
    if require_danger:
        print('   %-22s %8s %14s  %s'
              % ('danger_status_ok', '—', '—',
                 'ok 连续 %.2f s / 需要 %.1f s'
                 % ((now - ok_since['t']) if ok_since['t'] is not None else 0.0, stable_needed)),
              file=sys.stderr)
    print('   （机器可读的 satisfied：%s）' % satisfied, file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    raise SystemExit(main())

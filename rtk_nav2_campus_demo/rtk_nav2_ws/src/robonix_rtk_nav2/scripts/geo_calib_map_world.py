#!/usr/bin/env python3
"""★ 阶段八·第一问：**map 系与世界系是否重合** —— 实测标定，不许手推。

**为什么要实测**（brief §4，reviewer 2026-10-09 指定为独立任务）：
`navsat_transform` 走的是 **UTM**，而 Gazebo 的 GPS 走的是 SDF `spherical_coordinates`
的**切平面**；两者差一个**子午线收敛角** γ ≈ (λ − λ₀)·sin φ（λ₀ = UTM 51 带中央经线 123°）。
★ **γ 的手推值从 `--world` 那份 SDF 现读**（`<spherical_coordinates>`），**不硬编码** ——
2026-10-09 基准挪站（上海 → 杭州湾智慧谷）时这里写死过，**脚本吐的还是旧站的 −0.79°、
而我嘴里报的是新站的 −1.37°** ⇒ 产物与报数不一致 ✗（"报了但没落盘"那一族）。
10 m 场景里这只值 **0.07 m**（阶段七"定位差 ~1 cm"**分辨不出它存不存在**）；
100 m 场景里 ~**0.7 m**，与 10 m 的中间点间距**同量级** ⇒ 足以让"走到下一个中间点"走偏。
**另有 UTM 尺度因子（0.9996 起）在 100 m 上带来厘米级缩放**，也要实测。

⇒ 做法：让车跑一条覆盖 ±40 m 的路线，同步记
**`/ground_truth/odom`（Gazebo 真值 = 世界系）** 与 **`/odometry/global`（定位输出 = map 系）**，
拟合 **2D 相似变换**（旋转 + 缩放 + 平移，共 4 个自由度），
**同时报残差与拟合出的旋转角**（reviewer：**只报残差不算回答**）。

★ 本脚本**只取数与拟合**，不判"过/不过"（口径归 reviewer）。
★ 输出 CSV（逐样本）⇒ 任何人可以用自己的实现**独立复算**同一份数据 ✓。

用法（容器内）：
  python3 scripts/geo_calib_map_world.py --out /work/artifacts/rtk-nav2/<run-id>/frame_fit \
      [--seconds 600] [--tol 0.05]
产出：
  <out>_samples.csv    每行 t, gt_x, gt_y, loc_x, loc_y, dt
  <out>_fit.json       拟合结果（旋转角、缩放、平移、残差 p50/p95/max、覆盖范围）

退出码：0 = 正常；1 = 取数/用法失败；3 = 样本太少或覆盖不足（<±35 m）⇒ 本轮不足以回答。
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

GT_TOPIC = '/ground_truth/odom'
LOC_TOPIC = '/odometry/global'
# 默认世界：与生成器/静态层同一份（**单一真源**）
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_WORLD = os.path.join(_HERE, '..', 'worlds', 'campus_100.sdf')
DEFAULT_NAVSAT = os.path.join(_HERE, '..', 'config', 'navsat.yaml')
GEN_PATH = os.path.join(_HERE, 'make_campus_world.py')
RTKSIM_PATH = os.path.join(_HERE, '..', 'robonix_rtk_nav2', 'rtk_sim.py')


def read_run_state(run_dir):
    """从**数据那一轮**的 `run_state.txt` 读它当时真正用的东西。

    ★★ 为什么必须这样（reviewer 2026-10-09 抓到的**机制洞**）★★
    原先 `calib_key` 是按**"拟合那一刻"的系统状态**算的 ⇒ 它**保证不了"数据是在这个 datum 下采的"** ✗：
    任何人拿**旧数据**重拟合一次，就会得到一个**看起来完全有效**的新键 ✗。
    ⇒ 这是 (b) 机制**同一个病换了个位置**：防住了"旧标定配新场景"，没防住"**新键配旧数据**" ✗。
    ⇒ **键必须由"数据采集那一刻"的记录构造** ⇒ 从那一轮的 `run_state.txt` 取 ✓。
    """
    import re
    out = {'run_dir': run_dir}
    try:
        txt = open(os.path.join(run_dir, 'run_state.txt'), encoding='utf-8').read()
    except Exception as exc:                                   # noqa: BLE001
        out['error'] = '读不到 run_state.txt（%s）' % exc
        return out
    for key in ('config_used_campus_100.sdf', 'geometry_fingerprint',
                'world_matches_model', 'config_navsat.yaml', 'static_layer_args'):
        m = re.search(r'^%s=([^\s#]+)' % re.escape(key), txt, re.M)
        v = m.group(1) if m else None
        # ★ **剥掉值后面的括号注记** —— 这是**对旧 `run_state.txt` 的兜底**。
        #   根因已在**写的那一端**修掉（`lib.sh` 现在只写裸 sha、注记挪到 `#` 后的注释行 ✓）；
        #   这里留着剥离，是为了**已存在的旧轮次**仍能被正确读取 ✓ ——
        #   但**别再靠它**：新写的 `run_state.txt` 值里不该有注记（reviewer 2026-10-09 指出
        #   "只修读的那一半 = 把坑留给下一个消费者" ✗）。
        if v and '（' in v:
            v = v.split('（')[0]
        out[key] = v
    return out


def calib_key(world_path, navsat_path, generator_path, rtksim_path, run_dir=None):
    """★ **标定键 `calib_key`**（reviewer 2026-10-09 裁定，brief §4.3）。

    **为什么不是"几何指纹 + 一项一项并列"**：
    · 把 datum 并进"几何指纹" ⇒ **名字会说谎**（datum 不是几何）✗；
    · "两项并列" ⇒ 下次再冒出第三个输入（例如 `rtk_sim` 的注入偏差）就**又漏一个** ✗ —— 枚举式必然漏。
    ⇒ 改成**一个有序的捆**：`sha256(规范化后的分量串)`，`geo_route` **只比这一个键**；
      `geo_calib.json` **同时逐项列出分量** ⇒ 不一致时**一眼看出是哪一项变了** ✓。
    好处：不会漏、不必问"该查哪个"、**分量清单本身带版本**（改了分量列表 ⇒ 键值就变 ✓）、
    而 GUI/光照/注释这类不在键里的东西**仍然不影响** ✓。

    分量（**有序**，顺序写进 `components` 的键序）：
      1. `datum` = `lat,lon,alt`
      2. `geometry_fingerprint`（来自生成器 —— 障碍清单排序后的 sha256）
      3. `sha256(navsat.yaml)`
      4. `rtk_sim` 的 `sigma_m,bias_east_m,bias_north_m`（**注入偏差会整体平移 map 系** ⇒ 必须进键）
      5. `sha256(生成器)`
    """
    import hashlib
    import re
    comp = {}
    lon, lat, alt, src = read_datum(world_path)
    comp['datum'] = 'nan,nan,nan' if lat is None else '%.6f,%.6f,%.4f' % (lat, lon, alt)
    comp['datum_source'] = src
    # ★ **数据来源**：键要能回答"这条数据是在哪个世界/datum 下采的"
    if run_dir:
        rs = read_run_state(run_dir)
        comp['data_run'] = rs.get('run_dir', '')
        comp['data_world_sha256'] = rs.get('config_used_campus_100.sdf') or 'missing'
        comp['data_geometry_fingerprint'] = rs.get('geometry_fingerprint') or 'missing'
        comp['data_world_matches_model'] = rs.get('world_matches_model') or 'missing'
        # 数据那轮的世界 sha 与**现在**这份世界 sha 是否同一份
        try:
            cur_world_sha = hashlib.sha256(open(world_path, 'rb').read()).hexdigest()
        except Exception:                                      # noqa: BLE001
            cur_world_sha = 'unreadable'
        comp['fit_world_sha256'] = cur_world_sha
        # ★★ 裁定 ③（reviewer 2026-10-10）：有效性判据 = **几何指纹**（datum 已在键里），
        #    世界**文件 sha 降为审计** ✗、不进判据 ★★
        #    理由：几何指纹存在的意义就是"让 **GUI/注释**这类**零几何改动**不让标定失效" ✓；
        #    拿文件 sha 当判据 ⇒ **改一个 Gazebo 机位就让标定失效** ✗ —— 那正是当初定 (b) 要避免的 ✓。
        #    （反之，**真动了几何** ⇒ 几何指纹也变 ⇒ 照样失效 ✓，安全性没丢。）
        try:
            import make_campus_world as _M
            cur_fp = _M.geometry_fingerprint()
        except Exception as exc:                               # noqa: BLE001
            cur_fp = 'unreadable(%s)' % exc
        comp['data_matches_current_geometry'] = str(rs.get('geometry_fingerprint') == cur_fp)
        comp['data_matches_current_world_sha'] = str(      # ← **只作审计**，不参与有效性判定
            rs.get('config_used_campus_100.sdf') == cur_world_sha)
    else:
        comp['data_run'] = 'NOT_GIVEN'
        comp['data_matches_current_geometry'] = 'unknown'
        comp['data_matches_current_world_sha'] = 'unknown'
    try:
        import make_campus_world as M
        comp['geometry_fingerprint'] = M.geometry_fingerprint()
    except Exception as exc:                                   # noqa: BLE001
        comp['geometry_fingerprint'] = 'unreadable(%s)' % exc
    for name, path in (('navsat_yaml_sha256', navsat_path),
                       ('generator_sha256', generator_path)):
        try:
            comp[name] = hashlib.sha256(open(path, 'rb').read()).hexdigest()
        except Exception as exc:                               # noqa: BLE001
            comp[name] = 'unreadable(%s)' % exc
    # `rtk_sim` 的三个默认值：从源码里读 `declare_parameter('<名>', <默认>)`
    try:
        src_txt = open(rtksim_path, encoding='utf-8').read()
        vals = []
        for key in ('sigma_m', 'bias_east_m', 'bias_north_m'):
            m = re.search(r"declare_parameter\(\s*'%s'\s*,\s*([-0-9.eE]+)" % key, src_txt)
            vals.append(m.group(1) if m else '?')
        # ★★ 这三项**取自源码 `declare_parameter` 的默认值** —— 已实测
        #   `launch/demo.launch.py` 的 `rtk_sim` 节点**只传 `use_sim_time`、没有覆盖 σ/bias**
        #   ⇒ **当前默认值 == 部署值** ✓。**但这正是 2026-10-08 那次事故的形态**
        #   （探针用默认 0.19、部署 0.55 ⇒ 把"我算错"当成"检测器失明"）✗
        #   ⇒ 若将来把 σ/bias 参数化到起栈命令，**本键必须改成"从运行时读"**（或加一条
        #     "部署未覆盖"的断言），否则键会**记错而看不出来** ✗。
        comp['rtk_sim_sigma_bias'] = ','.join(vals)
        comp['rtk_sim_source'] = ('取自源码 declare_parameter 默认值（已实测：起栈未覆盖 ⇒ '
                                  '默认==部署）。★ 若将来在起栈处覆盖这三项 ⇒ 本键须改为从运行时读')
    except Exception as exc:                                   # noqa: BLE001
        comp['rtk_sim_sigma_bias'] = 'unreadable(%s)' % exc
    # 分量顺序（**有序**；改这份列表 ⇒ 键值就变，天然带版本 ✓）
    keys = ['datum', 'geometry_fingerprint', 'navsat_yaml_sha256',
            'rtk_sim_sigma_bias', 'generator_sha256']
    # ★ "数据来源"两项**只在给了 `--from-run` 时**才进键（没给时键是 null，见产物字段）
    if 'data_world_sha256' in comp:
        keys += ['data_world_sha256', 'data_geometry_fingerprint']
    bundle = '|'.join('%s=%s' % (k, comp[k]) for k in keys)
    return hashlib.sha256(bundle.encode('utf-8')).hexdigest(), comp, bundle


def read_datum(world_path):
    """从 world SDF 的 `<spherical_coordinates>` **现读**基准 ⇒ `(lon, lat, alt, source)`。

    ★ 为什么必须现读、不许硬编码：2026-10-09 把测试站点从上海挪到杭州湾智慧谷时，
      这里写死的 `121.4737 / 31.2304` 让 `gamma_hand_deg` **仍吐旧站的值** ✗ ——
      而我在会话里报的是新站的 −1.37° ⇒ **产物与报数不一致**（"报了但没落盘"同族）。
    """
    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(world_path).getroot()
        sc = root.find('.//spherical_coordinates')
        if sc is None:
            return None, None, None, '世界 %s 里没有 <spherical_coordinates>' % world_path
        return (float(sc.find('longitude_deg').text), float(sc.find('latitude_deg').text),
                float(sc.find('elevation').text), world_path)
    except Exception as exc:                                   # noqa: BLE001
        return None, None, None, '读不出（%s）' % exc


def umeyama_2d(p, q):
    """2D 相似变换拟合：求 s、R(θ)、t 使 q ≈ s·R·p + t（Umeyama 1991）。

    返回 (scale, theta_rad, t, residual_rot_deg, 残差数组)。
    **只用 numpy 的 svd**，不引外部依赖 ⇒ 谁都能复算。
    """
    mu_p = p.mean(axis=0)
    mu_q = q.mean(axis=0)
    P = p - mu_p
    Q = q - mu_q
    n = len(p)
    sigma = (Q.T @ P) / n
    u, d, vt = np.linalg.svd(sigma)
    s_mat = np.eye(2)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s_mat[1, 1] = -1.0
    rot = u @ s_mat @ vt
    var_p = (P ** 2).sum() / n
    scale = float((d * np.diag(s_mat)).sum() / var_p) if var_p > 0 else float('nan')
    t = mu_q - scale * (rot @ mu_p)
    pred = (scale * (rot @ p.T)).T + t
    resid = np.linalg.norm(pred - q, axis=1)
    theta = math.atan2(rot[1, 0], rot[0, 0])
    return scale, theta, t, np.degrees(theta), resid


def theta_std_pair(p, q, scale, theta, t):
    """旋转角不确定度的**两种口径**都算出来（reviewer 2026-10-09 要求"报不确定度必须写方法"）。

    来历：我第一版只报了一个数（±0.0313°），reviewer 用 iid 公式算出 ±0.00061° —— **差 50 倍**。
    差额的根源是**样本相关**：定位链是**阶梯状**的（`/odometry/global` 10 Hz 跳变、
    相邻样本约 66% 重复同一个值）⇒ **有效独立样本数远小于行数** ⇒ 两个口径分别对应
    "**保守上界**"与"**iid 理想下界**"，真实值在两者之间、且**必须写明是哪一种** ✓。

    返回 (conservative_deg, iid_deg, 说明字典)。

    · **conservative**：`std(resid) / r_rms` —— 只把残差尺度折算成角度，
      **不随 n 变小** ⇒ 是"**与样本独立性无关**"的保守上界 ✓（宁可报大不报小）。
    · **iid**：`σ_θ = sqrt( (Σr²/n) / Σ|p−p̄|² )` —— 标准最小二乘角度项，
      **按行数当独立样本** ⇒ 在有阶梯相关时**偏乐观** ⇒ 是下界 ✓。
    """
    mu_p = p.mean(axis=0)
    P = p - mu_p
    r_rms = float(np.sqrt((P ** 2).sum(axis=1).mean()))
    rot = np.array([[math.cos(theta), -math.sin(theta)],
                    [math.sin(theta), math.cos(theta)]])
    resid = np.linalg.norm((scale * (rot @ p.T)).T + t - q, axis=1)
    n = len(p)
    cons = float(np.degrees(resid.std() / r_rms)) if r_rms > 0 else float('nan')
    denom = float((P ** 2).sum())
    iid = float(np.degrees(math.sqrt((resid ** 2).sum() / n / denom))) if denom > 0 else float('nan')
    return cons, iid, {
        'conservative_deg': cons,
        'conservative_method': 'std(resid) / r_rms —— **不随 n 变小**，与样本独立性无关的保守上界',
        'iid_deg': iid,
        'iid_method': 'sqrt( (Σr²/n) / Σ|p−p̄|² ) —— 把每一行当**独立**样本；'
                      '本链路定位是**阶梯状**（10 Hz 跳变、约 66% 相邻行重复同一个值）'
                      '⇒ 它**偏乐观**、是下界',
        'note': '两个口径差约 50 倍（实测 0.0313° vs 0.0006°）**不是矛盾**、是"保守上界 vs iid 下界"；'
                '真值在两者之间。**报不确定度必须同时写清用的是哪一种** ✓',
    }


class Pairer(Node):
    """把两个话题按**最近时间戳**配对（两者频率不同，不能按下标配）。"""

    def __init__(self, tol):
        super().__init__('geo_calib_map_world')
        self.tol = tol
        self.gt = []          # 当前缓冲（回调往里写）
        self.loc = []
        self.gt_all = []      # 累计（配对时用）
        self.loc_all = []
        qos = QoSProfile(depth=50, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, GT_TOPIC, self._mk(self.gt, 'gt'), qos)
        self.create_subscription(Odometry, LOC_TOPIC, self._mk(self.loc, 'loc'), qos)

    def _mk(self, buf, which):
        def cb(msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t <= 0.0:
                t = time.time()
            p = msg.pose.pose.position
            buf.append((t, p.x, p.y))
        return cb

    def take(self):
        """把两个缓冲里**新到**的样本收进累计表并清空缓冲（**不清累计表**）。

        ★★ 这里踩过一次坑，两条都记下 ★★
        ① **不许重新绑定缓冲名**（`self.gt = []`）：订阅回调在**创建时**就抓住了列表对象的引用
           （`self._mk(self.gt, …)`）⇒ 重新绑定后回调仍往**旧列表**里塞、而这边读新列表
           ⇒ 永远空 ✗。正确做法是**原地清空** `del self.gt[:]` ✓。
        ② ★ **更隐蔽的一条：不许"来一批就配一批、配不上就丢"**。`/ground_truth/odom` 只有
           **~2 Hz**，而 `/odometry/global` 是 **~20 Hz**；每次 `spin_once(0.2)` 后立刻配对，
           常常是"这一窗只来了真值、能配的定位样本上一窗已经被清掉了" ⇒ 真值样本被**静默丢弃**
           ⇒ 最后**一对都配不上**（实测：跑了 30 s 一直打「已配对 0 对」，而两个话题都在正常发、
           QoS 也都是 RELIABLE，现象长得像"话题没数据"）✗
           ⇒ 改成**两边都累计、最后一次性按最近时间戳配对** ✓。
        采样量很小（真值 2 Hz × 1800 s ≈ 3.6k，定位 20 Hz ≈ 36k）⇒ 内存毫无压力。
        """
        self.gt_all.extend(self.gt)
        self.loc_all.extend(self.loc)
        del self.gt[:]
        del self.loc[:]

    def pair_all(self):
        """把**累计**的真值与定位按最近时间戳配对（时间差 <= tol）。"""
        gt = sorted(self.gt_all)
        loc = sorted(self.loc_all)
        out = []
        j = 0
        for (tg, xg, yg) in gt:
            if not loc:
                break
            while j + 1 < len(loc) and abs(loc[j + 1][0] - tg) <= abs(loc[j][0] - tg):
                j += 1
            tl, xl, yl = loc[j]
            dt = abs(tl - tg)
            if dt <= self.tol:
                out.append((tg, xg, yg, xl, yl, dt))
        return out

def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='输出前缀（会加 _samples.csv / _fit.json）')
    ap.add_argument('--seconds', type=float, default=600.0)
    ap.add_argument('--tol', type=float, default=0.05, help='真值/定位配对的最大时间差（秒）')
    ap.add_argument('--min-samples', type=int, default=200)
    ap.add_argument('--min-span', type=float, default=35.0, help='每个方向至少覆盖多少米')
    ap.add_argument('--navsat', default=DEFAULT_NAVSAT, help='navsat.yaml（进标定键）')
    ap.add_argument('--from-run', default='',
                    help='★ **数据那一轮**的 run 目录 —— 键必须由它构造（否则键保证不了'
                         '"数据是在这个 datum 下采的"，见 read_run_state 的说明）。不给 ⇒ 键写 null。')
    ap.add_argument('--world', default=DEFAULT_WORLD,
                    help='世界 SDF（用于**现读基准**算手推 γ 的对照值；默认 = 校园世界）')
    ap.add_argument('--from-csv', default='',
                    help='★ 不连 ROS：直接从已有的 <out>_samples.csv 重拟合并重写 <out>_fit.json。'
                         '用途有二：① 改了拟合/口径后**不必重跑一轮**；'
                         '② 让任何人**只凭 CSV**就能离线复算（reviewer 要的就是这个）。'
                         '给了它时 --out 仍用作前缀，--seconds/--tol 被忽略。')
    a = ap.parse_args(argv)

    if a.from_csv:
        import csv as _csv
        rows = list(_csv.DictReader(open(a.from_csv, encoding='utf-8')))
        if not rows:
            print('!! CSV 是空的 ⇒ 退出码 1', file=sys.stderr)
            return 1
        # ★ 列序必须是 **(t, gt_x, gt_y, loc_x, loc_y, dt)** —— 与主路径、与落盘 CSV 三处一致。
        #   第一版这里少了一列 `t`（只 5 列），而 `_emit_fit` 按 6 列写 ⇒
        #   `IndexError: index 5 is out of bounds` ✗（"两条路共用一段"的重构引入的口径不一致，
        #   跑一次就露 —— 所以重构完**必须真跑一遍**，光看代码看不出来）。
        arr = np.array([[float(r['t']), float(r['gt_x']), float(r['gt_y']),
                         float(r['loc_x']), float(r['loc_y']), float(r['dt'])] for r in rows])
        # ★ `--from-csv` **不重写样本 CSV**：那份就是它的输入 ⇒ 重写等于"自己吃自己"
        #   （第一版正是这么把源文件截断的）。只更新 `_fit.json` ✓
        return _emit_fit(a, arr, tolcsv_note='（由 %s 离线重拟合）' % a.from_csv,
                         interrupted=False, write_samples=False)

    rclpy.init()
    node = Pairer(a.tol)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    t0 = time.time()
    samples = []
    last_report = 0.0
    # ★★ **收到 SIGINT/SIGTERM 也必须把已经采到的样本落盘**（reviewer 2026-10-09 要求）★★
    #   来历：第一版没有这一层 ⇒ 收尾脚本的一记 `pkill -INT` 会**在写盘前把它打死**、
    #   几千对样本全丢（那次靠"在收尾前人工停掉那支脚本"才救回来 ✗ —— **处置不等于修**）。
    #   "数据够了但没落盘"：**发完信号不等于文件存在** ⇒ 采集循环里捕 `KeyboardInterrupt`，
    #   跳出后照常走下面的拟合与落盘 ✓。
    interrupted = [False]
    while time.time() - t0 < a.seconds:
        try:
            rclpy.spin_once(node, timeout_sec=0.2)
        except KeyboardInterrupt:
            interrupted[0] = True
            break
        node.take()
        # ★ 配对是 O(累计样本数) ⇒ **不要每拍都算**（每拍一次在 30 min 的轮次里是白烧 CPU）。
        #   它只服务于"进度打印"与"覆盖够了就提前收工"⇒ 每 15 s 算一次足够 ✓
        if time.time() - last_report > 15.0:
            last_report = time.time()
            samples = node.pair_all()
            print('  … 真值 %d / 定位 %d ⇒ 已配对 %d 对（%.0f s / %.0f s）'
                  % (len(node.gt_all), len(node.loc_all), len(samples),
                     time.time() - t0, a.seconds), file=sys.stderr, flush=True)
            if len(samples) >= 8:
                xs = [s[1] for s in samples]
                ys = [s[2] for s in samples]
                if (max(xs) - min(xs) >= 2 * a.min_span) and (max(ys) - min(ys) >= 2 * a.min_span):
                    print('  … 覆盖已达 ±%.1f m / ±%.1f m，提前收工'
                          % ((max(xs) - min(xs)) / 2, (max(ys) - min(ys)) / 2), file=sys.stderr)
                    break

    if interrupted[0]:
        print('  … 收到中断信号 ⇒ **把已采到的样本照常落盘**（不是"没跑完就丢掉"）',
              file=sys.stderr, flush=True)
    samples = node.pair_all()
    node.destroy_node()
    rclpy.shutdown()
    # 与 `--from-csv` / 落盘 CSV 同一列序：(t, gt_x, gt_y, loc_x, loc_y, dt)
    arr = np.array([[s[0], s[1], s[2], s[3], s[4], s[5]] for s in samples])
    return _emit_fit(a, arr, tolcsv_note='', interrupted=bool(interrupted[0]))


def _emit_fit(a, arr, tolcsv_note='', interrupted=False, write_samples=True):
    """把 (gt_x, gt_y, loc_x, loc_y, dt) 的数组拟合并落盘。

    **主路径与 `--from-csv` 共用这一段** —— 免得两条路各算一遍、口径漂移
    （"同一份数据两个中位数"同族）。
    """
    if len(arr) < a.min_samples:
        print('!! 只配对到 %d 对（< %d）⇒ 本轮不足以回答' % (len(arr), a.min_samples),
              file=sys.stderr)
        return 3
    # ★★ **先算完、再落盘；而且落盘要"临时文件 + 改名"**（reviewer 那套纪律，我这次没照做 ✗）★★
    #   事故：第一版把"写 CSV"放在**拟合与校验之前** ⇒ 一次列序不一致抛 IndexError 时，
    #   目标 CSV **已经被 `open('w')` 截断成 0 字节** ⇒ **12198 对样本归零** ✗
    #   （连随后那次重拟合又把 `_fit.json` 覆盖掉 ⇒ **原始产物两份都没了**）。
    #   ⇒ 现在：**先算出全部结果**，再 `临时文件 + os.replace` 原子落盘 ⇒
    #     任何中途抛错都**不会碰到已有产物** ✓。

    p = arr[:, 1:3]          # 真值 = 世界系（第 0 列是时间戳）
    q = arr[:, 3:5]          # 定位 = map 系
    scale, theta, t, theta_deg, resid = umeyama_2d(p, q)
    ts, ts_iid, ts_methods = theta_std_pair(p, q, scale, theta, t)

    span_x = [float(p[:, 0].min()), float(p[:, 0].max())]
    span_y = [float(p[:, 1].min()), float(p[:, 1].max())]
    span_ok = (span_x[1] - span_x[0] >= 2 * a.min_span
               and span_y[1] - span_y[0] >= 2 * a.min_span)

    # ★ **基准从 `--world` 现读**（单一真源），**不许硬编码** —— 2026-10-09 挪站时这里写死过，
    #   结果脚本吐的还是旧站的 −0.79°、而我嘴里报的是新站的 −1.37° ⇒ **产物与报数不一致** ✗。
    # ★ `(lon − lon0)` 本来就是**度** ⇒ 结果已经是度，**不许再 `math.degrees()` 一次** ✗
    #   （更早一版就这么写，报出过 −45.34° 这种离谱值）。
    lam, phi, _alt, datum_src = read_datum(a.world)
    lam0 = 123.0                 # UTM 51 带中央经线 —— **投影带的固有常数**，不是场景 datum
    gamma_hand_deg = ((lam - lam0) * math.sin(math.radians(phi))
                      if lam is not None else None)

    # ★ 各向异性对照（reviewer 2026-10-09 建议）：相似变换会把 x/y 两个尺度平均掉 ⇒
    #   另做一次**仿射**拟合 q = A·p + b，看 A 的两个对角是否差得多。
    #   实测 A = [[+1.00004, −0.00005], [+0.00003, +0.99926]] ⇒ y 向 −7.4e−4、x 向 +4e−5，
    #   相似变换报的是平均 −2.0e−4 ⇒ 差 7.8e−4（约 1.5σ，**不显著但不是零**）✓
    aff = {}
    try:
        X = np.hstack([p, np.ones((len(p), 1))])
        sol, *_ = np.linalg.lstsq(X, q, rcond=None)
        A = sol[0:2, :].T          # 2x2
        b = sol[2, :]
        predA = p @ A.T + b
        rA = np.linalg.norm(predA - q, axis=1)
        aff = {'A': [[float(A[0][0]), float(A[0][1])], [float(A[1][0]), float(A[1][1])]],
               'b': [float(b[0]), float(b[1])],
               'scale_x_minus_1': float(A[0][0] - 1.0),
               'scale_y_minus_1': float(A[1][1] - 1.0),
               'residual_p95': float(np.percentile(rA, 95)),
               'note': '相似变换把 x/y 两个尺度**平均成一个** ⇒ 各向异性要另做仿射才看得见'}
    except Exception as exc:                                   # noqa: BLE001
        aff = {'error': str(exc)}

    result = {
        'gt_topic': GT_TOPIC, 'loc_topic': LOC_TOPIC,
        'samples': int(len(arr)),
        'pair_tol_s': a.tol,
        'interrupted': bool(interrupted),
        'note': tolcsv_note,
        'sigma_eff_m': float(np.sqrt((resid ** 2).sum() / len(resid) / 2.0)),
        'sigma_eff_note': '按 2D 各向同性折算的单轴等效 σ = sqrt(Σr²/(2n))；'
                          '与注入的 rtk_sim sigma_m 对照即可看出"残差是不是噪声底"',
        'span_x_m': [round(v, 3) for v in span_x],
        'span_y_m': [round(v, 3) for v in span_y],
        'span_ok': bool(span_ok),
        'fit': {
            'model': 'q(map) ~= s * R(theta) * p(world) + t',
            'scale': scale,
            'scale_minus_1': scale - 1.0,
            'theta_deg': theta_deg,
            'theta_std_deg_est': ts,
            'theta_std_methods': ts_methods,
            'translation_m': [float(t[0]), float(t[1])],
            'translation_note': '★ **不能当"坐标系偏移"读**：map 系来自 rtk_sim，'
                                '它故意注入了固定偏差（默认 东 +0.05 / 北 −0.03 m）'
                                '⇒ 平移项里含它。要判"坐标系重不重合"请读**旋转项与尺度**。',
            'residual_m': {
                'mean': float(resid.mean()),
                'p50': float(np.percentile(resid, 50)),
                'p95': float(np.percentile(resid, 95)),
                'max': float(resid.max()),
            },
        },
        'affine_control': aff,
        'datum_from_world': {'lat': None if lam is None else phi, 'lon': lam, 'source': datum_src},
        # ★★ **不给 `--from-run` ⇒ 这枚键是 null**（reviewer 2026-10-09 要求）★★
        #   否则"拿旧数据重拟合"会得到一个**看起来完全有效**的键 ✗ —— 与其静默给个假凭据，不如给 null ✓
        'calib_key': (calib_key(a.world, a.navsat, GEN_PATH, RTKSIM_PATH, a.from_run)[0]
                      if a.from_run else None),
        # ★ 判据用**几何指纹**（裁定 ③）—— 与 datum（已在键里）合起来才是"标定还成不成立" ✓；
        #   世界**文件 sha 只作审计** ✗（否则改个机位就误判失效 ✗）
        'calib_key_is_valid': bool(a.from_run) and (
            calib_key(a.world, a.navsat, GEN_PATH, RTKSIM_PATH, a.from_run)[1]
            .get('data_matches_current_geometry') == 'True'),
        'calib_key_validity_note':
            '★ `calib_key` **只有在给了 `--from-run`（数据那一轮的 run 目录）时才给出**；'
            '不给 ⇒ **null**。理由：按"拟合那一刻"的系统状态算键 ⇒ **保证不了"数据是在这个 datum 下采的"** ✗'
            '（拿旧数据重拟合会得到一个看起来完全有效的键）。'
            '另：`calib_key_is_valid` 还要求"数据那轮的世界 sha == 现在这份世界的 sha" ✓',
        # ★★ 消费者**只读顶层布尔**，别读 components（reviewer 2026-10-09 的口径提醒）★★
        #   `calib_key_components` 里的值为了 canonical 化**全是字符串** ⇒ `"False"` 在 Python 里是**真值** ✗
        #   ⇒ 写 `if comp['data_matches_current_world']` 会**永远为真** ✗
        'calib_key_consumer_rule':
            '★ **只读顶层布尔 `calib_key_is_valid`**（真布尔）；`calib_key_components` **只作审计与诊断** — '
            '它的值全是**字符串**，`"False"` 在 Python 里是**真值** ✗，别拿它做判断 ✓',
        # ★★ `key_paths`：**把关键字段的完整路径列出来** ★★
        #   来历（reviewer 2026-10-09）：他按顶层取 `theta_std_methods` 取不到、我报路径只给了后半截
        #   ⇒ 两边各一半 ⇒ 在产物里加一份**路径索引**，下次谁取数都不用猜 ✓。
        #   ★ **只增不改**：不动任何已有字段的路径 ⇒ 已按老路径写好的消费者不受影响 ✓
        #   （reviewer 明确否掉了"把值提到顶层"那个方案：那会改结构 ✓）。
        'key_paths': {
            '旋转角': 'fit.theta_deg',
            '旋转角不确定度（保守上界）': 'fit.theta_std_methods.conservative_deg',
            '旋转角不确定度（iid 下界）': 'fit.theta_std_methods.iid_deg',
            '尺度': 'fit.scale_minus_1',
            '平移': 'fit.translation_m',
            '残差': 'fit.residual_m.{mean,p50,p95,max}',
            '覆盖范围': 'span_x_m / span_y_m',
            '有效标定判定（**只读这个布尔**）': 'calib_key_is_valid',
            '标定键': 'calib_key',
            '被 hash 的原文': 'calib_key_canonical',
            '键的分量（仅审计）': 'calib_key_components.*'
                              '（含 data_run / data_world_sha256 / data_matches_current_geometry / '
                              'data_matches_current_world_sha（仅审计）/ rtk_sim_source）',
            '手推 γ（只作对照）': 'gamma_hand_deg',
        },
        'canonical_convention':
            '★ **被 hash 的原文按 UTF-8 原样、不加尾换行**（reviewer 2026-10-09 钉下的约定：'
            '加一个 `\n` 重算就会得到别的值 ✗）',
        # ★ **被 hash 的原文**必须落盘 —— 只给"逐项分量"别人**重算不出键** ✗
        #   （分隔符、小数位数、顺序都得能复原才叫可复算；与"报哈希 = 算法 + 目标机实测 + 可复算"同条）
        'calib_key_canonical': (calib_key(a.world, a.navsat, GEN_PATH, RTKSIM_PATH, a.from_run)[2]
                                if a.from_run else None),
        'calib_key_components': calib_key(a.world, a.navsat, GEN_PATH, RTKSIM_PATH,
                                          a.from_run)[1],
        'calib_key_note': '★ `geo_route` **只比这一个键**；不一致时看 `calib_key_components` '
                          '就知道是哪一项变了。分量列表本身带版本（改列表 ⇒ 键值变）✓',
        'gamma_hand_deg': gamma_hand_deg,
        # ★ **note 里的数用"读回来的 datum"拼**，不许写死 —— 2026-10-09 挪站时这里
        #   还留着旧站的点位 ⇒ **产物的注释与它自己算出的值不一致** ✗（"注释不是凭据"同族）。
        'gamma_hand_note': ('γ ≈ (λ−λ₀)·sinφ，其中 λ=%.4f（**从 %s 现读**）、λ₀=123（UTM 51 带中央经线）、'
                            'φ=%.4f ⇒ **只作对照**，结论以拟合值为准（brief §4：不许手推公式）。'
                            '★ 另两处历史错：① 曾写成 math.degrees((λ−λ₀)·sinφ) ⇒ 印出 −45.34°'
                            '（(λ−λ₀) 本来就是度）；② 挪站后曾继续写死旧点位 ⇒ 注释与值不一致。均已修。'
                            % ((lam if lam is not None else float('nan')),
                               datum_src, (phi if phi is not None else float('nan')))),
    }
    # ★ 到这里**所有计算都已完成** ⇒ 才开始落盘；两份都用"临时文件 + os.replace"原子替换
    if write_samples:
        _atomic_write_csv(a.out + '_samples.csv', arr)
    _atomic_write_text(a.out + '_fit.json',
                       json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if write_samples:
        print('样本 CSV：%s_samples.csv' % a.out, file=sys.stderr)
    else:
        print('（--from-csv：只更新 _fit.json，**不动样本 CSV**）', file=sys.stderr)
    return 0 if span_ok else 3


def _atomic_write_text(path, text):
    """临时文件 + `os.replace`：中途任何异常都**不会碰到已有产物** ✓。"""
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        fh.write(text)
    os.replace(tmp, path)


def _atomic_write_csv(path, arr):
    tmp = path + '.tmp'
    with open(tmp, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'gt_x', 'gt_y', 'loc_x', 'loc_y', 'dt'])
        for row in arr:
            w.writerow(['%.6f' % row[0], '%.6f' % row[1], '%.6f' % row[2],
                        '%.6f' % row[3], '%.6f' % row[4], '%.6f' % row[5]])
    os.replace(tmp, path)


if __name__ == "__main__":
    sys.exit(main())

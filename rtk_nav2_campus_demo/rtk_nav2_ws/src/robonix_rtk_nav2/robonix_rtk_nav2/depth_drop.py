#!/usr/bin/env python3
"""Detect a descending platform edge from measured depth and publish costmap hazard masks.

No scene coordinates, padded depth cloud, or ground-truth topics are used. The camera's
calibrated mounting height anchors the upper floor; the last measured upper-floor return
before measured lower-floor returns gives the edge. Missing depth is never ground evidence.
"""

import json
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


REF_BAND = (0.55, 1.20)      # 近场参照窗口（forward）
REF_NEIGH = 5                # 邻域列数（与其它门同一口径）
REF_MIN_POINTS = 6           # 拟合最少样本数
REF_RESIDUAL_TOL = 0.02      # 残差 RMS 上限（米）⇒ 超了回退全局 floor
TRUNC_GAP = 0.12             # 截断：forward 间隙（米）⇒ 视为缺测
TRUNC_DZ = 0.05              # 截断：绝对跃变下限（米，抗噪）
TRUNC_SLOPE = 0.6            # 截断：Δz/Δforward 上限（≈31°，比任何可走坡都陡）


def column_reference(forward, world, valid, ncols, floor, ground_ref='local',
                     truncate=True, stats=None):
    """每列的参照高度（长度 ncols 的数组）。`plane` ⇒ 全是 `floor`（= 改前行为）。

    **方案 ①（2026-10-08）**：`upper`/`lower` 的参照从"车脚下的固定水平面"换成
    "**该列自己实测到的局部地面**"——近场（`REF_BAND`）样本按 `forward` 升序、**可截断**后
    拟合 `z(forward)`，**外推回 `forward = 0`** 得该列参照。动机与验收见 docs §7.17/§7.20。

    两条纪律（reviewer 2026-10-08 定）：
    - **截断只作用于"喂给拟合的样本"**，绝不删 `upper` 的锚定样本 —— `upper` 是"可测地面"
      的锚，删了判据会变松、少标；
    - 近场点不足 ⇒ `fallback_short`、残差超限 ⇒ `fallback_residual`，两者都**回退全局 floor**
      （= 改前行为），所以平地/近场缺测时不回归。
    """
    if ground_ref not in ('local', 'local_tilt'):
        return np.full(ncols, floor), np.zeros(ncols), 'plane'
    band = []
    for cc in range(ncols):
        idx = np.where(valid[:, cc] & (forward[:, cc] > REF_BAND[0])
                       & (forward[:, cc] < REF_BAND[1]))[0]
        band.append(idx)
    refs = np.full(ncols, np.nan)
    slopes = np.full(ncols, np.nan)
    counts = {'near_fit': 0, 'fallback_short': 0, 'fallback_residual': 0}
    for col in range(ncols):
        lo = max(0, col - REF_NEIGH)
        hi = min(ncols, col + REF_NEIGH + 1)
        pts = []
        for cc in range(lo, hi):
            for i in band[cc]:
                pts.append((float(forward[i, cc]), float(world[i, cc, 2])))
        if not pts:
            counts['fallback_short'] += 1
            continue
        pts.sort()
        if truncate:
            keep = [pts[0]]
            for prev, cur in zip(pts, pts[1:]):
                dfwd = cur[0] - prev[0]
                drop = prev[1] - cur[1]          # 正 = 沿视线在下降
                if dfwd > TRUNC_GAP or (drop > TRUNC_DZ
                                        and drop / max(dfwd, 1e-6) > TRUNC_SLOPE):
                    break
                keep.append(cur)
            pts = keep
        if len(pts) < REF_MIN_POINTS:
            counts['fallback_short'] += 1
            continue
        f = np.asarray([p[0] for p in pts])
        zz = np.asarray([p[1] for p in pts])
        a, b = np.polyfit(f, zz, 1)
        resid = float(np.sqrt(np.mean((zz - (a * f + b)) ** 2)))
        if resid > REF_RESIDUAL_TOL:
            counts['fallback_residual'] += 1
            continue
        refs[col] = b                             # 外推回 forward=0
        slopes[col] = a                           # 局部面斜率（local_tilt 用）
        counts['near_fit'] += 1
    if stats is not None:
        stats.update(counts)
        stats['max_abs_ref_minus_floor'] = None if not np.any(~np.isnan(refs)) else \
            round(float(np.nanmax(np.abs(np.where(np.isnan(refs), floor, refs) - floor))), 4)
    if ground_ref == 'local_tilt':
        # ★ reviewer 2026-10-08：参照**按该像素自己的 forward 求值**（每列一条倾斜参照面）
        #   `ref(f) = a·f + b`。坡面自身因此落在 `ground_tolerance` 内（up_n 起得来），
        #   而 `lower = z ≤ ref(f) − T` 只对真台阶成立（斜面恒在参照上、不触发）。
        #   注意 `local`（v1）用的是 `b`（截距 = 车身后方的地面）= 整列常数 ⇒
        #   在均匀楔形上与全局 floor 等价（代数必然，非巧合）。
        return np.where(np.isnan(refs), floor, refs), \
            np.where(np.isnan(slopes), 0.0, slopes), 'local_tilt'
    return np.where(np.isnan(refs), floor, refs), np.zeros(ncols), 'local'


def edge_columns(depth, intr, transform, camera_height, drop_threshold,
                 ground_tolerance, stride=2, ground_ref='local',
                 ref_truncate=True, ref_stats=None):
    """Return edge endpoints, valid-ground mask, lower-ground mask in map coordinates.

    Require both sides of a transition to be *measured* ground. A hole, sky, or a
    far-point sentinel never supplies either side. Process image columns independently
    so a longer depth on an ordinary flat floor cannot masquerade as a drop.

    `ground_ref='plane'` 时**逐字复现改前行为**（参照 = 全局 `floor`），用于同栈 A/B。
    """
    fx, fy, cx, cy = intr
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0.05) & (z < 7.95)
    xyz = np.stack(((u-cx)*z/fx, (v-cy)*z/fy, z), axis=-1)
    q = transform.transform.rotation
    offset = transform.transform.translation
    t = np.array([offset.x, offset.y, offset.z])
    world = xyz @ rotation(q).T + t
    floor = t[2] - camera_height
    # Restrict to forward surfaces at an observable distance, on a simple flat apron.
    # Camera optical +Z points forward. Project onto its horizontal direction instead
    # of assuming that the platform edge is aligned with the map axes.
    heading = rotation(q)[:, 2].copy()
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    forward = (world - t) @ heading
    # 参照（方案 ①）：逐列实测局部面（外推回车所在位置）；`plane` 时 = 全局 floor ✓
    ref_arr, slope_arr, ref_mode = column_reference(forward, world, valid, z.shape[1],
                                                   floor, ground_ref, ref_truncate,
                                                   ref_stats)
    # 逐像素参照：ref(f) = ref_arr + slope_arr·f。`plane` 时 slope≡0、ref≡floor
    # ⇒ 与改前表达式**逐位相同**（加 0.0 是精确运算）；`local` 时 slope≡0（整列常数）；
    # `local_tilt` 时按像素自己的 forward 求值（reviewer 2026-10-08 的改法）。
    ref_pix = ref_arr[None, :] + slope_arr[None, :] * forward
    upper = valid & (np.abs(world[..., 2] - ref_pix) <= ground_tolerance)
    lower = valid & (world[..., 2] <= ref_pix - drop_threshold)
    upper &= (forward > 0.55) & (forward < 5.5)
    lower &= (forward > 0.55) & (forward < 7.5)
    edges = []
    for col in range(z.shape[1]):
        above = np.where(upper[:, col])[0]
        below = np.where(lower[:, col])[0]
        if len(above) < 2 or len(below) < 2:
            continue
        # Use the last upper-floor sample, never the first visible lower-floor sample:
        # occlusion makes the latter much farther than the actual edge.
        upper_range = forward[above, col]
        end_idx = above[np.argmax(upper_range)]
        end = world[end_idx, col]
        lower_range = forward[below, col]
        separation = lower_range - forward[end_idx, col]
        visible = (separation > 0.10) & (separation < 3.0)
        if not np.any(visible):
            continue
        # A drop *occludes* the surface behind its lip: no measurement exists
        # between the last upper-floor sample and the first visible lower-floor
        # sample. A ramp is one continuously measured surface, so that same
        # interval is full of returns. Decide on that, not on the lower points'
        # slope: the slope fit needs six lower points and mixes ramp points with
        # the flat floor beyond, so it let the ramp's own upper shelf through as
        # an "edge" (measured 2026-09-30: hazard cells on the ramp entrance at
        # x 0.21..0.36, y 0.45..1.05, which stranded the vehicle at x=0.12).
        #
        # **Count over a column neighborhood, not a single column.** Seen from
        # the ramp's own top edge the slope is nearly edge-on, so it projects to
        # a thin band: one column then holds only one or two samples in that
        # interval and a per-column threshold lets the ramp through again. The
        # reviewer's independent rerun (rev7-ramp-e1) hit exactly that: the
        # vehicle parked at the ramp's top edge and the mask put 29 lethal cells
        # inside the ramp rectangle (global costmap same 29 through KeepoutFilter).
        # Occlusion is a property of the drop's geometry, so it stays empty across
        # the whole neighborhood; a ramp fills it in every nearby column.
        lo = max(0, col - 5); hi = min(z.shape[1], col + 6)
        nearest = lower_range[visible].min()
        r_from = forward[end_idx, col] + 0.02
        r_to = nearest - 0.02
        between = 0
        for cc in range(lo, hi):
            if r_to > r_from:
                between += int(np.count_nonzero(
                    valid[:, cc] & (forward[:, cc] > r_from) & (forward[:, cc] < r_to)))
        if between >= 3:
            continue
        # **主判据（2026-09-30 加，起因是 reviewer 的 rev7-ramp-e1）**：
        # 「边缘 = 一片**水平**表面的尽头」。只看候选边缘**之前**那一段实测表面的斜率：
        #   · 断崖之前是平台（水平，斜率≈0）⇒ 保留为 hazard ✓
        #   · 坡面之前就是正在下降的坡面（斜率≈坡度）⇒ 判为坡、跳过 ✓
        # 为什么比"边缘之后有没有实测点"可靠：那个判据依赖**边缘之后**的采样密度，
        # 而车停在坡顶沿、几乎是贴着坡面看过去时，坡面在图像里被压得很薄，
        # 之后那一段可能只有一两个样点 ⇒ 单列/邻域的计数都可能落空、坡面又被报成边缘
        # （reviewer 独立复跑实测：车停在坡顶时 mask 在坡面矩形内出现 29 格）。
        # 本判据只用**边缘之前**的样点，而那一段永远是稠密的（车正看着它）。
        r_edge = forward[end_idx, col]
        rr, zz = [], []
        for cc in range(lo, hi):
            m = valid[:, cc] & (forward[:, cc] > r_edge - 0.40) & (forward[:, cc] <= r_edge + 0.02)
            if np.any(m):
                rr.extend(forward[m, cc].tolist())
                zz.extend(world[m, cc, 2].tolist())
        if len(rr) >= 6:
            pre_slope = float(np.polyfit(np.asarray(rr), np.asarray(zz), 1)[0])
            if abs(pre_slope) > 0.05:
                continue
        # Secondary guard, kept from the first ramp attempt: a level lower slab
        # behind a steep edge has near-zero slope, a smooth descent does not.
        # Use only measured lower points in a small image-column neighborhood;
        # never use scene geometry here.
        lo = max(0, col - 5); hi = min(z.shape[1], col + 6)
        lr, lz = [], []
        for cc in range(lo, hi):
            rr = np.where(lower[:, cc])[0]
            if len(rr):
                lr.extend(forward[rr, cc].tolist())
                lz.extend(world[rr, cc, 2].tolist())
        if len(lr) >= 6:
            slope = float(np.polyfit(np.asarray(lr), np.asarray(lz), 1)[0])
            # Smooth downward surfaces (ramps) are intentionally traversable.
            # A level lower slab behind a steep edge has slope near zero.
            if abs(slope) > 0.045:
                continue
        edges.append(end[:2])
    return np.array(edges).reshape(-1, 2), upper, lower, heading


def near_ground_z(depth, intr, transform, stride=2, near=(0.55, 1.10)):
    """近场实测地面高度（map 系 z 的中位，只用**实测**点）。

    2026-10-08 加（阶段七实验 P）：`floor = 发布 z − camera_height` 一旦因**发布 z 的偏置**
    或**本节点参数与部署值不一致**而偏掉，`upper` 判据就恒空、检测器就等于失明，而外观与
    "真干净"完全一样。本函数给出一个**不依赖发布 z** 的地面参照，做活性/一致性指纹：
    与 `floor` 差得远就是"基准不对"的铁证（**当时那场"失明"误判就属于这一类**：
    读数工具自己把 `camera_height` 写成 0.19、部署值是 0.55）。

    **不参与判据**（当前只落证据 + 告警）：把 `floor` 换成它会牵动坡面灵敏度——
    沿坡下行时近场中位比脚下低约 `band_mid · slope`（本坡 0.82 m × 0.187 ≈ 0.15 m），
    `lower` 的门就会整体下移同样的量。要换必须先有坡面实测，见文档 §7.11。
    """
    fx, fy, cx, cy = intr
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0.05) & (z < 7.95)
    if not np.any(valid):
        return None, 0
    xyz = np.stack(((u-cx)*z/fx, (v-cy)*z/fy, z), axis=-1)
    q = transform.transform.rotation
    offset = transform.transform.translation
    t = np.array([offset.x, offset.y, offset.z])
    world = xyz @ rotation(q).T + t
    heading = rotation(q)[:, 2].copy()
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    forward = (world - t) @ heading
    m = valid & (forward > near[0]) & (forward < near[1])
    n = int(m.sum())
    if n == 0:
        return None, 0
    return float(np.median(world[..., 2][m])), n


class DropDetector(Node):
    def __init__(self):
        super().__init__('depth_drop')
        self.declare_parameter('camera_height', 0.19)
        self.declare_parameter('drop_threshold', 0.15)
        self.declare_parameter('ground_tolerance', 0.06)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('min_columns', 12)
        self.declare_parameter('evidence_dir', '')
        self.declare_parameter('mask_stale_timeout', 1.0)
        # 禁行区按车体足迹外扩的量（米）。0.15 = 内切半径 0.105 + 5 cm 余量；
        # 目的：让"规划器避开的地方"与"控制器拒绝的地方"是同一条口径（见 publish_mask 的说明）。
        self.declare_parameter('hazard_margin', 0.15)
        # 活性自检容差（米）：`floor` 与近场实测地面差超过它 ⇒ 打告警（**只告警，不改行为**）。
        self.declare_parameter('floor_consistency_tol', 0.25)
        # ★ 方案 ①（2026-10-08 AuYang 批）：参照来源。`local` = 逐列实测局部面（新行为）、
        #   `plane` = 车脚下固定水平面（= 改前行为，用于**同栈 A/B**）。
        #   `ref_truncate` = 参照拟合前是否按"由近及远遇第一个不连续就截断"裁剪样本
        #   （reviewer 要求的"截断开/关逐格 A/B"用它）。
        # ★ 2026-10-08 reviewer 裁定：**默认 `plane`（= 改前行为）**。`local` / `local_tilt`
        #   保留为开关，但**记为负结果：对 Q（沿坡南行不成列）无效** ——
        #   合成实测（docs §7.22）：`local` 让 `up_px` 涨 ~2.8× 而出列数/框架跨度**一动没动**，
        #   且在 `H ≈ drop_threshold` 的边界例里把 6 列**压到 0**（"变松少标"真实发生）；
        #   `local_tilt` 反而把沿坡出列数 13→5、贴门槛 6→1，跨度仍 <0.5 ⇒ 无收益且有损失。
        self.declare_parameter('ground_ref', 'plane')
        # reviewer 2026-10-08 裁定：**默认关**（A/B 差集非空，且那 1 格正在 `H ≈ T` 边界上
        # ⇒ ② 把 T 降到 0.10 后必须重跑该 A/B，否则"边界翻转"只是换个位置再出现）。
        self.declare_parameter('ref_truncate', False)
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.drop_threshold = float(self.get_parameter('drop_threshold').value)
        self.ground_tolerance = float(self.get_parameter('ground_tolerance').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.min_columns = int(self.get_parameter('min_columns').value)
        self.evidence_dir = str(self.get_parameter('evidence_dir').value or '')
        self.mask_stale_timeout = float(self.get_parameter('mask_stale_timeout').value)
        self.hazard_margin = float(self.get_parameter('hazard_margin').value)
        self.floor_consistency_tol = float(self.get_parameter('floor_consistency_tol').value)
        self.ground_ref = str(self.get_parameter('ground_ref').value)
        self.ref_truncate = bool(self.get_parameter('ref_truncate').value)
        self.last_confirmed_wall = None
        # Persistent map-frame hazard store. Confirmed edges only add cells; loss of
        # view never removes them. Local masks are projected from this same store.
        self.hazard_points = []
        self.frame_log = None
        self.merge_log = None
        self.edges_log = None
        if self.evidence_dir:
            os.makedirs(self.evidence_dir, exist_ok=True)
            self.frame_log = open(os.path.join(self.evidence_dir, 'frames.jsonl'), 'a', encoding='utf-8')
            # 每次**入库事件**逐格记录（reviewer 2026-10-08 要求 (b) 的逐格证据）
            self.merge_log = open(os.path.join(self.evidence_dir, 'merge_events.jsonl'), 'a', encoding='utf-8')
            # ★ 逐帧 dump **通过了四道列判据、但还没过框架门（min_columns/ptp）** 的边坐标。
            #   用途（reviewer 2026-10-08）：回答"放宽跨度门会放进来哪些边、落在哪"——
            #   若那些边不在南沿上，放宽门只是把别处的短边放进图 = 新增假阳性。
            self.edges_log = open(os.path.join(self.evidence_dir, 'frame_edges.jsonl'), 'a', encoding='utf-8')
        self.intr = None
        self.info_stamp = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/rtk/drop/mask_map', latched)
        self.odom_pub = self.create_publisher(OccupancyGrid, '/rtk/drop/mask_odom', latched)
        self.map_info = self.create_publisher(CostmapFilterInfo, '/rtk/drop/filter_info_map', latched)
        self.odom_info = self.create_publisher(CostmapFilterInfo, '/rtk/drop/filter_info_odom', latched)
        for frame, pub in (('map', self.map_info), ('odom', self.odom_info)):
            info = CostmapFilterInfo()
            info.header.frame_id = frame
            info.type = 0  # nav2 KeepoutFilter
            info.filter_mask_topic = '/rtk/drop/mask_' + frame
            info.base = 0.0
            info.multiplier = 1.0
            pub.publish(info)
        self.display_pub = self.create_publisher(Image, '/rtk/drop/classification', 5)
        self.frames = 0
        self.create_subscription(CameraInfo, '/rtk/camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        self.pending_depth = None
        self.pending_since = None
        self.pending_attempts = 0
        self.create_subscription(Image, '/rtk/camera/depth_image', self.on_depth,
                                 qos_profile_sensor_data)
        # The camera bridge can deliver an image a few milliseconds before /tf.
        # Retry the same frame instead of replacing it with the next frame.
        self.create_timer(0.03, self.process_depth)

    def on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.intr = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.info_stamp = msg.header.stamp

    def on_depth(self, msg):
        if msg.encoding == '32FC1' and self.intr is not None:
            if self.pending_depth is None:
                self.pending_since = self.get_clock().now()
                self.pending_attempts = 0
            self.pending_depth = msg

    def process_depth(self):
        msg = self.pending_depth
        if msg is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        depth_stamp = stamp.nanoseconds * 1e-9
        retry_count = self.pending_attempts
        retry_start = time.time()
        self.pending_attempts += 1
        try:
            if abs((stamp - Time.from_msg(self.info_stamp)).nanoseconds) > 1_000_000_000:
                self.get_logger().warn('CameraInfo older than 1 s; frame undecidable', throttle_duration_sec=5.0)
                return
            # TF often lags the depth image by 2–20 ms. Ask for that image's stamp
            # with a bounded wait; do not substitute the latest transform.
            tf_map = self.tf.lookup_transform('map', msg.header.frame_id, stamp)
            tf_odom = self.tf.lookup_transform('odom', 'map', stamp)
            self.pending_depth = None
            self.pending_since = None
            self.pending_attempts = 0
            tf_stamp = stamp.nanoseconds * 1e-9
            data = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        except (Exception,) as exc:
            # Keep the pending frame for up to 0.25 s. A bounded retry makes a
            # 12–20 ms TF lag a recoverable transport skew, not a false detector
            # failure; after the bound the frame becomes undecidable and is dropped.
            if (self.pending_since is not None and
                    (self.get_clock().now() - self.pending_since).nanoseconds < 250_000_000):
                return
            self.pending_depth = None
            self.pending_since = None
            self.get_logger().warn('Depth/TF unavailable after bounded retry: %s' % exc,
                                   throttle_duration_sec=5.0)
            return
        ref_stats = {}
        # ★ 2026-10-08 修：`ptp_across` 必须在**写 frames.jsonl 之前**就有值 ——
        #   上一版把它初始化放在日志块之后 ⇒ 节点启动即 `UnboundLocalError` 崩掉
        #   （`ramp7-ab` 连拦三次就是它；合成测试只调 `edge_columns`，不经过这里，所以没暴露）。
        #   教训：`py_compile` 只查语法，**运行路径必须真跑一次**（就绪闸门就是这个作用）。
        ptp_across = None
        # ★ 2026-10-08（② 同栈 A/B 需要）：`drop_threshold` **每次调用重读**，
        #   这样 `ros2 param set /depth_drop drop_threshold 0.10` 当场生效、不必重启栈。
        #   与 `hazard_margin` 那处同一写法；**参数不变时行为与改前逐位相同**（只是多读一次）。
        drop_th = float(self.get_parameter('drop_threshold').value)
        edges, upper, lower, heading = edge_columns(
            data, self.intr, tf_map, self.camera_height, drop_th,
            self.ground_tolerance, ground_ref=self.ground_ref,
            ref_truncate=self.ref_truncate, ref_stats=ref_stats)
        # 活性指纹（2026-10-08，reviewer 定案的新验收协议）：**"没有标记"这种结论必须同时
        # 给出"检测器当时活着"的证据**——否则"真干净"和"瞎了"外观完全一样。
        # `live` = 站在其上的地面被认出来（`upper_px > 0`），这是任何检测成立的前提。
        # `floor − near_ground_z` = "地面基准偏了"的指纹（0.35 那种量级的偏置一眼可见）。
        # 注：本字段是在**一场把"读节点的工具参数拿错"误判成"检测器失明"的事故**之后加的
        # ——它本来会在当时就暴露两套 `floor` 不一致。`floor_consistency_tol` 取 0.25 m
        # 是留出**坡面**的正常差（沿坡下行时近场中位天生比脚下低 ~0.15 m）。
        floor = float(tf_map.transform.translation.z) - self.camera_height
        near_z, near_n = near_ground_z(data, self.intr, tf_map)
        delta = None if near_z is None else floor - near_z
        if delta is not None and abs(delta) > self.floor_consistency_tol:
            self.get_logger().warn(
                '地面基准与近场实测地面差 %.3f m（floor %.3f / 实测 %.3f，%d 点）'
                '——发布 z 有偏置时检测器会失明，这条是活性告警（**不改行为**）'
                % (delta, floor, near_z, near_n), throttle_duration_sec=5.0)
        # 记录用：**未过滤**的列数（与历史口径一致，不要改成过滤后的）+ 宽松预筛结果
        n_cols_raw = int(len(edges))
        if self.edges_log is not None and n_cols_raw:
            self.edges_log.write(json.dumps({
                # ★ 连接键（reviewer 2026-10-08）：`depth_stamp` 与 `frames.jsonl` **同一个值** ⇒ 可精确 join；
                #   `wall` 在本函数里两处各调一次 `time.time()`，会差几毫秒 ⇒ **不要用 wall 做精确连接** ✗
                'depth_stamp': depth_stamp,
                'wall': time.time(), 'drop_th': drop_th, 'ground_ref': self.ground_ref,
                'n_edges': n_cols_raw,
                'edges': [[round(float(q[0]), 3), round(float(q[1]), 3)]
                          for q in np.asarray(edges).reshape(-1, 2)],
            }) + '\n'); self.edges_log.flush()
        prefilter_ok = bool(n_cols_raw >= self.min_columns)
        display = np.zeros((msg.height, msg.width), dtype=np.uint8)
        display[::2, ::2][upper] = 110
        display[::2, ::2][lower] = 210
        confirmed = len(edges) >= self.min_columns
        if confirmed:
            # Reject isolated columns without assuming the edge is x=constant.
            axis = np.array([-heading[1], heading[0]])
            along = edges @ heading[:2]
            across = edges @ axis
            median = np.median(along)
            edges = edges[np.abs(along-median) <= 0.20]
            ptp_across = float(np.ptp(across))     # 只记录（判据仍用同一个值，未改）
            confirmed = (len(edges) >= self.min_columns and
                         np.ptp(across) >= 0.5)
        if confirmed:
            self.last_confirmed_wall = time.time()
            # Merge only measured, confirmed edge points in fixed map coordinates.
            # No clearing path is allowed to remove this store during view loss.
            if self.hazard_points:
                merged = np.vstack([np.asarray(self.hazard_points), edges])
            else:
                merged = edges
            before = np.asarray(self.hazard_points) if self.hazard_points else np.zeros((0, 2))
            keep = []
            for point in merged:
                if not any(np.linalg.norm(point-other) < 0.08 for other in keep):
                    keep.append(point)
            # ★ 2026-10-08（reviewer 要求 (b) 的逐格证据）：记下**本次入库真正新增**的格 ——
            #   判据 = "与入库前的任何格距离 ≥0.08 m"（与上面的去重口径一致）。
            added = [list(map(float, p_)) for p_ in keep
                     if not len(before) or np.min(np.linalg.norm(before - np.asarray(p_), axis=1)) >= 0.08]
            self.hazard_points = keep
            if self.merge_log is not None:
                self.merge_log.write(json.dumps({
                    'wall': time.time(), 'n_before': int(len(before)),
                    'n_after': int(len(keep)), 'n_added': int(len(added)),
                    'added_cells': added, 'drop_th': drop_th, 'ground_ref': self.ground_ref,
                }) + '\n'); self.merge_log.flush()
        if self.frame_log:
            self.frame_log.write(json.dumps({
                'wall': time.time(), 'depth_stamp': depth_stamp,
                'tf_requested_stamp': depth_stamp, 'tf_return_stamp': tf_stamp,
                'tf_retry_count': retry_count,
                'tf_delay_ms': round((tf_stamp-depth_stamp)*1000, 3),
                'valid_pixels': int(np.isfinite(data).sum()),
                # 口径不变：仍是**未过滤**的列数（历史可比）
                'edge_columns': n_cols_raw,
                # ★ 改名（reviewer 2026-10-08）：旧的 'confirmed' 只是"列数≥12"的**宽松预筛**，
                #   不是入库确认 ⇒ 现名 `prefilter_ok`，真正的入库条件单列 `merged`。
                'prefilter_ok': prefilter_ok,
                'merged': bool(confirmed),          # = 列数≥12 **且** ptp(across)≥0.5 且随后入库
                'ptp_across': ptp_across,           # ★ 现在写的是**真值**（在过滤块里算出来的那个）
                'upper_px': int(upper.sum()), 'lower_px': int(lower.sum()),
                'floor': round(floor, 4),
                'near_ground_z': None if near_z is None else round(near_z, 4),
                'near_ground_n': near_n,
                'floor_minus_near_ground': None if delta is None else round(delta, 4),
                'live': bool(int(upper.sum()) > 0),
                'ground_ref': self.ground_ref,
                'drop_th': drop_th,
                'ref_source': ref_stats or None,
            }) + '\n'); self.frame_log.flush()
        if self.hazard_points:
            map_edges = np.asarray(self.hazard_points)
            self.publish_mask('map', map_edges, msg.header.stamp, self.map_pub)
            q = tf_odom.transform.rotation
            p = tf_odom.transform.translation
            odom_edges = map_edges @ rotation(q)[:2, :2].T + np.array([p.x, p.y])
            self.publish_mask('odom', odom_edges, msg.header.stamp, self.odom_pub)
            # Magenta/bright edge in the classification image (valid ground=110, lower=210).
            for u in range(0, msg.width, 2):
                if np.any(upper[:, u//2]) and np.any(lower[:, u//2]):
                    v = np.where(upper[:, u//2])[0][0] * 2
                    display[max(0, v-2):min(msg.height, v+3), u:u+2] = 255
        # Lifecycle evidence: do not keep publishing a stale keepout mask forever.
        # The filter receives no synthetic free mask; after TTL the mask remains the
        # last measured mask, while the evidence stream records STALE_UNKNOWN.
        if self.frame_log and self.last_confirmed_wall is not None:
            age = time.time() - self.last_confirmed_wall
            if age > self.mask_stale_timeout:
                self.frame_log.write(json.dumps({'wall': time.time(), 'lifecycle': 'STALE_UNKNOWN',
                                                 'last_confirmed_age_s': age}) + '\n'); self.frame_log.flush()
        image = Image()
        image.header = msg.header
        image.height = msg.height
        image.width = msg.width
        image.encoding = 'mono8'
        image.step = msg.width
        image.data = display.tobytes()
        self.display_pub.publish(image)
        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info('upper=%d lower=%d edge_columns=%d confirmed=%s' %
                                   (upper.sum(), lower.sum(), len(edges), confirmed))

    def publish_mask(self, frame, points, stamp, publisher):
        # Sparse local window. Unknown (-1) everywhere outside measured edge samples:
        # no synthetic free ground is emitted by this detector.
        origin = np.min(points, axis=0) - 0.40
        max_xy = np.max(points, axis=0) + 0.40
        width, height = (np.ceil((max_xy-origin)/self.resolution).astype(int) + 1)
        if width > 240 or height > 240:
            return
        grid = np.full((height, width), -1, dtype=np.int8)
        # Neighboring camera columns can land more than one 5 cm cell apart.
        # Connect only adjacent, coherent measurements; never fill a missing span.
        cells = np.floor((points-origin)/self.resolution).astype(int)
        order = np.argsort(points[:, 1])
        for index in order:
            ix, iy = cells[index]
            if 0 <= ix < width and 0 <= iy < height:
                grid[iy, ix] = 100
        for left, right in zip(order[:-1], order[1:]):
            a, b = points[left], points[right]
            if np.linalg.norm(b-a) > 0.16:
                continue
            steps = max(1, int(np.ceil(np.linalg.norm(cells[right]-cells[left]))))
            for ix, iy in np.rint(np.linspace(cells[left], cells[right], steps+1)).astype(int):
                if 0 <= ix < width and 0 <= iy < height:
                    grid[iy, ix] = 100
        # ★ 2026-10-08：把禁行区**按车体足迹外扩**再交给 KeepoutFilter。
        #
        # 为什么（AuYang 的"直连目标"用例 `(-2,0) → (2,-1)` 实测死锁，reviewer 取证）：
        #   规划器（NavFn）把 keepout 的**膨胀环**当"代价高但可通行"，于是路径**擦着**边缘给出；
        #   控制器（RPP）按车体足迹判断，凡是离致命格 < 内切半径(0.105 m) 就拒绝 ⇒ **一步不能动**，
        #   重规划又给同一条路径 ⇒ 死锁（实测 `collision ahead` 580、`patience exceeded` 101、
        #   前方 15 cm 就有 cost=100 的格）。
        #   根因是**规划与控制对"离禁行区留多少余量"口径不一致**：mask 写进代价地图后没有按
        #   车体半径外扩到控制器同一条口径。这里在**产生 mask 的地方**补上，两个消费端（全局图/
        #   局部图）就自动一致。
        # 外扩量取 `hazard_margin`（默认 0.15 m = 内切半径 0.105 + 5 cm 余量）：
        #   **只补到"控制器会拒绝的那一圈"**，比 inflation_radius（0.25，那是代价不是禁行）小，
        #   所以不会把可行走区吃掉更多。它同时是对"检测器只标已看见的那段边缘"的补偿：
        #   没看见的那段不会有人拦，但车只要离**已标出**的部分留 0.15 m，就不会从边缘蹭过去。
        # 每次调用都重读参数：这样 `ros2 param set /rtk_depth_drop hazard_margin X` 就能做
        # 0.15/0.25/0.30 的对照跑，不必改文件、不必重启（默认值仍是 0.15）。
        margin = float(self.get_parameter('hazard_margin').value)
        margin_cells = int(np.ceil(margin / self.resolution))
        if margin_cells > 0:
            dilated = grid >= 100
            for _ in range(margin_cells):
                shifted = [np.roll(dilated, 1, axis=0), np.roll(dilated, -1, axis=0),
                           np.roll(dilated, 1, axis=1), np.roll(dilated, -1, axis=1)]
                for s in shifted:
                    dilated = dilated | s
            grid[dilated] = 100
        out = OccupancyGrid()
        out.header.stamp = stamp
        out.header.frame_id = frame
        out.info.resolution = self.resolution
        out.info.width = int(width)
        out.info.height = int(height)
        out.info.origin.position.x = float(origin[0])
        out.info.origin.position.y = float(origin[1])
        out.info.origin.orientation.w = 1.0
        out.data = grid.reshape(-1).astype(np.int8).tolist()
        publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DropDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.frame_log:
            node.frame_log.close()
        if node.merge_log:
            node.merge_log.close()
        if node.edges_log:
            node.edges_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

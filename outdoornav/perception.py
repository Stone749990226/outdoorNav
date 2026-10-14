"""ROS-independent depth geometry and negative-obstacle detection."""

import numpy as np


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


REF_BAND = (0.55, 1.20)  # 近场参照窗口（forward）
REF_NEIGH = 5  # 邻域列数（与其它门同一口径）
REF_MIN_POINTS = 6  # 拟合最少样本数
REF_RESIDUAL_TOL = 0.02  # 残差 RMS 上限（米）⇒ 超了回退全局 floor
TRUNC_GAP = 0.12  # 截断：forward 间隙（米）⇒ 视为缺测
TRUNC_DZ = 0.05  # 截断：绝对跃变下限（米，抗噪）
TRUNC_SLOPE = 0.6  # 截断：Δz/Δforward 上限（≈31°，比任何可走坡都陡）


def column_reference(
    forward, world, valid, ncols, floor, ground_ref="local", truncate=True, stats=None
):
    """Fit a per-column ground reference; plane mode uses the fixed floor."""
    if ground_ref not in ("local", "local_tilt"):
        return np.full(ncols, floor), np.zeros(ncols), "plane"
    band = []
    for cc in range(ncols):
        idx = np.where(
            valid[:, cc]
            & (forward[:, cc] > REF_BAND[0])
            & (forward[:, cc] < REF_BAND[1])
        )[0]
        band.append(idx)
    refs = np.full(ncols, np.nan)
    slopes = np.full(ncols, np.nan)
    counts = {"near_fit": 0, "fallback_short": 0, "fallback_residual": 0}
    for col in range(ncols):
        lo = max(0, col - REF_NEIGH)
        hi = min(ncols, col + REF_NEIGH + 1)
        pts = []
        for cc in range(lo, hi):
            for i in band[cc]:
                pts.append((float(forward[i, cc]), float(world[i, cc, 2])))
        if not pts:
            counts["fallback_short"] += 1
            continue
        pts.sort()
        if truncate:
            keep = [pts[0]]
            for prev, cur in zip(pts, pts[1:]):
                dfwd = cur[0] - prev[0]
                drop = prev[1] - cur[1]  # 正 = 沿视线在下降
                if dfwd > TRUNC_GAP or (
                    drop > TRUNC_DZ and drop / max(dfwd, 1e-6) > TRUNC_SLOPE
                ):
                    break
                keep.append(cur)
            pts = keep
        if len(pts) < REF_MIN_POINTS:
            counts["fallback_short"] += 1
            continue
        f = np.asarray([p[0] for p in pts])
        zz = np.asarray([p[1] for p in pts])
        a, b = np.polyfit(f, zz, 1)
        resid = float(np.sqrt(np.mean((zz - (a * f + b)) ** 2)))
        if resid > REF_RESIDUAL_TOL:
            counts["fallback_residual"] += 1
            continue
        refs[col] = b  # 外推回 forward=0
        slopes[col] = a  # 局部面斜率（local_tilt 用）
        counts["near_fit"] += 1
    if stats is not None:
        stats.update(counts)
        stats["max_abs_ref_minus_floor"] = (
            None
            if not np.any(~np.isnan(refs))
            else round(
                float(np.nanmax(np.abs(np.where(np.isnan(refs), floor, refs) - floor))),
                4,
            )
        )
    if ground_ref == "local_tilt":
        #   `ref(f) = a·f + b`。坡面自身因此落在 `ground_tolerance` 内（up_n 起得来），
        #   而 `lower = z ≤ ref(f) − T` 只对真台阶成立（斜面恒在参照上、不触发）。
        #   注意 `local`（v1）用的是 `b`（截距 = 车身后方的地面）= 整列常数 ⇒
        #   在均匀楔形上与全局 floor 等价（代数必然，非巧合）。
        return (
            np.where(np.isnan(refs), floor, refs),
            np.where(np.isnan(slopes), 0.0, slopes),
            "local_tilt",
        )
    return np.where(np.isnan(refs), floor, refs), np.zeros(ncols), "local"


def edge_columns(
    depth,
    intr,
    transform,
    camera_height,
    drop_threshold,
    ground_tolerance,
    stride=2,
    ground_ref="local",
    ref_truncate=True,
    ref_stats=None,
):
    """Find measured depth discontinuities with upper/lower support and slope rejection."""
    fx, fy, cx, cy = intr
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0.05) & (z < 7.95)
    xyz = np.stack(((u - cx) * z / fx, (v - cy) * z / fy, z), axis=-1)
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

    ref_arr, slope_arr, ref_mode = column_reference(
        forward, world, valid, z.shape[1], floor, ground_ref, ref_truncate, ref_stats
    )
    # 逐像素参照：ref(f) = ref_arr + slope_arr·f。`plane` 时 slope≡0、ref≡floor
    # ⇒ 与改前表达式**逐位相同**（加 0.0 是精确运算）；`local` 时 slope≡0（整列常数）；

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

        # x 0.21..0.36, y 0.45..1.05, which stranded the vehicle at x=0.12).
        #
        # **Count over a column neighborhood, not a single column.** Seen from
        # the ramp's own top edge the slope is nearly edge-on, so it projects to
        # a thin band: one column then holds only one or two samples in that
        # interval and a per-column threshold lets the ramp through again. The

        # vehicle parked at the ramp's top edge and the mask put 29 lethal cells
        # inside the ramp rectangle (global costmap same 29 through KeepoutFilter).
        # Occlusion is a property of the drop's geometry, so it stays empty across
        # the whole neighborhood; a ramp fills it in every nearby column.
        lo = max(0, col - 5)
        hi = min(z.shape[1], col + 6)
        nearest = lower_range[visible].min()
        r_from = forward[end_idx, col] + 0.02
        r_to = nearest - 0.02
        between = 0
        for cc in range(lo, hi):
            if r_to > r_from:
                between += int(
                    np.count_nonzero(
                        valid[:, cc]
                        & (forward[:, cc] > r_from)
                        & (forward[:, cc] < r_to)
                    )
                )
        if between >= 3:
            continue

        # 而车停在坡顶沿、几乎是贴着坡面看过去时，坡面在图像里被压得很薄，
        # 之后那一段可能只有一两个样点 ⇒ 单列/邻域的计数都可能落空、坡面又被报成边缘

        # 本判据只用**边缘之前**的样点，而那一段永远是稠密的（车正看着它）。
        r_edge = forward[end_idx, col]
        rr, zz = [], []
        for cc in range(lo, hi):
            m = (
                valid[:, cc]
                & (forward[:, cc] > r_edge - 0.40)
                & (forward[:, cc] <= r_edge + 0.02)
            )
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
        lo = max(0, col - 5)
        hi = min(z.shape[1], col + 6)
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
    """Measure median near-field ground height for diagnostics, without changing detection thresholds."""
    fx, fy, cx, cy = intr
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > 0.05) & (z < 7.95)
    if not np.any(valid):
        return None, 0
    xyz = np.stack(((u - cx) * z / fx, (v - cy) * z / fy, z), axis=-1)
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

#!/usr/bin/env python3
"""方案 ① 的**合成预检**（不起栈、不碰节点）：三条，都在花栈时间之前做完。

1. **`ground_ref='plane'` 必须复现改前行为**：同一张合成深度图，改前函数与新代码（`plane`）
   的 `edges / upper / lower` 必须**完全一致**（逐元素比）。
2. **`local` 要解决 Q**：合成"车在坡面上朝南看、南沿在正前方"的深度图 ⇒ 期望 `local` 出列
   （改前/`plane` 出 0 列），且 `ref_source` 显示参照大多来自近场拟合。
3. **截断开/关逐格 A/B**（reviewer 要求 (c)）：`ref_truncate=True/False` 的 `edges` 集合
   差集必须为**空**；非空就报出来（不并进 ①）。

场景（解析面，map 系）：高台 `x<0` 顶面 `z=0`；坡面 `x∈[0,1.6], y∈[0.5,3.0]`
顶面 `z=-0.15-0.18752(x-0.8)`；其余低台面 `z=-0.30`。深度按针孔模型取**沿光轴的 z**
（与 `edge_columns` 的假设一致）。
"""
import importlib.util
import sys

import numpy as np

sys.path.insert(0, '/work/rtk_nav2_ws/src/robonix_rtk_nav2')


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def z_surf_vec(x, y):
    z = np.full_like(x, -0.30)
    z = np.where(x < 0.0, 0.0, z)
    on_ramp = (x >= 0.0) & (x <= 1.60) & (y >= 0.5) & (y <= 3.0)
    z = np.where(on_ramp, -0.15 - 0.18752 * (x - 0.8), z)
    return z


R_CAM2BODY = np.array([[0.0, 0.0, 1.0],      # 相机 x（右） → 车体 −y
                       [-1.0, 0.0, 0.0],     # 相机 y（下） → 车体 −z
                       [0.0, -1.0, 0.0]])    # 相机 z（前） → 车体 +x


def body_to_map(yaw, roll, pitch=0.0):
    """车体→map：先滚（绕车体 x）再俯仰（绕 y）再偏航（绕 z）。"""
    cy, sy = np.cos(yaw), np.sin(yaw)
    cx, sx = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Rz @ Ry @ Rx


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat(R):
    tr = np.trace(R)
    w = np.sqrt(max(1e-9, 1 + tr)) / 2
    return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
            (R[1, 0] - R[0, 1]) / (4 * w), w)


class _T:
    pass


class Tf:
    def __init__(self, R, t):
        self.transform = _T()
        self.transform.rotation = _T()
        x, y, z, w = quat(R)
        self.transform.rotation.x, self.transform.rotation.y = x, y
        self.transform.rotation.z, self.transform.rotation.w = z, w
        self.transform.translation = _T()
        self.transform.translation.x = float(t[0])
        self.transform.translation.y = float(t[1])
        self.transform.translation.z = float(t[2])


def render(R, t, intr, W=320, H=240, far=8.0):
    """解析场景 → z 深度图（沿光轴），按 t 步进向量化求交。"""
    fx, fy, cx, cy = intr
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    dirs = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=float)], axis=-1)
    d_map = dirs @ R.T
    d_map /= np.linalg.norm(d_map, axis=-1, keepdims=True)
    z_axis_dir = d_map @ R[:, 2]                       # 沿光轴分量
    depth = np.full((H, W), np.nan)
    hit = np.zeros((H, W), bool)
    for tt in np.arange(0.30, far, 0.01):
        P = t[None, None, :] + tt * d_map
        below = (P[..., 2] <= z_surf_vec(P[..., 0], P[..., 1]) + 0.002)
        new = below & ~hit
        if new.any():
            depth[new] = tt * z_axis_dir[new]
            hit |= new
        if hit.all():
            break
    return depth


def main():
    ws = '/work/rtk_nav2_ws/src/robonix_rtk_nav2'
    new = load(ws + '/robonix_rtk_nav2/depth_drop.py', 'newdd')
    base = load(sys.argv[1] if len(sys.argv) > 1 else '/tmp/depth_drop_baseline.py', 'basedd')
    intr = (120.0, 120.0, 160.0, 120.0)
    CH, DT, GT = 0.55, 0.15, 0.06
    # ★ 三档姿态按 reviewer 要求取"离坎 0.6/0.8/1.0 m"（坎在 y=0.5）：
    #   x 取 0.5（该处台阶 H=0.206 m，**明显高于** drop_threshold 0.15 ⇒ 用来判"local 该不该出列"）；
    #   x=0.8（H 恰好 0.150 ⇒ **贴门槛**）单独跑一条，用来记录"贴及时的表现"，不当失败依据。
    cases = [
        ('坡上朝南·离坎 0.6 m（x=0.5,y=1.1）', 0.5, 1.1, -np.pi / 2, 0.18534),
        ('坡上朝南·离坎 0.8 m（x=0.5,y=1.3）', 0.5, 1.3, -np.pi / 2, 0.18534),
        ('坡上朝南·离坎 1.0 m（x=0.5,y=1.5）', 0.5, 1.5, -np.pi / 2, 0.18534),
        ('坡上朝南·贴门槛（x=0.8,y=1.5，H=0.150）', 0.8, 1.5, -np.pi / 2, 0.18534),
        ('高台上朝东看断崖（x=-1.9,y=0）', -1.9, 0.0, 0.0, 0.0),
        ('坡上朝东顺坡下行（x=0.5,y=1.5）', 0.5, 1.5, 0.0, 0.18534),
    ]
    all_ok = True
    for name, x, y, yaw, roll in cases:
        R = body_to_map(yaw, roll) @ R_CAM2BODY   # 相机→map
        t = np.array([x, y, float(z_surf_vec(np.array(x), np.array(y))) + CH])
        depth = render(R, t, intr)
        tf = Tf(R, t)
        e_b, u_b, l_b, _ = base.edge_columns(depth, intr, tf, CH, DT, GT)
        st_p = {}
        e_p, u_p, l_p, _ = new.edge_columns(depth, intr, tf, CH, DT, GT,
                                            ground_ref='plane', ref_stats=st_p)
        same = np.array_equal(np.asarray(e_b), np.asarray(e_p))
        same_m = np.array_equal(u_b, u_p) and np.array_equal(l_b, l_p)
        modes = {}
        for mode in ('plane', 'local', 'local_tilt'):
            stm = {}
            em, um, lm, hdm = new.edge_columns(depth, intr, tf, CH, DT, GT,
                                               ground_ref=mode, ref_stats=stm)
            em_a = np.asarray(em).reshape(-1, 2)
            if len(em_a) >= 2:
                axis = np.array([-hdm[1], hdm[0]])
                spm = float(np.ptp(em_a @ axis))
            else:
                spm = 0.0
            modes[mode] = {'edges': len(em), 'up_px': int(um.sum()),
                           'span': spm, 'stats': stm}
        st_l = modes['local']['stats']
        e_l = np.asarray([]).reshape(-1, 2)
        e_l2, _, _, _ = new.edge_columns(depth, intr, tf, CH, DT, GT, ground_ref='local')
        e_l = np.asarray(e_l2).reshape(-1, 2)
        st_n = {}
        e_n, _, _, _ = new.edge_columns(depth, intr, tf, CH, DT, GT,
                                        ground_ref='local', ref_truncate=True, ref_stats=st_n)
        # 截断点证据（按 TRUNC_* 同规则复算，只作报告）：取"最靠车前的那一列"的近场点序
        trunc_info = ('n/a', 0.0, 0.0)
        try:
            fx0, fy0, cx0, cy0 = intr
            vv0, uu0 = np.mgrid[0:depth.shape[0]:2, 0:depth.shape[1]:2]
            z0 = depth[::2, ::2]
            ok0 = np.isfinite(z0) & (z0 > 0.05) & (z0 < 7.95)
            xyz0 = np.stack([(uu0 - cx0) * z0 / fx0, (vv0 - cy0) * z0 / fy0, z0], axis=-1)
            w0 = xyz0 @ R.T + t
            fwd0 = (w0 - t) @ R[:, 2]
            fwd0 = fwd0 / max(1e-9, np.linalg.norm(R[:, 2]))
            mid = z0.shape[1] // 2
            pts0 = sorted((float(fwd0[i, mid]), float(w0[i, mid, 2]))
                          for i in range(z0.shape[0])
                          if ok0[i, mid] and 0.55 < fwd0[i, mid] < 1.20)
            for a0, b0 in zip(pts0, pts0[1:]):
                dg = b0[0] - a0[0]
                dr = a0[1] - b0[1]
                if dg > 0.12 or (dr > 0.05 and dr / max(dg, 1e-6) > 0.6):
                    trunc_info = ('%.3f m 处（Δz=%.3f, Δz/Δfwd=%.2f）'
                                  % (a0[0], dr, dr / max(dg, 1e-6)), dr, dr / max(dg, 1e-6))
                    break
        except Exception as e:
            trunc_info = ('复算失败 %s' % e, 0.0, 0.0)
        s_l = {tuple(np.round(p, 3)) for p in np.asarray(e_l).reshape(-1, 2)}
        s_n = {tuple(np.round(p, 3)) for p in np.asarray(e_n).reshape(-1, 2)}
        diff = s_l ^ s_n
        valid_frac = float(np.isfinite(depth).mean())
        print('=== %s ===' % name)
        print('  有效深度 %d/%d（%.0f%%）列数=%d'
              % (np.isfinite(depth).sum(), depth.size, 100 * valid_frac, depth.shape[1] // 2))
        if valid_frac < 0.20:
            print('  ！合成数据无效（有效深度 <20%%）⇒ 本用例**判 FAIL**，不得当作通过')
            all_ok = False
            continue
        # 重建自检：近场实测地面 vs 解析面（防"相机朝着天上"这类建模错）
        fx, fy, cx, cy = intr
        uu, vv = np.meshgrid(np.arange(depth.shape[1]), np.arange(depth.shape[0]))
        zc = depth
        ok_px = np.isfinite(zc)
        xyz = np.stack([(uu - cx) * zc / fx, (vv - cy) * zc / fy, zc], axis=-1)
        wz = xyz.reshape(-1, 3) @ R.T + t
        near = ok_px.reshape(-1) & (np.linalg.norm(wz[:, :2] - t[:2], axis=1) < 1.2)
        if near.sum() > 50:
            err = np.abs(wz[near, 2] - z_surf_vec(wz[near, 0], wz[near, 1]))
            print('  重建自检：近场 %d 点，|z 实测 − 解析面| 中位 %.3f m'
                  % (near.sum(), float(np.median(err))))
            if float(np.median(err)) > 0.03:
                print('  ！重建与解析面差 >3 cm ⇒ 合成建模有错，判 FAIL')
                all_ok = False
                continue
        print('  plane vs 改前：edges %d vs %d  一致=%s  masks 一致=%s'
              % (len(e_p), len(e_b), same, same_m))
        for mode in ('plane', 'local', 'local_tilt'):
            mm = modes[mode]
            print('  [%s] up_px=%6d  edges=%3d  跨度=%.3f  ref_stats=%s'
                  % (mode, mm['up_px'], mm['edges'], mm['span'],
                     {k: v for k, v in mm['stats'].items() if k != 'max_abs_ref_minus_floor'}))
        print('  ★ 代数核对：local 的 max|ref−floor| = %s（坡上应 ≈0 ⇒ 与 plane 结构性等价）'
              % st_l.get('max_abs_ref_minus_floor'))


        print('  截断开/关：edges %d vs %d，差集 %d ⇒ %s'
              % (len(e_l), len(e_n), len(diff), '空 ✓' if not diff else '**非空 ✗（逐格报，不并进 ①）**'))
        print('  截断点证据（中间列）：%s' % trunc_info[0])
        if diff:
            print('  差集明细（前 6 个）：%s' % sorted(diff)[:6])
        if not (same and same_m):
            all_ok = False
    print('\n判定：`plane` 复现改前行为 =', 'PASS ✓' if all_ok else 'FAIL ✗')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())

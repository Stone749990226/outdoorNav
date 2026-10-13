#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把箱那一格**反投影到相机像平面**：报 (u, v) 与**水平角 / 垂直角**。

★ 硬要求（本项目被这条坑过）：**必须用当帧 TF（活节点）实算**，不许只用静态 TF 推 ——
  车停在哪、相机朝哪，都要**现查 tf2 Buffer**；并把**查到的那一帧 TF 的 stamp** 与
  **查表时刻的墙钟**一起写出来（否则"这个角是哪一刻的"说不清）。

光学约定：相机光学帧 `camera_optical_frame`：+x 右、+y 下、+z 前。
  u = fx * X/Z + cx ； v = fy * Y/Z + cy
  水平角 = atan2(X, Z)（右为正）； 垂直角 = atan2(Y, Z)（下为正）
内参优先取活节点的 `/rtk/camera/camera_info`（K[0],K[4],K[2],K[5]）；
取不到再按 SDF 的 horizontal_fov 1.0472 rad + 320×240 推（fx=fy=160/tan(30°)=277.13）。

用法（容器内）：
  reproj.py --point 4.5,-3.3,0.5 [--at-sim 123.456] [--out /work/.../r4_reproj.json]
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node


def _jsonable(o):
    """numpy 标量不是 JSON 可序列化的（`np.bool_` 尤其：它不是 Python `bool` 的子类）
    ⇒ 统一降级。2026-09-21 实测：nav10-r4 的 A 轮 `r4_reproj_A.json` 就是这么丢的
    （`.txt` 那份仍在；数值本身没错，只是落盘那一步炸了）。"""
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.integer):
        return int(o)
    return str(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--point", required=True, help="map 帧的世界点 x,y,z")
    ap.add_argument("--z", type=float, default=0.5, help="只给 x,y 时用的 z（箱中心高度）")
    ap.add_argument("--at-sim", type=float, default=None,
                    help="按这个仿真时刻查 TF（当帧）；不给则用最新一帧")
    ap.add_argument("--out", default="")
    ap.add_argument("--timeout", type=float, default=40.0)
    a = ap.parse_args()

    parts = [float(v) for v in a.point.split(",")]
    if len(parts) == 2:
        px, py, pz = parts[0], parts[1], a.z
    else:
        px, py, pz = parts

    rclpy.init()
    node = Node("reproj")
    import tf2_ros
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    node.create_timer(0.05, lambda: None)

    from sensor_msgs.msg import CameraInfo
    ci = {}
    node.create_subscription(CameraInfo, "/rtk/camera/camera_info",
                             lambda m: ci.setdefault("m", m), 10)

    t0 = time.time()
    while "m" not in ci and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    K = None
    if "m" in ci:
        m = ci["m"]
        K = {"fx": m.k[0], "fy": m.k[4], "cx": m.k[2], "cy": m.k[5],
             "width": m.width, "height": m.height, "frame_id": m.header.frame_id,
             "distortion_model": m.distortion_model, "source": "live /rtk/camera/camera_info"}
    else:
        fx = (320 / 2.0) / math.tan(1.0472 / 2.0)
        K = {"fx": fx, "fy": fx, "cx": 160.0, "cy": 120.0, "width": 320, "height": 240,
             "frame_id": "camera_optical_frame", "distortion_model": "fallback",
             "source": "SDF horizontal_fov=1.0472 + 320x240 推算（camera_info 没收到）"}

    # ---- 当帧 TF（活节点）----
    query = rclpy.time.Time() if a.at_sim is None else rclpy.time.Time(seconds=int(a.at_sim),
                                                                      nanoseconds=int((a.at_sim % 1) * 1e9))
    tf = None
    last_exc = None
    t1 = time.time()
    while tf is None and time.time() - t1 < 20:
        try:
            tf = buf.lookup_transform("map", "camera_optical_frame", query)
        except Exception as exc:
            last_exc = exc
            rclpy.spin_once(node, timeout_sec=0.1)
    if tf is None:
        print("** 查不到 map←camera_optical_frame（%s）⇒ 算不了 (u,v) **" % last_exc)
        return 1
    looked_up_at_wall = time.time()

    tr = tf.transform.translation
    q = tf.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                  [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    t_map = np.array([tr.x, tr.y, tr.z])
    t_stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9

    def project(pt):
        loc = R.T @ (np.asarray(pt, dtype=float) - t_map)     # map → 光学帧
        X, Y, Z = float(loc[0]), float(loc[1]), float(loc[2])
        u = K["fx"] * X / Z + K["cx"]
        v = K["fy"] * Y / Z + K["cy"]
        h = math.degrees(math.atan2(X, Z))
        vv = math.degrees(math.atan2(Y, Z))
        return {"map_xyz": [round(float(c), 4) for c in pt],
                "optical_XYZ": [round(X, 4), round(Y, 4), round(Z, 4)],
                "u": round(u, 2), "v": round(v, 2),
                "horizontal_deg": round(h, 3), "vertical_deg": round(vv, 3),
                "in_image": (0 <= u < K["width"]) and (0 <= v < K["height"]),
                "in_hfov_30": abs(h) <= 30.0, "in_vfov_23p4": abs(vv) <= 23.4}

    center = project((px, py, pz))
    extra = {"top": project((px, py, pz + 0.5)), "bottom": project((px, py, pz - 0.5)),
             "near_face_toward_cam": None}
    # 箱的近面（朝向相机那一面，沿光学 x/z 方向退 0.2 m）—— 只作参照，不改判据
    axis_z = R @ np.array([0.0, 0.0, 1.0])
    near = np.array([px, py, pz]) - 0.2 * np.array([axis_z[0], axis_z[1], 0.0])
    extra["near_face_toward_cam"] = project((near[0], near[1], pz))

    print("=== 箱格反投影（**当帧 TF 实算**）===")
    print("内参：%s  fx=%.3f fy=%.3f cx=%.2f cy=%.2f  %dx%d  帧=%s"
          % (K["source"], K["fx"], K["fy"], K["cx"], K["cy"], K["width"], K["height"],
             K["frame_id"]))
    print("TF map←camera_optical_frame：stamp=%.3f（sim）  位置=(%.4f, %.4f, %.4f)"
          % (t_stamp, tr.x, tr.y, tr.z))
    print("  查表时刻墙钟=%.3f（epoch）  查询方式=%s"
          % (looked_up_at_wall, "最新一帧" if a.at_sim is None else "指定 sim=%.3f" % a.at_sim))
    print("箱中心 map(%.3f, %.3f, %.3f) → 光学帧 (%.4f, %.4f, %.4f)"
          % (px, py, pz, *center["optical_XYZ"]))
    print("** (u, v) = (%.2f, %.2f)   水平角 = %+.3f°（限 ±30°）   垂直角 = %+.3f°（限 ±23.4°）**"
          % (center["u"], center["v"], center["horizontal_deg"], center["vertical_deg"]))
    print("  在像平面内=%s  水平在限内=%s  垂直在限内=%s"
          % (center["in_image"], center["in_hfov_30"], center["in_vfov_23p4"]))
    for k in ("top", "bottom", "near_face_toward_cam"):
        e = extra[k]
        print("  参照 %-20s → (u,v)=(%.2f, %.2f) 水平=%+.2f° 垂直=%+.2f°"
              % (k, e["u"], e["v"], e["horizontal_deg"], e["vertical_deg"]))

    if a.out:
        json.dump({"intrinsics": K, "tf": {"stamp_sim": t_stamp,
                                           "translation": [tr.x, tr.y, tr.z],
                                           "quaternion": [qx, qy, qz, qw],
                                           "lookup_wall": looked_up_at_wall,
                                           "query": "latest" if a.at_sim is None else "sim=%.3f" % a.at_sim},
                   "center": center, "extra": extra,
                   "note": "全部用活节点 tf2 Buffer 查到的那一帧 TF 计算，不是静态 TF 推的"},
                  open(a.out, "w"), ensure_ascii=False, indent=1, default=_jsonable)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

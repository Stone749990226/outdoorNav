#!/usr/bin/env python3
"""frontend-smoke.py —— ★ 一句命令验完"前端那条链" ✓（✗ 不用开浏览器 ✓）

★ 它做的事 ✓（★ 与我今天实跑 30 遍 + 定向 10 遍的**同一套请求** ✓，只是抽成一条命令 ✓）：
  ① `GET  /api/status` ✓ —— ★ 拿**车当前位置**（★ 桥自己的口径 ✓）
  ② 造一条两点路径 ✓ —— ★ 起点 ＝ 车位置 ✓、★ 终点 ＝ 车前方 12 m ✓
  ③ `POST /api/routes` ✓ —— ★ body 与原脚本**逐字段相同** ✓（★ frame／version／origin 必须与
     `calibration.json` 一致 ✓ ⇒ ★ 手写必挂 ✓，所以用这个脚本 ✓）
  ④ 轮询 `GET /api/status` ✓ ⇒ ★ 打相位与终点距离 ✓

★ 为什么起点要**现读** ✓：★ 业主口径"起始点就是小车位置" ✓ ⇒ ★ 写死的起点要么被 `sim_reset` 挪车、
  ★ 要么车已经在别处 ⇒ ★ 两种都不对 ✓。

用法（★ 容器内**或**宿主都行 ✓ —— 容器是 `--network host` ✓ ⇒ 两边 `127.0.0.1:8765` 是同一个 ✓）：
    python3 /work/scripts/rtk-nav2/frontend-smoke.py            # 容器内
    python3 scripts/rtk-nav2/frontend-smoke.py                  # 宿主（WSL）
    python3 scripts/rtk-nav2/frontend-smoke.py --len 6 --timeout 120

★ 预期 ✓：★ 先打 `phase=queued` ✓ ⇒ `running` ✓ ⇒ ★ 最后一行 ★ `phase=completed` 且离目标 ≤ 0.5 m ✓
  （★ 到点门槛：★ 系统自己的 `xy_goal_tolerance` ＝ 0.25 m ✓；★ 物理落点会略大 ✓ —— 见
   `docs/frontend-integration.md` §三 ✓）
★ 退出码 ✓：★ `0` ＝ 走完 ✓；★ `1` ＝ 没走完（★ 打实际相位 ✓，✗ 不谎报成功 ✓）
"""
import argparse
import json
import math
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8765"
# ★ 与 `docs/frontend-integration.md` 里那条一致的标定字段 ✓（★ 逐字来自他们的 `calibration.json` ✓）
# ★★ **故意写死、故意不跟随** ✗（2026-10-13 ✓）：★ 标定一变（version／origin 改了 ✓）⇒
#   ★ 这个脚本 ★ **会失败** ✓ ⇒ ★ ★ **那是对的** ✗ —— ★ 别让它自动去读 `calibration.json` 跟随 ✗：
#   ★ 静默跟随 ⇒ ★ 它验的**变成另一件事**（★ 拿新标定去对旧世界 ✓）⇒ ★ 而它**照样打"通过"** ✗
#   ★ 同族：本仓"**判据拿具体值比、别拿否定式比**" ✓ —— ★ 这里是把"我期望的那个版本"写死 ✓
CAL = {"frame_id": "map", "calibration_version": "survey-388df4ebbef8",
       "calibration_status": "survey_dgps_unverified", "map_version": "survey-388df4ebbef8",
       "executable_on_robot": False,
       "origin_wgs84": {"latitude": 30.22413029, "longitude": 120.25979315833334}}


def api(path, body=None, timeout=10):
    req = urllib.request.Request(BASE + path,
                                 data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=timeout).read())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--len", type=float, default=12.0, help="路径长度（米）")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--target", default="", help="可选：写死的目标 x,y（默认＝车前方 --len 米）")
    args = ap.parse_args()

    st = api("/api/status")
    p = st.get("pose") or {}
    if "x" not in p:
        print("✗ `/api/status` 里没有 pose ⇒ 桥没在跑，或车没在发 /odom ✗")
        return 1
    x, y, yaw = float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0))
    if args.target:
        tx, ty = (float(v) for v in args.target.split(","))
    else:
        tx, ty = x + args.len * math.cos(yaw), y + args.len * math.sin(yaw)

    print("★ 车（★ 桥自己的口径 ✓）: (%.4f, %.4f) yaw=%+.3f rad" % (x, y, yaw))
    print("★ 目标: (%.4f, %.4f) ｜ 距离 %.2f m" % (tx, ty, math.dist((x, y), (tx, ty))))

    body = dict(CAL)
    body["path"] = [{"x": x, "y": y}, {"x": tx, "y": ty}]
    body["waypoints"] = [{"x": x, "y": y, "yaw": yaw}, {"x": tx, "y": ty, "yaw": yaw}]
    r = api("/api/routes", body)
    print("★ POST /api/routes ⇒ phase=%s" % r.get("phase"))

    t0 = time.time()
    seen = []
    while time.time() - t0 < args.timeout:
        s = api("/api/status")
        ph = s.get("phase")
        if not seen or seen[-1] != ph:
            seen.append(ph)
            print("   t+%5.1fs phase=%-10s pose=(%.3f, %.3f)" %
                  (time.time() - t0, ph, s["pose"]["x"], s["pose"]["y"]))
        if ph in ("completed", "failed", "canceled"):
            break
        time.sleep(1)
    s = api("/api/status")
    d = math.dist((s["pose"]["x"], s["pose"]["y"]), (tx, ty))
    print("=" * 62)
    print("★ 结果 ✓：★ phase=%s ｜ ★ 离目标 %.4f m ｜ 相位轨迹 %s" %
          (s.get("phase"), d, "→".join(seen)))
    return 0 if s.get("phase") == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())

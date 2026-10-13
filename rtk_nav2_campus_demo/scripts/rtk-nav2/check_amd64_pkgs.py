#!/usr/bin/env python3
"""实测移植清单里那条关键假设：★ x86_64(amd64) 上 Fortress 与 ROS 2 Humble 到底有没有包？
   ★ 清单里这条是从 Dockerfile 的**注释**读到的 ✗ —— 按我们自己的规矩，注释不是凭据 ⇒ 必须实测 ✓。
   只读 ✓、不需要 sudo ✓、不需要 docker ✓。
★ 三态：取不到索引 ⇒ UNDETERMINED（★ 不许落成"没有" ✗）
"""
import gzip, io, sys, urllib.request

WANT = {
    "ros2":  ("https://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages.gz",
              ["ros-humble-ros-gz", "ros-humble-ros-gz-sim", "ros-humble-desktop",
               "ros-humble-navigation2", "ros-humble-nav2-bringup", "ros-humble-robot-localization"]),
    "gazebo": ("http://packages.osrfoundation.org/gazebo/ubuntu-stable/dists/jammy/main/binary-amd64/Packages.gz",
               ["ignition-fortress", "ignition-gazebo6", "libignition-gazebo6-dev"]),
}

def fetch(url, verify=True):
    ctx = None
    if not verify:
        import ssl
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
    req = urllib.request.Request(url, headers={"User-Agent": "rtk-port-check/1.0"})
    with urllib.request.urlopen(req, timeout=240, context=ctx) as r:
        return r.read()

def scan(url, names):
    raw = None
    tried = []
    cands = [url]
    if url.startswith("https://"):
        cands.append("http://" + url[len("https://"):])
    for u in cands:
        for verify in (True, False):
            try:
                raw = fetch(u, verify=verify)
                tag = "已校验" if verify else "★ 未校验证书（只读公开索引，本次可接受，但已如实标注 ✗）"
                print("  取到 ✓ 来源=%s 证书校验=%s" % (u, tag))
                break
            except Exception as e:
                tried.append("%s [%s] %s: %s" % (u, "verify" if verify else "noverify", type(e).__name__, e))
        if raw is not None:
            break
    if raw is None:
        print("  UNDETERMINED 取不到索引 —— 试过 %d 种：" % len(tried))
        for t in tried[:4]:
            print("      %s" % t)
        print("  （★ 这是「没取到」，不是「没有这个包」 ✗）")
        return None
    try:
        data = gzip.decompress(raw).decode(errors="replace")
    except Exception:
        data = raw.decode(errors="replace")
    print("  索引大小：%.1f MB（解压后 %.1f MB）" % (len(raw)/1e6, len(data)/1e6))
    found = {}
    # 逐段扫（各包一段，段间空行分隔）
    for para in data.split("\n\n"):
        if "\nPackage: " not in "\n" + para:
            continue
        pkg = arch = ver = ""
        for line in para.splitlines():
            if line.startswith("Package: "): pkg = line[9:].strip()
            elif line.startswith("Architecture: "): arch = line[14:].strip()
            elif line.startswith("Version: "): ver = line[8:].strip()
        if pkg in names:
            found.setdefault(pkg, []).append((arch, ver))
    for n in names:
        if n in found:
            archs = sorted({a for a, _ in found[n]})
            vs = sorted({v for _, v in found[n]})
            print("  ✓ %-32s amd64在? %s ｜ 架构=%s ｜ 版本=%s（%d 条）"
                  % (n, "YES" if "amd64" in archs else "NO", ",".join(archs), vs[-1] if vs else "?", len(found[n])))
        else:
            print("  ✗ %-32s ★ 索引里没有这个名字" % n)
    return found

print("=== 0. 来源自检（★ 先qualify这份读数 ✓）===")
import socket
_probe = "zzz-does-not-exist-rtk-probe-9271.example"
_seen = []
for h in ["packages.ros.org", "packages.osrfoundation.org", "github.com", _probe]:
    try:
        _seen.append((h, socket.gethostbyname(h)))
    except Exception as e:
        _seen.append((h, "✗%s" % type(e).__name__))
for h, ip in _seen:
    print("  %-46s → %s" % (h, ip))
_all198 = all(isinstance(ip, str) and ip.startswith("198.18.") for _, ip in _seen)
if _all198:
    print("  ★★ 含**不存在的名字** ⇒ catch-all 拦截器（DNS 被接管）✗")
    print("  ★★ 198.18.0.0/15 是 RFC 2544 保留段 ⇒ **不是公网** ✗")
    print("  ⇒ ★ 下面读到的索引**来自拦截器、不是上游** ⇒ 只能算「经本机网络看到的」✗")
    print("  ⇒ ★ 对移植的直接含义：★ **容器内 apt-get update 也走同一个拦截器** ⇒ 可能当场挂 ✗")
    print("  ⇒ ★ 建议先单独验一条：docker run --rm ubuntu:22.04 bash -c 'apt-get update'")
else:
    print("  ✓ 未发现 catch-all 迹象（不存在名字不给地址）")
print()

for label, (url, names) in WANT.items():
    print("=== %s ===\n  %s" % (label, url))
    scan(url, names)
print("\n★ 判定：只要上面出现 UNDETERMINED ⇒ 本次**不算验过** ✗（不是「没有」 ✗）")

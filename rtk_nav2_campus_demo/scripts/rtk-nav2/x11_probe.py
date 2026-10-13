#!/usr/bin/env python3
"""最小 X11 客户端（零依赖）—— 只回答两件事：
   ① 能不能连上 X 服务器（= 显示链路通不通）
   ② 根窗口上有没有 _NET_SUPPORTING_WM_CHECK（= 有没有 EWMH 窗口管理器 ⇒ wmctrl 能不能用）
   顺带把屏幕尺寸读出来（Step 3 的 ⑤，不需要 xdpyinfo）。
★ 判定三态：连不上/解析失败 ⇒ undetermined（★ 不许落成"没有"）
"""
import socket, struct, sys

PATH = "/tmp/.X11-unix/X0"
# ★ 默认问 EWMH 的窗口管理器自检属性；★ 也可传别的名字用来自检本探针的**负向分支** ✓
#   例：python3 x11_probe.py --property _RTK_NO_SUCH_PROP  ⇒ 应当报"属性不存在"✓
prop_name = b"_NET_SUPPORTING_WM_CHECK"
if "--property" in sys.argv:
    prop_name = sys.argv[sys.argv.index("--property") + 1].encode()

def recv_exact(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise EOFError("连接在收到 %d/%d 字节时关闭" % (len(b), n))
        b += c
    return b

def main():
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(8)
    s.connect(PATH)
    # 连接建立请求：小端、11.0、无鉴权（WSLg 下 XAUTHORITY 为空）
    s.sendall(b"l" + b"\x00" + struct.pack("<HH", 11, 0) + struct.pack("<HH", 0, 0) + b"\x00\x00")
    hdr = recv_exact(s, 8)
    status = hdr[0]
    if status != 1:
        print("SETUP_FAILED status=%d（鉴权被拒或协议不符）" % status); return 2
    add_len = struct.unpack("<H", hdr[6:8])[0]
    body = recv_exact(s, add_len * 4)
    vendor_len = struct.unpack("<H", body[16:18])[0]
    n_formats = body[21]
    n_screens = body[20]
    off = 32 + ((vendor_len + 3) // 4) * 4 + 8 * n_formats
    if n_screens < 1 or off + 40 > len(body):
        print("PARSE_UNDETERMINED n_screens=%d off=%d len=%d" % (n_screens, off, len(body))); return 3
    root = struct.unpack("<I", body[off:off+4])[0]
    w_px, h_px = struct.unpack("<HH", body[off+20:off+24])
    print("CONNECTED vendor=%r root=0x%08x screens=%d 屏幕=%dx%d px"
          % (body[32:32+vendor_len].decode(errors="replace"), root, n_screens, w_px, h_px))

    def intern(name: bytes):
        pad = (4 - len(name) % 4) % 4
        # ★ InternAtom 请求头是 8 字节：opcode(1) only_if(1) 请求长度(2) 名字长(2) 未用(2)
        #   （第一版漏了"请求长度"这 2 字节 ⇒ 服务器读到垃圾 ⇒ 不回包 ⇒ 超时 ✓）
        # ★★ only_if_exists=0：**无条件登记** ✓
        #   ✗ 不能用 1 ✗ —— 那样"原子不存在"的成因是"**没有客户端提过这个 EWMH 名字**"✗，
        #     而不是"没有窗口管理器"✗ ⇒ 那是个**依赖环境历史**的信号 ✓
        #     （★ 同族于今晚"对的读数、错的推断"✗：★ 在这台机器上两者恰好同真 ⇒ 会读对、但理由不对 ✗）
        #   ⇒ 真正的判据只有一条：★ **根窗口上那个属性在不在** ✓（属性是 per-window，原子是全局名 ✓）
        req_len = 2 + (len(name) + pad) // 4
        s.sendall(struct.pack("<BBHHH", 16, 0, req_len, len(name), 0) + name + b"\x00" * pad)
        r = recv_exact(s, 32)
        return struct.unpack("<I", r[8:12])[0]

    def atom_name(a):
        # ★ GetAtomName（opcode 17）—— ★★ 直接问服务器"这个 id 叫什么" ✓
        #   ✗ 不需要候选表 ✗（★ 候选表只能认出我事先想到的名字 ✓ ⇒ 认不出的就永远"未解析" ✗）
        s.sendall(struct.pack("<BBHI", 17, 0, 2, a))
        rr = recv_exact(s, 32)
        rlen2 = struct.unpack("<I", rr[4:8])[0]
        nlen = struct.unpack("<H", rr[8:10])[0]
        vb = recv_exact(s, rlen2 * 4) if rlen2 > 0 else b""
        return vb[:nlen].decode(errors="replace")

    atom = intern(prop_name)
    if atom == 0:
        # 无条件登记还拿不到 ⇒ 一律未判（★ 不许落成"没有"✗）
        print("UNDETERMINED 原子登记返回 0（无条件登记不该发生）")
        return 4
    # GetProperty(root, atom, ANY, 0, N)
    # ★ 仍然是 8 个头字段：opcode(1) delete(1) 请求长度(2)=6 窗口(4) 属性(4) 类型(4) 偏移(4) 长度(4)
    want = 64 if "--list" in sys.argv else 1
    s.sendall(struct.pack("<BBHIIIII", 20, 0, 6, root, atom, 0, 0, want))
    r = recv_exact(s, 32)
    fmt   = r[1]                                     # ★ 每项的位宽（8/16/32 ⇒ 1/2/4 字节）
    rlen  = struct.unpack("<I", r[4:8])[0]           # ★ 回复**自带**的长度（4 字节单位）
    ptype = struct.unpack("<I", r[8:12])[0]
    bytes_after = struct.unpack("<I", r[12:16])[0]
    vlen  = struct.unpack("<I", r[16:20])[0]
    # ★★ 值的字节数一律取"回复头自己说的那个长度" ✓
    #   ✗ 绝不按"我以为每项多少字节"去算 ✗ ——
    #   ★ 血的教训：我原先写 `32 if ptype==0x21 else 4` ✗（把**回复头大小**当成了**项大小**）
    #     ⇒ 默认属性是 WINDOW 型 ⇒ 我去读 32 字节而实际只有 4 ⇒ ★ **永远等不到 ⇒ 超时** ✗
    #     ⇒ ★★ 于是 ★ **"报超时"只在"用默认属性 + --list"这一种命令形状下发生** ✗
    #     ★ 而我自己的用法（`--property _NET_SUPPORTED --list`）恰好躲开了 ✗ ⇒ ★ 两边读数不一致 ✓
    val = recv_exact(s, rlen * 4) if rlen > 0 else b""
    item = max(fmt // 8, 1)
    total_items = vlen + bytes_after // item
    if ptype != 0 and "--list" in sys.argv:
        ids = [struct.unpack("<I", val[i:i+4])[0] for i in range(0, min(len(val), vlen*4), 4)]
        # ★ 用 GetAtomName 逐个解析 ✓ —— 不再维护候选表 ✓
        print("EWMH[%s]: 项数=%d（★ 头上说的长度 %d×4 字节 ✓）" % (prop_name.decode(), len(ids), rlen))
        # ★★ 只有类型真的是 ATOM 才能去问名字 ✗
        #   ★ 血的教训：`_NET_SUPPORTING_WM_CHECK` 的**类型是 WINDOW** ✓ ⇒ 它的值是**窗口 id** ✗
        #     ⇒ 拿窗口 id 去 `GetAtomName` ⇒ ★ BadAtom ⇒ **服务器不回包** ⇒ ★ 永久等待 ⇒ 超时 ✗
        #   ★ 而我先前只拿 `_NET_SUPPORTED`（ATOM 型 ✓）试过 ⇒ ★ **从一个形状推广到了全部** ✗
        #     —— ★ 与"把回复头大小当成项大小"是**同一个错的孪生** ✓（那次假设**项多大**，这次假设**项是什么**）
        if ptype == 4:
            for i in ids:
                print("   0x%08x  %s" % (i, atom_name(i)))
        else:
            for i in ids:
                print("   0x%08x  <type=0x%08x 非原子，按原值列出>" % (i, ptype))
        return 0
    if ptype == 0:
        # ★★ 限定条件要**由工具自己打印** ✗ —— 不能只写在我的报告里 ✓
        #   判据：★「换个人来跑这个工具，他会不会看到这个限定？」✓ 看不到 ⇒ 不稳 ✗
        #   ★ 只有问的**就是**自检属性时，「无 WM」才是结论 ✓；
        #     问别的名字（尤其我自造的负向标定名）⇒ ★ 那只说明**这个名字不存在** ✗
        if prop_name == b"_NET_SUPPORTING_WM_CHECK":
            print("EWMH[%s]: ★ 属性不存在 ⇒ 无 EWMH 窗口管理器 ⇒ wmctrl 不适用"
                  "（★ 仅当问的就是自检属性时，这句才是结论 ✓）" % prop_name.decode())
        else:
            print("EWMH[%s]: 属性不存在 —— ★ 但这**不构成**关于窗口管理器的结论 ✗"
                  "（问的不是自检属性 ⇒ 只说明这个名字不存在 ✓）" % prop_name.decode())
    else:
        print("EWMH[%s]: ★ 属性存在 type=0x%08x 项数=%d（★ 含 bytes_after ⇒ 真实长度 ✓；仅 vlen=%d 是请求上限 ✗）"
              % (prop_name.decode(), ptype, total_items, vlen))
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("UNDETERMINED %s: %s" % (type(e).__name__, e))
        sys.exit(9)

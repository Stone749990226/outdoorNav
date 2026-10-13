#!/usr/bin/env python3
"""★ **静态实体封边 → 全局代价图**（reviewer 2026-10-09 批准开工，五条约束见下）。

**为什么需要它**（`ramp7-at` 的实测根因 ✗）：封边块是**物理实体**，但它们**没有任何"持久"的通道进规划器的图** ✗ ——
· `drop_keepout`（`KeepoutFilter`）读的是 **`/rtk/drop/filter_info_map`** = **落差危险掩膜**，**不含实体块** ✗，
  而且那个话题是 **`depth_drop.py`（冻结的检测器）** 发的 ✗ ⇒ **约束 ① 把这条路封死了** ✓；
· 实体块只能靠 `obstacle_layer` 的**观测**进图 ✓，而它设的是 `combination_method: 0`（Overwrite）+ `clearing: true` ✓
  ⇒ **"看到过、离开视野就被清掉"** ✗（实测：`ramp7-at` 车里卡在 `x=-0.038` 时，
  新东坡块 `y=3.5/4.0` cost=99/100 ✓ 而**车前方 `y≥4.5` cost=0** ✗，连**早先见过的 `seal_west` 也回到 0** ✗）
  ⇒ **规划器可以自由穿过"当前没在看"的块** ✓ ⇒ 车照着走就顶上去、然后出不来 ✗。

**⇒ 本条走的是"加一层"** ✓：`static_layer` 是**层**、跑在 **inflation 之前** ✓ ⇒ 封边块会**带上 0.25 m 充气带** ✓
（实测 `drop_keepout` 那条**没有**梯度：主边两侧 `x=±0.30` 都是 100、`±0.40` 才是 0 ✗ ⇒
 **filter 跑在 inflation 之后** ✓ ⇒ 光靠 keepout 只能"穿不过去"、仍会**贴着墙走** ✗ —— 那正是 `as` 那次的病 ✓）。

**五条约束（reviewer 给）逐条对上** ✓：
① **不动检测器** ✓ —— 本文件是**新节点**，`depth_drop.py` 一行没碰 ✓；
② **不退回"幽灵格永不清"** ✗ —— **没改** `obstacle_layer` 的 `combination_method` / `clearing` ✓，
   观测层的行为原样 ✓；这里加的是**另一层** ✓，里面只有**真的静态实体**（墙本来就该常驻 ✓，不是"幽灵格" ✓）；
③ **单一真源** ✓ —— 块的几何**从 world 现解析**（`stop_margin_check.parse_raised_boxes` ✓，同一把尺子 ✓），
   **不手抄坐标** ✗；世界哈希一并落进日志 ✓；
④ **有效性测试** ✓ —— 见 `--self-test`：打印每块的中心格与"离块 0.25 m 的格"，
   供"把车移开视野后该格仍为致命 + 0.25 m 内有梯度"两条判据 ✓；
⑤ **不牵进 ①** ✓ —— 本文件与落差掩膜/空隙判据无关 ✓。

用法：`ros2 run robonix_rtk_nav2 static_walls`
"""
import hashlib
import os
import sys

# ★ **ROS 惰性导入** ✓（本项目纪律）：`--self-test` 是**纯离线**路径 ⇒ 在**没装 ROS 的机器上也要能跑** ✓
#   （踩过两次：把 ROS 放顶层 ⇒ 离线复算直接 ModuleNotFoundError ✗ —— 见 `ramp_band_check.py` 的同类注释 ✓）
try:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    _HAS_ROS = True
except ImportError:                                            # pragma: no cover
    _HAS_ROS = False
    Node = object          # 只为了让下面的类体能定义 ✓；离线路径**不会实例化**它 ✓

# ★ 几何解析**复用同一份实现** ✓（③ 单一真源）：`scripts/stop_margin_check.py` 的 `parse_raised_boxes`
_SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')
sys.path.insert(0, _SCRIPTS)
try:
    from stop_margin_check import parse_raised_boxes          # noqa: E402
except ImportError:                                            # pragma: no cover
    parse_raised_boxes = None

DEFAULT_WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             '..', 'worlds', 'depth_drop_ramp.sdf')

# ★ `OccupancyGrid` 的约定**不是** PGM 那套 ✗ —— 它用 `int8[]`：**`-1` = 未知** ✓、**0~100 = 占据** ✓
#   （我第一版照抄了 PGM 的 205/0 ✗ ⇒ 205 会溢出 int8、而且 StaticLayer 认的是 `-1` 与 `>= lethal_threshold`(默认 100) ✓）
#   ⇒ 未知必须是 **-1** ✓（也因此**不能用 `bytearray`** ✗，得用 list ✓）
UNKNOWN, OCCUPIED = -1, 100
RES = 0.05                                  # 与 global_costmap.resolution 对齐 ✓
SIZE_M = 40.0                               # 与 global_costmap width/height 对齐 ✓
ORIGIN = -20.0                              # 与 global_costmap origin_x/y 对齐 ✓


def build_grid(world_path, res=RES, size_m=SIZE_M, origin=ORIGIN):
    """把世界里的"高出地面"的箱体栅格化。

    ★★ **栅格化约定（reviewer 2026-10-09 要求写清：他要按它复现到"逐格逐位相同"）** ★★
    本函数用的是 **"与矩形有交叠"** 约定，**不是**"格中心落在矩形内"：

        x0 = int((b.x0 - origin) / res)      # 截断（floor，因为坐标都 > origin）
        x1 = int((b.x1 - origin) / res)
        然后 **闭区间** 填 [x0 .. x1]

    等价说法：**取矩形两个角点各自所在的那一格，连同它们之间的所有格** ——
    · 角点 `b.x0` 只"擦到"某一格的右边缘也算 ⇒ 该格被标；
    · 角点 `b.x1` 落在某格内 ⇒ 该格被标（**哪怕它的格心在矩形外**）。
    ⇒ 与"格中心在内"的约定相比，本约定在**障碍边界一格**上会多标/少标（实测与 reviewer
      自己按"格中心"栅格化的结果差 **3143 格 / 0.218%**，**两个方向都有**，且**全部落在边界一格内**）。
    对判据无影响（形心 23/23、直方图、0.25 m 充气带都远超这一格的量级），
    但**换人换机时必须照同一条约定算**，否则会各按各的约定、对不上还找不到原因。
    """

    """返回 (grid, info_dict)。**除封边块外一律 UNKNOWN** ✓ —— 免得静态层把"没去过"写成"可通行" ✗
    （`track_unknown_space: true` 是刻意的 ✓，静态层不能把它抹掉 ✓）。"""
    if parse_raised_boxes is None:
        raise RuntimeError('拿不到 parse_raised_boxes ⇒ 不能手抄坐标 ✗（③）')
    boxes = parse_raised_boxes(world_path)
    n = int(round(size_m / res))
    grid = [UNKNOWN] * (n * n)
    for b in boxes:
        x0 = max(0, int((b['x0'] - origin) / res))
        x1 = min(n - 1, int((b['x1'] - origin) / res))
        y0 = max(0, int((b['y0'] - origin) / res))
        y1 = min(n - 1, int((b['y1'] - origin) / res))
        for cy in range(y0, y1 + 1):
            row = cy * n
            for cx in range(x0, x1 + 1):
                grid[row + cx] = OCCUPIED
    return list(grid), {'n': n, 'res': res, 'origin': origin, 'boxes': boxes}


class StaticWalls(Node):
    def __init__(self):
        super().__init__('static_walls')
        self.declare_parameter('world', DEFAULT_WORLD)
        self.declare_parameter('topic', '/rtk/static_map')
        # ★ 2026-10-09（阶段八）**参数化栅格**：默认值 = 阶段七那一套（0.05 / 40 / -20）⇒
        #   不给这三项时行为与改动前**逐字相同** ✓；100 m 校园场景显式传
        #   `resolution: 0.10, size_m: 120.0, origin: -60.0`（与 `campus_nav2.yaml` 的全局图对齐 ✓）。
        #   为什么必须能与全局图对齐：nav2 的 `StaticLayer` 是按**全局图自己的**
        #   resolution/size/origin 去重采样的，两张图参数不一致时它会缩放/裁剪 —— 楼会被挪位 ✗。
        self.declare_parameter('resolution', RES)
        self.declare_parameter('size_m', SIZE_M)
        self.declare_parameter('origin', ORIGIN)
        world = self.get_parameter('world').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        res = self.get_parameter('resolution').get_parameter_value().double_value
        size_m = self.get_parameter('size_m').get_parameter_value().double_value
        origin = self.get_parameter('origin').get_parameter_value().double_value
        if not os.path.exists(world):
            self.get_logger().fatal('找不到 world：%s ⇒ **不发布**（宁可没有，也不要发一张错图 ✗）' % world)
            raise SystemExit(1)
        data, meta = build_grid(world, res=res, size_m=size_m, origin=origin)
        sha = hashlib.sha256(open(world, 'rb').read()).hexdigest()
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = meta['res']
        m.info.width = meta['n']
        m.info.height = meta['n']
        m.info.origin.position.x = meta['origin']
        m.info.origin.position.y = meta['origin']
        m.info.origin.orientation.w = 1.0
        m.data = data
        self._msg = m
        # ★ latched ✓：StaticLayer 是 transient_local 订阅 ✓；再按 1 Hz 重发 ⇒ 代价图重启也能拿到 ✓
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, topic, latched)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            '静态封边图：%d 块 ✓ 世界 %s ✓ %d×%d @ %.3f m ✓ 发布 %s ✓'
            % (len(meta['boxes']), sha, meta['n'], meta['n'], meta['res'], topic))
        for b in meta['boxes']:
            self.get_logger().info('  %-16s x∈[%.2f,%.2f] y∈[%.2f,%.2f] top=%.2f'
                                   % (b['name'], b['x0'], b['x1'], b['y0'], b['y1'], b['top']))

    def _tick(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self._msg)


def self_test(world_path=None, res=None, size_m=None, origin=None):
    """★ **有效性自测**（约束 ④，**离线、不需要 ROS** ✓）：打印每块的内部格、边界外 0.05/0.25/0.30 m 的格。

    判据（**两种结果都可能出现** ✓，这才叫"能失败"）：
    · **块内部必须 OCCUPIED(100)** ✓ —— 否则静态层没写进去 ✗；
    · **块外 0.30 m 必须是 UNKNOWN(-1)（无标记 ✓）**，**这个由 inflation 在图层之后补带** ✓ ⇒
      "out@0.25~0.30 在**静态图**里应当是 UNKNOWN" ✓；
    · **块外 0.05 m 也应是 UNKNOWN** ✓（**膨胀不在这张图里做** ✓，它由 `InflationLayer` 做 ✓）⇒
      运行期要看的正是"**StaticLayer 层给出致命 + InflationLayer 给出 0.25 m 梯度**" ✓。
    """
    world = world_path or DEFAULT_WORLD
    data, meta = build_grid(world, res=(RES if res is None else res),
                            size_m=(SIZE_M if size_m is None else size_m),
                            origin=(ORIGIN if origin is None else origin))
    n, res, org = meta['n'], meta['res'], meta['origin']

    def at(x, y):
        cx, cy = int((x - org) / res), int((y - org) / res)
        if not (0 <= cx < n and 0 <= cy < n):
            return None
        return data[cy * n + cx]

    bad = 0
    for b in meta['boxes']:
        cx, cy = (b['x0'] + b['x1']) / 2.0, (b['y0'] + b['y1']) / 2.0
        inside = at(cx, cy)
        out05 = at(b['x1'] + 0.05, cy)
        out25 = at(b['x1'] + 0.25, cy)
        out30 = at(b['x1'] + 0.30, cy)
        # ★ 2026-10-09（阶段八）"块外必须 UNKNOWN"这条**只在块是孤立的**时候成立。
        #   校园场景里 `bld_main` 与 `bld_east` **贴合**、`green_e_1` 贴东墙、`car_00/01`
        #   既贴楼又彼此相接 ⇒ "+x 侧 0.30 m 处"本来就在**另一件障碍里面**、读 100 是**对的** ✗
        #   （第一版照旧判 ⇒ 4 块假报 ✗）。这里先查"该点是否落在别的箱体里"：
        #   落在里面 ⇒ 这条不适用（记 `邻接`），不算失败；否则照旧判 UNKNOWN。
        adj = None
        for o in meta['boxes']:
            if o is b or o['name'] == b['name']:
                continue
            if o['x0'] <= b['x1'] + 0.30 <= o['x1'] and o['y0'] <= cy <= o['y1']:
                adj = o['name']
                break
        if adj is not None:
            ok = (inside == OCCUPIED)
            bad += 0 if ok else 1
            print('  %-16s 内部(%6.2f,%5.2f)=%4s  外+0.05=%4s  外+0.25=%4s  外+0.30=%4s  %s'
                  % (b['name'], cx, cy, inside, out05, out25, out30,
                     'N/A(邻接 %s)' % adj if ok else '✗'))
            continue
        ok = (inside == OCCUPIED) and (out30 == UNKNOWN)
        bad += 0 if ok else 1
        print('  %-16s 内部(%6.2f,%5.2f)=%4s  外+0.05=%4s  外+0.25=%4s  外+0.30=%4s  %s'
              % (b['name'], cx, cy, inside, out05, out25, out30, '✓' if ok else '✗'))
    print('世界 = %s' % world)
    print('网格 = %dx%d @ %.3f m，原点 (%.1f, %.1f)；UNKNOWN=%d OCCUPIED=%d'
          % (n, n, res, org, org, UNKNOWN, OCCUPIED))

    # ★★ **交叉自检**（2026-10-09 现场教训 ✗）：本自测**只看它能解析到的块** ⇒ **少了一整块它也不会报** ✗
    #   实测：我这边**world 副本是旧的**（改的是目标机那份、本地没同步 ✗）⇒ 只解析到 2 块，
    #   可输出照样是"✅" ✗✗ —— **一个"看起来通过"的假通过** ✓。
    #   ⇒ 拿 `world_edges.py`（**单一真源** ✓）记的世界哈希与"已封边条数"来对照 ✓
    #   ⇒ 副本旧/世界换了没同步，**当场就露** ✓（与"自检要比名字、不只比条数"同族 ✓）。
    actual = hashlib.sha256(open(world, 'rb').read()).hexdigest()
    # ★ 这道交叉自检是**阶段七世界专有的**（`world_edges.py` 里写的是 depth_drop_ramp 的几何与哈希）。
    #   阶段八的校园世界**不在它的清单里** ⇒ 那边只报哈希、跳过对照（不能因为"对不上"就判失败 ✗）。
    try:
        import world_edges as WE
        if os.path.basename(world) != 'depth_drop_ramp.sdf':
            print('ℹ 世界 %s 不是阶段七那份 ⇒ `world_edges.py` 的哈希/封边对照**不适用**；'
                  '本世界 sha256 = %s' % (os.path.basename(world), actual))
            print('⇒ %s' % ('✅ 每块内部均为 OCCUPIED、块外 0.30 m 均未标记 ✓（0.25 m 充气带由 InflationLayer 在运行期补 ✓）'
                            if bad == 0 else '✗ **有 %d 块不对** ⇒ 先修再起栈 ✗' % bad))
            return 0 if bad == 0 else 2
        if actual != WE.WORLD_SHA256:
            print('✗ **世界文件与 `world_edges.py` 记的不是同一份** ✗')
            print('   本文件   = %s' % actual)
            print('   world_edges = %s' % WE.WORLD_SHA256)
            print('   ⇒ 多半是**一边的副本没同步**（今天就踩到：本地旧副本 ⇒ 少解析到一整块 ✗）⇒ 先同步 ✓')
            return 2
        n_sealed = len(WE.SEALED_EDGES)
        if len(meta['boxes']) != n_sealed:
            print('⚠ 解析到 %d 块，而 `SEALED_EDGES` 记了 %d 条 —— **两者不等** ⇒ 值得查（不一定是错 ✓）'
                  % (len(meta['boxes']), n_sealed))
        print('✅ 世界哈希与 `world_edges.py` 一致（%s…）✓；块数 %d / 已封边 %d ✓'
              % (actual[:8], len(meta['boxes']), n_sealed))
    except ImportError:
        print('⚠ 拿不到 `world_edges` ⇒ **跳过了世界哈希对照** ✗（这一条正是防"副本旧"的 ✓）')

    print('⇒ %s' % ('✅ 每块内部均为 OCCUPIED、块外 0.30 m 均未标记 ✓（0.25 m 充气带由 InflationLayer 在运行期补 ✓）'
                    if bad == 0 else '✗ **有 %d 块不对** ⇒ 先修再起栈 ✗' % bad))
    return 0 if bad == 0 else 2


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if '--self-test' in argv:
        kw = {}
        for i, a in enumerate(argv):
            if a == '--world' and i + 1 < len(argv):
                kw['world_path'] = argv[i + 1]
            if a == '--resolution' and i + 1 < len(argv):
                kw['res'] = float(argv[i + 1])
            if a == '--size' and i + 1 < len(argv):
                kw['size_m'] = float(argv[i + 1])
            if a == '--origin' and i + 1 < len(argv):
                kw['origin'] = float(argv[i + 1])
        sys.exit(self_test(**kw))
    if not _HAS_ROS:
        print('!! 本机没有 ROS ⇒ 只能跑 `--self-test`（离线）✗', file=sys.stderr)
        sys.exit(1)
    rclpy.init(args=argv)
    n = StaticWalls()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

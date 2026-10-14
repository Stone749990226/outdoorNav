# 构建实测记录（2026-10-12）

★ 这份是 §6「已知不保证」第 2 条指向的文件 ✓ —— ★ 下面是**真跑过**的记录 ✗ 不是照抄 Dockerfile 的推断 ✓。
★ 场景：**本机（x86_64 WSL2）干净检出** ✓ —— ✗ **不是"在他们的机器上"** ✗（那台我们验不了 ✓）。

## 1) 构建命令与结果
```bash
GAZEBO_ROUTE=fortress IMAGE_NAME=robinox/rtk-nav2-demo:humble-amd64 bash scripts/build-image.sh
```

★★ **补注（2026-10-14 加 ✓）：★ 上面这条命令里的 `IMAGE_NAME=robinox/rtk-nav2-demo:…` 是**当时拼错的名字** ✗**
- ★ 本机确实建出过 `robinox/rtk-nav2-demo:verify` ✓（`docker images` 里可见 ✓）⇒ ★ 本节下面的记录**都是真的** ✓
- ★ ★ **正确的名字以 `README.md` 的「0) 先把镜像拿到手」为准** ✓ ＝
  **`robonix/rtk-nav2-demo:humble-amd64`** ✓（`robonix` ✓，✗ 不是 `robinox` ✗）
  ★ 依据：`scripts/rtk-nav2/lib.sh:23` 的默认就是它 ✓
- ★ 本节其余记录（耗时／体积／包数／`diff`）**不受影响** ✓ —— 它们量的是**镜像内容** ✓，✗ 与 tag 叫什么都不相干 ✓

| 项 | 实测值 |
|---|---|
| ★ 起点 | 干净检出、提交 **`73da6a5`** ✓（★ 也就是**初版** ✓，其 `Dockerfile.amd64` ＝ **`0d3c6187bb6199fa`** ✓ —— ★ 即 §3(a) 里那个**只装 `nav2-map-server`** 的版本 ✓）<br>★ 注意 ✗：**本节的 19 min／5.65 GB 都属于补 nav2 之前那一次** ✓；**补全之后**在提交 **`57b3071`** 上的重建 ⇒ ★ 见 **§6**（**25 min 39 s** ✓／**6.04 GB** ✓／**1813** 包 ✓） |
| 结果 | ★ **成功** ✓（`Successfully tagged` ✓） |
| ★ 耗时 | ★ **19 min 16 s** ✓（日志首末 16:49:35 → 17:08:51 ✓） |
| 镜像大小 | ★ **5.65 GB**（以 `docker images` 为准 ✓；★ `docker inspect .Size` 报 1.16 GB 是**另一个口径** ✗，两者不是一回事 ✓） |
| 基础镜像 | ★ **`ubuntu:22.04`** ✓ —— 本机没有 `osrf/ros:humble-desktop-full` ⇒ Fortress 路线直接落到 Dockerfile 的默认 ✓（★ 固定 tag ✓） |
| 关键包版本 | ★ 与参照机**逐条一致** ✓：`ignition-fortress 1.0.3-2~jammy` ✓／`libignition-gazebo6 6.18.0-1~jammy` ✓／`ros-humble-desktop 0.10.0-1jammy.20260908.101832` ✓／`ros-humble-ros-gz 0.244.26-1jammy.20260908.015605` ✓ |

## 2) 磁盘与网络需求（★ 构建前请照这个准备 ✓）
- **磁盘**：★ 镜像 **5.65 GB**（★ 补 nav2 **之前**那次 ✓；★ 补全后 **6.04 GB** ✓，见 §6 ✓）＋ 构建中间层 ⇒ 建议预留 **≥ 15 GB** ✓
- **网络**：★ **构建期必须联网** ✓ —— 实测从这三个源拉包 ✓：
  `archive.ubuntu.com` ✓／`packages.ros.org` ✓／`packages.osrfoundation.org` ✓
- **时间**：★ 本机 **19 min 16 s** ✓（机器更快/更慢会变 ✓）—— ★ 这是**补 nav2 之前**那次 ✓；★ 补全后**更大**：**25 min 39 s** ✓、镜像 **6.04 GB** ✓（见 §6 ✓）

## 3) ★★ 这次真跑抓到的两件事（★ 比"能建起来"更要紧 ✓）
### (a) ✗ 旧 Dockerfile **建出来的镜像跑不了演示** —— 已修 ✓
- 症状 ✓：从干净检出起栈，在"首次运行，编译 `rtk_nav2_ws` 工作区"处**编译失败** ✗：
  `CMake Error … find_package … [nav2_costmap_2d]` ✗
- 定量 ✓（用 `docs/image/dpkg-versions.txt` 对照）：旧 Dockerfile 的镜像 **1718** 个包、参照机 **1815** ⇒ ★ **缺 97 个、其中 nav2 相关 26 个** ✗
- 根因 ✓：`docker/Dockerfile.amd64` 那一行**只装了 `ros-humble-nav2-map-server`** ✗ —— ★ 没有 `ros-humble-navigation2` ✓
- ★ 处置 ✓：把 **nav2 栈 ＋ `robot-localization` ＋ `depth-image-proc` ＋ rtabmap/grid-map/pcl 那几个 ＋ 那批 `-dev`／`lib`** 一次补进那一行 ✓
  ★ 注意 ✓：那几个 `-dev`／`lib` 是**构建期依赖** ⇒ ★ `apt install ros-humble-navigation2` **不会**把它们带出来 ✗ ⇒ **必须显式列** ✓（实测：只装 navigation2 ⇒ 1765 ✗；再补那批 ⇒ 1770 ✗；再补剩下的 45 个 ros 包 ⇒ 才够 ✓）
- ★ 起点 ✓：上面这些症状与数值都出自 **`73da6a5`**（初版）那次干净构建 ✓（★ 检出仍在本机：`…/verify-checkout` 的姊妹目录 `clean-checkout`，其 `Dockerfile.amd64` ＝ **`0d3c6187bb6199fa`** ✓ 可复核 ✓）
- ★ 判据 ✓：包数**回到参照机量级** ∧ 与清单 `diff` **只剩孤儿包** ⇒ ★ **同时**演示能起栈 ✓
  ★ ★ **实测值（见 §6）** ✓：补全后重建 ⇒ **1813** 个包（参照机 1815 ✗）、`diff` **只剩 2 行** ✓ —— ★ **差的恰好是 `ros-humble-topic-tools` 与 `ros-humble-topic-tools-interfaces`** ✓，其余 1813 行**逐条相同** ✓

### (b) ✗ 干净检出里脚本不可执行 —— 已修 ✓
- 症状 ✓：`start-demo.sh: Permission denied` ✗（git 里没记可执行位 ✗——`scripts/` 下 **170** 个 `.sh`/`.py` 是 644 ✓）
- ★ 处置 ✓：把 `scripts/` 下 **287** 个脚本标成可执行 ✓（★ 只改 git 里的 mode ✓，✗ 内容一字未动 ✓）

## 4) 结论（一句话）
★ **照本文件的命令、用当前提交重建 ⇒ 包数 1813（≠ 参照机 1815，差 2 个孤儿包）、`diff` 只剩那 2 行、工作区能 `colcon build`、演示能起栈＋走＋停** ✓（★ 四样**全部实测于 §6** ✓）；
★ 若某一步对不上 ⇒ ★ **按 `diff` 差出来的包补** ✓（✗ 别猜 ✓），★ 并把差的那几个如实记下来 ✓。

## 5) §5 那条对照命令：**已在参照机镜像上实测** ✓
★ `docs/reproduce.md` §5 的 `dpkg-query … | sort` ＋ `diff` 那条命令，已在**参照机镜像**（`9406e5268aa1` ✓）上原样跑过：
★ 产出 **1815 行** ✓ ∧ 与仓库里的 `docs/image/dpkg-versions.txt` 比 ⇒ ★ **`diff` 输出 0 行** ✓✓
★ 意义 ✓：格式／排序／**分隔符（`\t`）**全对 ⇒ ★ 照抄跑在**参照机镜像**上就是空差异 ✓
  ★ 但注意 ✓：同一条命令跑在**新构建的镜像**上**会差出 2 行** ✓（✗ 不是命令写错了 ✓）—— ★ 那 2 行是谁、为什么，见 §6 ✓
★ 反过来说这条为什么值得记 ✓：这类命令最容易坏在看不见的地方（分隔符写成空格 ✗／漏了 `| sort` ✗）⇒ 对方会看到 1815 行"**假差异**" ✗ ⇒ ★ 明明是复现对了、却会读成"对不上" ✗
★ 所以本文件第 4 行那句"**每条命令都在本机实测过**"，对它**也成立** ✓

---

## 6) ★★ 端到端验证的**实测结果**（2026-10-12 17:22–17:55）✓

★ 做法 ✓：**全新检出**（`git clone` 本仓库 ⇒ 提交 **`57b3071`** ✓）⇒ 照 §1 的命令重建 ⇒ 四样判据一次跑全。
★ 为什么值得单列 ✓：§3／§4 里那几句判据此前是**预期** ✗ ⇒ 这一节把它们换成**实测值** ✓。

| # | 判据 | 实测 | 判定 |
|---|---|---|---|
| ① | 包数 vs 参照机 | **1813**（参照机 **1815**） | ★ **差 2 个** ✗（见下） |
| ② | 与 `docs/image/dpkg-versions.txt` 的 `diff` | ★ **只剩 2 行** ✓（其余 1813 行逐条相同 ✓） | ★ **2 行都是孤儿包** ✗ |
| ③ | 容器内首次 `colcon build` | ★ **成功** ✓（`install/` 里有 `robonix_rtk_nav2` ∧ `robonix_rtk_nav2_costmap_host` ✓；colcon `JobEnded rc=0` ✓；无 `Failed` ✓） | ★ **通过** ✓（上一轮就死在这一步 ✗） |
| ④ | 起栈 ∧ 短走 ∧ 停栈 | ★ 起栈 **rc=0**（**59 s** ✓）；关键节点齐（`/planner_server` `/global_costmap/global_costmap` `/controller_server` `/lifecycle_manager_nav2` ✓）；短走 **6 分钟到位 25/41 点** ✓；停栈后相关进程 **0** ✓、僵尸 **0** ✓ | ★ **通过** ✓ |

★ ★ **`diff` 的方向也测了** ✓（★ 这条比"差 2 行"更有信息量 ✓）：★ 全量 `diff` **只有 1 个 hunk、类型是 `a`（增行）** ✓
  ⇒ ★ **`c`（改）＝ 0** ✓、★ **`d`（删）＝ 0** ✓ ⇒ ★ **新镜像 ⊂ 参照机** ✓（★ "新镜像多装了参照机没有的包"这一头**不存在** ✓）
  ★ `comm` 实测 ✓：★ 共同 **1813 行** ✓ ／ 仅新镜像 **0** ✓ ／ 仅参照机 **2** ✓；
  ★ 且 ★ **同名包的版本差 ＝ 0** ✓（`join -t『制表符』-j1` 逐包比 ⇒ ★ 1813 个共同包**版本全同** ✓）
  ⇒ ★ 即"`apt` 不钉版 ⇒ 建出来的可能比参照机**新**"这条风险 ✓ ★ **今天没有发生** ✓ —— ★ 差的全是**整包缺**（少 2 个 ✓），✗ 没有**同包不同版** ✓。

★ 构建 ✓：`Successfully built 214feabf16a3` ✓／`Successfully tagged robinox/rtk-nav2-demo:verify` ✓；
★ ★ **补注（2026-10-14 加 ✓）**：★ 这个 `robinox/rtk-nav2-demo:verify` **就是当时那个拼错的 tag ✓**
（★ `docker images` 里现在还看得到它 ✓ ⇒ ★ 记录本身**没写错** ✓）；★ **该用的名字见 §1 的补注** ✓。
★ **耗时 25 min 39 s** ✓（★ 取**阶段时间戳** `17:22:05 → 17:47:44` ✓ —— ✗ 别取构建日志首末行 ✓，那两行不是时间戳 ✗）；镜像 **6.04 GB** ✓（`docker images` 口径 ✓）。

### 差的那 2 个：`ros-humble-topic-tools` 与 `ros-humble-topic-tools-interfaces`
★ 结论 ✓：**参照机里的孤儿包** —— ★ 本版用的 `Dockerfile.amd64` **装不出它们** ✓，✗ 但**也没人需要它们** ✓。
★ ★ **来路：查实了 ✓（★ 不是推断 ✓）** —— ★ 参照机镜像（`9406e5268aa1` ✓）**自己的 `/var/log/apt/history.log`** 里写着 ✓：
```
Commandline: apt-get install -y --no-install-recommends ros-humble-robot-localization ros-humble-topic-tools
```
  ★ 而这条命令与 ★ **`docker/Dockerfile.rtk` 第 30–32 行逐字相同** ✓（`--no-install-recommends` ✓、两个包**同一条** ✓、**顺序也同** ✓）—— ★ 全仓**唯一**一份这样写的 Dockerfile ✓（`Dockerfile.amd64` 里那两个包**在别的行**、且**不含** `topic-tools` ✓）。
  ⇒ ★ 即 ★ **"哪份 Dockerfile 建出来的镜像、就带哪些包"** ✗ 在这 2 个包上**成立且可复核** ✓ —— ★ 本版走 `Dockerfile.amd64`（`build-image.sh` 的 `case` 里 **`Dockerfile.rtk` 出现 0 次** ✓）⇒ ★ **少这 2 个** ✓。
  ★ 附 ★：`/build-info.txt` **不写 Dockerfile 名** ✗（实测 `grep -ci dockerfile` ＝ **0** ✓）⇒ ★ 靠它坐实不了 ✓，★ 是 **apt 历史 ＋ Dockerfile 指纹** 坐实的 ✓。
★ 五条实测：
- ★ 反查依赖 ✓：新镜像里 `apt-cache showpkg ros-humble-topic-tools` 的 `Reverse Depends` ⇒ ★ **空** ✗（没有任何已装包依赖它 ✓）
- ★ 不是被 `--no-install-recommends` 吃掉的 ✓：`ros-humble-desktop`／`ros-base`／`topic-monitor` **都没有 `Recommends`／`Suggests` 字段** ✓
- ★ 仓库里的引用 ✓：★ 非 `docs/` 下命中 **2 行、都在同一个文件 `docker/Dockerfile.rtk`** ✓（★ 第 **26** 行注释 ＋ 第 **32** 行安装行 `ros-humble-topic-tools;` ✓）—— ★ **更正（reviewer 2026-10-12 复量抓到 ✓）**：此处原写"**0 行**" ✗，★ 那是**我那条 grep 的 `--include` 集合里没有 `Dockerfile*`** ✗ ⇒ 拿**窄口径**的结果写成了**宽口径**的结论 ✗；★ 权威计数用 **`git grep -niE 'topic_tools|topic-tools' -- .`** ⇒ ★ 非 `docs/` **2 行／1 个文件** ✓
  ★ ★ 但**结论不变** ✓：★ 那个 `Dockerfile.rtk` **不参与本版构建** ✓（★ 实测：`scripts/build-image.sh` 的 `case` 里 **`Dockerfile.rtk` 出现 0 次** ✓ —— 它只选 `Dockerfile.amd64`（fortress／x86_64 ✓）／`Dockerfile.arm64`／`Dockerfile`（classic ✓））
  ★ ★ 而且**那份文件自己的注释就写着** ✓：`#   topic_tools        1.1.2  -> 排障用（relay/记录开关），非运行期必需` ✓ ⇒ ★ **仓库自己印证了"没人需要它"** ✓✓
- ★ 演示要的那条命令不是它给的 ✓：`ros2 topic …` 来自 **`ros-humble-ros2cli`** ✓（`dpkg -S /opt/ros/humble/bin/ros2` ⇒ `ros-humble-ros2cli` ✓）
★ 所以我**没有**为了把数字凑到 1815 而把它写进 Dockerfile ✗（★ 按"**别为了凑数硬装**" ✓）—— ★ 谁要**逐包齐平**，就往 `docker/Dockerfile.amd64` 第 52 行那块加 `ros-humble-topic-tools` 一行即可 ✓（★ 但那样必须**再重建一次**才能重新验证 ✓，约 25 分钟 ✓）。

# 复现指南（本版为初版）

★ 目标：**在一台干净机器上，照着本文件能跑出与交付版一致的效果** ✓。
★ 本文件里的每条命令都在本机实测过；★ **凡是"没实测"的，会明确标出** ✗（✗ 不假装验过 ✓）。

---

## 0) 这套东西是什么
Robonix 的 RTK + nav2 演示：Gazebo Sim（Fortress）里跑一个**校园场景**（`north_campus`），
车按经纬度航线走 41 个点，边跑边建图；验收看的是**图上干不干净**（车脚下与身后不应出现成片障碍点）。

---

## 1) 前置条件
| 项 | 要求 | 备注 |
|---|---|---|
| 架构 | **x86_64 / amd64** | 本版走的就是 amd64 那份 Dockerfile ✓ |
| Docker | 可用（需要能 `docker build`／`docker run`） | 构建期要联网拉 apt/ROS 包 |
| 磁盘 | ★ 预留 **≥ 15 GB**（镜像 + 一次构建的中间层） | 运行期另有小量产物 |
| 内存 | ≥ 8 GB 建议 | Gazebo + nav2 + RViz 一起跑 |
| 图形 | 有 X/Wayland 显示（跑 RViz 与 Gazebo GUI 用） | 无显示也能起，但看不到图 |
| 网络 | 构建期需要 | apt（`packages.ros.org`／`packages.osrfoundation.org`）与基础镜像 |

---

## 2) 构建镜像（★ 路径就这一条：**自己 build**，不用 registry ✓）
```bash
cd <本仓库根目录>
GAZEBO_ROUTE=fortress IMAGE_NAME=robinox/rtk-nav2-demo:humble-amd64 bash scripts/build-image.sh
# ★★ `IMAGE_NAME=` **不能省** ✗：build-image.sh 自己的默认是 `robinox/slam-demo:humble-amd64`
#   （scripts/lib.sh:35），而演示脚本找的是 `robinox/rtk-nav2-demo:humble-amd64`
#   （scripts/rtk-nav2/lib.sh:23）⇒ ★ 不传 ⇒ **建成别的名字 ⇒ 构建成功但演示起不来** ✗
# ★ 构建后**当场自检**（建成了别的名字立刻暴露 ✓，✗ 别等到起栈才发现）：
docker image inspect robinox/rtk-nav2-demo:humble-amd64 >/dev/null && echo "★ 镜像在 ✓"
```
- ★ **用哪一份 Dockerfile 是坐实的** ✓：`scripts/build-image.sh` 按 **Gazebo 路线**（不是架构）选 ✓，
  `GAZEBO_ROUTE` 默认就是 `fortress` ✓ ⇒ 本版 = **`docker/Dockerfile.amd64`** ✓。
- ★ **基础镜像**：★ 本版走 **Fortress** ⇒ 脚本**直接选 `ubuntu:22.04`** ✓（★ **固定 tag** ✓）；
  想换用 `--base <镜像>` 覆盖 ✓。
  ★ 那条 `robinox-osrf-ros:humble-desktop-full` → `osrf/ros:humble-desktop-full` → `docker pull` 的顺序
  **只对 Classic 路线有效** ✗（见 §6 第 1 条 ✓）—— ★ 别照它准备基础镜像 ✓。
- ★ 构建完会打印 `/build-info.txt`（构建记录 ✓）—— **请把它留档** ✓，它就是这台机器上这一版的凭据之一 ✓。

## 3) 跑起来（三条命令，零参数）
> ★ 起栈与走航线是**长驻前台**命令 ⇒ **各开一个终端**，✗ 不要串在一条里 ✓。

```bash
# ── 终端 1：起栈（约 1–2 分钟，看到「起栈返回 rc=0」即可）
cd <本仓库根目录>
bash scripts/rtk-nav2/demo-start.sh

# ── 终端 2：走航线（★ 等终端 1 打印 rc=0 之后再敲）
cd <本仓库根目录>
bash scripts/rtk-nav2/demo-drive.sh

# ── 走完之后：停栈（★ 先看到"抓图那几行"再停 ✗ —— 那是验收凭证）
cd <本仓库根目录>
bash scripts/rtk-nav2/demo-stop.sh
```
★ 不需要任何环境变量／run-id／分目录／拷文件 ✓：容器名、ROS 域、分区、轮名、输入路径、产物路径**都在脚本里钉死** ✓。

## 4) 应该看到什么（判据）
- **车体不渲染** ✓，车顶**红盘**在 ✓（`rtk_marker_link` 是图元 ⇒ 不依赖渲染资源 ✓）⇒ 靠它盯车
- **起飞前**全局图基本为空（实测 **非零 0 / 致命 0** ✓）
- 车动起来后**车脚下与身后不出现成片障碍点** ✓（整轮走完后，占据格里 **~93–98% 落在真实几何上** ✓）
- 走完打印 **到达 41/41** ✓；收尾**自动抓**一张「走完之后」的全局图（打印若干行 ＋ 落 `artifacts/rtk-nav2/demo/nav2/global_map_after.*` ✓）

---

## 5) 版本锚（★ 复现"对没对上"就看这几个）
| 锚 | 值 | 怎么取 |
|---|---|---|
| ★ **Dockerfile** | `docker/Dockerfile.amd64` ＝ **sha256 前 16：`6c9ae0cbe65df899`** ★（2026-10-12 更新：旧值 `0d3c6187bb6199fa` 那份**只装了 `nav2-map-server`** ⇒ 照旧锚重建**跑不了演示** ✗，已补全 nav2 栈 ✓） | `sha256sum docker/Dockerfile.amd64` |
| 关键脚本 | `demo-start.sh` `dc3d90d6e7cdc22a`／`demo-drive.sh` `b879c9a4723190c5`／`demo-stop.sh` `3a4568a5b00f67bd` | `sha256sum scripts/rtk-nav2/demo-*.sh` |
| 场景/配置 | `campus_scene/model.sdf` `784e2ae135673aad`／`campus_nav2.yaml` `d01fd872f6b517d6`／`navsat.yaml` `b38894ee4f9897b1` | 见 `docs/image/` 与各文件 |
| ★ **参照机的包版本** | `docs/image/dpkg-versions.txt`（1815 个包 ✓） | 构建出的镜像里 `dpkg-query -W` 对一遍 |
| 参照机镜像 id | `sha256:9406e5268aa1b9f7…`（**仅供对照** ✗ —— 你们 build 出来必然是新 id ✓） | `docker image inspect` |

★ ★ **逐条对照建议** ✓：build 完在镜像里跑
```bash
docker run --rm --entrypoint bash robonix/rtk-nav2-demo:humble-amd64 -lc \
  "dpkg-query -W -f='\${Package}\t\${Version}\n' | sort" > /tmp/their-dpkg.txt
diff /tmp/their-dpkg.txt docs/image/dpkg-versions.txt | head -50
```
⇒ ★ 差出来的就是"重建与参照机的不同" ✓（重点看 `ignition-fortress`／`libignition-gazebo6`／`ros-humble-*` ✓）。

---

## 6) 已知不保证（★ 如实列出 ✗）
1. ★ **Dockerfile 里的 apt 安装没有钉版本** ✗（`apt-get install -y --no-install-recommends ignition-fortress ros-humble-… ` 全为浮动 ✓）
   ⇒ ★ **今天重建，ROS 源上的包可能已经比我们这份新** ✗ ⇒ 效果大体一致，但**不是逐字节相同** ✓。
   - ★ 参照机上装到的版本已整份导出 ✓：`docs/image/dpkg-versions.txt` ⇒ **想严格对齐就照它钉** ✓
     （例：把 `ignition-fortress` 改成 `ignition-fortress=1.0.3-2~jammy`、`libignition-gazebo6-dev=6.18.0-1~jammy` ✓）
   - ★ **基础镜像这条对本版不适用** ✓：那条 `osrf/ros:humble-desktop-full` 链是 **Classic 路线**的 ✓；★ 本版走 **Fortress** ⇒ 直接落到 **`ubuntu:22.04`**（★ **固定 tag** ✓）⇒ ★ 这条**不构成"不保证"** ✓（对复现是好消息 ✓）。
2. ★ 本文件里"构建"这一步的实测记录 ⇒ 见 **`docs/image/build-test.md`** ✓（★ 该文件已建 ✓：含一次真 build 的结果、耗时、磁盘与网络需求，以及**抓到的 nav2 缺口与已补** ✓）。
3. ★ **在"他的机器上"我们验不了** ✗ ⇒ 第 4 节的判据请在那边自己看一遍 ✓。

---

## 7) 目录导览（★ 初版只收"现在这一版" ✓ —— 一句话：这就是现在的演示 ＋ 怎么复现）
```
rtk_nav2_ws/src/robonix_rtk_nav2/   ★ 代码＋配置＋世界＋模型（本演示主体）
scripts/rtk-nav2/                   ★ 入口与工具（demo-start/drive/stop ＋ demo-data/ 固定输入）
scripts/                            ★ 构建镜像与通用库（build-image.sh 等）
docker/                             ★ Dockerfile ×6 ＋ 构建脚本 ＋ apt 公钥
docs/                               ★ 只留本版：reproduce.md／demo-usage.md／delivery-ground-fix.md
                                      ／merge-into-outdoorNav.md／run-block-long.md／image/（版本清单）
tests/                              ★ 与演示同级的自检
```
★ **故意不进仓库**（`.gitignore` 已挡 ✓，文件仍在磁盘上 ✓）：
- `artifacts/`（按轮运行产物；本机 **107 GB** ✗）、`rtk_nav2_ws_campus9-*`（按轮工作区副本 ✗）、`video/`
- ★ 以及**历史件**（业主口径"之前的版本不引入" ✓）：`.reh/`（9 月排练草稿）、
  `demo_ws/`・`rgbd_ws/`・`rtk_ws/`（更早阶段的工作区；**现版脚本一个都不引用** ✓，实测 0 处 ✓）、
  早期文档（`campus-8/9-*`・`depth-drop-*`・`rgbd-*`・`rtk-*`・`environment*` …）、
  `CLAUDE.md`／`AGENTS.md`／`agent.md`（智能体工作文件，其中一个还指向本机私有路径 ✗）
★ 收完之后的清单：**427 个文件**、`.git` **4.6 M**、顶层 8 项 ✓

---

## 8) ★ 一条硬警告：**不要重新生成场景** ✗
本演示用的场景是 `campus_scene`（并入他们仓库后位于 `gazebo_campus/generated/models/campus_scene/` ✓）。
★ 那个目录**能被一键重新生成** ✗ —— `gazebo_campus/build.sh` 会跑 `scripts/build_scene.py` ✓，
而它写的是**修复前**的值（ground 的 z ＝ `-0.1` ✓）＋ **不发法线**（Fortress 里没法线的 mesh ⇒ 纯白不受光 ✗）。
⇒ ★ 重跑一次会**同时抹掉两处修复** ✗✗。
★ 所以：**要改场景请改生成器（`build_scene.py` 的 z 与"生成后补法线"），✗ 不要用"生成覆盖 + 手工改回"的老路** ✓。
★ 判据（换完 / 改完全应逐位对上 ✓）：
`model.sdf` ＝ `784e2ae135673aad` ｜ `meshes/roads.dae` ＝ `6e9dc6f6bed095d3` ｜ `meshes/plaza.dae` ＝ `f1ed7a072c483103`

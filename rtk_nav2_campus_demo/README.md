# RTK ＋ Nav2 校园导航演示（`rtk_nav2_campus_demo/`）

在 Gazebo Sim（Fortress）里跑一个**校园场景**（`north_campus`）：车按**经纬度航线**走 41 个点，
边跑边建图。★ 验收看的是**图上干不干净** —— 车脚下与身后**不应出现成片障碍点**。

★ 本子树里的东西 ✓：Dockerfile、脚本、固定输入、文档**都在里面** ✓。

★ ★ **一处例外（★ 别把它当"完全自包含" ✗）**：★ **场景不是** ✗ ——
**它取自外层** ✓：`<本仓根>/gazebo_campus/generated/models/` ✓
（起容器时 `scripts/rtk-nav2/lib.sh` 把它**只读挂进**容器 ✓，容器内路径 `/opt/rtk_nav2_external_models` ✓）。
⇒ ★ ★ **别把本子树单独拷走** ✗ —— ★ 拷走了**世界加载失败** ✓，而表相是「**看着像卡在起栈**」✗
（★ 这个坑我们记过 ✓：`docs/delivery-ground-fix.md` 那族 ✓）。

---

## 一、怎么跑（★ 三步，但**前面还有第 0 步**）

### 0) 先把镜像拿到手（★ 这一步不能省 ✗）

★ 本演示用的镜像是 **`robonix/rtk-nav2-demo:humble-amd64`** ✓。
★ **实情**：★ 它**只在我们那台机器上** ✗ —— **没有推到 registry**，也**没有导出 tar** ✓。
⇒ ★ **必须自己 build 一次** ✓（首次约 **20 分钟**；构建期**要联网**拉 apt 与 ROS 包 ✓）。

```bash
cd <本仓库根目录>          # 即含 scripts/ 与 docker/ 的那一层
GAZEBO_ROUTE=fortress IMAGE_NAME=robonix/rtk-nav2-demo:humble-amd64 bash scripts/build-image.sh
```

★ **`IMAGE_NAME=` 不能省** ✗：`scripts/build-image.sh` 自己走的是**另一个默认名**
（`scripts/lib.sh` 里写的是 `robinox/slam-demo:humble-amd64`，属更早的阶段 ✓）
⇒ 不传就会**建成别的名字** ⇒ ★ **构建成功、而演示起不来** ✗。
★ 构建完**当场自检**（建错名字立刻暴露 ✓，别等到起栈才发现 ✗）：

```bash
docker image inspect robonix/rtk-nav2-demo:humble-amd64 >/dev/null && echo "★ 镜像在 ✓"
```

前置条件（架构／Docker／磁盘 ≥15 GB／内存／图形／网络）与细节：
见 [`docs/reproduce.md`](docs/reproduce.md) §1–§2 ✓。

### 1) 起栈

```bash
bash scripts/rtk-nav2/demo-start.sh
```

### 2) 走航线（★ 另开一个终端）

```bash
bash scripts/rtk-nav2/demo-drive.sh
```

### 3) 停栈

```bash
bash scripts/rtk-nav2/demo-stop.sh
```

★ 三条命令**都零参数** ✓ —— 容器名／ROS 域／Gazebo 分区／轮名全部钉在脚本里 ✓；
输入固定在 `scripts/rtk-nav2/demo-data/` ✓；产物固定写 `artifacts/rtk-nav2/demo/` ✓。
★ ★ **走完别马上停** ✗ —— 走完那一刻 `demo-drive.sh` 会自动抓一张「走完之后」的全局图
（会打印几行，含尺寸与值域自检 ✓）；★ **先看到那几行再停栈** ✓，否则那张凭证就没了 ✓。

### 4)（可选）把前端那条链也开起来

```bash
bash scripts/rtk-nav2/frontend-bridge.sh
# 起完浏览器开 http://localhost:5173
```

★ 它起三样：发 `/odom` 的适配器、**你们的** HTTP 桥（`127.0.0.1:8765`）、前端 dev server ✓。
★ `:5173` 上跑的就是本子树里的 `outdoor-patrol-demo/` ✓（脚本起前端**之前**会把它同步进容器
并**逐文件比 sha256** ✓，不一致就报错停 ✓）。

---

## 二、不依赖什么（★ 这一节是给"克隆完发现跑不起来"的人看的）

- ★ **先说那一处例外** ✓：★ 本子树**依赖外层**的 `gazebo_campus/generated/models/` ✓（场景 ✓，见开头）⇒ ✗ 别单独拷走 ✗。
- ✗ 本子树**不依赖** `demo_ws/` ✗、✗ 也**不依赖** `scripts/container/` ✗。
  ★ 它们属于**更早的 slam 阶段** ✓，★ 且**在 `.gitignore` 里、不在本子树里** ✓
  ⇒ ★ 克隆本子树后**看不到它们是正常的** ✓（✗ 不是缺件 ✓）。
- ✗ 同样**不需要** `rtk_ws/`・`rgbd_ws/`・`campus9-*` 那些工作区 ✗。

---

## 三、文档

| 文件 | 讲什么 |
|---|---|
| [`docs/demo-usage.md`](docs/demo-usage.md) | 怎么跑 ＋ ★ **起栈前必读** ✓ ＋ 跑起来该看到什么 ＋ 产物口径 |
| [`docs/reproduce.md`](docs/reproduce.md) | 在**一台干净机器**上复现（前置条件／构建／跑／验收） |
| [`docs/frontend-integration.md`](docs/frontend-integration.md) | 前端那条链（`POST /api/routes` → 桥 → nav2 → 车）＋ ★ 已知问题 ✓ |
| [`docs/frontend-integration-recon.md`](docs/frontend-integration-recon.md) | 接前端之前做的只读侦察（接口与约定） |
| [`docs/delivery-ground-fix.md`](docs/delivery-ground-fix.md) | 校园场景那处**地面物理／渲染对齐**的修复说明 |
| [`docs/run-block-long.md`](docs/run-block-long.md) | 长跑（完整航线）那一版的跑法 |

★ 一句提醒 ✓：★ 本子树的**场景**与 `gazebo_campus/` 共用**同一份** ✓ ——
✗ 本子树里**没有**第二份 `campus_scene` ✗（两份会分叉 ✓）。

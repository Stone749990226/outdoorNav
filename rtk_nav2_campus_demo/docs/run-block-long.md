# ★★★ 最终交付版：物理地面抬到可见铺装之下 ＋ 不渲染小车 ★★★
# 目录：/mnt/c/Users/xjy/projects/robonix （**主树，就地改的**）
#
# 改了三处（其它一字未动，哈希见产物）：
#   ① worlds/models/campus_scene/model.sdf 里 **ground 两处**（碰撞盒 ＋ 可见地面）的 pose z：-0.1 → -0.071
#      ⇒ 两面顶面都从 0.000 抬到 **0.029**（**铺装渲染面仍是 0.030** ⇒ 留 1 mm 缝，不共面、不会闪）
#      ★ 两面同高 ⇒ 裸地处车也不会浮起来 ✓；★ 铺装处激光看到的仍是 0.030 ⇒ 读数不变 ✓
#   ② scripts/rtk-nav2/start-northcampus.sh 里 z_pose:=0.01 → **z_pose:=0.039**（地面抬 0.029 ⇒ 出生点同抬）
#   ③ scripts/rtk-nav2/start-northcampus-cut.sh 里同样 0.01 → **0.039**（它也跑 north_campus）
#   ★ 另外四个 `z_pose:=0.01` **没改**：它们跑的是 campus_100／depth_drop，**那个世界不含这个地面盒**
#     （实测：全树只有 north_campus.sdf 引用 campus_scene）⇒ 改了反而会把车生成在地面里 ✗
#
# ★ 下面两段是**整段粘就能跑**的。
# ★★ 终端 1 那条是**长驻前台**：它会一直占着那个终端，别关窗口 ——
#    看到最后一行 `起栈返回 rc=0`（约 1–2 分钟）再去终端 2。


# ───────────────────────── 终端 1（长驻前台） ─────────────────────────
cd /mnt/c/Users/xjy/projects/robonix
mkdir -p artifacts/rtk-nav2/final-owner/input
touch artifacts/rtk-nav2/final-owner/.no_render_fix
cp artifacts/rtk-nav2/final-off/geo_calib.json artifacts/rtk-nav2/final-owner/
cp artifacts/rtk-nav2/final-off/input/route_geo.json artifacts/rtk-nav2/final-owner/input/
CONTAINER_NAME=rtknav2-main ROS_DOMAIN_ID=55 GZ_PARTITION=rtknav2main bash scripts/rtk-nav2/start-northcampus.sh final-owner


# ───────────────────── 终端 2（等终端 1 打出 `起栈返回 rc=0` 再粘） ─────────────────────
cd /mnt/c/Users/xjy/projects/robonix
CONTAINER_NAME=rtknav2-main ROS_DOMAIN_ID=55 GZ_PARTITION=rtknav2main bash scripts/rtk-nav2/geo-route.sh final-owner --tol 0.40


# ★ 四段说明（都是逐字可跑的，没有占位符）：
#   · 终端 1 第 2–5 行：这一轮的产物目录 ＋ **`.no_render_fix` 标记** ＋ **两个输入文件**
#     —— `touch .../.no_render_fix` 就是"不渲染小车"的开关（`container/start_stack.sh` 里按轮判断的 ✓）；
#       少了两行 `cp` ⇒ `geo-route.sh` 找不到标定/航线，会直接报缺文件 ✗（★ 换轮名时最容易漏这两行 ✓）。
#       ★ 想换别的轮名：把四条 `final-owner` **一起**换成那个名字即可（四行要一致 ✓）
#   · 终端 1 里的 `z_pose` 是**新值 0.039**（写在 `start-northcampus.sh` 第 115 行里 ✓）
#     —— 如果你手上还有把 `z_pose:=0.01` 抄下来的老命令，**别用**，那会把车生成在地里 ✗
#   · 只想让车动几米、不整条航线：把终端 2 那条换成
#       CONTAINER_NAME=rtknav2-main ROS_DOMAIN_ID=55 GZ_PARTITION=rtknav2main bash scripts/rtk-nav2/nav-to.sh final-owner -0.4 5.0
#     （或直接在 RViz 里按 2D Goal Pose 点一个几米外的点）

# ★ 跑起来该看到什么（判据，都是"图上/眼里"就够的）：
#   · 车体**不渲染**，但 **车顶那个红盘还在** ✓ —— `rtk_marker_link` 的红色圆盘是**图元**
#     （`<cylinder radius="0.5" length="0.012">`，URDF 第 70–74 行 ✓）⇒ **不依赖 resource path** ⇒
#     渲染关着它照样画 ✓（★ 这也是"看不见车"时仍能盯住车在哪的标记 ✓）
#   · **起跑前**（还没发目标）全局图上应当**基本是空的**（实测量到的是 非零 0 / 致命 0 ✓）
#   · 车动起来后，**车脚下与车身后方不应再出现成片的障碍点**（这是这一版要修的）✓
#   · ★ 中途可能有一次**几分钟的停顿**（车不动、日志还在写）：这是本项目**已知**现象
#     （发目标后的发现竞态；实测交付轮停过 **420.8 s**、之后自己恢复并继续走 ✓）⇒ **别以为它坏了** ✓

# ★ 新增两处口径（2026-10-12，业主裁定放宽到点容差之后）：
#   ① 按固定航线走那条（★ **不在当前演示流程里** ✓）**末尾加了 --tol 0.40** —— 到点判据是「进容差 ∧ 停稳」两条同时成立；
#      默认 0.25 会让车在**距目标约 0.29 m** 处收敛停住 ⇒ 差约 4 cm 进不去 ⇒ 判不了到达 ⇒
#      原地等超时（实测停过 420 s，还会掉一个点的到位判定）。放宽到 0.40 ⇒ 不再卡、判据不变。
#      ★ 只动了命令行这一个参数；config/campus_nav2.yaml **一个字节没动**。
#   ② ★ **等脚本自己打印完成再收栈** —— 走完之后 `geo-route.sh` 会**自动抓**一张
#      「走完之后」的全局图（会打印抓图的几行，含尺寸与值域自检）⇒ 先看到那几行再停栈，
#      否则那张图就没了（它是验收凭证）。★ 万一没抓到，日志里会写明。

# 卫星底图（2026-11-11）

已完成 `web/` 智慧谷北区演示界面的卫星底图。此页面与 Robonix `/outdoor` Scene 分开。

## 使用

在 `web/` 运行 `npm run dev`，访问终端中的地址。A→B 页通过“显示／隐藏卫星底图”切换；巡航页通过“图层设置 → 卫星底图”切换。卫星底图默认开启，示意底图默认关闭，两个图层可独立操作。

底图随前端一起加载，浏览页面不需要连接瓦片服务器。缩放、拖动及切换代价地图不会改变坐标或已选目标。

## 重新生成

使用带 Pillow 的 Python 环境运行：

```bash
.venv/bin/python tools/fetch_satellite_tiles.py
```

可加一个非负数字设置地图外侧的余量，单位为米，默认为 10。脚本下载 Esri World Imagery 的 z=19 瓦片，生成两个配套文件：

- `web/src/assets/north-campus-satellite.webp`：默认 1136×1112，约 77 KiB。
- `web/src/assets/north-campus-satellite.json`：影像的经纬度范围。

两个文件需要一起保留。`MapView.tsx` 使用已有坐标换算函数摆放图片，重新生成后无需手改前端尺寸。遮罩透明度为 0.52，便于看清路线和文字。页面显示影像来源，来源与使用条款见 [Esri World Imagery](https://goto.arcgisonline.com/maps/World_Imagery)。

按经纬度对齐不能消除影像自身的位置误差或建筑倾斜效果，底图不用于判断道路是否安全。

## 验证

- 生产构建通过，18 项单元测试通过。
- 底图／代价地图、山猫导航和道路权重相关的 4 项浏览器测试通过。底图测试覆盖图片解码、两个底图开关独立操作及切换代价地图后恢复。
- 已检查巡航页图层开关、缩放／拖动／复位、桌面和 390 像素宽页面；未发现页面脚本错误或横向溢出。
- `navigation.spec.ts` 的两个失败在干净 HEAD `29fe49c` 上同样复现：旧断言漏了 `request_id`；障碍物测试未展开“路线详情”工具面板。未修改这两项无关测试。
- `demo.spec.ts` 的旧失败由前一轮交接记录，本轮未重跑整组。

截图位于 `artifacts/satellite-basemap-20261111/`。本次没有创建 Git 提交。

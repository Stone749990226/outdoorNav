# OutdoorNav - 园区巡航与路径规划演示

一个可在浏览器运行的园区二维地图与巡航规划原型。支持道路巡航、A→B 区域规划、模拟封路重规划、关键途经点提取，以及地图和路线导出。

地图由参考图片人工辅助标注；道路宽度、比例尺和机器人包络均为演示假设。项目尚未连接真实机器狗、RTK 或 ROS，导出路线不能直接用于实机导航。

## 运行

需要 Node.js 22.12 或更新版本。

```bash
cd outdoor-patrol-demo
npm ci
npm run dev
```

打开终端显示的地址，通常是 <http://localhost:5173>。页面右上角的“一键演示”可运行预置巡航任务。

## 验证

```bash
cd outdoor-patrol-demo
npm test
npm run build
```

## 文档

- [演示项目说明](outdoor-patrol-demo/README.md)：功能、交互、算法和导出格式。
- [三分钟演示稿](outdoor-patrol-demo/DEMO.md)：演示顺序与讲解要点。
- [实机导航路线图](outdoor-patrol-demo/MAPPING_ROADMAP.md)：地图校准、定位与机器人对接。
- [ROS 2 / Nav2 源码阅读笔记](docs/ROS2_SOURCE_NOTES.md)：本地学习用的上游源码阅读入口。

`nav2/`、`ros2_src/` 是本机单独克隆的上游仓库，不属于本项目的提交内容；根目录的机器狗用户手册 PDF 也只保留在本机。

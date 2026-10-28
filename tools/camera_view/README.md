# 山猫相机网页（低带宽）

每秒抓一帧、压成宽度 640 的 JPEG，通过网页显示；只在你打开某个相机时才启动对应的
ffmpeg 拉流，`idle_stop_s` 秒无人查看即自动停止，隧道带宽约 20–40 kB/s。

## 运行

山猫上：

```bash
bash tools/camera_view/start.sh          # 监听 127.0.0.1:8080
```

本地电脑（与户外导航 8766 一起转发也行）：

```bash
ssh -p 6003 -N -L 8080:127.0.0.1:8080 nvidia@svr.gddg08.top
```

浏览器打开 <http://127.0.0.1:8080>，上方按钮切换相机；按钮灰掉表示该路当前没有推流。

## 配置

`config.json` 的 `cameras` 是"名字 → RTSP 地址"的映射；改名字即可把某一路标成
前/后/左/右。默认：

| 名字 | 地址 |
| --- | --- |
| front | rtsp://10.21.33.103:8554/video2 |
| back | rtsp://10.21.33.103:8554/video3 |
| left | rtsp://10.21.33.103:8554/video4 |
| right | rtsp://10.21.33.103:8554/video5 |

（当前只有 `video2` 在推流，其余返回 404，页面会显示为不可用。）

`width`、`fps`、`jpeg_quality`（ffmpeg `-q:v`，数值越大越小越糊）用于调节画质与带宽。
服务默认只监听 127.0.0.1，必须经 SSH 隧道访问，不直接暴露到网络。

## 接口

- `GET /`：网页
- `GET /api/cameras`：各路可用状态与帧年龄
- `GET /frame?cam=front`：最新一帧 JPEG（无流时 503）

相机网页部署：见 tools/camera_view/README.md。

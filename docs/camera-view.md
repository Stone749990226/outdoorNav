# 山猫相机网页（远程看状态）

在不方便到车边时，用浏览器看山猫四路相机画面，确认车的位置、朝向和周围情况。
每秒取一帧、按需压成 JPEG，经 SSH 隧道传到本地；默认 640 px、1 秒一帧时约 20–40 kB/s。

## 组成

| 文件 | 作用 |
| --- | --- |
| `tools/camera_view/server.py` | HTTP 服务：按需启动 ffmpeg 拉 RTSP，缓存最新一帧并输出 JPEG |
| `tools/camera_view/config.json` | 监听地址、画质等级、更新间隔、相机路径映射 |
| `tools/camera_view/start.sh` | 前台启动（调试用） |
| `tools/camera_view/lynx-camera.service` | 用户级 systemd 单元（常驻） |

服务只监听 `127.0.0.1`，没有认证，**必须经 SSH 隧道访问，不要直接暴露到网络或公网**。

## 部署（山猫）

仓库位于 `/home/nvidia/outdoorNav-git`：

```bash
cd /home/nvidia/outdoorNav-git
mkdir -p ~/.config/systemd/user
cp tools/camera_view/lynx-camera.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now lynx-camera.service
systemctl --user status lynx-camera.service      # 应为 active (running)
```

`Linger` 已开启时重启后会自动启动。临时调试也可以直接前台运行：

```bash
bash tools/camera_view/start.sh
```

## 本地使用

一条 SSH 同时转发导航与相机：

```bash
ssh -p 6003 -N -L 8766:127.0.0.1:8766 -L 8080:127.0.0.1:8080 nvidia@svr.gddg08.top
```

浏览器打开 <http://127.0.0.1:8080>：

- 顶部显示**当前电量百分比**，充电时标注“充电中”，低于或等于 20% 时用醒目颜色提示。
  每约 5–7 秒读取一次本体状态；超过 15 秒未更新会明确显示过期，不把旧值当作当前电量。
- 顶部按钮切换 **front / back / left / right**；打开页面会自动选中第一个可用相机。按钮灰掉表示该路当前没有推流。
- **画质/带宽**滑杆 0–100：同时调整分辨率与 JPEG 质量，240 px·q28（最小）→ 1280 px·q5（最高）。
- **更新间隔**滑杆：0.1–5 秒，对数刻度。
- 状态行显示：采集时刻、山猫侧帧龄、端到端延迟（两机时钟不一致时会标注不可用）、本次请求耗时、帧大小；右侧显示估算带宽。
- 没人查看 60 秒后 ffmpeg 自动退出，不占带宽和 CPU。

实测（同一画面）：等级 10 ≈ 2.9 kB/帧、55 ≈ 12.6 kB/帧、95 ≈ 52 kB/帧。

## 配置

```json
{
  "listen_host": "127.0.0.1",
  "listen_port": 8080,
  "level": 55,
  "interval_s": 1.0,
  "idle_stop_s": 60,
  "probe_interval_s": 30,
  "cameras": {
    "front": "rtsp://10.21.33.103:8554/video4",
    "back":  "rtsp://10.21.33.103:8554/video0",
    "left":  "rtsp://10.21.33.103:8554/video2",
    "right": "rtsp://10.21.33.103:8554/video6"
  }
}
```

改 `cameras` 的名字即可重命名或增删通道；改完 `systemctl --user restart lynx-camera.service`。
页面上的滑杆只改运行中的参数，重启后回到 `level` / `interval_s`。

## 相机现状（2026-10-28 现场核对）

- 四路都在机载 MediaMTX（`10.21.33.103:8554`）上，分辨率 1024×576 H.264；
  命名按厂商文档是 `video{0,2,4,6}`，方向经现场画面确认：
  **front = video4、back = video0、left = video2、right = video6**。
- 曾观察到一次“遥控器关闭视频界面后四路全部 404”，在遥控器里把相机界面关掉再打开后恢复；
  之后遥控器完全关闭时四路仍有流。**目前不认为依赖遥控器**，该现象按偶发处理；
  若再次出现，先确认 MediaMTX 上四路是否还有流。

## 接口

| 路径 | 说明 |
| --- | --- |
| `GET /` | 网页 |
| `GET /api/battery` | 电量百分比、电压、充电状态、数据年龄与可用性 |
| `GET /api/cameras` | 各路可用状态、运行状态、最近帧时间；含当前 `settings` |
| `GET /api/settings` | 当前 `level` / `interval_s` 及换算后的宽高、质量、fps |
| `POST /api/settings` | `{"level":0-100,"interval_s":0.1-5}`，校验后立即重建 ffmpeg |
| `GET /frame?cam=<name>` | 最新一帧 JPEG；无流时 503，长时间取不到帧时 504 |

`/frame` 响应头：`X-Frame-Time`（采集时刻，epoch 秒）、`X-Frame-Age-Ms`（服务端帧龄）、`X-Camera`。

## 故障排查

| 现象 | 处理 |
| --- | --- |
| 页面显示“后台不可达” | 隧道没加 `-L 8080:127.0.0.1:8080`；先访问 `/api/cameras` 看是否有 JSON |
| 某一路按钮灰色 | 该路径在 MediaMTX 上不存在（推流端未开），不是前端问题 |
| 画面停在旧帧 | 切换一次画质滑杆（会重建 ffmpeg），或 `systemctl --user restart lynx-camera.service` |
| 改了参数但画面没变 | 确认页面已强刷（Ctrl+Shift+R），旧页面不会加载新逻辑 |

## 电量来源

通过本体监控协议 `10.21.33.103:30004` 发送只读状态订阅心跳，读取设备状态中的
`BatteryList[0].BatteryLevel`（单位 %）、`Voltage`（V）和 `charge`（是否充电）。
不发送站立、移动或模式切换命令，不订阅／使用本体 ODOM。
如连接地址不同，可在 `config.json` 设置 `battery_host`、`battery_port`。

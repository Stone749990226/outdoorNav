# 山猫相机网页

每秒一帧、可调画质与间隔的低带宽相机查看工具；只监听 127.0.0.1，经 SSH 隧道访问。

完整说明（部署、隧道、页面功能、配置、接口、排查）见 **[docs/camera-view.md](../../docs/camera-view.md)**。

快速开始：

```bash
# 山猫
cp tools/camera_view/lynx-camera.service ~/.config/systemd/user/
systemctl --user daemon-reload && systemctl --user enable --now lynx-camera.service

# 本地
ssh -p 6003 -N -L 8766:127.0.0.1:8766 -L 8080:127.0.0.1:8080 nvidia@svr.gddg08.top
# 浏览器 http://127.0.0.1:8080
```

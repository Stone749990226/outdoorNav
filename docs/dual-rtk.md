# 双天线 RTK 接入

> 历史保留方案。2026-10-26 默认航向已切换为新十轴 WIT IMU，位置仍为原 u-blox RTK；双天线不在默认采集／导航链路中。

## 当前结果（2026-10-22）

同事包标注 BT-982G1／Unicore UM982，USB 为 PL2303G `067b:23a3`。
USB ID 只能确认转串口芯片，接收机具体型号仍以标签／版本查询为准。
原包 SHA256：`ac546f3de557da3125bd6abb0c0c5f8b60f429ef9a1a8077517a2fca02884887`。

山猫为 Ubuntu 22.04、aarch64、`5.15.148-tegra`，原内核配置未启用 PL2303。
已确认现有 Tegra 头文件可编译包内 5.15 源码；运行原 `install.sh` 后，DKMS 安装并加载成功：

- `pl2303-rtk/1.0, 5.15.148-tegra, aarch64: installed`
- 模块：`/lib/modules/5.15.148-tegra/updates/dkms/pl2303.ko`
- `/dev/rtk -> ttyUSB0`，普通 nvidia 用户已有 dialout 权限。
- 旧 u-blox 保持在 `/dev/ttyACM0`，没有改动其采集配置。

安装流程安装了 DKMS 及其依赖，包含 generic 头文件和部分 GCC 运行库更新；运行内核未更换，未重启。
原安装包与构建目录保留在山猫 `/tmp/outdoornav-dual-rtk/rtk-driver-ubuntu2204`。
原压缩包和未修改源码还复制到持久目录 `/home/nvidia/outdoor-rtk/` 及其 `vendor/rtk-driver-ubuntu2204/`。
卸载可使用该目录的原 `sudo ./uninstall.sh`；本机原本没有 PL2303 模块，卸载后新接收机将再次无串口。

原 `rtk-test --port /dev/rtk --duration 10 --no-diag` 运行成功，收到八类语句各约 10 条：
GGA、GSA、GSV、RMC、VTG、HDT、THS、TRA，校验错误为零。
独立采集工具另测 10 秒收到 2120 字节，校验错误和格式错误均为零。
持久部署入口再采样 3 秒正常；新增解析核心测试在山猫系统 Python 下 4 项通过，
本地相关回归 19 项通过、3 项因无 ROS 跳过。实际日志回放的无效状态与实机结果一致。
室内 GGA quality=0、0 星、THS=V，无有效位置或航向；HDT 仍输出 0°，**不能把这个 0°当作真实北向**。
串口链路成功不等于双天线定向成功；也不能仅凭室内零星判定天线故障。

## 室外操作

已将独立测试工具放在山猫持久目录 `/home/nvidia/outdoor-rtk`，只依赖系统 Python 标准库。
它不运行导航，不发布速度，不发送接收机配置命令。与原 `rtk-test` 二选一，顺序运行，不同时读取串口。

```bash
ssh nvidia@10.29.1.4
lsusb -d 067b:23a3
ls -l /dev/rtk
cd /home/nvidia/outdoor-rtk
bash deploy/lynx/check_dual_rtk.sh --duration 180
```

每 2 秒显示状态，结束写入 `~/rtk_logs/dual_<时间>.json` 和同名 `.nmea`；Ctrl+C 会保存已采集结果。
也可用原工具：`rtk-test --port /dev/rtk --duration 180 --no-diag`。
本项目采集器不自动回退到 ttyACM0，避免误读旧 u-blox；退出恢复串口设置。

两根天线需固定在同一刚体、保持合适间距和开阔天空视野，间距要求以设备说明为准。
先确认 ANT1／ANT2 的接线与接收机航向基线定义，再记录该基线与机身前方的夹角。
不要仅凭 NMEA 的 true heading 标签推断天线顺序。

航向转换为校园 ENU：`body_yaw = π/2 - true_heading + baseline_to_body_offset`。
其中 offset 是从接收机所报基线方向转到机身前方的逆时针角度，弧度；尚未测量时不填。
实际测试可使用 `--baseline-to-body-yaw-deg <实测角度>`，不填写则只显示原始真航向，不生成机身朝向。

## 解析和判定

- 只接受校验通过的完整 NMEA；支持串口分片。
- GGA 区分单点、差分、RTK 浮点和固定解，重复 UTC 不刷新位置有效期。
- THS 必须为 A 才报告有效航向；V 撤销旧航向，断流超时后失效。
- HDT 没有解状态，只作为未验证观测显示；TRA 的字段定义未确认，不猜测字段解析为航向。
- THS A 不是“固定基线解、已知航向精度”的证明；GGA 的 RTK 固定状态针对位置，也不能代替基线解状态。
- 回放：`bash deploy/lynx/check_dual_rtk.sh --replay <日志.nmea>`。回放仅检查协议，不能验证实际更新率和延迟。

同事原脚本有已确认的判定问题：最高质量 `>=4` 会把浮点 5、INS 6 等误报为固定解；
TRA 首字段被当作航向，但本机收到的是类似 `-00001.00` 的字段，实际协议应另行核对。
原脚本保持原样安装，结论以原始数据和本项目解析为准。

## 距离正式导航还缺什么

10 月 23 日曾按“原 u-blox RTK 给位置＋新双天线只给航向”接入定位和网页诊断（当前为非默认旧模式）；**未替换位置来源，未放行运动**。
现有生产 GNSS 解码器使用 u-blox UBX NAV-PVT；UM982 的 NMEA 不能直接代入。
运动验收前仍需获得并校验原 RTK 位置误差、双天线基线解状态／航向误差、观测时间、天线杆臂与安装角，
再与 IMU／连续里程计和 GNSS 地图对齐集成。
GGA 只有 HDOP，不能将其当作米制 hAcc，不能因此降低现有定位门槛。

位置 RTK 通常还需要基站／NTRIP 差分；双天线相对定向与全局位置 RTK 固定是不同状态。
本次没有给新接收机注入改正，也没有改写已有 u-blox 的 NTRIP 串口配置。
后续只验证新接收机的有效航向、安装角和精度；位置继续使用旧 RTK 及既有千寻客户端，不再推进新接收机替代原 RTK。

室内原始证据保存在本机 `artifacts/lynx/dual-rtk-20261022/`；运动开关保持关闭。

## 分离接入接口

新 `deploy/lynx/heading_stream.py --config <robot.json>` 独占只读 `/dev/rtk`，输出到独立本机 UDP；不回退旧串口、不发送接收机命令，退出恢复串口参数。`start.sh` 管理它与原 GNSS 包装采集者的生命周期，诊断默认两者均 observe。配置与隔离验证入口见 [部署说明](../deploy/lynx/README.md#位置与航向分别接入)。

原 RTK 的位置不会被新设备的 GGA 覆盖；两个来源独立显示有效／失效状态。航向未安装标定时，仅显示原始真航向，不生成默认 0° 的车体朝向。当前 THS 缺少采样时间字段，时序检查依据主机单调接收时间，真实传输延迟仍待测量。

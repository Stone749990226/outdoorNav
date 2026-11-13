# 实机复盘 2026-11-13:航向确认门槛删除与部署就绪

## 背景

当日重新部署 field 清单后导航迟迟不就绪,排查发现三个卡点,最终全部解决,
导航服务 ready=true。用户要求:RTK 室外固定解即放行,不再逐次确认航向精度;
已生效并固化到本地工作区。

## 问题 1:quality_confirmed 门控每次启动都拦(已删除)

**现象**:定位故障 `heading_quality_unconfirmed`,融合停在 initializing,
`stationary_collected_s=0`,导航未就绪。

**根因**:清单里 `quality_confirmed: false`、`yaw_std_deg: null`(防呆默认),
eskf 模式下 heading 观测 ready 要求 quality_confirmed=true,每次重启都被拦,
现场配置易被同步覆盖丢失。

**处理**(按用户要求删除该机制):
- `localization_outdoor/inputs.py`:heading ready 只看 body_yaw 是否有值,
  不再要求 quality_confirmed(安装旋转、磁偏角等持久确认项仍保留门控)。
- `localization_outdoor/configuration.py`:删除 quality_confirmed 字段与校验;
  yaw_std_deg 默认 20(2026-11-11 现场授权值:磁航向实测误差约 18°)。
- 两个清单固化 `yaw_std_deg: 20`、`max_yaw_2sigma_deg: 45`,删除
  quality_confirmed 字段;同步更新 web 前端提示与测试(28 个本地测试通过)。
- 效果:RTK fixed + 连续静止 5 秒后定位自动初始化,无需人工确认。
- 车头朝向(IMU 负 Y 轴朝机头、磁偏角 -0.0995)未丢失,已随清单持久保存。

## 问题 2:加载室外地图报"Indoor and outdoor maps must have distinct IDs"

**现象**:`POST /api/maps/load {"map_id":"outdoor-20261106-dff1ba32",...}`
返回 502 上述错误,定位停在 "Load an outdoor map to bind localization"。

**根因**:scene 服务 `web.py::_map_request` 里
`kind = body.get("map_type", "indoor")`,不带 `map_type` 时默认 indoor,
`check_identity` 发现同 ID 地图归 outdoor 所有即报错。

**临时处理**:请求带 `"map_type": "outdoor"` 后加载成功。
**待修复**:加载/保存时按地图归属自动判定类型,不应要求请求方带 map_type;
前端 outdoor 页面若未带该字段,同样会踩中。

## 问题 3:底盘反馈步态不是导航步态

**现象**:现场遥控站起后 `state=17`、`usage_mode=1` 均正确,但
`/outdoor/chassis_status` 报 `gait=4097`(基础平地步态 0x1001),
导航服务 health.chassis=false,提示"底盘导航模式与步态"。

**处理**:按 S10 指南 1.2.4 经 UDP 30004 明文帧
(Type=0x00100001、Command=0x00300002、GaitParam=12290、ActionParam=0)
切换平地导航步态,反馈确认 gait=12290 后 chassis 就绪。

**说明**:场景菜单栏就绪状态是实时的(底盘状态每秒发布、页面 5 Hz 轮询),
显示未就绪是因为步态确实不是导航步态,不是刷新问题。

## 新增工具

- `robot-deep_robotics-lynx_s10/outdoor/scripts/prepare_chassis.py`:
  读 BasicStatus;`--mode 1` 切使用模式、`--gait flat|stairs` 切步态、
  `--stand` 站起(1→17)、`--apply` 一键全部。
- 已知不足:UDP 回复偶尔慢,脚本 wait_state 读回偶发失败但命令已生效;
  已放宽 recv 超时(1 s)与总等待(5 s)。实际使用以
  `/outdoor/chassis_status` 话题反馈为准。

## 当天最终状态

- 定位:ready=true、grade=fixed、无故障(未做任何航向人工确认)。
- 导航:ready=true、not_ready 为空、pose 正常。
- 页面:`http://127.0.0.1:50110/outdoor`(本地转发 50110 → 山猫 50108)。

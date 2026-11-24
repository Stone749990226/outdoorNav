# 室外语义地点路线预览

用户在室外 Scene 加载一张保存好的地图，用“标记地点”在地图上点选并命名。保存后，直接在 `rbnx chat` 说「帮我在室外导航到北门访客入口」即可。地点名称和坐标不写在代码里，新增地点不需要重启 Scene 或 Pilot。

## 一条指令怎么变成路线

1. **Scene 保存标注。** 地点作为一个点（`poi`）按地图保存，有自己的 ID、名称和坐标；道路网仍由地图服务管理。
2. **Pilot 查地点。** 真实 DeepSeek 从用户的话中提取地点名，生成 RTDL（工具调用计划）。`list_outdoor_places` 每次读取当前室外地图的标注，按名称精确匹配，返回地图 ID、版本、地点 ID 和是否过期。当前接口按名称查找，不做模糊搜索，也不跨地图查找。
3. **地图服务算路线。** Pilot 用查到的地图 ID、版本和地点 ID 调用 `preview_outdoor_place`。Scene 重新核对地图和标注，用标注坐标作终点，再让地图服务沿道路网规划。地图切换、标注删除或过期时，不会拿旧数据继续规划；没有唯一有效匹配时，Pilot 应说明原因或请用户澄清。
4. **Scene 画线。** 网页从 `/api/outdoor/state` 读取路线，Pilot 回复距离。预览最多保留两分钟，换图或修改该标注会使旧路线失效。只规划清单不启动导航执行或底盘服务，也不需要山猫的里程计。

## 改了哪些模块

| 模块 | 负责什么 |
|---|---|
| [Robonix Scene 能力](../robonix/capabilities/system/scene/list_outdoor_places.v1.toml)、[MCP 接口](../robonix/system/scene/scene_service/mcp_tools.py) | 注册查地点与预览两个工具；让 Pilot 的调用和网页在 Scene 同一事件循环里处理。 |
| [Scene 室外逻辑](../robonix/system/scene/scene_service/outdoor.py)、[室外网页](../robonix/system/scene/scene_service/web_assets/outdoor.html) | 按当前地图查标注，校验地点和地图版本，显示预览路线。 |
| [地图服务](../service-map-outdoor-rbnx/outdoor_mapping/main.py) | 复用现有道路规划；只规划模式不接传感器，不允许录制、合并、保存或删除地图。空地图库仍会导入仓库自带示例。 |
| [Pilot 模型调用](../robonix/system/pilot/src/vlm.rs) | 由 Robonix 随清单启动，模型地址和名称可配置，密钥通过环境传入；模型生成 RTDL。 |
| [现有部署清单](../robot-deep_robotics-lynx_s10/outdoor/robonix_manifest.yaml)、[同步脚本](../tools/sync_robot.sh) | `OUTDOOR_MODE=preview` 启用预览和模拟起点；同步示例地图文件，保留山猫已有地图和标注。 |

## 演示数据和验证

- `example-campus` 是空地图库的备用地图，不是规划器指定的地图。已有地图时不会自动导入或切图。山猫上演示用的“快递点”是保存到这张地图里的标注，不是默认目的地。独立硬件模拟脚本里也有同名测试点，不参与真实 Pilot 链路。
- 只规划清单默认取**当前地图第一条有点的道路的首点**作为起点。模拟的只有起点，不是模型、终点或规划结果。
- 山猫上已用真实 DeepSeek 验证两种问法都能查找示例标注并预览约 77.2 米的路线。[回归测试](../robonix/system/scene/tests/test_outdoor.py) 还覆盖另一名称和坐标：先查不到，在网页接口新增后立即可查，预览终点随新标注变化。
- 现场操作见[部署文档](DEPLOYMENT.md#现场路线预览)。验收时可在同一张地图上另标任意名称的地点，再用新名称发起预览，不改代码也不重启服务。

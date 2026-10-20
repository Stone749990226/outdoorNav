# 室外地点导航

“走到公共路口”：先调用 `places(query="公共路口")`，使用唯一、已启用的地点 ID 调用
`navigate`，随后用返回的 `run_id` 查询 `navigate/status`。
`blocked_road_ids` 默认传空数组，`request_id` 可传空字符串。
需要预览时调用 `preview`；取消使用 `navigate/cancel` 并继续查询直到停车终态。

不要调用原单点导航绕过道路规划，不要生成底盘速度。定位失锁、传感器断流或道路不可通行时任务停止。

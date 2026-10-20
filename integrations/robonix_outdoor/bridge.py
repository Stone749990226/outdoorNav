"""MCP capability facade; HTTP and Pilot share the same navigation owner."""

import json
import os
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from robonix_api import Service, Ok, Deferred
from outdoor_mcp import (
    Places_Request,
    Places_Response,
    Preview_Request,
    Preview_Response,
    Navigate_Request,
    Navigate_Response,
    Status_Request,
    Status_Response,
    Cancel_Request,
    Cancel_Response,
)

service = Service(id="outdoor_navigation", namespace="robonix/service/outdoor")
base_url = os.environ.get("OUTDOORNAV_URL", "http://127.0.0.1:8765")


def request(path, payload=None):
    body = None if payload is None else json.dumps(payload).encode()
    req = Request(
        base_url + path, data=body, headers={"Content-Type": "application/json"}
    )
    try:
        with urlopen(req, timeout=5) as response:
            return json.load(response)
    except HTTPError as error:
        raise ValueError(json.load(error).get("error", str(error))) from error


def goal_payload(place_id, blocked):
    info = request("/api/map")
    return {
        "place_id": place_id,
        "blocked_road_ids": blocked,
        "frame_id": "map",
        "scene_version": info["scene_version"],
        "route_version": info["route_version"],
    }


@service.mcp("robonix/service/outdoor/places")
def places(req: Places_Request) -> Places_Response:
    """查询室外目的地。用户说“走到xx点”时，先查询地点并使用唯一 ID；enabled=false 的地点尚未现场确认，不可行驶。query 为空返回全部。"""
    result = request("/api/places")
    if req.query:
        result["places"] = [
            p
            for p in result["places"]
            if req.query in (p["id"], p["name"], *p.get("aliases", []))
        ]
    return Places_Response(places_json=json.dumps(result, ensure_ascii=False))


@service.mcp("robonix/service/outdoor/preview")
def preview(req: Preview_Request) -> Preview_Response:
    """预览当前位置到指定室外地点的道路路线，不移动机器人；blocked_road_ids 为空表示不额外封路。"""
    result = request("/api/plans", goal_payload(req.place_id, req.blocked_road_ids))
    return Preview_Response(route_json=json.dumps(result, ensure_ascii=False))


@service.mcp("robonix/service/outdoor/navigate")
def navigate(req: Navigate_Request) -> Navigate_Response:
    """沿路网前往已确认的室外地点，Nav2 局部避障，到达并停车。必须使用 places 返回的 ID，不生成速度指令。request_id 可为空；网络重试须复用同一32位十六进制ID。返回 run_id 后查询配套 status；新任务会取消旧任务并等待停车。"""
    payload = goal_payload(req.place_id, req.blocked_road_ids)
    if req.request_id:
        payload["request_id"] = req.request_id
    try:
        result = request("/api/tasks", payload)
        return Navigate_Response(
            accepted=True, run_id=result["task_id"], detail="accepted"
        )
    except ValueError as error:
        return Navigate_Response(accepted=False, run_id="", detail=str(error))


def task(run_id):
    if run_id:
        import re

        if not re.fullmatch(r"[a-f0-9]{32}", run_id):
            raise ValueError("Invalid run_id")
        return request("/api/tasks/" + run_id)
    return request("/api/status")["task"]


@service.mcp("robonix/service/outdoor/navigate/status")
def status(req: Status_Request) -> Status_Response:
    """查询室外导航任务。空 run_id 查询最近任务；SUCCEEDED 表示确认到达并停车。"""
    try:
        result = task(req.run_id)
    except ValueError as error:
        return Status_Response(known=False, state="UNKNOWN", detail=str(error))
    if result is None:
        return Status_Response(known=False, state="UNKNOWN", detail="No task")
    state = {"succeeded": "SUCCEEDED", "failed": "FAILED", "canceled": "CANCELED"}.get(
        result["state"], "RUNNING"
    )
    return Status_Response(
        known=True, state=state, detail=result.get("reason") or result["state"]
    )


@service.mcp("robonix/service/outdoor/navigate/cancel")
def cancel(req: Cancel_Request) -> Cancel_Response:
    """请求取消并停车。accepted 只表示收到请求，继续查询 status 直到任务终止。"""
    result = task(req.run_id)
    if result is None:
        return Cancel_Response(accepted=False, detail="No task")
    request("/api/tasks/" + result["task_id"] + "/cancel", {})
    return Cancel_Response(accepted=True, detail="Stopping; poll status")


@service.on_init
def init(config):
    try:
        if request("/api/status").get("mode") != "lynx":
            return Deferred("Waiting for the Lynx outdoor backend")
        request("/api/activation", {"active": False})
    except Exception:
        return Deferred("Outdoor backend is not reachable")
    return Ok()


@service.on_activate
def activate():
    request("/api/activation", {"active": True})
    return Ok()


@service.on_deactivate
def deactivate():
    request("/api/activation", {"active": False})
    return Ok()


@service.on_shutdown
def shutdown():
    try:
        request("/api/activation", {"active": False})
    except Exception:
        # The enclosing start script also stops the worker and watchdog processes.
        pass


if __name__ == "__main__":
    service.run()

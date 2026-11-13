"""Exercise the real Scene API and assert motion using independent simulator truth."""

import json
import math
import time
import uuid

from .launch import request, wait_for


def check(base, control, run):
    def navigation():
        return request(base + "/api/outdoor/state")["navigation"]

    def truth():
        return request(control + "/state")["world"]

    def execute(goal):
        state = request(base + "/api/outdoor/state")
        plan = request(base + "/api/outdoor/plan_route", {
            "map_id": state["binding"]["map_id"], "version": state["binding"]["version"], "goal": goal})
        result = request(base + "/api/outdoor/execute", {
            "preview_id": plan["preview_id"], "request_id": uuid.uuid4().hex})
        assert result.get("accepted"), result
        return result["run_id"], plan

    def finished(task_id):
        nav = navigation()
        task = nav.get("task") or {}
        record = {"time": time.time(), "task_id": task_id, "state": task.get("state"),
                  "reason": task.get("reason"), "remaining_m": task.get("distance_remaining_m"),
                  "ready": nav.get("ready"), "blocked": nav.get("not_ready_reasons"),
                  "fusion": nav.get("fusion"), "pose": nav.get("pose"), "truth": truth()}
        with (run / "smoke-observations.jsonl").open("a") as stream:
            stream.write(json.dumps(record) + "\n")
        if task.get("task_id") != task_id:
            raise AssertionError("Task identity changed")
        if task["state"] in ("failed", "canceled", "succeeded"):
            return task
        return None

    def stopped():
        w = truth()
        return abs(w["linear_mps"]) < 0.01 and abs(w["angular_rps"]) < 0.01

    checks = []
    print("smoke: navigating to courier point, including gait switches", flush=True)
    state = request(base + "/api/outdoor/state")
    place = next(p for p in state["places"] if p["name"] == "快递点" and p["enabled"])
    task_id, plan = execute({"x": place["x"], "y": place["y"]})
    task = wait_for(lambda: finished(task_id), timeout=120)
    assert task["state"] == "succeeded", task.get("reason")
    wait_for(stopped, timeout=5)
    w = truth()
    tolerance = task["arrival_tolerance_m"]
    error = math.hypot(w["x"] - plan["goal"]["x"], w["y"] - plan["goal"]["y"])
    assert error <= tolerance + 0.2, (w, error, tolerance)
    assert w["distance_m"] > 1 and w["command_count"] > 0, w
    if any(i["gait"] == "stairs" for i in plan.get("intervals", [])):
        assert 0x3003 in w["gait_history"] and w["gait_history"][-1] == 0x3002, w
    checks.append({"check": "arrival_and_gait_feedback", "truth_error_m": error,
                   "distance_m": w["distance_m"], "gait_history": w["gait_history"]})

    start = request(base + "/api/outdoor/state")["map"]["network"]["roads"][0]["points"][0]
    wait_for(lambda: navigation().get("ready"))
    print("smoke: canceling a moving robot", flush=True)
    task_id, _ = execute({"x": start["x"], "y": start["y"]})
    wait_for(lambda: abs(truth()["linear_mps"]) > 0.03, timeout=20)
    request(base + "/api/outdoor/cancel", {"run_id": task_id})
    task = wait_for(lambda: finished(task_id), timeout=20)
    assert task["state"] == "canceled", task.get("reason")
    wait_for(stopped, timeout=5)
    checks.append({"check": "cancel_stops_actual_motion"})

    print("smoke: unplugging IMU while moving", flush=True)
    wait_for(lambda: navigation().get("ready"))
    task_id, _ = execute({"x": start["x"], "y": start["y"]})
    wait_for(lambda: abs(truth()["linear_mps"]) > 0.03, timeout=20)
    at = time.monotonic()
    request(control + "/faults", {"imu_connected": False})
    try:
        wait_for(stopped, timeout=4)
        assert not navigation()["ready"]
        checks.append({"check": "imu_dropout_stops_actual_motion", "stop_delay_s": time.monotonic() - at})
        request(base + "/api/outdoor/cancel", {"run_id": task_id})
    finally:
        request(control + "/faults", {"imu_connected": True})
    wait_for(lambda: finished(task_id), timeout=25)
    wait_for(lambda: navigation().get("ready"), timeout=30)
    wait_for(stopped, timeout=5)
    checks.append({"check": "reconnection_does_not_restart_task"})
    result = {"passed": True, "simulated": True, "robot_tested": False, "checks": checks}
    (run / "smoke-result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False), flush=True)

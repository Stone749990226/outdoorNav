import json
import struct
import tempfile
from pathlib import Path

import pytest
from types import SimpleNamespace
from urllib.request import Request, urlopen

from outdoornav.bundle import load_bundle
from outdoornav.gnss import FixDecoder, usable_fix
from outdoornav.localization import GeoAlignment
from outdoornav.places import Places
from outdoornav.tasks import TaskStore
from outdoornav.outdoor import OutdoorService
from outdoornav.api import ApiError, serve


def nmea(body):
    checksum = 0
    for b in body.encode():
        checksum ^= b
    return f"${body}*{checksum:02X}\r\n".encode()


def pvt(carrier=2, tow=100):
    p = bytearray(92)
    struct.pack_into("<I", p, 0, tow)
    p[20], p[21], p[23] = 3, 3 | (carrier << 6), 20
    struct.pack_into("<iiiiII", p, 24, 1200000000, 300000000, 123456, 100000, 100, 150)
    data = bytes([1, 7, 92, 0]) + p
    a = b = 0
    for value in data:
        a = (a + value) & 255
        b = (b + a) & 255
    return b"\xb5\x62" + data + bytes([a, b])


def fix(tow):
    return {
        "valid": True,
        "fixed": True,
        "differential": True,
        "gga_quality": 4,
        "hacc_m": 0.1,
        "correction_age_s": 1.0,
        "latitude": 30.0,
        "longitude": 120.0,
        "tow_ms": tow,
    }


def test_receiver_fixed_units_and_missing_age():
    decoder = FixDecoder()
    decoder.feed(
        nmea("GNGGA,123456,3000.0,N,12000.0,E,4,20,1.0,100,M,23.456,M,1.2,0000"), 0
    )
    result = decoder.feed(pvt(), 0.1)
    assert usable_fix(result)
    assert result["hacc_m"] == 0.1
    assert result["altitude_m"] == 123.456
    assert result["msl_m"] == 100
    assert not usable_fix(decoder.feed(pvt(carrier=0), 0.2))
    assert not usable_fix(decoder.feed(pvt(), 2))
    decoder.feed(
        nmea("GNGGA,123456,3000.0,N,12000.0,E,4,20,1.0,100,M,23.456,M,,0000"), 3
    )
    assert not usable_fix(decoder.feed(pvt(), 3.1))


def test_alignment_antenna_stability_loss_and_reset():
    align = GeoAlignment(
        {"latitude": 30.0, "longitude": 120.0},
        {
            "calibrated": True,
            "initial_heading_rad": 0.0,
            "antenna_xy_m": [1.0, 0.0],
            "map_offset_xy_m": [0.0, 0.0],
        },
    )
    for i in range(61):
        t = i / 10
        assert align.odom(10, 20, 0, 100 + t, t)
        align.update_fix(fix(i), t)
    assert align.ready(6)
    assert align.pose()["x"] == pytest.approx(-1)
    assert align.pose()["y"] == pytest.approx(0)
    assert not align.ready(7.1)
    assert not align.odom(0, 0, 0, 106.1, 6.1)
    assert not align.ready(6.1)


def test_repeated_epoch_cannot_keep_localization_fresh():
    align = GeoAlignment(
        {"latitude": 30.0, "longitude": 120.0},
        {
            "calibrated": True,
            "initial_heading_rad": 0.0,
            "antenna_xy_m": [0.0, 0.0],
            "map_offset_xy_m": [0.0, 0.0],
        },
    )
    for i in range(70):
        t = i / 10
        align.odom(0, 0, 0, t + 1, t)
        align.update_fix(fix(1), t)
    assert not align.ready(6.9)


def test_places_bind_existing_nodes_and_require_confirmation():
    bundle = load_bundle()
    places = Places("scenes/north_campus/places.json", bundle)
    assert places.resolve("公共路口")["id"] == "J"
    assert places.resolve("取餐点")["id"] == "P"
    assert not any(p["enabled"] for p in places.items)
    for p in places.items:
        node = next(n for n in bundle["network"]["nodes"] if n["id"] == p["id"])
        assert (p["x"], p["y"]) == (node["x"], node["y"])
    with pytest.raises(ValueError):
        Places(
            "scenes/north_campus/places.json", {**bundle, "route_version": "another"}
        )


def test_restart_records_failure_without_resuming_and_preserves_terminal():
    with tempfile.TemporaryDirectory() as directory:
        journal = Path(directory) / "tasks.json"
        store = TaskStore(journal=journal)
        done = store.submit({"x": 1.0, "y": 2.0})
        store.update(done["task_id"], state="succeeded")
        active = store.submit({"x": 3.0, "y": 4.0})
        recovered = TaskStore(journal=journal)
        assert recovered.snapshot(done["task_id"])["state"] == "succeeded"
        assert recovered.snapshot(active["task_id"])["state"] == "failed"
        assert "not resumed" in recovered.snapshot()["reason"]


def test_shared_service_rejects_unconfirmed_places_and_deduplicates_after_fix_loss():
    bundle = load_bundle()
    places = Places("scenes/north_campus/places.json", bundle)
    tasks = TaskStore(bundle["scene_version"], bundle["route_version"])
    state = {"ready": True, "pose": {**bundle["network"]["nodes"][0], "age_s": 0.1}}
    service = OutdoorService(bundle, places, tasks, lambda *_: state)
    payload = {
        "frame_id": "map",
        "scene_version": bundle["scene_version"],
        "route_version": bundle["route_version"],
        "place_id": "J",
        "blocked_road_ids": [],
        "request_id": "a" * 32,
    }
    assert service.preview(payload)["goal"] == {
        k: places.resolve("J")[k] for k in ("x", "y")
    }
    with pytest.raises(ApiError, match="confirmation"):
        service.submit(payload)
    places.items[1]["enabled"] = True
    first = service.submit(payload)
    state.update(ready=False, pose=None)
    assert service.submit(payload)["task_id"] == first["task_id"]
    assert len(tasks.tasks) == 1
    with pytest.raises(ApiError, match="not ready"):
        service.submit({**payload, "request_id": "b" * 32})
    with pytest.raises(ApiError, match="another goal"):
        service.submit({**payload, "place_id": "P"})
    tasks.cancel(first["task_id"])
    assert service.submit(payload)["state"] == "canceling"
    state.update(ready=True, pose={**bundle["network"]["nodes"][0], "age_s": 0.1})
    replacement = service.submit({**payload, "request_id": "b" * 32})
    assert replacement["state"] == "accepted"
    assert tasks.snapshot(first["task_id"])["cancel_requested"]
    with pytest.raises(ApiError, match="version"):
        service.submit({**payload, "route_version": "old"})


def test_http_places_submit_and_cancel_share_the_same_task():
    bundle = load_bundle()
    places = Places("scenes/north_campus/places.json", bundle)
    places.items[1]["enabled"] = True
    tasks = TaskStore(bundle["scene_version"], bundle["route_version"])
    status = lambda *_: {
        "ready": True,
        "pose": {**bundle["network"]["nodes"][0], "age_s": 0.1},
    }
    service = OutdoorService(bundle, places, tasks, status)
    backend = SimpleNamespace(
        places=places,
        tasks=tasks,
        obstacles=None,
        status=status,
        bundle=bundle,
        map_snapshot=lambda: bundle,
        submit=service.submit,
        preview=service.preview,
    )
    server = serve(backend, port=0)

    def request(path, body=None):
        req = Request(
            f"http://127.0.0.1:{server.server_port}" + path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Content-Type": "application/json"},
        )
        with urlopen(req, timeout=2) as response:
            return json.load(response)

    try:
        assert request("/api/places")["places"][1]["id"] == "J"
        result = request(
            "/api/tasks",
            {
                "frame_id": "map",
                "place_id": "J",
                "scene_version": bundle["scene_version"],
                "route_version": bundle["route_version"],
            },
        )
        path = "/api/tasks/" + result["task_id"]
        assert request(path)["goal"]["place_id"] == "J"
        assert request(path + "/cancel", {})["state"] == "canceling"
        assert tasks.snapshot()["cancel_requested"]
    finally:
        server.shutdown()
        server.server_close()

#!/usr/bin/env python3
"""Drive the outdoor stack to a named Scene place (POI) through the Scene HTTP API.

Scene marks places as `poi` annotations; `/api/outdoor/state` exposes them as
`places` with map coordinates once their map is the loaded one. This script is
the stand-in for a person tapping the frontend: preview the route, execute it,
then poll the task until it reaches a terminal state.
"""

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
import uuid


def call(base, path, body=None, timeout=30.0):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(base + path, data=data,
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        return json.load(error)


def state(base):
    return call(base, "/api/outdoor/state")


def find_place(current, name):
    matches = [p for p in current.get("places") or [] if p.get("name") == name]
    if len(matches) > 1:
        raise SystemExit(f"Multiple places named {name!r}; rename them in Scene first")
    if matches:
        place = matches[0]
        if not place.get("enabled"):
            raise SystemExit(f"Place {name!r} is marked stale; confirm it in Scene first")
        return place
    names = ", ".join(sorted(p.get("name", "?") for p in current.get("places") or [])) or "none"
    raise SystemExit(f"No place named {name!r}; Scene has: {names}")


def require_ready(current):
    navigation = current.get("navigation") or {}
    if not current.get("is_active"):
        raise SystemExit("Scene has no active outdoor map; load the map first")
    if not navigation.get("available") or not navigation.get("ready"):
        raise SystemExit("Navigation is not ready: " + str(navigation.get("not_ready_reasons")))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="http://127.0.0.1:50108")
    parser.add_argument("--list", action="store_true", help="print Scene places and exit")
    parser.add_argument("--status", action="store_true", help="print the navigation task and exit")
    parser.add_argument("--cancel", metavar="RUN_ID", help="cancel a running task and exit")
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("place", nargs="?", help="place name to drive to")
    arguments = parser.parse_args(argv)

    if arguments.list:
        current = state(arguments.base)
        for place in current.get("places") or []:
            print(f"{place['name']}\t{place['x']:.2f}\t{place['y']:.2f}\tenabled={place['enabled']}")
        return 0
    if arguments.status:
        task = (state(arguments.base).get("navigation") or {}).get("task") or {}
        print(json.dumps(task, ensure_ascii=False, indent=2))
        return 0
    if arguments.cancel:
        print(json.dumps(call(arguments.base, "/api/outdoor/cancel", {"run_id": arguments.cancel}),
                         ensure_ascii=False))
        return 0
    if not arguments.place:
        parser.error("give a place name, or use --list/--status/--cancel")

    current = state(arguments.base)
    require_ready(current)
    place = find_place(current, arguments.place)
    binding = current["binding"]
    goal = {"x": place["x"], "y": place["y"]}
    planned = call(arguments.base, "/api/outdoor/plan_route",
                   {"map_id": binding["map_id"], "version": binding["version"], "goal": goal})
    if not planned.get("ok"):
        raise SystemExit("Preview failed: " + str(planned.get("detail")))
    accepted = call(arguments.base, "/api/outdoor/execute",
                    {"preview_id": planned["preview_id"], "request_id": uuid.uuid4().hex})
    if not accepted.get("accepted"):
        raise SystemExit("Execute refused: " + str(accepted))
    task_id = accepted["run_id"]
    print(f"accepted run {task_id} -> {arguments.place} ({goal['x']:.2f}, {goal['y']:.2f})")
    deadline = time.monotonic() + arguments.timeout
    last = None
    while time.monotonic() < deadline:
        task = (state(arguments.base).get("navigation") or {}).get("task") or {}
        if task.get("task_id") != task_id:
            raise SystemExit("Task was superseded; this monitor will not report another task as its result")
        remaining = task.get("distance_remaining_m")
        line = (task.get("state"), round(-1.0 if remaining is None else remaining, 2))
        if line != last:
            print(f"{line[0]}\tremaining {line[1]:.2f} m", flush=True)
            last = line
        if task.get("state") in ("succeeded", "failed", "canceled"):
            print(json.dumps({k: task.get(k) for k in ("state", "reason", "distance_remaining_m")},
                             ensure_ascii=False))
            return 0 if task["state"] == "succeeded" else 1
        time.sleep(2.0)
    print("still running after timeout; use --status or --cancel", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())

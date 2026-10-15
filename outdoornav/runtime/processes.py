"""Process identity and run ownership; detached Gazebo children remain identifiable."""

import os
from pathlib import Path
import signal
import time


def birth(pid):
    try:
        # comm may contain spaces and parentheses. Fields after its last ')' start at 3.
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return None if fields[0] == "Z" else fields[19]
    except (OSError, IndexError):
        return None


def identity(pid):
    return {"pid": pid, "birth": birth(pid)}


def matches(record):
    if (
        not isinstance(record, dict)
        or not isinstance(record.get("pid"), int)
        or record["pid"] <= 0
    ):
        return False
    return bool(record.get("birth")) and birth(record["pid"]) == record["birth"]


def owned(run_id):
    token = ("OUTDOORNAV_RUN_ID=" + run_id).encode()
    result = []
    for path in Path("/proc").iterdir():
        if not path.name.isdigit() or int(path.name) == os.getpid():
            continue
        try:
            if path.stat().st_uid == os.getuid() and token in (
                path / "environ"
            ).read_bytes().split(b"\0"):
                result.append(identity(int(path.name)))
        except OSError:
            continue
    return result


def stop_owned(run_id, grace=15):
    targets = owned(run_id)
    pids = {record["pid"] for record in targets}
    for record in targets:
        if matches(record):
            try:
                # Launch supervisors forward SIGINT. Signaling their children as well
                # would interrupt cleanup a second time and corrupt final evidence.
                fields = (
                    Path(f"/proc/{record['pid']}/stat")
                    .read_text()
                    .rsplit(")", 1)[1]
                    .split()
                )
                if int(fields[1]) in pids:
                    continue
                os.kill(record["pid"], signal.SIGINT)
            except (ProcessLookupError, FileNotFoundError):
                pass
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline and any(matches(p) for p in targets):
        time.sleep(0.1)
    # Rescan for children created during graceful shutdown, always with the run token.
    for record in owned(run_id):
        if matches(record):
            try:
                os.kill(record["pid"], signal.SIGKILL)
            except ProcessLookupError:
                pass

"""Atomic runtime records and supervisor identity."""

import json
from outdoornav.bundle import ROOT
from .processes import matches

RUNS = ROOT / "artifacts/runs"
ACTIVE = RUNS / "active.json"


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    tmp.replace(path)


def active():
    if not ACTIVE.is_file():
        return None
    try:
        state = json.loads(ACTIVE.read_text())
        return state if matches(state["supervisor"]) else None
    except (ValueError, KeyError, TypeError):
        return None

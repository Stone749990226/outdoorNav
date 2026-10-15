"""Acceptance scenarios shared by the CLI and validation runners."""

NAVIGATION_SCENARIOS = (
    "short",
    "cancel",
    "replace",
    "turnaround",
    "detour",
    "injected-static",
    "moving",
    "blocked",
    "ring",
    "branch",
)
FAULT_SCENARIOS = ("version-mismatch", "pause", "rewind", "sensor-loss")

#!/usr/bin/env bash
# Run the camera web service (default 127.0.0.1:8080; tunnel it, do not expose).
set -eo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$root/server.py" --config "${CAMERA_CONFIG:-$root/config.json}"

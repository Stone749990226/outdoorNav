#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
if [[ ! -x "$root/.venv/bin/python" ]]; then
  uv venv --python /usr/bin/python3 --system-site-packages "$root/.venv"
fi
uv pip install --python "$root/.venv/bin/python" -e "$root" \
  -e "$(rbnx path robonix-api)[codegen]" 'numpy<2'
rbnx codegen -p "$root/integrations/robonix_outdoor" --mcp --python "$root/.venv/bin/python"
# No colcon: this consumer does not build or start another ROS navigation stack.

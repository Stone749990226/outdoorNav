#!/usr/bin/env bash
set -eo pipefail
campus_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(dirname "$campus_dir")"
if [[ -x "$repo_dir/.venv/bin/python" ]]; then
  exec uv run --python "$repo_dir/.venv/bin/python" python "$campus_dir/scripts/build_scene.py" "$@"
elif [[ -x "$repo_dir/venv/bin/python" ]]; then
  exec uv run --python "$repo_dir/venv/bin/python" python "$campus_dir/scripts/build_scene.py" "$@"
else
  exec conda run --no-capture-output -n torch python "$campus_dir/scripts/build_scene.py" "$@"
fi

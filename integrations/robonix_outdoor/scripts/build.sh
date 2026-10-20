#!/usr/bin/env bash
set -euo pipefail
task_pkg="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
bash "$task_pkg/../../deploy/lynx/build_existing.sh"

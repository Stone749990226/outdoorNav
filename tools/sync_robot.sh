#!/usr/bin/env bash
# Copy the workspace sources; build and lifecycle remain native rbnx commands.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REMOTE="nvidia@svr.gddg08.top"
WORKSPACE="/home/nvidia/outdoorNav-workspace"
DEPLOY="/home/nvidia/robot-deep_robotics-lynx_s10/outdoor"
FLAGS=(-az --delete --exclude=.git --exclude=.venv --exclude=__pycache__
  --exclude='*.pyc' --exclude='*.egg-info' --exclude=.pytest_cache
  --exclude=rbnx-build --exclude=rbnx-boot --exclude=target --exclude=node_modules
  --exclude=build --exclude=dist --exclude=run --exclude=logs --exclude=data
  --exclude=backups --exclude=.robonix)
if [[ "${1:-}" == "--dry-run" ]]; then
  FLAGS+=(--dry-run --itemize-changes)
elif [[ $# != 0 ]]; then
  echo "Usage: bash tools/sync_robot.sh [--dry-run]" >&2
  exit 2
fi
export RSYNC_RSH='ssh -p 6003 -o BatchMode=yes -o ConnectTimeout=8'
if [[ "${1:-}" != "--dry-run" ]]; then
  ssh -p 6003 -o BatchMode=yes -o ConnectTimeout=8 "$REMOTE" \
    "flock -n '$DEPLOY/run/boot.lock' true" || {
      echo "Stop the deployment with rbnx shutdown before syncing." >&2
      exit 1
    }
fi
# Keep root-level tools/docs and every independent repository, including uncommitted files.
rsync "${FLAGS[@]}" --exclude=secrets.json --exclude=gnss.base.yaml \
  "$ROOT/" "$REMOTE:$WORKSPACE/"
rsync "${FLAGS[@]}" --exclude=packages --exclude=framework \
  --exclude=secrets.json --exclude=gnss.base.yaml \
  "$ROOT/robot-deep_robotics-lynx_s10/outdoor/" "$REMOTE:$DEPLOY/"
for package in primitive-g70-gnss-rbnx primitive-wit-imu-rbnx \
  service-localization-outdoor-rbnx service-map-outdoor-rbnx service-navigation-outdoor-rbnx; do
  rsync "${FLAGS[@]}" "$ROOT/$package/" "$REMOTE:$DEPLOY/packages/$package/"
done
rsync "${FLAGS[@]}" "$ROOT/robonix/" "$REMOTE:$DEPLOY/framework/robonix/"
for primitive in primitive-lynx_chassis primitive-lynx_lidar primitive-lynx_camera; do
  rsync "${FLAGS[@]}" "$ROOT/robot-deep_robotics-lynx_s10/primitives/$primitive/" \
    "$REMOTE:/home/nvidia/robot-deep_robotics-lynx_s10/primitives/$primitive/"
done

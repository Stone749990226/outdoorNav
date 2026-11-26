#!/usr/bin/env bash
# Copy the workspace into the single robot tree; build and lifecycle stay native.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REMOTE="nvidia@svr.gddg08.top"
WORKSPACE="/home/nvidia/outdoorNav-workspace"
DEPLOY="$WORKSPACE/robot-deep_robotics-lynx_s10/outdoor"
FLAGS=(-az --delete --exclude=.git --exclude=.venv --exclude=__pycache__
  --exclude='*.pyc' --exclude='*.egg-info' --exclude=.pytest_cache
  --exclude=rbnx-build --exclude=rbnx-boot --exclude=target --exclude=node_modules
  --exclude=build --exclude=dist --exclude=run --exclude=logs --exclude=data
  --exclude=backups --exclude=.robonix
  # Logs and recordings written on the robot must never be deleted by a sync.
  --exclude='*.log' --exclude='*.jsonl' --exclude=recordings
  # Robot-side build output and runtime assets live inside the deployment and
  # are produced by rbnx build, not by this mirror.
  --exclude=robot-deep_robotics-lynx_s10/outdoor/packages
  --exclude=robot-deep_robotics-lynx_s10/outdoor/framework
  --exclude=robot-deep_robotics-lynx_s10/outdoor/bin
  --exclude=robot-deep_robotics-lynx_s10/outdoor/models
  # Present on the robot only; cloning it here would delete it on sync.
  --exclude=primitive-orbbec-camera
  # Site files created on the robot: RTK base config, verified ground reference,
  # the Pilot soul and dated backups survive every sync.
  --exclude=ground.local.json
  --exclude=preview-soul.md
  --exclude='*.bak-*')
DRY_RUN=()
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=(--dry-run --itemize-changes)
  FLAGS+=("${DRY_RUN[@]}")
elif [[ $# != 0 ]]; then
  echo "Usage: bash tools/sync_robot.sh [--dry-run]" >&2
  exit 2
fi
export RSYNC_RSH='ssh -p 6003 -o BatchMode=yes -o ConnectTimeout=8'
EXAMPLE_MAP=data/examples/example-campus/map.json
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
# Deployment packages and framework are assembled from their own sources.
for package in primitive-g70-gnss-rbnx primitive-wit-imu-rbnx \
  service-localization-outdoor-rbnx service-map-outdoor-rbnx service-navigation-outdoor-rbnx; do
  rsync "${FLAGS[@]}" --mkpath "$ROOT/$package/" "$REMOTE:$DEPLOY/packages/$package/"
done
rsync -az --relative "${DRY_RUN[@]}" \
  "$ROOT/./service-map-outdoor-rbnx/$EXAMPLE_MAP" "$REMOTE:$WORKSPACE/"
rsync -az --relative --mkpath "${DRY_RUN[@]}" \
  "$ROOT/service-map-outdoor-rbnx/./$EXAMPLE_MAP" "$REMOTE:$DEPLOY/packages/service-map-outdoor-rbnx/"
rsync "${FLAGS[@]}" --mkpath "$ROOT/robonix/" "$REMOTE:$DEPLOY/framework/robonix/"

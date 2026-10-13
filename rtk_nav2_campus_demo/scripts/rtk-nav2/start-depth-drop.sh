#!/usr/bin/env bash
# Stage 6 only. The stage-5 world, nav2_params.yaml and demo.rviz remain defaults.
# Run from the Ubuntu host: scripts/rtk-nav2/start-depth-drop.sh <run-id> [-2.3|-1.7]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${1:?usage: start-depth-drop.sh <run-id> [-2.3|-1.7]}"
X="${2:--2.3}"
case "$X" in -2.3|-2.0|-1.7) ;; *) printf 'Observation x must be -2.3, -2.0 or -1.7\n' >&2; exit 2;; esac
OWNER="$ROOT/artifacts/rtk-nav2/.stack-owner"
if [ -e "$OWNER" ]; then
    read -r owner phase _ < "$OWNER"
    if [ "$owner" != stage6-developer ] && [ "$owner" != stage7-developer ]; then
        printf 'Shared container is owned: %s\n' "$(< "$OWNER")" >&2
        exit 1
    fi
    if [ "$phase" != "$RUN_ID" ]; then
        printf 'Shared container is owned: %s\n' "$(< "$OWNER")" >&2
        exit 1
    fi
fi
# No bag is needed for the first visual tuning pass. Explicitly opt in later to record.
# Existing container may have been created for somebody else's display (:10).
# The volume-mounted X authority belongs to this account; explicitly select their :11.
# The container's DISPLAY was fixed when it was created; update that single env var
# for this launch without changing the other user's X socket or authorization file.
if /usr/bin/docker inspect -f '{{.State.Running}}' robonix-rtk-nav2-demo 2>/dev/null | grep -qx true; then
    /usr/bin/docker exec robonix-rtk-nav2-demo bash -lc 'DISPLAY=:11.0 XAUTHORITY=/xauth timeout 5 xdpyinfo >/dev/null 2>&1' || {
        printf 'Display :11 not accessible inside container\n' >&2; exit 1;
    }
fi
DEMO_DISPLAY="${DEMO_DISPLAY:-:11}" ROBONIX_DEPTH_DROP_DISPLAY="${DEMO_DISPLAY:-:11}.0" \
    SKIP_BAG="${SKIP_BAG:-1}" "$ROOT/scripts/rtk-nav2/start-demo.sh" "$RUN_ID" --gui -- \
    "world:=${DROP_WORLD:-/work/rtk_nav2_ws/src/robonix_rtk_nav2/worlds/depth_drop.sdf}" \
    "nav2_config:=/work/rtk_nav2_ws/src/robonix_rtk_nav2/config/depth_drop_nav2.yaml" \
    "rviz_config:=/work/rtk_nav2_ws/src/robonix_rtk_nav2/config/depth_drop.rviz" \
    "depth_drop:=true" "drop_camera_height:=${DROP_CAMERA_HEIGHT:-0.55}" "camera_tf_height:=${CAMERA_TF_HEIGHT:-0.55}" \
    "drop_evidence_dir:=${DROP_EVIDENCE_DIR:-/work/artifacts/rtk-nav2/$RUN_ID/regime_in_view}" \
    "hazard_margin:=${HAZARD_MARGIN:-0.15}" \
    "model_file:=/work/rtk_nav2_ws/src/robonix_rtk_nav2/models/depth_drop_robot.urdf.xacro" \
    "x_pose:=$X" "y_pose:=0.0" "z_pose:=0.01" \
    "gz_gui:=true" "rviz:=true"

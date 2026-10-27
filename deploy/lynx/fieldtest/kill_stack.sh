#!/usr/bin/env bash
# Stop only this directory's navigation processes. Patterns use bracket form so
# the script's own command line never matches. Keeps lidar driver and static TF.
set -u
self=$$
mapfile -t pids < <(ps -eo pid=,args= | awk -v me="$self" '
  $1==me {next}
  /[o]utdoornav_ros\/lynx_(localizer|navigation)/ {print $1; next}
  /[w]it_stream\.py/ {print $1; next}
  /[g]nss_stream\.py/ {print $1; next}
  /[r]os2 launch outdoornav_ros/ {print $1; next}
  /[c]md_vel_bridge\.py/ {print $1; next}
  /[m]ock_virtual_base\.py/ {print $1; next}
  /[r]un\/mock\.launch\.py/ {print $1; next}')
echo "targets: ${pids[*]:-none}"
for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null; done
sleep 4
for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
sleep 1
ps -eo pid=,args= | awk '/[o]utdoornav_ros\/lynx_|[w]it_stream\.py|[g]nss_stream\.py/ {print "LEFT",$0}'

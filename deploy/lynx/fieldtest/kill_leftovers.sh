#!/usr/bin/env bash
set -u
self=$$
mapfile -t pids < <(ps -eo pid=,args= | awk -v me="$self" '
  $1==me {next}
  /[p]ointcloud_to_laserscan_node/ {print $1; next}
  /[n]av2_controller\/controller_server/ {print $1; next}
  /[n]av2_planner\/planner_server/ {print $1; next}
  /[n]av2_velocity_smoother/ {print $1; next}
  /[n]av2_lifecycle_manager/ {print $1; next}
  /[l]ynx_watchdog / {print $1; next}
  /[r]un\/mock\.launch\.py/ {print $1; next}')
echo "leftover targets: ${pids[*]:-none}"
for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null; done
sleep 3
for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null; done
ps -eo pid=,args= | awk '/[p]ointcloud_to_laserscan|[c]ontroller_server|[p]lanner_server|[v]elocity_smoother|[l]ifecycle_manager|[l]ynx_watchdog/ {print "LEFT",$0}'

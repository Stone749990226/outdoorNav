#!/usr/bin/env bash
set -u
self=$$
mapfile -t pids < <(ps -eo pid=,args= | awk -v me="$self" '$1==me{next} /[r]slidar_sdk_node|[s]tatic_transform_publisher/ {print $1}')
echo "lidar targets: ${pids[*]:-none}"
for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null; done
sleep 2
for pid in "${pids[@]}"; do kill -KILL "$pid" 2>/dev/null; done

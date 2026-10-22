# Shared child process groups for the navigation stack and its MCP wrapper.
task_children=()
task_stop_grace_s=${task_stop_grace_s:-20}
start_child() {
  setsid "$@" &
  task_children+=("$!")
}
stop_children() {
  trap - EXIT INT TERM
  local task_pid task_until=$((SECONDS + task_stop_grace_s)) task_alive
  for task_pid in "${task_children[@]}"; do
    kill -TERM -- "-$task_pid" 2>/dev/null || true
  done
  while (( SECONDS < task_until )); do
    task_alive=false
    for task_pid in "${task_children[@]}"; do
      if kill -0 -- "-$task_pid" 2>/dev/null; then task_alive=true; fi
    done
    [[ "$task_alive" == true ]] || break
    sleep 0.1
  done
  for task_pid in "${task_children[@]}"; do
    kill -KILL -- "-$task_pid" 2>/dev/null || true
  done
  wait || true
}
trap stop_children EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

#!/usr/bin/env bash
# Stops avoid_sim / avoid_real: Ctrl-C to every pane (roslaunch shuts its nodes down cleanly),
# then the tmux session, then whatever the session started that is still alive.
#
#   avoid_stop.sh [--quiet] [session ...]     (default: avoid_sim avoid_real)
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

QUIET=0 SESSIONS=()
for a in "$@"; do
  case "$a" in
    --quiet) QUIET=1 ;;
    -h|--help) sed -n '2,5p' "$0"; exit 0 ;;
    *) SESSIONS+=("$a") ;;
  esac
done
[ ${#SESSIONS[@]} -gt 0 ] || SESSIONS=(avoid_sim avoid_real)

# Run from a pane of the session being stopped: killing the session would kill this script too,
# so finish the job detached.
if [ -n "$AVOID_SESSION" ] && [ -z "$AVOID_STOP_DETACHED" ]; then
  log="${XDG_RUNTIME_DIR:-/tmp}/avoid_bringup/stop.log"
  mkdir -p "$(dirname "$log")"
  echo "[avoid] stopping in the background (log: $log)"
  env -u AVOID_SESSION AVOID_STOP_DETACHED=1 setsid "$0" "$@" > "$log" 2>&1 < /dev/null &
  exit 0
fi

alive() { [ -n "$(session_pids "$1")" ]; }

for s in "${SESSIONS[@]}"; do
  if ! tmux has-session -t "$s" 2>/dev/null && ! alive "$s"; then
    [ $QUIET = 1 ] || info "$s is not running"
    continue
  fi
  info "stopping $s"
  if tmux has-session -t "$s" 2>/dev/null; then
    # the core window last: roscore must outlive the nodes that unregister from it
    for w in nav monitor web shell core; do
      for p in $(tmux list-panes -t "$s:$w" -F '#{pane_id}' 2>/dev/null | tac); do
        tmux send-keys -t "$p" C-c
      done
      [ "$w" = core ] || sleep 0.3
    done
    for _ in $(seq 1 12); do alive "$s" || break; sleep 0.5; done   # gzserver can take 15 s+
    tmux kill-session -t "$s" 2>/dev/null
  fi
  pids="$(session_pids "$s")"
  if [ -n "$pids" ]; then
    warn "still running after Ctrl-C, terminating: $(ps -o comm= -p $(echo $pids | tr ' ' ',') | sort -u | tr '\n' ' ')"
    kill -TERM $pids 2>/dev/null
    for _ in $(seq 1 10); do alive "$s" || break; sleep 0.5; done
    pids="$(session_pids "$s")"
    [ -z "$pids" ] || kill -KILL $pids 2>/dev/null
  fi
  info "$s stopped"
done

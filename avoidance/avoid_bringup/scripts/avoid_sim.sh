#!/usr/bin/env bash
# Simulation in one command: tmux session "avoid_sim" with Gazebo (avoid_sim), nav_node, rosbridge,
# the web UI and a status monitor. Pure Gazebo: no robot drivers.
#
#   avoid_sim.sh [camera|lidar] [--no-gui] [--seed N] [--layout random|route|classic] [--no-attach]
set -e
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SENSOR=camera GUI=true SEED=1 LAYOUT=random NO_ATTACH=${NO_ATTACH:-0}
while [ $# -gt 0 ]; do
  case "$1" in
    camera|lidar) SENSOR="$1" ;;
    --no-gui) GUI=false ;;
    --seed) SEED="$2"; shift ;;
    --layout) LAYOUT="$2"; shift ;;
    --no-attach) NO_ATTACH=1 ;;
    -h|--help) sed -n '2,6p' "$0"; exit 0 ;;
    *) fail "unknown argument '$1' (see --help)" ;;
  esac
  shift
done

command -v tmux >/dev/null || fail "tmux is not installed (sudo apt install tmux)"
SESSION="$(instance avoid_sim)"
[ -z "$AVOID_SESSION" ] || fail "this terminal belongs to the running '$AVOID_SESSION' session: run this from another terminal"
"$BRINGUP_DIR/scripts/avoid_stop.sh" --quiet "$SESSION" "$(instance avoid_real)"
make_env "$SESSION"
check_pkgs avoid_bringup avoid_sim avoid_nav gazebo_ros rosbridge_server
if [ "$SENSOR" = camera ]; then
  check_pkgs mjpeg_server            # the virtual camera's images in the web UI
  check_ports "$ROS_PORT" "$BRIDGE_PORT" "$WEB_PORT" "$GAZEBO_PORT" "$MJPEG_PORT"
else
  check_ports "$ROS_PORT" "$BRIDGE_PORT" "$WEB_PORT" "$GAZEBO_PORT"
fi
if [ "$GUI" = true ] && [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
  warn "no display: starting Gazebo without its window (--no-gui)"; GUI=false
fi

info "simulation, $SENSOR configuration, layout $LAYOUT, seed $SEED, gui $GUI"
start_session "$SESSION" \
  "roslaunch avoid_bringup sim.launch nav:=false sensor:=$SENSOR gui:=$GUI layout:=$LAYOUT seed:=$SEED bridge_port:=$BRIDGE_PORT mjpeg_port:=$MJPEG_PORT" \
  "$SENSOR"
attach_session "$SESSION"

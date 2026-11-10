#!/usr/bin/env bash
# Real robot in one command: tmux session "avoid_real" with the drivers of one sensor
# configuration, the base (motion, IMU), nav_node, rosbridge, the web UI and a status monitor.
#
#   avoid_real.sh camera|lidar [--no-attach]
#     camera  prosilica + black_item (the lidars are not started)
#     lidar   three Hokuyo (the camera is not started)
#   Lidar serial ports: AVOID_LIDAR_PORTS="/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2" (back, left, right)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SESSION=avoid_real
SENSOR= NO_ATTACH=${NO_ATTACH:-0}
CAMERA_IP=192.168.16.100
while [ $# -gt 0 ]; do
  case "$1" in
    camera|lidar) SENSOR="$1" ;;
    --no-attach) NO_ATTACH=1 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) fail "unknown argument '$1' (see --help)" ;;
  esac
  shift
done
[ -n "$SENSOR" ] || fail "choose the sensor configuration: avoid_real.sh camera|lidar"

command -v tmux >/dev/null || fail "tmux is not installed (sudo apt install tmux)"
[ -z "$AVOID_SESSION" ] || fail "this terminal belongs to the running '$AVOID_SESSION' session: run this from another terminal"
"$BRINGUP_DIR/scripts/avoid_stop.sh" --quiet "$SESSION" avoid_sim
make_env "$SESSION"
check_pkgs avoid_bringup avoid_nav avoid_sim motion imu_3d rosbridge_server robot_state_publisher xacro
EXTRA_ARGS=
if [ "$SENSOR" = camera ]; then
  check_pkgs vision prosilica_camera mjpeg_server fira_launch
  check_ports "$ROS_PORT" "$BRIDGE_PORT" "$WEB_PORT" "$MJPEG_PORT"
  ensure_config "$REPO_DIR/fira_launch/default_config/AvoidChallenge.yaml" \
                "$REPO_DIR/vision/config/FIRA.yaml" \
                "$REPO_DIR/vision/config/prosilica_driver.yaml" \
                "$REPO_DIR/vision/config/HSVcolormap.bin"
  if ping -c 1 -W 1 "$CAMERA_IP" >/dev/null 2>&1; then info "camera $CAMERA_IP answers"
  else warn "camera $CAMERA_IP does not answer ping (cable / network settings?); starting anyway"; fi
else
  check_pkgs urg_node
  check_ports "$ROS_PORT" "$BRIDGE_PORT" "$WEB_PORT"
  read -r -a PORTS <<< "${AVOID_LIDAR_PORTS:-/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2}"
  [ ${#PORTS[@]} = 3 ] || fail "AVOID_LIDAR_PORTS needs 3 devices (back, left, right)"
  for i in 0 1 2; do
    [ -e "${PORTS[$i]}" ] || fail "lidar $i: ${PORTS[$i]} does not exist (plugged in? set AVOID_LIDAR_PORTS)"
    [ -r "${PORTS[$i]}" ] && [ -w "${PORTS[$i]}" ] || fail "lidar $i: no permission on ${PORTS[$i]} (add yourself to the dialout group)"
    EXTRA_ARGS="$EXTRA_ARGS lidar_port_$i:=${PORTS[$i]}"
  done
fi

info "real robot, $SENSOR configuration"
start_session "$SESSION" \
  "roslaunch avoid_bringup real.launch nav:=false sensor:=$SENSOR bridge_port:=$BRIDGE_PORT$EXTRA_ARGS" \
  "$SENSOR"
attach_session "$SESSION"

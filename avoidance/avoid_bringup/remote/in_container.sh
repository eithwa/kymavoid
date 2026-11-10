#!/usr/bin/env bash
# Runs inside the kymavoid-noetic container on the build machine (started by remote.sh, not by hand).
#
#   in_container.sh build
#   in_container.sh test
#   in_container.sh eval <sensor> <key=value ...> -- <evaluate.py args>
#   in_container.sh run  <sensor> <key=value ...> -- <command>
#
# Environment (from remote.sh): KYM_WS (catkin workspace), KYM_OUT (results directory), JOBS.
# The container has its own network namespace: its roscore (default ports) is invisible to the
# build machine's other ROS processes.
set -uo pipefail
mode=$1; shift
PKGS="avoid_msgs;avoid_nav;avoid_sim;avoid_bringup"
mkdir -p "$HOME" "$KYM_OUT"
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
cd "$KYM_WS" || exit 1

build() {
  echo "== catkin_make ($PKGS, -j$JOBS)"
  catkin_make -DCATKIN_WHITELIST_PACKAGES="$PKGS" -j"$JOBS" > "$KYM_OUT/build.log" 2>&1
  local rc=$?
  grep -E "error|Error" "$KYM_OUT/build.log" | grep -v "Policy CMP0046\|non-existent dependency" | head -20
  [ $rc = 0 ] && echo "== build ok" || { echo "== build FAILED (exit $rc): $KYM_OUT/build.log"; tail -30 "$KYM_OUT/build.log"; }
  return $rc
}

unit_tests() {
  local rc=0
  echo "== gtest (avoid_nav)"
  catkin_make tests -DCATKIN_WHITELIST_PACKAGES="$PKGS" -j"$JOBS" > "$KYM_OUT/build_tests.log" 2>&1 \
    || { echo "test build FAILED: $KYM_OUT/build_tests.log"; tail -30 "$KYM_OUT/build_tests.log"; return 1; }
  # shellcheck source=/dev/null
  source devel/setup.bash
  # every gtest registered in CMakeLists must have been built (a stale build system once left one out)
  for t in $(sed -n 's/^ *catkin_add_gtest(\([a-z_0-9]*\) .*/\1/p' src/kymavoid/avoidance/avoid_nav/CMakeLists.txt); do
    [ -x "devel/lib/avoid_nav/$t" ] || { echo "   $t: NOT BUILT (registered in CMakeLists)"; rc=1; }
  done
  for t in devel/lib/avoid_nav/test_*; do
    [ -x "$t" ] || continue
    if "$t" > "$KYM_OUT/$(basename "$t").log" 2>&1; then
      echo "   $(basename "$t"): $(grep -E '^\[  PASSED  \]' "$KYM_OUT/$(basename "$t").log")"
    else
      rc=1
      echo "   $(basename "$t"): FAILED"
      grep -E "FAILED|Failure|Expected|actual" "$KYM_OUT/$(basename "$t").log" | head -20
    fi
  done
  echo "== unittest (avoid_sim)"
  if python3 -m unittest discover -s src/kymavoid/avoidance/avoid_sim/test -p 'test_*.py' > "$KYM_OUT/avoid_sim_unittest.log" 2>&1; then
    echo "   $(tail -1 "$KYM_OUT/avoid_sim_unittest.log") ($(grep -E '^Ran ' "$KYM_OUT/avoid_sim_unittest.log"))"
  else
    rc=1
    echo "   FAILED"; tail -30 "$KYM_OUT/avoid_sim_unittest.log"
  fi
  return $rc
}

# headless simulation + navigation, as avoid_bringup/scripts/avoid_sim.sh does, without tmux / web
start_sim() {
  local sensor=$1; shift
  # shellcheck source=/dev/null
  source devel/setup.bash
  export ROS_MASTER_URI=http://localhost:11311 ROS_IP=127.0.0.1 GAZEBO_MASTER_URI=http://localhost:11345
  export PYTHONUNBUFFERED=1
  roscore > "$KYM_OUT/roscore.log" 2>&1 &
  for _ in $(seq 1 30); do rosparam get /run_id > /dev/null 2>&1 && break; sleep 1; done
  for kv in "$@"; do                          # /full/names: before the simulation starts
    case "$kv" in /*) rosparam set "${kv%%=*}" "${kv#*=}" && echo "   ${kv%%=*} = ${kv#*=}" ;; esac
  done
  roslaunch avoid_sim sim.launch gui:=false sensor:="$sensor" > "$KYM_OUT/sim.log" 2>&1 &
  for _ in $(seq 1 90); do rosparam get /robot_description > /dev/null 2>&1 && break; sleep 1; done
  roslaunch avoid_nav nav.launch sensor:="$sensor" > "$KYM_OUT/nav.log" 2>&1 &
  local ok=0
  for _ in $(seq 1 90); do
    if timeout -k 2 4 rostopic echo -n 1 /avoid/status/state_name 2>/dev/null | grep -q IDLE; then ok=1; break; fi
    sleep 1
  done
  [ $ok = 1 ] || { echo "== nav_node did not come up (see $KYM_OUT/sim.log, nav.log)"; tail -20 "$KYM_OUT/nav.log"; return 1; }
  for _ in $(seq 1 60); do rosservice list 2>/dev/null | grep -q '^/avoid_sim/reset$' && break; sleep 1; done
  for kv in "$@"; do                          # plain names: nav_node parameters for the runs
    case "$kv" in /*) ;; *) rosparam set "/nav_node/${kv%%=*}" "${kv#*=}" && echo "   /nav_node/${kv%%=*} = ${kv#*=}" ;; esac
  done
  echo "== simulation up ($sensor): $(rosparam get /nav_node/v_max 2>/dev/null | sed 's/^/v_max /')"
}

stop_sim() {
  pkill -INT -f roslaunch; sleep 3
  pkill -INT -f rosmaster; sleep 1
  pkill -KILL -f "gzserver|roslaunch|rosmaster|rosout" 2> /dev/null || true
}

case "$mode" in
  build) build ;;
  test) build && unit_tests ;;
  eval|run)
    sensor=$1; shift
    sets=()
    while [ $# -gt 0 ] && [ "$1" != "--" ]; do sets+=("$1"); shift; done
    shift
    start_sim "$sensor" "${sets[@]}" || { stop_sim; exit 1; }
    if [ "$mode" = eval ]; then
      rosrun avoid_sim evaluate.py "$@" --csv "$KYM_OUT/eval.csv" 2>&1 | tee "$KYM_OUT/eval.log" \
        | grep --line-buffered -vE "^\[(INFO|WARN)\]|^$"
      rc=${PIPESTATUS[0]}
    else
      (cd "$KYM_OUT" && "$@")
      rc=$?
    fi
    stop_sim
    exit $rc
    ;;
  *) echo "unknown mode $mode" >&2; exit 2 ;;
esac

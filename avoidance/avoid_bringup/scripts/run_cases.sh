#!/usr/bin/env bash
# Run the recorded simulation cases (avoid_sim/cases/*.yaml) as a test suite.
#
#   run_cases.sh [camera|lidar] [--repeat N] [--only id1,id2] [case files...]
#
# Uses the running avoid_sim session if it has the same sensor configuration; otherwise starts one
# without the Gazebo window, runs the cases and stops it again. Cases recorded with the other
# configuration are skipped. Exit code 1 when a `fixed` case fails (a regression).
set -e
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

SENSOR=camera REPEAT=1 ONLY= FILES=()
while [ $# -gt 0 ]; do
  case "$1" in
    camera|lidar) SENSOR="$1" ;;
    --repeat) REPEAT="$2"; shift ;;
    --only) ONLY="$2"; shift ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    -*) fail "unknown option '$1' (see --help)" ;;
    *) FILES+=("$1") ;;
  esac
  shift
done
[ -z "$AVOID_SESSION" ] || fail "run this from a terminal outside the '$AVOID_SESSION' session"

SESSION="$(instance avoid_sim)"
make_env "$SESSION" >/dev/null
unset AVOID_SESSION            # this script is not part of the session (avoid_sim.sh / avoid_stop.sh check it)
CASES_DIR="$REPO_DIR/avoidance/avoid_sim/cases"
if [ ${#FILES[@]} = 0 ]; then
  mapfile -t FILES < <(ls "$CASES_DIR"/*.yaml 2>/dev/null)
  [ ${#FILES[@]} -gt 0 ] || fail "no case files in $CASES_DIR"
fi

running_sensor() { timeout 5 rostopic echo -n 1 /avoid/status/sensor_config 2>/dev/null | head -1 | tr -d '"' | sed 's/^data: //'; }
STARTED=0
if tmux has-session -t "$SESSION" 2>/dev/null && rosparam get /run_id >/dev/null 2>&1; then
  cur="$(running_sensor)"
  [ "$cur" = "$SENSOR" ] || fail "the running $SESSION session is '$cur', not '$SENSOR': stop it (avoid_stop.sh) or pick '$cur'"
  info "using the running $SESSION session ($SENSOR)"
else
  info "starting a simulation without the Gazebo window ($SENSOR)"
  NO_ATTACH=1 "$BRINGUP_DIR/scripts/avoid_sim.sh" "$SENSOR" --no-gui --no-attach >/dev/null
  STARTED=1
  for _ in $(seq 1 60); do [ "$(running_sensor)" = "$SENSOR" ] && break; sleep 1; done
  [ "$(running_sensor)" = "$SENSOR" ] || { "$BRINGUP_DIR/scripts/avoid_stop.sh" --quiet "$SESSION"; fail "nav_node did not come up"; }
fi

ARGS=(--cases "${FILES[@]}" --repeat "$REPEAT")
[ -n "$ONLY" ] && ARGS+=(--only "$ONLY")
set +e
rosrun avoid_sim evaluate.py "${ARGS[@]}"
rc=$?
set -e
[ $STARTED = 1 ] && "$BRINGUP_DIR/scripts/avoid_stop.sh" --quiet "$SESSION"
exit $rc

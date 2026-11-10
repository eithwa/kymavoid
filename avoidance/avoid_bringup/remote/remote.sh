#!/usr/bin/env bash
# Build and test kymavoid on the build machine instead of this computer: Gazebo plus the navigation
# stack at full speed fills a laptop's CPU, and timing-sensitive results (latency, near misses)
# suffer from it. Everything runs in a container (ROS Noetic + Gazebo 11; the build machine itself is
# Ubuntu 22.04 with ROS 2 only) with its own network namespace, so its roscore cannot reach, or be
# reached by, any other ROS process on that machine.
#
#   remote.sh setup                      build the image on the build machine (once, ~4 GB)
#   remote.sh build [sync]               sync the source, catkin_make (avoidance packages)
#   remote.sh test  [sync]               ... and the unit tests (avoid_nav gtest, avoid_sim unittest)
#   remote.sh eval  [sync] [opts] -- <evaluate.py arguments>
#                                        ... and the headless simulation with avoid_sim/evaluate.py
#   remote.sh run   [sync] [opts] -- <command>
#                                        ... any command while the simulation runs (cwd: results dir)
#   remote.sh status                     this tool's containers on the build machine
#
# sync (default: the working tree as it is, uncommitted changes included):
#   --rev <git rev>     a committed revision instead (git archive)
#   --with <path>       with --rev: add this working-tree file or directory on top (repeatable);
#                       <path>=<file> puts <file> there instead (e.g. a CMakeLists without
#                       someone else's uncommitted lines)
#   --no-sync           build what is on the build machine already
# opts:
#   --sensor camera|lidar   (camera)
#   --set key=value         nav_node parameter for the runs (repeatable), e.g. --set v_max=3;
#                           a /full/name is set before the simulation starts, e.g.
#                           --set /virtual_camera/latency=0.1 (the camera later than nav assumes)
#   --tag <name>            results directory name (default: mode-date-time)
#   --cpus <n>              container CPU limit (eval/run 4, build/test 8)
#
# Results come back to <repo>/remote_results/<tag>/ (git-ignored): build / test / simulation logs,
# eval.csv, and whatever `run` wrote to its working directory.
#
# Configuration: KYM_REMOTE_HOST=user@host (ssh key login), or remote.env next to this script
# (git-ignored) with that line. KYM_REMOTE_DIR: directory on the build machine (kymavoid_remote).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO="$(cd "$HERE/../../.." && pwd -P)"
[ -f "$HERE/remote.env" ] && source "$HERE/remote.env"
HOST="${KYM_REMOTE_HOST:-}"
DIR="${KYM_REMOTE_DIR:-kymavoid_remote}"
IMAGE=kymavoid-noetic:1
die() { echo "remote.sh: $*" >&2; exit 1; }
[ -n "$HOST" ] || die "set KYM_REMOTE_HOST=user@host (or put it in $HERE/remote.env)"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST")
on_host() { "${SSH[@]}" "bash -s -- $(printf '%q ' "$@")"; }

mode="${1:-}"; shift || true
rev="" withs=() nosync=0 sensor=camera sets=() tag="" cpus="" rest=()
while [ $# -gt 0 ]; do
  case "$1" in
    --rev) rev="$2"; shift ;;
    --with) withs+=("$2"); shift ;;
    --no-sync) nosync=1 ;;
    --sensor) sensor="$2"; shift ;;
    --set) sets+=("$2"); shift ;;
    --tag) tag="$2"; shift ;;
    --cpus) cpus="$2"; shift ;;
    --) shift; rest=("$@"); break ;;
    -h|--help) sed -n '2,37p' "$0"; exit 0 ;;
    *) die "unknown option '$1' (see --help)" ;;
  esac
  shift
done

case "$mode" in
  setup)
    "${SSH[@]}" "mkdir -p ~/$DIR/image"
    scp -q "$HERE/Dockerfile" "$HOST:$DIR/image/Dockerfile"
    "${SSH[@]}" "cd ~/$DIR/image && docker build -t $IMAGE ."
    exit 0 ;;
  status)
    "${SSH[@]}" "docker ps --filter name=^kymavoid- --format '{{.Names}}  {{.Status}}  {{.Command}}'; uptime"
    exit 0 ;;
  build|test|eval|run) ;;
  *) sed -n '2,37p' "$0"; exit 1 ;;
esac
[ "$mode" != run ] || [ ${#rest[@]} -gt 0 ] || die "run needs a command after --"
tag="${tag:-$mode-$(date +%m%d-%H%M%S)}"
case "$mode" in build|test) cpus="${cpus:-8}" ;; *) cpus="${cpus:-4}" ;; esac

# ---- sync: the build machine's repo/ becomes exactly this source (no .git, no build output)
if [ $nosync = 0 ]; then
  src="$REPO"
  tmp=""
  if [ -n "$rev" ]; then
    tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
    git -C "$REPO" archive "$rev" | tar -x -C "$tmp"
    for w in "${withs[@]}"; do
      if [[ "$w" == *=* ]]; then                     # path=file: that file in place of path
        mkdir -p "$tmp/$(dirname "${w%%=*}")"; cp -a "${w#*=}" "$tmp/${w%%=*}"
      else
        (cd "$REPO" && cp -a --parents "$w" "$tmp/")
      fi
    done
    src="$tmp"
    echo "== sync: $(git -C "$REPO" rev-parse --short "$rev")${withs[*]:+ + ${withs[*]}}"
  else
    echo "== sync: working tree ($(git -C "$REPO" rev-parse --short HEAD), $(git -C "$REPO" status --short | wc -l) changed files)"
  fi
  "${SSH[@]}" "mkdir -p ~/$DIR/repo ~/$DIR/ws/src ~/$DIR/results && ln -sfn ../../repo ~/$DIR/ws/src/kymavoid"
  # By content, not by time: a changed file arrives with a new mtime, so make / cmake rebuild it. With
  # -a the local mtime came along (git archive: the commit's), older than the remote build output,
  # and changed sources or CMakeLists were silently not rebuilt.
  rsync -rlpD --checksum --delete --exclude .git --exclude __pycache__ --exclude '*.pyc' --exclude /remote_results \
    --exclude 'avoidance/avoid_bringup/remote/remote.env' -e "ssh -o BatchMode=yes" "$src/" "$HOST:$DIR/repo/"
fi

# ---- run in the container; one build at a time (shared workspace), simulations in parallel
echo "== $mode on $HOST (container kymavoid-$tag, $cpus CPUs)"
set +e
on_host "$DIR" "$IMAGE" "$tag" "$cpus" "$mode" "$sensor" "${sets[@]+"${sets[@]}"}" -- "${rest[@]+"${rest[@]}"}" <<'REMOTE'
set -uo pipefail
base="$HOME/$1"; image=$2; tag=$3; cpus=$4; mode=$5; sensor=$6; shift 6
out="$base/results/$tag"; mkdir -p "$out"
docker image inspect "$image" > /dev/null 2>&1 || { echo "no image $image on $(hostname): run 'remote.sh setup' first" >&2; exit 1; }
args=("$mode")
case "$mode" in
  eval|run) args+=("$sensor" "$@") ;;
esac
lock=()
case "$mode" in build|test) lock=(flock -w 600 "$base/.build.lock") ;; esac
"${lock[@]}" docker run --rm --name "kymavoid-$tag" --cpus "$cpus" -u "$(id -u):$(id -g)" \
  -e HOME=/tmp/home -e USER="$(id -un)" -e KYM_WS="$base/ws" -e KYM_OUT="$out" -e JOBS="$cpus" \
  -v "$base:$base" -w "$base/ws" "$image" \
  bash "$base/repo/avoidance/avoid_bringup/remote/in_container.sh" "${args[@]}"
REMOTE
rc=$?
set -e
mkdir -p "$REPO/remote_results/$tag"
rsync -a -e "ssh -o BatchMode=yes" "$HOST:$DIR/results/$tag/" "$REPO/remote_results/$tag/" || true
echo "== results: $REPO/remote_results/$tag  (exit $rc)"
exit $rc

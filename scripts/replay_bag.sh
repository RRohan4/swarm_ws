#!/usr/bin/env bash
# Replay the most recent exploration bag at 10x speed with a Foxglove bridge.
#
# Usage:
#   ./scripts/replay_bag.sh              # replay latest bag
#   ./scripts/replay_bag.sh bags/foo     # replay a specific bag
#
# Then open Foxglove → New connection → Foxglove WebSocket → ws://localhost:8765

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags}"
RATE="${RATE:-10}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
START_DELAY="${START_DELAY:-5}"
ROBOT_IDS_CSV="${ROBOT_IDS:-robot_0,robot_1,robot_2,robot_3}"
ROBOT_URDF="${ROBOT_URDF:-${REPO_ROOT}/src/swarm_exploration/urdf/turtlebot3_waffle.urdf}"
DEFAULT_POSES=(
  "0.6 0.6 0 1.5708"
  "1.8 0.6 0 1.5708"
  "0.6 1.8 0 1.5708"
  "1.8 1.8 0 1.5708"
)

if [[ $# -ge 1 ]]; then
  BAG_PATH="$1"
else
  shopt -s nullglob
  bag_candidates=("${BAG_DIR}"/exploration_*)
  shopt -u nullglob
  if [[ ${#bag_candidates[@]} -eq 0 ]]; then
    echo "No bags found in ${BAG_DIR}. Run a recording first." >&2
    exit 1
  fi
  IFS=$'\n' bag_candidates=($(printf '%s\n' "${bag_candidates[@]}" | sort -r))
  unset IFS
  BAG_PATH="${bag_candidates[0]}"
fi

echo "Replaying: $BAG_PATH  (rate=${RATE}x)"

# ROS setup scripts read some optional variables before defining them, which
# breaks under `set -u`. Temporarily relax nounset while sourcing overlays.
source_ros_setup() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

# Always source the workspace overlay when available so replay can deserialize
# custom messages from the bag (for example swarm_msgs/*).
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_ros_setup /opt/ros/jazzy/setup.bash
fi
if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  source_ros_setup "${REPO_ROOT}/install/setup.bash"
fi
if ! command -v ros2 &>/dev/null; then
  echo "ROS 2 environment not found. Source your workspace first." >&2
  exit 1
fi
if [[ ! -f "${ROBOT_URDF}" ]]; then
  echo "Robot URDF not found at ${ROBOT_URDF}" >&2
  exit 1
fi

IFS=',' read -r -a ROBOT_IDS_ARR <<< "${ROBOT_IDS_CSV}"
ROBOT_DESCRIPTION="$(<"${ROBOT_URDF}")"
CHILD_PIDS=()
cleanup_ran=0

# Start Foxglove bridge and robot_state_publishers, then play the bag
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p port:="${FOXGLOVE_PORT}" -p use_sim_time:=true &
FOXGLOVE_PID=$!
CHILD_PIDS+=("${FOXGLOVE_PID}")

RSP_PIDS=()
for robot_id in "${ROBOT_IDS_ARR[@]}"; do
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -r __ns:="/${robot_id}" \
    -p use_sim_time:=true \
    -p frame_prefix:="${robot_id}/" \
    -p robot_description:="${ROBOT_DESCRIPTION}" &
  RSP_PIDS+=($!)
  CHILD_PIDS+=($!)
done

STATIC_TF_PIDS=()
for i in "${!ROBOT_IDS_ARR[@]}"; do
  robot_id="${ROBOT_IDS_ARR[$i]}"
  pose="${DEFAULT_POSES[$i]:-${DEFAULT_POSES[-1]}}"
  IFS=' ' read -r px py pz pyaw <<< "${pose}"
  ros2 run tf2_ros static_transform_publisher \
    --x "${px}" \
    --y "${py}" \
    --z "${pz}" \
    --yaw "${pyaw}" \
    --pitch 0 \
    --roll 0 \
    --frame-id world \
    --child-frame-id "${robot_id}/map" &
  STATIC_TF_PIDS+=($!)
  CHILD_PIDS+=($!)
done

cleanup() {
  if [[ ${cleanup_ran} -eq 1 ]]; then
    return
  fi
  cleanup_ran=1
  for pid in "${CHILD_PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${CHILD_PIDS[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  wait "${CHILD_PIDS[@]}" 2>/dev/null || true
}

handle_interrupt() {
  cleanup
  exit 130
}
trap cleanup EXIT
trap handle_interrupt INT TERM

# Brief pause so the bridge is ready before playback starts
sleep 2

echo "Connect Foxglove to ws://localhost:${FOXGLOVE_PORT}"
echo "Press Enter after Foxglove is connected so it receives /tf_static."
read -r
echo "Starting playback in ${START_DELAY} seconds..."
sleep "${START_DELAY}"

ros2 bag play "$BAG_PATH" \
  --rate "$RATE" \
  --clock &
PLAY_PID=$!
CHILD_PIDS+=("${PLAY_PID}")
wait "${PLAY_PID}"
cleanup
exit 0

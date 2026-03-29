#!/usr/bin/env bash
# Replay the most recent exploration bag at 10x speed with a Foxglove bridge.
#
# Usage:
#   ./scripts/replay_bag.sh              # replay latest bag
#   ./scripts/replay_bag.sh bags/foo     # replay a specific bag
#
# Then open Foxglove → New connection → Foxglove WebSocket → ws://localhost:8765

set -euo pipefail

BAG_DIR="${BAG_DIR:-./bags}"
RATE="${RATE:-10}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

if [[ $# -ge 1 ]]; then
  BAG_PATH="$1"
else
  BAG_PATH=$(ls -dt "${BAG_DIR}"/exploration_* 2>/dev/null | head -1)
  if [[ -z "$BAG_PATH" ]]; then
    echo "No bags found in ${BAG_DIR}. Run a recording first." >&2
    exit 1
  fi
fi

echo "Replaying: $BAG_PATH  (rate=${RATE}x)"
echo "Connect Foxglove to ws://localhost:${FOXGLOVE_PORT}"
echo

# Source ROS if not already sourced
if ! command -v ros2 &>/dev/null; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi

# Start Foxglove bridge (background), then play the bag
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p port:="${FOXGLOVE_PORT}" -p use_sim_time:=true &
FOXGLOVE_PID=$!

cleanup() {
  kill "$FOXGLOVE_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Brief pause so the bridge is ready before playback starts
sleep 2

ros2 bag play "$BAG_PATH" \
  --rate "$RATE" \
  --clock \
  --loop

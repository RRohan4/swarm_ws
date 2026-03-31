#!/usr/bin/env python3
"""
Records a ROS 2 bag during swarm exploration and stops automatically when all
robots reach DONE state.

Per-robot raw scans and individual SLAM maps are excluded — they are large and
not needed for visualization replay.  Everything else is captured, including
/merged_map, /frontiers, /frontier_markers, /robot_poses, /tf, /clock, and all
per-robot status/goal/odom topics.

Configuration via environment variables:
  SWARM_ROBOT_IDS   comma-separated list of robot IDs  (default: robot_0,...,robot_3)
  BAG_OUTPUT_DIR    directory for bag files             (default: /bags)
  EXPLORE_CUTOFF    stop once exploration reaches N %   (default: 0, disabled)
  TIMEOUT_CUTOFF    stop after N wall-clock seconds     (default: 0, disabled)
"""

import os
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from swarm_msgs.msg import RobotStatus

DONE = 2

# Exclude raw laser scans and per-robot SLAM maps — together they dwarf every
# other topic and make 10x replay choppy.  /merged_map is sufficient for replay.
EXCLUDE_PATTERN = r"/robot_[0-9]+/(scan|map)"

OUTPUT_DIR = os.environ.get("BAG_OUTPUT_DIR", "/bags")
ROBOT_IDS_ENV = os.environ.get("SWARM_ROBOT_IDS", "robot_0,robot_1,robot_2,robot_3")
EXPLORE_CUTOFF = float(os.environ.get("EXPLORE_CUTOFF", "0"))  # 0 = disabled
TIMEOUT_CUTOFF = float(os.environ.get("TIMEOUT_CUTOFF", "0"))  # 0 = disabled


class BagRecorderNode(Node):
    def __init__(self, robot_ids: list[str]) -> None:
        super().__init__("bag_recorder")
        self._robot_ids = set(robot_ids)
        self._states: dict[str, int] = {}
        self._proc: subprocess.Popen | None = None
        self._stopped = False
        self._timeout_timer = None
        self._proc_watchdog = self.create_timer(0.5, self._check_recorder_proc)

        for rid in robot_ids:
            self.create_subscription(
                RobotStatus,
                f"/{rid}/status",
                lambda msg, r=rid: self._on_status(msg, r),
                10,
            )

        if EXPLORE_CUTOFF > 0:
            self.create_subscription(
                Float32, "/exploration_pct", self._on_explore_pct, 10
            )

        if TIMEOUT_CUTOFF > 0:
            self._timeout_timer = self.create_timer(
                TIMEOUT_CUTOFF, self._on_timeout_cutoff
            )

        self._start_recording()
        cutoff_parts = []
        if EXPLORE_CUTOFF > 0:
            cutoff_parts.append(f"exploration cutoff {EXPLORE_CUTOFF}%")
        if TIMEOUT_CUTOFF > 0:
            cutoff_parts.append(f"timeout cutoff {TIMEOUT_CUTOFF}s")
        cutoff_info = f", {', '.join(cutoff_parts)}" if cutoff_parts else ""
        self.get_logger().info(
            f"Watching {robot_ids} — will stop when all reach DONE{cutoff_info}."
        )

    def _start_recording(self) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(OUTPUT_DIR, f"exploration_{timestamp}")
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        cmd = [
            "ros2",
            "bag",
            "record",
            "--all-topics",
            "--exclude-regex",
            EXCLUDE_PATTERN,
            "--output",
            bag_path,
        ]
        self._proc = subprocess.Popen(cmd, start_new_session=True)
        self.get_logger().info(f"Recording → {bag_path}")

    def _check_recorder_proc(self) -> None:
        if self._stopped or self._proc is None:
            return
        exit_code = self._proc.poll()
        if exit_code is None:
            return
        self.get_logger().error(
            f"ros2 bag record exited unexpectedly with code {exit_code}."
        )
        self._stop("Recorder process failed")

    def _on_explore_pct(self, msg: Float32) -> None:
        if EXPLORE_CUTOFF > 0 and msg.data >= EXPLORE_CUTOFF:
            self.get_logger().info(
                f"Exploration {msg.data:.1f}% >= cutoff {EXPLORE_CUTOFF}% — stopping."
            )
            self._stop("Exploration cutoff reached")

    def _on_timeout_cutoff(self) -> None:
        if TIMEOUT_CUTOFF > 0:
            self.get_logger().info(
                f"Timeout {TIMEOUT_CUTOFF:.1f}s reached — stopping."
            )
            self._stop("Timeout cutoff reached")

    def _on_status(self, msg: RobotStatus, robot_id: str) -> None:
        self._states[robot_id] = msg.state
        if self._robot_ids <= self._states.keys() and all(
            self._states[r] == DONE for r in self._robot_ids
        ):
            self._stop("All robots DONE")

    def _stop(self, reason: str) -> None:
        if self._stopped:
            return
        self._stopped = True
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        if self._proc_watchdog is not None:
            self._proc_watchdog.cancel()
        self.get_logger().info(f"{reason} — stopping bag recorder.")
        if self._proc and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=15)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    "ros2 bag record did not exit after SIGINT; sending SIGTERM."
                )
                self._proc.terminate()
                self._proc.wait(timeout=5)
        self.get_logger().info("Bag saved.")
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    robot_ids = [r.strip() for r in ROBOT_IDS_ENV.split(",") if r.strip()]
    node = BagRecorderNode(robot_ids)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop("Interrupted")
    node.destroy_node()


if __name__ == "__main__":
    main()

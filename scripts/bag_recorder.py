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
"""

import os
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node

from swarm_msgs.msg import RobotStatus

DONE = 2

# Exclude raw laser scans and per-robot SLAM maps — together they dwarf every
# other topic and make 10x replay choppy.  /merged_map is sufficient for replay.
EXCLUDE_PATTERN = r"/robot_[0-9]+/(scan|map)"

OUTPUT_DIR = os.environ.get("BAG_OUTPUT_DIR", "/bags")
ROBOT_IDS_ENV = os.environ.get("SWARM_ROBOT_IDS", "robot_0,robot_1,robot_2,robot_3")


class BagRecorderNode(Node):
    def __init__(self, robot_ids: list[str]) -> None:
        super().__init__("bag_recorder")
        self._robot_ids = set(robot_ids)
        self._states: dict[str, int] = {}
        self._proc: subprocess.Popen | None = None
        self._stopped = False

        for rid in robot_ids:
            self.create_subscription(
                RobotStatus,
                f"/{rid}/status",
                lambda msg, r=rid: self._on_status(msg, r),
                10,
            )

        self._start_recording()
        self.get_logger().info(f"Watching {robot_ids} — will stop when all reach DONE.")

    def _start_recording(self) -> None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(OUTPUT_DIR, f"exploration_{timestamp}")
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        cmd = [
            "ros2",
            "bag",
            "record",
            "--all",
            "--exclude",
            EXCLUDE_PATTERN,
            "--output",
            bag_path,
        ]
        self._proc = subprocess.Popen(cmd)
        self.get_logger().info(f"Recording → {bag_path}")

    def _on_status(self, msg: RobotStatus, robot_id: str) -> None:
        self._states[robot_id] = msg.state
        if self._robot_ids <= self._states.keys() and all(
            self._states[r] == DONE for r in self._robot_ids
        ):
            self._stop()

    def _stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        self.get_logger().info("All robots DONE — stopping bag recorder.")
        if self._proc and self._proc.poll() is None:
            self._proc.send_signal(signal.SIGINT)
            self._proc.wait(timeout=15)
        self.get_logger().info("Bag saved.")
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    robot_ids = [r.strip() for r in ROBOT_IDS_ENV.split(",") if r.strip()]
    node = BagRecorderNode(robot_ids)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop()
    node.destroy_node()


if __name__ == "__main__":
    main()

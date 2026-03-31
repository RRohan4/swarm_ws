#!/usr/bin/env python3
"""
Measures time from first exploration activity to EXPLORE_TARGET% map coverage.

"Not including setup time" — the clock starts when the first non-empty
/frontiers message arrives (robots have left the WAITING state and begun
navigating), not from process start.

Writes a JSON result file and exits so that docker compose
--abort-on-container-exit / --exit-code-from measurer can stop the run.

Environment variables:
  EXPLORE_TARGET   target exploration percentage          (default: 80.0)
  RESULTS_FILE     path to write JSON result              (default: none)
  SIM_TIMEOUT_S    sim-time seconds before giving up      (default: 1800)
  NUM_ROBOTS       informational — included in result     (default: "?")
"""

import json
import os
import time

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32

from swarm_msgs.msg import FrontierArray

EXPLORE_TARGET = float(os.environ.get("EXPLORE_TARGET", "80.0"))
RESULTS_FILE = os.environ.get("RESULTS_FILE", "")
SIM_TIMEOUT_S = float(os.environ.get("SIM_TIMEOUT_S", "1800"))
WATCHDOG_WALL_S = float(os.environ.get("BENCHMARK_WATCHDOG_S", "1.0"))
NUM_ROBOTS = os.environ.get("NUM_ROBOTS", "?")


class ExplorationMeasurer(Node):
    def __init__(self) -> None:
        super().__init__("exploration_measurer")

        self._t_start_wall: float | None = None
        self._t_start_sim: float | None = None
        self._sim_now: float = 0.0  # latest sim time in seconds
        self._current_pct: float = 0.0
        self._done = False

        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(FrontierArray, "/frontiers", self._on_frontiers, 10)
        self.create_subscription(Float32, "/exploration_pct", self._on_explore_pct, 10)

        # Faster watchdog avoids extra wall-time after sim-time timeout.
        self._watchdog = self.create_timer(WATCHDOG_WALL_S, self._check_timeout)
        self._last_progress_pct: float = -1.0

        self.get_logger().info(
            f"Waiting for first frontiers to start timer "
            f"(target={EXPLORE_TARGET}%, sim_timeout={SIM_TIMEOUT_S}s, "
            f"robots={NUM_ROBOTS}, watchdog={WATCHDOG_WALL_S:.1f}s)."
        )

    # ── Clock tracking ─────────────────────────────────────────────────────────

    def _on_clock(self, msg: Clock) -> None:
        self._sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ── Start trigger: first non-empty /frontiers ──────────────────────────────

    def _on_frontiers(self, msg: FrontierArray) -> None:
        if self._t_start_wall is not None or len(msg.frontiers) == 0:
            return
        if self._sim_now == 0.0:
            return  # clock not yet valid

        self._t_start_wall = time.monotonic()
        self._t_start_sim = self._sim_now
        self.get_logger().info(
            f"Timer started — first frontiers received "
            f"(sim_t={self._sim_now:.1f}s, {len(msg.frontiers)} frontier(s))."
        )

    # ── Stop trigger: /exploration_pct reaches target ──────────────────────────

    def _on_explore_pct(self, msg: Float32) -> None:
        self._current_pct = msg.data
        if self._t_start_wall is None or self._done:
            return
        if msg.data >= EXPLORE_TARGET:
            self._finish(success=True)

    # ── Sim-time timeout watchdog + periodic progress ─────────────────────────

    def _check_timeout(self) -> None:
        if self._done:
            return

        # Log current % whenever it changes by ≥1 point (or timer first fires).
        if self._t_start_sim is not None:
            elapsed_sim = self._sim_now - self._t_start_sim
            if self._current_pct - self._last_progress_pct >= 1.0:
                self._last_progress_pct = self._current_pct
                print(
                    f"BENCHMARK_PROGRESS: {self._current_pct:.1f}%  "
                    f"sim={elapsed_sim:.0f}s",
                    flush=True,
                )

            if elapsed_sim >= SIM_TIMEOUT_S:
                self.get_logger().warning(
                    f"Sim-time timeout ({SIM_TIMEOUT_S:.0f}s) reached "
                    f"at {self._current_pct:.1f}% — giving up."
                )
                self._finish(success=False)

    # ── Record result and shut down ────────────────────────────────────────────

    def _finish(self, *, success: bool) -> None:
        if self._done:
            return
        self._done = True
        self._watchdog.cancel()

        elapsed_wall = time.monotonic() - self._t_start_wall
        elapsed_sim = self._sim_now - self._t_start_sim

        result = {
            "num_robots": NUM_ROBOTS,
            "target_pct": EXPLORE_TARGET,
            "achieved_pct": round(self._current_pct, 2),
            "elapsed_sim_s": round(elapsed_sim, 2),
            "elapsed_wall_s": round(elapsed_wall, 2),
            "success": success,
        }

        status = "REACHED" if success else "TIMED OUT"
        print(f"\n{'=' * 52}")
        print(f"  Robots:            {NUM_ROBOTS}")
        print(f"  Target:            {EXPLORE_TARGET:.0f}%  →  {status}")
        print(f"  Achieved:          {self._current_pct:.1f}%")
        print(f"  Sim time elapsed:  {elapsed_sim:.1f} s")
        print(f"  Wall time elapsed: {elapsed_wall:.1f} s")
        print(f"{'=' * 52}\n")
        print(f"BENCHMARK_RESULT: {json.dumps(result)}", flush=True)

        if RESULTS_FILE:
            os.makedirs(os.path.dirname(RESULTS_FILE) or ".", exist_ok=True)
            with open(RESULTS_FILE, "w") as fh:
                json.dump(result, fh, indent=2)
            self.get_logger().info(f"Result written to {RESULTS_FILE}")

        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = ExplorationMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node._done:
            node.get_logger().info("Interrupted — no result written.")
    node.destroy_node()


if __name__ == "__main__":
    main()

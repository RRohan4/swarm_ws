"""
coordinator_node.py

Central coordinator. Runs at 1 Hz.

Subscriptions:
  /frontiers           (swarm_msgs/FrontierArray)
  /robot_poses         (geometry_msgs/PoseArray)
  /robot_N/status      (swarm_msgs/RobotStatus)   for each robot

Publications:
  /robot_N/exploration_target  (geometry_msgs/PoseStamped)  for each robot
  /assignment_markers          (visualization_msgs/MarkerArray)

Scoring:
  score = cluster_size / euclidean_distance(robot, frontier_centroid)
  (Nav2 path cost fallback is left as future work — Euclidean is used here)

Assignment:
  Greedy submodular: iterate robots in descending "map_cells_known" order;
  each picks the highest-scored unassigned frontier. This yields ≥ 63% of the
  optimal submodular assignment.

Parameters:
  robot_ids   : list[str]  (default ["robot_0", "robot_1"])
  rate        : float      Hz (default 1.0)
"""

import math

import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from swarm_msgs.msg import Frontier, FrontierArray, RobotStatus

# How far from a robot a frontier must be (metres) to be worth sending
MIN_DISPATCH_DIST = 0.5
# Distance threshold to consider two frontier centroids the "same" target
BLACKLIST_DIST = 1.0
# Seconds before a blacklist entry expires (map may have changed)
BLACKLIST_TTL = 120.0
# Seconds to wait after a robot finishes a goal before reassigning.
# Gives the SLAM → map_merge → frontier_detector pipeline time to reveal
# new frontiers (e.g. further down a corridor the robot was already exploring).
REASSIGN_COOLDOWN = 1.0


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("coordinator_node")

        self.declare_parameter("robot_ids", ["robot_0", "robot_1"])
        self.declare_parameter("rate", 1.0)

        self._robot_ids: list[str] = (
            self.get_parameter("robot_ids").get_parameter_value().string_array_value
        )
        rate: float = self.get_parameter("rate").get_parameter_value().double_value

        self._frontiers: list[Frontier] = []
        self._frontiers_ever_received: bool = False  # guard against early DONE
        self._robot_poses: dict[str, Point] = {}  # robot_id → latest position
        self._robot_status: dict[str, RobotStatus] = {}
        self._prev_state: dict[str, int] = {}  # previous state per robot
        self._assigned_target: dict[str, Point] = {}  # current assignment per robot
        # Last heading direction per robot (unit vector) — for momentum bonus
        self._last_heading: dict[str, tuple[float, float]] = {}
        # Timestamp when each robot last entered WAITING (for reassignment cooldown)
        self._waiting_since: dict[str, float] = {}
        # Per-robot blacklist: list of (centroid, ros_timestamp) for failed targets
        self._blacklist: dict[str, list[tuple[Point, float]]] = {
            rid: [] for rid in self._robot_ids
        }

        # Subscriptions
        self.create_subscription(FrontierArray, "/frontiers", self._frontiers_cb, 10)
        self.create_subscription(PoseArray, "/robot_poses", self._poses_cb, 10)
        for rid in self._robot_ids:
            self.create_subscription(
                RobotStatus,
                f"/{rid}/status",
                lambda msg, r=rid: self._status_cb(r, msg),
                10,
            )

        # Publishers
        self._target_pubs: dict[str, any] = {
            rid: self.create_publisher(PoseStamped, f"/{rid}/exploration_target", 10)
            for rid in self._robot_ids
        }
        self._marker_pub = self.create_publisher(MarkerArray, "/assignment_markers", 10)

        self.create_timer(1.0 / rate, self._assign)
        self.get_logger().info(f"coordinator_node started — managing {self._robot_ids}")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _frontiers_cb(self, msg: FrontierArray) -> None:
        self._frontiers = list(msg.frontiers)
        self._frontiers_ever_received = True

    def _poses_cb(self, msg: PoseArray) -> None:
        for i, pose in enumerate(msg.poses):
            if i < len(self._robot_ids):
                self._robot_poses[self._robot_ids[i]] = pose.position

    def _status_cb(self, robot_id: str, msg: RobotStatus) -> None:
        prev = self._prev_state.get(robot_id)
        # EXPLORING (0) → WAITING (1) means navigation finished.
        # If the old target is still in the frontier list, the robot likely
        # failed to reach it — blacklist so the coordinator picks a different one.
        if prev == 0 and msg.state == 1:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._waiting_since[robot_id] = now
            if robot_id in self._assigned_target:
                target = self._assigned_target.pop(robot_id)
                # Check if the target frontier still exists (success removes it)
                for f in self._frontiers:
                    if _dist(f.centroid, target) < BLACKLIST_DIST:
                        self._blacklist[robot_id].append((target, now))
                        self.get_logger().info(
                            f"[coordinator] blacklisted ({target.x:.1f}, "
                            f"{target.y:.1f}) for {robot_id}"
                        )
                        break
        self._prev_state[robot_id] = msg.state
        self._robot_status[robot_id] = msg

    # ── Assignment ─────────────────────────────────────────────────────────────

    def _assign(self) -> None:
        if not self._frontiers:
            # Only signal DONE once we've actually received frontier data at least
            # once.  Before the first update the list is empty because the map
            # isn't ready yet — robots must not be sent to DONE prematurely.
            if not self._frontiers_ever_received:
                return
            for rid in self._robot_ids:
                status = self._robot_status.get(rid)
                if status is not None and status.state == 1:  # WAITING
                    done_msg = PoseStamped()
                    done_msg.header.frame_id = "done"
                    done_msg.header.stamp = self.get_clock().now().to_msg()
                    self._target_pubs[rid].publish(done_msg)
            return

        if not self._robot_poses:
            return

        # Only assign to WAITING robots (state == 1) past their cooldown.
        # The cooldown lets SLAM/map_merge/frontier_detector catch up so new
        # nearby frontiers (e.g. further down a corridor) appear before we
        # reassign the robot to a distant frontier.
        now_ts = self.get_clock().now().nanoseconds * 1e-9
        waiting_robots = [
            rid
            for rid in self._robot_ids
            if self._robot_status.get(rid) is not None
            and self._robot_status[rid].state == 1
            and now_ts - self._waiting_since.get(rid, 0.0) >= REASSIGN_COOLDOWN
        ]

        if not waiting_robots:
            return

        # Sort robots: those that know more of the map explore first
        waiting_robots.sort(
            key=lambda r: self._robot_status[r].map_cells_known, reverse=True
        )

        # Expire old blacklist entries
        now = self.get_clock().now().nanoseconds * 1e-9
        for rid in self._robot_ids:
            self._blacklist[rid] = [
                (pt, t) for pt, t in self._blacklist[rid] if now - t < BLACKLIST_TTL
            ]

        # Collect positions of robots that are currently EXPLORING (busy targets)
        busy_targets: list[Point] = [
            self._assigned_target[rid]
            for rid in self._robot_ids
            if rid in self._assigned_target
            and self._robot_status.get(rid) is not None
            and self._robot_status[rid].state == 0  # EXPLORING
        ]

        available = list(self._frontiers)
        assignments: dict[str, Frontier] = {}

        for rid in waiting_robots:
            if not available:
                break
            pos = self._robot_poses.get(rid)
            if pos is None:
                continue

            # If a robot is WAITING but still has an assigned target, clear it
            # so a new one can be assigned. Don't blacklist — the failure may
            # have been transient (e.g. Nav2 not ready yet).
            if rid in self._assigned_target:
                self._assigned_target.pop(rid)

            # Filter out blacklisted frontiers for this robot
            candidates = [
                f for f in available if not self._is_blacklisted(rid, f.centroid)
            ]
            if not candidates:
                candidates = available  # fallback: ignore blacklist

            best_f = max(
                candidates,
                key=lambda f: self._score(rid, pos, f, busy_targets),
            )
            dist = _dist(pos, best_f.centroid)
            if dist < MIN_DISPATCH_DIST:
                # Already at the frontier; skip
                available.remove(best_f)
                continue

            best_f.assigned_to = rid
            best_f.info_score = self._score(rid, pos, best_f, busy_targets)
            assignments[rid] = best_f
            available.remove(best_f)
            # Track as a busy target for subsequent robots in this loop
            busy_targets.append(best_f.centroid)

        # Publish targets and record assignments
        stamp = self.get_clock().now().to_msg()
        for rid, frontier in assignments.items():
            # Record heading direction (robot → frontier) for momentum bonus
            pos = self._robot_poses.get(rid)
            if pos is not None:
                dx = frontier.centroid.x - pos.x
                dy = frontier.centroid.y - pos.y
                norm = math.sqrt(dx * dx + dy * dy)
                if norm > 0.01:
                    self._last_heading[rid] = (dx / norm, dy / norm)
            self._assigned_target[rid] = frontier.centroid
            ps = PoseStamped()
            ps.header = Header(stamp=stamp, frame_id="world")
            ps.pose.position = frontier.centroid
            ps.pose.orientation.w = 1.0
            self._target_pubs[rid].publish(ps)
            self.get_logger().info(
                f"[coordinator] {rid} → ({frontier.centroid.x:.2f}, "
                f"{frontier.centroid.y:.2f}) score={frontier.info_score:.3f}"
            )

        self._publish_assignment_markers(assignments, stamp)

    def _is_blacklisted(self, robot_id: str, centroid: Point) -> bool:
        for pt, _ in self._blacklist[robot_id]:
            if _dist(pt, centroid) < BLACKLIST_DIST:
                return True
        return False

    def _score(
        self,
        robot_id: str,
        robot_pos: Point,
        frontier: Frontier,
        busy_targets: list[Point] | None = None,
    ) -> float:
        d = _dist(robot_pos, frontier.centroid)
        # Strongly prefer closer frontiers: inverse-square distance weighting
        proximity = 1.0 / max(d, 0.1) ** 2
        # Cluster size provides a mild bonus (sqrt to dampen large clusters)
        score = math.sqrt(frontier.cluster_size) * proximity

        # Momentum bonus: strongly prefer frontiers in the same direction
        # the robot was already heading. This makes robots commit to corridors
        # and continue exploring forward rather than backtracking.
        heading = self._last_heading.get(robot_id)
        if heading is not None and d > 0.1:
            dx = frontier.centroid.x - robot_pos.x
            dy = frontier.centroid.y - robot_pos.y
            # cos(angle) between last heading and frontier direction
            cos_angle = (heading[0] * dx + heading[1] * dy) / d
            # Range [-1, 1] → [0.1, 3.0]: forward = 3×, backward = 0.1×
            # Strong bias keeps robots committed to their exploration corridor
            score *= 0.1 + 1.45 * (1.0 + cos_angle)

        # Penalize frontiers near OTHER robots (not just busy targets) —
        # encourages each robot to explore its own neighbourhood
        for other_id, other_pos in self._robot_poses.items():
            if other_id == robot_id:
                continue
            sep = _dist(other_pos, frontier.centroid)
            # Boost if frontier is far from the other robot, penalize if close
            if sep < 2.0:
                score *= 0.2
            elif sep < 4.0:
                score *= 0.5

        # Also penalize frontiers near already-assigned targets
        if busy_targets:
            for bt in busy_targets:
                sep = _dist(bt, frontier.centroid)
                if sep < 3.0:
                    score *= 0.3
        return score

    # ── Markers ────────────────────────────────────────────────────────────────

    def _publish_assignment_markers(
        self, assignments: dict[str, Frontier], stamp
    ) -> None:
        ma = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        colors = [
            ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.9),
            ColorRGBA(r=0.0, g=0.4, b=1.0, a=0.9),
            ColorRGBA(r=1.0, g=0.0, b=0.8, a=0.9),
            ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.9),
        ]

        for i, (rid, frontier) in enumerate(assignments.items()):
            robot_pos = self._robot_poses.get(rid)
            if robot_pos is None:
                continue

            color = colors[i % len(colors)]
            marker_id = i * 2

            # Arrow from robot to frontier
            arrow = Marker()
            arrow.header = Header(stamp=stamp, frame_id="world")
            arrow.ns = "assignments"
            arrow.id = marker_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(
                    x=robot_pos.x,
                    y=robot_pos.y,
                    z=0.1,
                ),
                Point(
                    x=frontier.centroid.x,
                    y=frontier.centroid.y,
                    z=0.1,
                ),
            ]
            arrow.scale.x = 0.05
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            arrow.color = color
            arrow.lifetime.sec = 2
            ma.markers.append(arrow)

            # Sphere at frontier
            sphere = Marker()
            sphere.header = Header(stamp=stamp, frame_id="world")
            sphere.ns = "assignments"
            sphere.id = marker_id + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = frontier.centroid
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.2
            sphere.scale.y = 0.2
            sphere.scale.z = 0.2
            sphere.color = color
            sphere.lifetime.sec = 2
            ma.markers.append(sphere)

        self._marker_pub.publish(ma)


def _dist(a: Point, b: Point) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""
robot_fsm_node.py

Per-robot finite-state machine that drives autonomous exploration.

States:
  WAITING    (1) — no target assigned; self-assigns frontiers
  EXPLORING  (0) — navigating to an assigned frontier via Nav2 NavigateToPose
  DONE       (2) — no more frontiers (frontier list becomes empty)

Subscriptions:
  /robot_N/map                 (nav_msgs/OccupancyGrid) — for map_cells_known
  /merged_map                  (nav_msgs/OccupancyGrid) — BFS ground-distance
  /frontiers                   (swarm_msgs/FrontierArray)
  /robot_poses                 (geometry_msgs/PoseArray) — all robot positions

Publications:
  /robot_N/status  (swarm_msgs/RobotStatus)
  /robot_N/goal    (geometry_msgs/PoseStamped) — this robot's current goal

Coordination:
  Geodesic Voronoi partitioning using all robot positions on the merged map.
  Multi-source BFS from every robot simultaneously; each cell is owned by the
  robot whose wavefront reaches it first (geodesic nearest).  Within its own
  partition a robot picks the closest frontier by ground distance.  If the
  partition contains no frontiers, falls back to the globally closest frontier.

Parameters:
  robot_id        : str        (default "robot_0")
  robot_ids       : list[str]  (default ["robot_0", "robot_1"])
  status_rate     : float Hz   (default 2.0)
  goal_timeout    : float seconds before aborting a goal (default 60.0)
"""

import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from swarm_msgs.msg import Frontier, FrontierArray, RobotStatus

# Minimum distance to a frontier before it's worth navigating to
MIN_DISPATCH_DIST = 0.5
# Distance threshold to consider a frontier the "same" as a blacklisted target
BLACKLIST_DIST = 1.0
# Seconds before a blacklist entry expires
BLACKLIST_TTL = 120.0
# If no frontier centroid is within this distance of the current target,
# the target is considered resolved (mapped) and navigation is cancelled.
FRONTIER_VANISHED_DIST = 1.0

MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# State constants matching RobotStatus message
EXPLORING = 0
WAITING = 1
DONE = 2


class RobotFSMNode(Node):
    def __init__(self):
        super().__init__("robot_fsm_node")

        self.declare_parameter("robot_id", "robot_0")
        self.declare_parameter("status_rate", 2.0)
        self.declare_parameter("goal_timeout", 60.0)

        self._robot_id: str = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        rate: float = (
            self.get_parameter("status_rate").get_parameter_value().double_value
        )
        self._goal_timeout: float = (
            self.get_parameter("goal_timeout").get_parameter_value().double_value
        )

        # Peer list is received dynamically from /robot_ids (published by global_node).
        self._robot_ids: list[str] = []
        self._subscribed_peers: set[str] = set()

        # FSM state
        self._state = WAITING
        self._current_target: PoseStamped | None = None
        self._goal_handle = None
        self._goal_start_time: float | None = None
        self._pending_target: PoseStamped | None = None
        self._nav2_ready = False
        self._goal_seq = 0

        # Map knowledge
        self._map_cells_known = 0
        self._frontiers: list[Frontier] = []
        self._frontiers_ever_received = False
        self._merged_map: OccupancyGrid | None = None

        # Peer awareness
        self._my_pos: Point | None = None  # own position
        self._peer_poses: dict[str, Point] = {}  # peer robot positions

        # Local assignment state
        self._blacklist: list[tuple[Point, float]] = []
        self._assign_needed = False  # set True when frontiers arrive while WAITING

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, f"/{self._robot_id}/navigate_to_pose"
        )

        # Subscriptions
        self.create_subscription(
            OccupancyGrid,
            f"/{self._robot_id}/map",
            self._local_map_cb,
            MAP_QOS,
        )
        self.create_subscription(
            OccupancyGrid,
            "/merged_map",
            self._merged_map_cb,
            10,
        )
        self.create_subscription(
            FrontierArray,
            "/frontiers",
            self._frontiers_cb,
            10,
        )
        self.create_subscription(
            PoseArray,
            "/robot_poses",
            self._poses_cb,
            10,
        )
        self.create_subscription(
            String,
            "/robot_ids",
            self._robot_ids_cb,
            MAP_QOS,
        )

        # Color index derived from the numeric suffix of robot_id (robot_N → N)
        try:
            self._color_idx = int(self._robot_id.rsplit("_", 1)[-1])
        except ValueError:
            self._color_idx = 0

        # Publishers
        self._status_pub = self.create_publisher(
            RobotStatus, f"/{self._robot_id}/status", 10
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, f"/{self._robot_id}/goal", 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, f"/{self._robot_id}/goal_markers", 10
        )

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"robot_fsm_node started for {self._robot_id} (awaiting peer list)"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _local_map_cb(self, msg: OccupancyGrid) -> None:
        self._map_cells_known = int(
            np.count_nonzero(np.array(msg.data, dtype=np.int8) != -1)
        )

    def _merged_map_cb(self, msg: OccupancyGrid) -> None:
        self._merged_map = msg

    def _poses_cb(self, msg: PoseArray) -> None:
        for i, pose in enumerate(msg.poses):
            if i >= len(self._robot_ids):
                break
            rid = self._robot_ids[i]
            if rid == self._robot_id:
                self._my_pos = pose.position
            else:
                self._peer_poses[rid] = pose.position

    def _robot_ids_cb(self, msg: String) -> None:
        ids = [r.strip() for r in msg.data.split(",") if r.strip()]
        self._robot_ids = ids
        for peer_id in ids:
            if peer_id == self._robot_id or peer_id in self._subscribed_peers:
                continue
            self._subscribed_peers.add(peer_id)
            self.get_logger().info(f"[{self._robot_id}] discovered peer {peer_id}")

    def _frontiers_cb(self, msg: FrontierArray) -> None:
        self._frontiers = list(msg.frontiers)
        # Only mark "ever received" on non-empty lists so an early empty
        # message from the frontier detector (before the merged map is ready)
        # cannot trigger a premature DONE transition.
        if self._frontiers:
            self._frontiers_ever_received = True

        if (
            not self._frontiers
            and self._frontiers_ever_received
            and self._state == WAITING
        ):
            self._state = DONE
            self._clear_goal_markers()
            return

        # While navigating, check if the target frontier still exists.
        if (
            self._state == EXPLORING
            and self._current_target is not None
            and self._goal_handle is not None
        ):
            tx = self._current_target.pose.position.x
            ty = self._current_target.pose.position.y
            still_exists = any(
                math.hypot(f.centroid.x - tx, f.centroid.y - ty)
                < FRONTIER_VANISHED_DIST
                for f in self._frontiers
            )
            if not still_exists:
                self.get_logger().info(
                    f"[{self._robot_id}] Target frontier resolved by sensors, "
                    "cancelling navigation"
                )
                self._cancel_goal()
                self._state = WAITING

        # Trigger self-assignment if we're idle
        if self._state == WAITING and self._frontiers:
            self._assign_needed = True

    # ── Nav2 interaction ───────────────────────────────────────────────────────

    def _send_goal(self, target: PoseStamped) -> None:
        if not self._nav2_ready:
            self._pending_target = target
            return

        self._pending_target = None
        goal = NavigateToPose.Goal()
        goal.pose = target

        seq = self._goal_seq
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f, s=seq: self._goal_response_cb(f, s))

    def _goal_response_cb(self, future, seq: int) -> None:
        if seq != self._goal_seq:
            return
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f"[{self._robot_id}] Goal rejected by Nav2")
            self._state = WAITING
            self._goal_handle = None
            return

        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f, s=seq: self._goal_result_cb(f, s))

    def _goal_result_cb(self, future, seq: int) -> None:
        if seq != self._goal_seq:
            return
        self._goal_handle = None
        self._goal_start_time = None
        status = future.result().status

        from action_msgs.msg import GoalStatus

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{self._robot_id}] Reached frontier")
        else:
            # Blacklist the target if it still exists in the frontier list
            if self._current_target is not None:
                target_pt = self._current_target.pose.position
                now = self.get_clock().now().nanoseconds * 1e-9
                for f in self._frontiers:
                    if _dist(f.centroid, target_pt) < BLACKLIST_DIST:
                        self._blacklist.append((target_pt, now))
                        self.get_logger().info(
                            f"[{self._robot_id}] blacklisted "
                            f"({target_pt.x:.1f}, {target_pt.y:.1f})"
                        )
                        break
            self.get_logger().warn(
                f"[{self._robot_id}] Navigation ended with status={status}"
            )
        self._state = WAITING
        self._assign_needed = bool(self._frontiers)
        self._clear_goal_markers()

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ── Periodic tick ──────────────────────────────────────────────────────────

    def _tick(self) -> None:
        was_ready = self._nav2_ready
        self._nav2_ready = self._nav_client.server_is_ready()
        if not was_ready and self._nav2_ready:
            self.get_logger().info(f"[{self._robot_id}] Nav2 server is now available")

        # Retry pending target once Nav2 becomes ready
        if self._pending_target is not None and self._nav2_ready:
            self.get_logger().info(f"[{self._robot_id}] Sending queued goal")
            self._send_goal(self._pending_target)

        # Goal timeout
        if (
            self._state == EXPLORING
            and self._goal_start_time is not None
            and self._goal_handle is not None
        ):
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._goal_start_time
            if elapsed > self._goal_timeout:
                self.get_logger().warn(
                    f"[{self._robot_id}] Goal timeout after {elapsed:.1f}s, cancelling"
                )
                self._cancel_goal()
                self._state = WAITING
                self._assign_needed = bool(self._frontiers)

        # Self-assign when idle
        if self._assign_needed and self._state == WAITING and self._nav2_ready:
            self._assign_needed = False
            self._self_assign()

        if self._state == EXPLORING:
            self._publish_goal_markers()
        self._publish_status()

    # ── Local self-assignment ──────────────────────────────────────────────────

    def _self_assign(self) -> None:
        if not self._frontiers or not self._frontiers_ever_received:
            return
        if self._my_pos is None:
            return

        # Expire blacklist
        now = self.get_clock().now().nanoseconds * 1e-9
        self._blacklist = [
            (pt, t) for pt, t in self._blacklist if now - t < BLACKLIST_TTL
        ]

        # BFS from own position for ground distances
        my_dists, _ = self._bfs_ground_distances(self._my_pos)

        # Geodesic Voronoi partition
        partition = self._multi_source_bfs_partition()
        my_cells = partition.get(self._robot_id, set())

        candidates = [
            f
            for f in self._frontiers
            if not self._is_blacklisted(f.centroid)
            and _dist(self._my_pos, f.centroid) >= MIN_DISPATCH_DIST
        ]
        if not candidates:
            # Retry ignoring blacklist but still enforce minimum distance
            candidates = [
                f
                for f in self._frontiers
                if _dist(self._my_pos, f.centroid) >= MIN_DISPATCH_DIST
            ]
        if not candidates:
            return

        # Split into frontiers within our Voronoi partition and all candidates
        my_frontiers = [
            f for f in candidates
            if self._world_to_grid(f.centroid) in my_cells
        ]

        pool = my_frontiers if my_frontiers else candidates

        # Pick closest by ground distance
        best_f = min(
            pool,
            key=lambda f: my_dists.get(
                self._world_to_grid(f.centroid),
                _dist(self._my_pos, f.centroid) * 1.4,
            ),
        )

        # Build and send goal
        stamp = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header = Header(stamp=stamp, frame_id="world")
        ps.pose.position = best_f.centroid
        ps.pose.orientation.w = 1.0

        self._current_target = ps
        self._state = EXPLORING
        self._goal_start_time = self.get_clock().now().nanoseconds * 1e-9
        self._goal_seq += 1

        # Announce goal to peers before navigating
        self._goal_pub.publish(ps)

        gd = my_dists.get(
            self._world_to_grid(best_f.centroid),
            _dist(self._my_pos, best_f.centroid) * 1.4,
        )
        in_partition = "partition" if my_frontiers else "global-fallback"
        self.get_logger().info(
            f"[{self._robot_id}] self-assigned → "
            f"({best_f.centroid.x:.2f}, {best_f.centroid.y:.2f}) "
            f"gd={gd:.1f} [{in_partition}]"
        )

        if self._nav2_ready:
            self._send_goal(ps)
        else:
            self._pending_target = ps

    def _is_blacklisted(self, centroid: Point) -> bool:
        return any(_dist(pt, centroid) < BLACKLIST_DIST for pt, _ in self._blacklist)


    # ── Ground distance helpers ───────────────────────────────────────────────

    def _world_to_grid(self, point: Point) -> tuple[int, int] | None:
        m = self._merged_map
        if m is None:
            return None
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        res = m.info.resolution
        col = int((point.x - ox) / res)
        row = int((point.y - oy) / res)
        if 0 <= row < m.info.height and 0 <= col < m.info.width:
            return (row, col)
        return None

    def _bfs_ground_distances(
        self, start: Point
    ) -> tuple[dict[tuple[int, int], float], dict[tuple[int, int], tuple[int, int]]]:
        """BFS wavefront from ``start`` on the merged occupancy grid.

        Returns ``(dist, parent)`` where:
          - ``dist[cell]``   = ground distance in metres from start cell
          - ``parent[cell]`` = predecessor cell on the shortest path from start

        Both dicts are empty (not None) when the map isn't available yet, so
        callers can always treat the result as a valid pair.
        """
        m = self._merged_map
        if m is None:
            return {}, {}

        start_cell = self._world_to_grid(start)
        if start_cell is None:
            return {}, {}

        w = m.info.width
        h = m.info.height
        res = m.info.resolution
        grid = np.array(m.data, dtype=np.int8).reshape((h, w))

        passable = grid == 0
        sr, sc = start_cell
        passable[sr, sc] = True

        dist: dict[tuple[int, int], float] = {start_cell: 0.0}
        parent: dict[tuple[int, int], tuple[int, int]] = {}
        q: deque[tuple[int, int]] = deque()
        q.append(start_cell)

        neighbours = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 1.414),
            (-1, 1, 1.414),
            (1, -1, 1.414),
            (1, 1, 1.414),
        ]

        while q:
            r, c = q.popleft()
            d0 = dist[(r, c)]
            for dr, dc, cost_mul in neighbours:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and passable[nr, nc]:
                    nd = d0 + res * cost_mul
                    key = (nr, nc)
                    if key not in dist:
                        dist[key] = nd
                        parent[key] = (r, c)
                        q.append(key)

        return dist, parent

    def _multi_source_bfs_partition(self) -> dict[str, set[tuple[int, int]]]:
        """Geodesic Voronoi partition of the merged map.

        Seeds a BFS from every robot position simultaneously.  Each cell is
        claimed by whichever robot's wavefront reaches it first.  Expanding
        over both free AND unknown cells so that frontier cells (which sit on
        the free/unknown boundary) are reachable.
        """
        m = self._merged_map
        if m is None:
            return {}

        w = m.info.width
        h = m.info.height
        grid = np.array(m.data, dtype=np.int8).reshape((h, w))

        # Passable = free (0) or unknown (-1)
        passable = (grid == 0) | (grid == -1)

        # Collect all robot positions (self + peers)
        # Map each robot to an integer index for the owner array.
        seeds: list[tuple[str, tuple[int, int]]] = []
        my_cell = self._world_to_grid(self._my_pos)
        if my_cell is not None:
            seeds.append((self._robot_id, my_cell))
        for peer_id, peer_pos in self._peer_poses.items():
            cell = self._world_to_grid(peer_pos)
            if cell is not None:
                seeds.append((peer_id, cell))

        if not seeds:
            return {}

        rid_to_idx = {rid: i + 1 for i, (rid, _) in enumerate(seeds)}

        # owner array: 0 = unclaimed, positive int = robot index
        owner = np.zeros((h, w), dtype=np.int16)
        q: deque[tuple[int, int]] = deque()

        for rid, (sr, sc) in seeds:
            if owner[sr, sc] == 0:
                owner[sr, sc] = rid_to_idx[rid]
                q.append((sr, sc))

        neighbours = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        while q:
            r, c = q.popleft()
            oid = owner[r, c]
            for dr, dc in neighbours:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and passable[nr, nc] and owner[nr, nc] == 0:
                    owner[nr, nc] = oid
                    q.append((nr, nc))

        # Build partition dict using numpy indexing per robot
        partition: dict[str, set[tuple[int, int]]] = {}
        for rid, _ in seeds:
            idx = rid_to_idx[rid]
            rows, cols = np.where(owner == idx)
            partition[rid] = set(zip(rows.tolist(), cols.tolist()))

        return partition

    # ── Status and visualization publishing ───────────────────────────────────

    def _publish_status(self) -> None:
        msg = RobotStatus()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
        msg.robot_id = self._robot_id
        msg.state = self._state
        msg.map_cells_known = self._map_cells_known
        msg.frontiers_remaining = len(self._frontiers)
        self._status_pub.publish(msg)

    # Per-robot colors (matches coordinator palette that was removed)
    _COLORS = [
        ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.9),  # orange  — robot_0
        ColorRGBA(r=0.0, g=0.4, b=1.0, a=0.9),  # blue    — robot_1
        ColorRGBA(r=1.0, g=0.0, b=0.8, a=0.9),  # magenta — robot_2
        ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.9),  # green   — robot_3
    ]

    def _publish_goal_markers(self) -> None:
        """Arrow from robot → goal, sphere at goal.

        Published on /{robot}/goal_markers.
        """
        if self._current_target is None or self._my_pos is None:
            return

        stamp = self.get_clock().now().to_msg()
        color = self._COLORS[self._color_idx % len(self._COLORS)]
        goal_pt = self._current_target.pose.position

        arrow = Marker()
        arrow.header = Header(stamp=stamp, frame_id="world")
        arrow.ns = "goal"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=self._my_pos.x, y=self._my_pos.y, z=0.1),
            Point(x=goal_pt.x, y=goal_pt.y, z=0.1),
        ]
        arrow.scale.x = 0.05
        arrow.scale.y = 0.1
        arrow.scale.z = 0.1
        arrow.color = color
        arrow.lifetime.sec = 2

        sphere = Marker()
        sphere.header = Header(stamp=stamp, frame_id="world")
        sphere.ns = "goal"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = goal_pt
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.2
        sphere.scale.y = 0.2
        sphere.scale.z = 0.2
        sphere.color = color
        sphere.lifetime.sec = 2

        ma = MarkerArray()
        ma.markers = [arrow, sphere]
        self._marker_pub.publish(ma)

    def _clear_goal_markers(self) -> None:
        ma = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers = [delete_all]
        self._marker_pub.publish(ma)


def _dist(a: Point, b: Point) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = RobotFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

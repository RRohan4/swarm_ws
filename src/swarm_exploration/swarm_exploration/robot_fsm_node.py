"""
robot_fsm_node.py

Per-robot finite-state machine that drives autonomous exploration.

States:
  WAITING    (1) — no target assigned; idling
  EXPLORING  (0) — navigating to an assigned frontier via Nav2 NavigateToPose
  DONE       (2) — no more frontiers (coordinator signals via empty target)

Subscriptions:
  /robot_N/exploration_target  (geometry_msgs/PoseStamped)
  /robot_N/map                 (nav_msgs/OccupancyGrid) — for map_cells_known
  /frontiers                   (swarm_msgs/FrontierArray)  — for frontiers_remaining

Publications:
  /robot_N/status  (swarm_msgs/RobotStatus)

Parameters:
  robot_id        : str   (default "robot_0")
  status_rate     : float Hz (default 2.0)
  goal_timeout    : float seconds before aborting a goal (default 60.0)
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Header

from swarm_msgs.msg import FrontierArray, RobotStatus

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

        self._state = WAITING
        self._current_target: PoseStamped | None = None
        self._goal_handle = None
        self._goal_start_time: float | None = None
        self._map_cells_known = 0
        self._frontiers_remaining = 0
        self._latest_pose_stamped: PoseStamped | None = None

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, f"/{self._robot_id}/navigate_to_pose"
        )

        # Subscriptions
        self.create_subscription(
            PoseStamped,
            f"/{self._robot_id}/exploration_target",
            self._target_cb,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            f"/{self._robot_id}/map",
            self._map_cb,
            MAP_QOS,
        )
        self.create_subscription(
            FrontierArray,
            "/frontiers",
            self._frontiers_cb,
            10,
        )

        # Publisher
        self._status_pub = self.create_publisher(
            RobotStatus, f"/{self._robot_id}/status", 10
        )

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"robot_fsm_node started for {self._robot_id}")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _target_cb(self, msg: PoseStamped) -> None:
        """New target from coordinator. Empty position (0,0,0) means DONE."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        if x == 0.0 and y == 0.0 and msg.header.frame_id == "done":
            self._transition_to_done()
            return

        # Cancel any in-flight goal
        if self._goal_handle is not None:
            self._cancel_goal()

        self._current_target = msg
        self._state = EXPLORING
        self._goal_start_time = self.get_clock().now().nanoseconds * 1e-9
        self._send_goal(msg)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        data = msg.data
        self._map_cells_known = sum(1 for c in data if c != -1)

    def _frontiers_cb(self, msg: FrontierArray) -> None:
        self._frontiers_remaining = len(msg.frontiers)
        if self._frontiers_remaining == 0 and self._state == WAITING:
            self._state = DONE

    # ── Nav2 interaction ───────────────────────────────────────────────────────

    def _send_goal(self, target: PoseStamped) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"[{self._robot_id}] "
                "NavigateToPose server not available, staying WAITING"
            )
            self._state = WAITING
            return

        goal = NavigateToPose.Goal()
        goal.pose = target

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f"[{self._robot_id}] Goal rejected by Nav2")
            self._state = WAITING
            self._goal_handle = None
            return

        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        self._goal_handle = None
        self._goal_start_time = None
        status = future.result().status

        from action_msgs.msg import GoalStatus

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{self._robot_id}] Reached frontier")
        else:
            self.get_logger().warn(
                f"[{self._robot_id}] Navigation ended with status={status}"
            )
        self._state = WAITING

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ── Periodic tick ──────────────────────────────────────────────────────────

    def _tick(self) -> None:
        # Check for goal timeout while exploring
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

        self._publish_status()

    def _publish_status(self) -> None:
        msg = RobotStatus()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
        msg.robot_id = self._robot_id
        msg.state = self._state
        msg.map_cells_known = self._map_cells_known
        msg.frontiers_remaining = self._frontiers_remaining
        self._status_pub.publish(msg)

    def _transition_to_done(self) -> None:
        self._cancel_goal()
        self._state = DONE
        self.get_logger().info(f"[{self._robot_id}] Exploration complete (DONE)")


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

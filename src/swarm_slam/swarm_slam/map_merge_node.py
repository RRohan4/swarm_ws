"""
map_merge_node.py

Global singleton. Subscribes to /robot_N/map (OccupancyGrid) for each robot,
merges them with max-confidence voting (unknown < free < occupied), and
publishes /merged_map.  Also tracks latest odom poses and publishes /robot_poses.

Parameters:
  robot_ids       : list[str]  e.g. ["robot_0", "robot_1"]
  map_merge_rate  : float      publish rate Hz (default 2.0)
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Header

# QoS that matches slam_toolbox's latched map topic
MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class MapMergeNode(Node):
    def __init__(self):
        super().__init__("map_merge_node")

        self.declare_parameter("robot_ids", ["robot_0", "robot_1"])
        self.declare_parameter("map_merge_rate", 2.0)

        robot_ids: list[str] = (
            self.get_parameter("robot_ids").get_parameter_value().string_array_value
        )
        rate: float = (
            self.get_parameter("map_merge_rate").get_parameter_value().double_value
        )

        self._maps: dict[str, OccupancyGrid] = {}
        self._poses: dict[str, Pose] = {}

        # Subscribe to each robot's map and odom
        for rid in robot_ids:
            self.create_subscription(
                OccupancyGrid,
                f"/{rid}/map",
                lambda msg, r=rid: self._map_cb(r, msg),
                MAP_QOS,
            )
            self.create_subscription(
                Odometry,
                f"/{rid}/odom",
                lambda msg, r=rid: self._odom_cb(r, msg),
                10,
            )

        self._merged_pub = self.create_publisher(OccupancyGrid, "/merged_map", MAP_QOS)
        self._poses_pub = self.create_publisher(PoseArray, "/robot_poses", 10)

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f"map_merge_node started — tracking {robot_ids}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _map_cb(self, robot_id: str, msg: OccupancyGrid) -> None:
        self._maps[robot_id] = msg

    def _odom_cb(self, robot_id: str, msg: Odometry) -> None:
        self._poses[robot_id] = msg.pose.pose

    # ── Merge and publish ──────────────────────────────────────────────────────

    def _publish(self) -> None:
        if not self._maps:
            return

        merged = self._merge_maps(list(self._maps.values()))
        if merged is not None:
            self._merged_pub.publish(merged)

        if self._poses:
            pa = PoseArray()
            pa.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
            pa.poses = list(self._poses.values())
            self._poses_pub.publish(pa)

    def _merge_maps(self, maps: list[OccupancyGrid]) -> OccupancyGrid | None:
        """Max-confidence merge: unknown(-1) < free(0) < occupied(100).

        All maps are assumed to share the same origin and resolution (robots
        spawn at the same world location and SLAM uses an identity world→map TF).
        We expand the bounding box to cover all maps, then overlay in order.
        """
        if not maps:
            return None

        ref = maps[0]
        res = ref.info.resolution
        # Compute bounding box across all maps in grid coords
        max_col = max_row = 0
        for m in maps:
            max_col = max(max_col, m.info.width)
            max_row = max(max_row, m.info.height)

        # Start with all-unknown
        merged = np.full((max_row, max_col), -1, dtype=np.int8)

        for m in maps:
            arr = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            h = min(m.info.height, max_row)
            w = min(m.info.width, max_col)
            sub = arr[:h, :w]
            dst = merged[:h, :w]

            # unknown stays as-is; free overwrites unknown; occupied overwrites both
            known_mask = sub != -1
            dst[known_mask & (dst == -1)] = sub[known_mask & (dst == -1)]
            occupied_mask = sub == 100
            dst[occupied_mask] = 100
            merged[:h, :w] = dst

        out = OccupancyGrid()
        out.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="world",
        )
        out.info = MapMetaData(
            resolution=res,
            width=max_col,
            height=max_row,
            origin=ref.info.origin,
        )
        out.data = merged.flatten().tolist()
        return out


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

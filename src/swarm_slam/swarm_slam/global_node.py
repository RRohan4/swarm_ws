"""
global_node.py

Global singleton. Subscribes to /robot_N/map (OccupancyGrid) for each robot,
transforms each map into the world frame using TF, merges with max-confidence
voting (unknown < free < occupied), and publishes /merged_map.
Also tracks latest odom poses and publishes /robot_poses.

Parameters:
  robot_ids  : list[str]  e.g. ["robot_0", "robot_1"]
  rate       : float      publish rate Hz (default 2.0)
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

# QoS that matches slam_toolbox's latched map topic
MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _yaw_from_quat(q) -> float:
    """Extract yaw from a quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class GlobalNode(Node):
    def __init__(self):
        super().__init__("global_node")

        self.declare_parameter("robot_ids", ["robot_0", "robot_1"])
        self.declare_parameter("rate", 2.0)

        robot_ids: list[str] = (
            self.get_parameter("robot_ids").get_parameter_value().string_array_value
        )
        rate: float = self.get_parameter("rate").get_parameter_value().double_value

        self._robot_ids = robot_ids
        self._maps: dict[str, OccupancyGrid] = {}
        self._poses: dict[str, Pose] = {}

        # TF2 for looking up world → robot_N/map transforms
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

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
        self._ids_pub = self.create_publisher(String, "/robot_ids", MAP_QOS)

        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f"global_node started — tracking {robot_ids}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _map_cb(self, robot_id: str, msg: OccupancyGrid) -> None:
        self._maps[robot_id] = msg

    def _odom_cb(self, robot_id: str, msg: Odometry) -> None:
        # Store raw odom pose (in robot_N/odom frame); transformed to world in _publish.
        self._poses[robot_id] = msg.pose.pose

    # ── TF helpers ────────────────────────────────────────────────────────────

    def _get_world_transform(self, frame: str):
        """Look up static TF world → frame. Returns (tx, ty, yaw) or None."""
        try:
            tf: TransformStamped = self._tf_buf.lookup_transform(
                "world", frame, rclpy.time.Time()
            )
            t = tf.transform.translation
            yaw = _yaw_from_quat(tf.transform.rotation)
            return (t.x, t.y, yaw)
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed for {frame} → world: {e}",
                throttle_duration_sec=10.0,
            )
            return None

    # ── Merge and publish ──────────────────────────────────────────────────────

    def _publish(self) -> None:
        if not self._maps:
            return

        merged = self._merge_maps()
        if merged is not None:
            self._merged_pub.publish(merged)

        # Broadcast robot ID list so FSM nodes can discover peers dynamically.
        self._ids_pub.publish(String(data=",".join(self._robot_ids)))

        # Publish robot poses in world frame, in robot_ids order.
        world_poses: list[Pose] = []
        for rid in self._robot_ids:
            if rid not in self._poses:
                continue
            raw = self._poses[rid]
            # Transform odom-frame position into world frame via TF.
            try:
                tf: TransformStamped = self._tf_buf.lookup_transform(
                    "world", f"{rid}/odom", rclpy.time.Time()
                )
                t = tf.transform.translation
                yaw = _yaw_from_quat(tf.transform.rotation)
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                ox = raw.position.x
                oy = raw.position.y
                p = Pose()
                p.position.x = t.x + cos_y * ox - sin_y * oy
                p.position.y = t.y + sin_y * ox + cos_y * oy
                p.position.z = raw.position.z
                p.orientation = raw.orientation
                world_poses.append(p)
            except Exception as e:
                self.get_logger().warn(
                    f"TF world→{rid}/odom unavailable: {e}",
                    throttle_duration_sec=10.0,
                )
                world_poses.append(raw)  # fall back to raw odom if TF not ready
        if world_poses:
            pa = PoseArray()
            pa.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
            pa.poses = world_poses
            self._poses_pub.publish(pa)

    def _merge_maps(self) -> OccupancyGrid | None:
        """Transform each robot's map into world coords, then merge."""
        if not self._maps:
            return None

        res = list(self._maps.values())[0].info.resolution

        # Collect transformed bounding boxes in world coordinates
        map_data = []  # list of (array, world_origin_x, world_origin_y)
        for rid, m in self._maps.items():
            tf = self._get_world_transform(f"{rid}/map")
            if tf is None:
                continue
            wx, wy, wyaw = tf

            # Map origin in the map's own frame
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            oyaw = _yaw_from_quat(m.info.origin.orientation)

            # Combined rotation: world_yaw + map_origin_yaw
            total_yaw = wyaw + oyaw
            cos_y = math.cos(wyaw)
            sin_y = math.sin(wyaw)

            # Transform map origin to world frame
            world_ox = wx + cos_y * ox - sin_y * oy
            world_oy = wy + sin_y * ox + cos_y * oy

            arr = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            map_data.append((arr, world_ox, world_oy, total_yaw, res))

        if not map_data:
            return None

        # Compute world-frame bounding box across all maps
        min_wx = float("inf")
        min_wy = float("inf")
        max_wx = float("-inf")
        max_wy = float("-inf")

        for arr, world_ox, world_oy, total_yaw, r in map_data:
            h, w = arr.shape
            cos_t = math.cos(total_yaw)
            sin_t = math.sin(total_yaw)
            # Four corners of the map in world coords
            corners_local = [(0, 0), (w * r, 0), (0, h * r), (w * r, h * r)]
            for cx, cy in corners_local:
                wx = world_ox + cos_t * cx - sin_t * cy
                wy = world_oy + sin_t * cx + cos_t * cy
                min_wx = min(min_wx, wx)
                min_wy = min(min_wy, wy)
                max_wx = max(max_wx, wx)
                max_wy = max(max_wy, wy)

        # Snap to grid
        min_wx = math.floor(min_wx / res) * res
        min_wy = math.floor(min_wy / res) * res

        merged_w = int(math.ceil((max_wx - min_wx) / res))
        merged_h = int(math.ceil((max_wy - min_wy) / res))

        if merged_w <= 0 or merged_h <= 0:
            return None

        merged = np.full((merged_h, merged_w), -1, dtype=np.int8)

        # Place each map into the merged grid
        for arr, world_ox, world_oy, total_yaw, r in map_data:
            h, w = arr.shape
            cos_t = math.cos(total_yaw)
            sin_t = math.sin(total_yaw)

            # For each cell in the source map, compute its merged grid position
            rows, cols = np.mgrid[0:h, 0:w]
            # Position in map's local frame (from map origin)
            local_x = cols * r + r * 0.5  # cell center
            local_y = rows * r + r * 0.5

            # Rotate + translate to world frame
            wx = world_ox + cos_t * local_x - sin_t * local_y
            wy = world_oy + sin_t * local_x + cos_t * local_y

            # Convert to merged grid indices
            mi = ((wy - min_wy) / res).astype(int)
            mj = ((wx - min_wx) / res).astype(int)

            # Clip to bounds
            valid = (mi >= 0) & (mi < merged_h) & (mj >= 0) & (mj < merged_w)

            src = arr[valid]
            di = mi[valid]
            dj = mj[valid]

            # Merge priority: free > occupied > unknown.
            # Occupied only fills unknown cells — it cannot override confirmed
            # free space.  Free space is authoritative and overwrites occupied,
            # so ghost obstacles left by moving robots are cleared as soon as
            # any robot's raytrace sweeps through the vacated area.
            # Real walls are never marked free (no ray penetrates them), so
            # they accumulate occupied votes from multiple robots and remain.
            occupied = src == 100
            dst_unknown = merged[di[occupied], dj[occupied]] == -1
            occ_idx = np.where(occupied)[0]
            merged[di[occ_idx[dst_unknown]], dj[occ_idx[dst_unknown]]] = 100

            free = src == 0
            merged[di[free], dj[free]] = 0

        out = OccupancyGrid()
        out.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="world",
        )
        origin = Pose()
        origin.position.x = min_wx
        origin.position.y = min_wy
        out.info = MapMetaData(
            resolution=res,
            width=merged_w,
            height=merged_h,
            origin=origin,
        )
        out.data = merged.flatten().tolist()
        return out


def main(args=None):
    rclpy.init(args=args)
    node = GlobalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

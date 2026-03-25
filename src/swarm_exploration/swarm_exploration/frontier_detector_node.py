"""
frontier_detector_node.py

Subscribes to /merged_map (OccupancyGrid), detects frontier cells (free cells
adjacent to unknown cells), clusters them with connected-component labelling,
and publishes:
  /frontiers        (swarm_msgs/FrontierArray)
  /frontier_markers (visualization_msgs/MarkerArray)

Parameters:
  min_frontier_size : int    minimum cluster size to publish (default 5)
  detect_rate       : float  Hz (default 2.0)
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from scipy import ndimage
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from swarm_msgs.msg import Frontier, FrontierArray

MAP_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class FrontierDetectorNode(Node):
    def __init__(self):
        super().__init__("frontier_detector_node")

        self.declare_parameter("min_frontier_size", 5)
        self.declare_parameter("detect_rate", 5.0)

        self._min_size: int = (
            self.get_parameter("min_frontier_size").get_parameter_value().integer_value
        )
        rate: float = (
            self.get_parameter("detect_rate").get_parameter_value().double_value
        )

        self._latest_map: OccupancyGrid | None = None

        self.create_subscription(OccupancyGrid, "/merged_map", self._map_cb, MAP_QOS)

        self._frontier_pub = self.create_publisher(FrontierArray, "/frontiers", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/frontier_markers", 10)

        self.create_timer(1.0 / rate, self._detect)
        self.get_logger().info("frontier_detector_node started")

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _detect(self) -> None:
        if self._latest_map is None:
            return

        msg = self._latest_map
        info = msg.info
        res = info.resolution
        w, h = info.width, info.height
        ox = info.origin.position.x
        oy = info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)

        # Free cells (0) adjacent to unknown cells (-1) are frontiers
        free = grid == 0
        unknown = grid == -1
        occupied = grid > 50  # occupied cells (typically 100)

        # Exclude free cells near obstacles — prevents frontiers in tight corners
        # that the robot cannot physically reach without clipping walls.
        obstacle_buffer = ndimage.binary_dilation(
            occupied,
            structure=np.ones((3, 3)),  # ~1 cell clearance at 0.05m res
        )
        safe_free = free & ~obstacle_buffer

        # Dilate unknown mask by 1 cell; intersection with safe free gives frontiers
        unknown_dilated = ndimage.binary_dilation(unknown, structure=np.ones((3, 3)))
        frontier_mask = safe_free & unknown_dilated

        # Label connected components
        labeled, num_features = ndimage.label(frontier_mask)

        frontiers: list[Frontier] = []
        for label_id in range(1, num_features + 1):
            cells = np.argwhere(labeled == label_id)  # (row, col)
            if len(cells) < self._min_size:
                continue

            # Centroid in world frame
            mean_row = cells[:, 0].mean()
            mean_col = cells[:, 1].mean()
            cx = ox + (mean_col + 0.5) * res
            cy = oy + (mean_row + 0.5) * res

            f = Frontier()
            f.centroid = Point(x=cx, y=cy, z=0.0)
            f.cluster_size = len(cells)
            f.info_score = 0.0  # filled by coordinator
            f.assigned_to = ""
            frontiers.append(f)

        stamp = self.get_clock().now().to_msg()

        fa = FrontierArray()
        fa.header = Header(stamp=stamp, frame_id="world")
        fa.frontiers = frontiers
        self._frontier_pub.publish(fa)

        self._publish_markers(frontiers, stamp)

    def _publish_markers(self, frontiers: list[Frontier], stamp) -> None:
        ma = MarkerArray()

        # Delete all previous markers first
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=0.8)
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header = Header(stamp=stamp, frame_id="world")
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position = f.centroid
            m.pose.orientation.w = 1.0
            # Scale cylinder height by cluster size (capped)
            height = min(0.05 + f.cluster_size * 0.005, 0.5)
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = height
            m.color = color
            m.lifetime.sec = 2
            ma.markers.append(m)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

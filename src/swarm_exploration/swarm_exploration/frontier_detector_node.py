"""
frontier_detector_node.py

Subscribes to /merged_map (OccupancyGrid), detects frontier cells (free cells
adjacent to unknown cells), clusters them with connected-component labelling,
and publishes:
  /frontiers        (swarm_msgs/FrontierArray)
  /frontier_markers (visualization_msgs/MarkerArray)

Parameters:
  min_frontier_size  : int    minimum cluster size to publish (default 5)
  detect_rate        : float  Hz (default 2.0)
  merge_distance     : float  merge centroids within this distance metres (default 0.3)
  min_unknown_backing: int    minimum unique unknown neighbours per cluster (default 10)
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
        self.declare_parameter("merge_distance", 0.3)
        self.declare_parameter("min_unknown_backing", 10)

        self._min_size: int = (
            self.get_parameter("min_frontier_size").get_parameter_value().integer_value
        )
        rate: float = (
            self.get_parameter("detect_rate").get_parameter_value().double_value
        )
        self._merge_distance: float = (
            self.get_parameter("merge_distance").get_parameter_value().double_value
        )
        self._min_unknown_backing: int = (
            self.get_parameter("min_unknown_backing")
            .get_parameter_value()
            .integer_value
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

        # Exclude free cells too close to obstacles.  The clearance must match
        # Nav2's inflation_radius (0.55 m) so that frontier centroids always
        # land outside the inflated costmap — otherwise the planner sees the
        # goal as inside a wall and fails to produce a path.
        clearance_cells = max(1, round(0.55 / res))
        obstacle_buffer = ndimage.binary_dilation(
            occupied,
            structure=np.ones((3, 3)),
            iterations=clearance_cells,
        )
        safe_free = free & ~obstacle_buffer

        # Dilate unknown mask by 1 cell; intersection with safe free gives frontiers
        unknown_dilated = ndimage.binary_dilation(unknown, structure=np.ones((3, 3)))
        frontier_mask = safe_free & unknown_dilated

        # Strip cells within EDGE_MARGIN pixels of the map boundary.
        # Without this, frontier centroids land at the costmap edge, causing
        # the Nav2 planner to try inflating obstacles off the grid
        # ("worldToMap failed") and fail to make progress.
        EDGE_MARGIN = 4
        frontier_mask[:EDGE_MARGIN, :] = False
        frontier_mask[-EDGE_MARGIN:, :] = False
        frontier_mask[:, :EDGE_MARGIN] = False
        frontier_mask[:, -EDGE_MARGIN:] = False

        # Label connected components
        labeled, num_features = ndimage.label(frontier_mask)

        frontiers: list[Frontier] = []
        if num_features > 0:
            # Collect all frontier cell positions in one pass — avoids the
            # O(K×N) cost of calling np.argwhere(labeled == id) per cluster.
            rows, cols = np.where(frontier_mask)
            cell_labels = labeled[rows, cols]

            # Sort once by label so clusters form contiguous slices.
            order = np.argsort(cell_labels, kind="stable")
            rows = rows[order]
            cols = cols[order]
            cell_labels = cell_labels[order]

            boundaries = np.flatnonzero(np.diff(cell_labels)) + 1
            groups_r = np.split(rows, boundaries)
            groups_c = np.split(cols, boundaries)

            for cells_r, cells_c in zip(groups_r, groups_c):
                if len(cells_r) < self._min_size:
                    continue

                # Vectorised unknown-neighbor count (replaces nested Python loop)
                nr = np.concatenate([cells_r - 1, cells_r + 1, cells_r, cells_r])
                nc = np.concatenate([cells_c, cells_c, cells_c - 1, cells_c + 1])
                in_bounds = (nr >= 0) & (nr < h) & (nc >= 0) & (nc < w)
                nr, nc = nr[in_bounds], nc[in_bounds]
                is_unk = unknown[nr, nc]
                unique_unk = (
                    len(np.unique(np.column_stack([nr[is_unk], nc[is_unk]]), axis=0))
                    if is_unk.any()
                    else 0
                )
                if unique_unk < self._min_unknown_backing:
                    continue

                # Snap centroid to nearest frontier cell (medoid)
                mean_r = cells_r.mean()
                mean_c = cells_c.mean()
                best = np.argmin((cells_r - mean_r) ** 2 + (cells_c - mean_c) ** 2)
                cx = ox + (cells_c[best] + 0.5) * res
                cy = oy + (cells_r[best] + 0.5) * res

                f = Frontier()
                f.centroid = Point(x=cx, y=cy, z=0.0)
                f.cluster_size = len(cells_r)
                f.info_score = 0.0  # filled by coordinator
                f.assigned_to = ""
                frontiers.append(f)

        frontiers = self._merge_nearby(frontiers)

        stamp = self.get_clock().now().to_msg()

        fa = FrontierArray()
        fa.header = Header(stamp=stamp, frame_id="world")
        fa.frontiers = frontiers
        self._frontier_pub.publish(fa)

        self._publish_markers(frontiers, stamp)

    def _merge_nearby(self, frontiers: list[Frontier]) -> list[Frontier]:
        """Greedily merge frontiers whose centroids are within merge_distance."""
        if not frontiers:
            return frontiers
        # Sort largest first so they become anchors
        frontiers.sort(key=lambda f: f.cluster_size, reverse=True)
        merged: list[Frontier] = []
        for f in frontiers:
            absorbed = False
            for m in merged:
                dx = f.centroid.x - m.centroid.x
                dy = f.centroid.y - m.centroid.y
                if dx * dx + dy * dy <= self._merge_distance**2:
                    # Absorb into existing — keep larger's centroid, sum sizes
                    m.cluster_size += f.cluster_size
                    absorbed = True
                    break
            if not absorbed:
                merged.append(f)
        return merged

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

# Swarm Maze-Solving Architecture

## Package Structure

```
swarm_ws/
├── Dockerfile
├── compose.yaml
└── src/
    ├── swarm_msgs/          # ament_cmake — custom ROS interfaces only (no executables)
    ├── swarm_slam/          # ament_python — per-robot SLAM wrapper + global map merge
    ├── swarm_exploration/   # ament_python — frontier detector, coordinator, robot FSM
    └── swarm_bringup/       # ament_python — launch files, worlds, configs
```

**Dropped from earlier design:** `swarm_radio` (comm simulation), `swarm_comms` (FEC/ARQ/heartbeat),
`swarm_perception` (pseudo-3D elevation), `swarm_coordinator` (distributed peer consensus),
`ViewpointClaim` protocol. All robots communicate freely via shared ROS 2 topics — no radio
god node, no comm radius, no packet loss simulation.

---

## Custom Messages — `swarm_msgs`

**`RobotStatus.msg`**
```
std_msgs/Header header
string               robot_id
uint8                state          # EXPLORING=0  WAITING=1  DONE=2
geometry_msgs/Pose   pose
uint32               map_cells_known
uint32               frontiers_remaining
```

**`Frontier.msg`**
```
geometry_msgs/Point  centroid       # world frame
uint32               cluster_size   # number of frontier pixels in cluster
float32              info_score     # cluster_size / path_cost (filled by coordinator)
string               assigned_to    # robot_id or "" if unassigned
```

**`FrontierArray.msg`**
```
std_msgs/Header header
swarm_msgs/Frontier[] frontiers
```

---

## Node Graph

### `map_merge_node` — `swarm_slam` (global namespace, singleton)

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/map` | `nav_msgs/OccupancyGrid` — one per robot (dynamic) |
| Sub | `/robot_N/odom` | `nav_msgs/Odometry` — one per robot (dynamic) |
| Pub | `/merged_map` | `nav_msgs/OccupancyGrid` — max-confidence merge |
| Pub | `/robot_poses` | `geometry_msgs/PoseArray` — all robot positions |

Merge rule: unknown(−1) < free(0) < occupied(100). All robots spawn at the same world origin
so map frames are co-located — direct numpy overlay, no ICP required.

---

### `frontier_detector_node` — `swarm_exploration` (global namespace, singleton)

| | Topic | Type |
|---|---|---|
| Sub | `/merged_map` | `nav_msgs/OccupancyGrid` |
| Pub | `/frontiers` | `swarm_msgs/FrontierArray` |
| Pub | `/frontier_markers` | `visualization_msgs/MarkerArray` |

**Algorithm:**

```python
import numpy as np
from scipy import ndimage

def detect_frontiers(grid, map_info):
    arr = np.array(grid.data).reshape(grid.info.height, grid.info.width)
    free    = (arr == 0)
    unknown = (arr == -1)
    # Frontier = free cell adjacent to at least one unknown cell
    kernel = np.array([[0,1,0],[1,0,1],[0,1,0]])
    unknown_neighbors = ndimage.convolve(unknown.astype(int), kernel, mode='constant') > 0
    frontier_cells = free & unknown_neighbors
    # Cluster (4-connectivity) and filter small clusters
    labeled, n = ndimage.label(frontier_cells)
    clusters = []
    for i in range(1, n + 1):
        cells = np.argwhere(labeled == i)
        if len(cells) < 5:
            continue
        r, c = cells.mean(axis=0)
        cx = map_info.origin.position.x + c * map_info.resolution
        cy = map_info.origin.position.y + r * map_info.resolution
        clusters.append({'centroid': (cx, cy), 'size': len(cells)})
    return clusters
```

---

### `coordinator_node` — `swarm_exploration` (global namespace, singleton)

| | Topic | Type |
|---|---|---|
| Sub | `/frontiers` | `swarm_msgs/FrontierArray` |
| Sub | `/robot_poses` | `geometry_msgs/PoseArray` |
| Sub | `/robot_N/status` | `swarm_msgs/RobotStatus` — one per robot |
| Pub | `/robot_N/exploration_target` | `geometry_msgs/PoseStamped` — one per robot |
| Pub | `/assignment_markers` | `visualization_msgs/MarkerArray` |

Runs at **1 Hz** (and on any robot status change). Greedy submodular frontier assignment:

```python
def assign(frontiers, robot_poses):
    assignments = {}
    claimed = set()

    for robot_id in sorted(robot_poses.keys()):
        best_frontier, best_score = None, -1
        for f in frontiers:
            if f in claimed:
                continue
            path_cost = get_nav2_path_cost(robot_poses[robot_id], f.centroid)
            score = f.cluster_size / max(path_cost, 0.1)
            if score > best_score:
                best_score, best_frontier = score, f
        if best_frontier:
            assignments[robot_id] = best_frontier
            claimed.add(best_frontier)

    return assignments
```

Nav2 path cost: call `/robot_N/compute_path_to_pose` (ComputePathToPose action) to get real
path length. Falls back to Euclidean distance if Nav2 is not ready.

---

### `robot_fsm_node` — `swarm_exploration` (per robot, namespace `/robot_N`)

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/exploration_target` | `geometry_msgs/PoseStamped` |
| Sub | `/robot_N/map` | `nav_msgs/OccupancyGrid` |
| Pub | `/robot_N/status` | `swarm_msgs/RobotStatus` — published at 2 Hz |
| Action | `/robot_N/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

**State machine:**

```
WAITING   → EXPLORING : new target received from coordinator
EXPLORING → WAITING   : Nav2 goal succeeded  OR  local frontier exhausted
EXPLORING → WAITING   : Nav2 goal failed (obstacle / unreachable) — coordinator reassigns
WAITING   → DONE      : coordinator publishes empty FrontierArray
```

---

### Nav2 stack — per robot, namespace `/robot_N`

Standard Nav2 lifecycle nodes. Global and local costmaps consume `/robot_N/map`.
Controller publishes `/robot_N/cmd_vel`.

---

## Gazebo World

**`swarm_bringup/worlds/maze_world.sdf`** — pre-generated 12×12 m maze arena.

Robot spawns (both inside maze entrance, one grid cell apart):

| Robot | x | y | z | yaw |
|---|---|---|---|---|
| robot_0 | 0.6 | 0.6 | 0.05 | 1.5708 |
| robot_1 | 1.8 | 0.6 | 0.05 | 1.5708 |

All Gazebo plugin topics namespaced per robot: `/robot_N/cmd_vel`, `/robot_N/odom`,
`/robot_N/scan`, `/robot_N/tf`.

**TF frames:** `map_frame: robot_N/map` per SLAM instance. A static
`world → robot_N/map` identity transform is published at startup so all robots share
a common reference frame without ICP.

---

## Launch Architecture — `swarm_bringup`

```
launch/
  swarm.launch.py           # top-level: Gazebo + N × robot_stack + global nodes
  robot_stack.launch.py     # per-robot: RSP + gz_bridge + SLAM lifecycle + Nav2
  single_robot.launch.py    # kept for single-robot regression testing

config/
  nav2_params.yaml          # Nav2 config (robot_N namespace via LaunchConfiguration)
  slam_params.yaml          # slam_toolbox per-robot config
```

The Gazebo bridge config is generated programmatically per robot in `robot_stack.launch.py`
so that `/model/robot_N/cmd_vel` maps to `/robot_N/cmd_vel`.

---

## Key Architectural Trade-offs

| Decision | Choice | Rationale |
|---|---|---|
| Communication | Global ROS 2 topics, no radio simulation | Eliminates comm complexity; focus on exploration strategy |
| Exploration strategy | Greedy submodular frontier assignment | ≥ (1−1/e) ≈ 63 % of optimal coverage; simple to implement and reason about |
| Coordination | Central `coordinator_node` | Single point of assignment is correct and deterministic; resilience not required for this scope |
| Map sharing | Direct `/robot_N/map` subscriptions, numpy overlay | No compression overhead; bandwidth is unconstrained in sim |
| Frontier scoring | `cluster_size / nav2_path_cost` | Balances information gain against travel cost without raycasting |
| Frontier detection | `scipy.ndimage` label on merged map | Fast, dependency-free, well-tested |
| TF alignment | Identity static transform at spawn | Robots start at known offsets; no ICP needed |

---

## Node Graph Summary

```
Gazebo Sim
  /robot_N/scan, /robot_N/odom, /robot_N/tf
        │
        ▼
  slam_toolbox (per robot)
  /robot_N/map
        │
        ▼
  map_merge_node ──────────────────────────────────┐
  /merged_map                /robot_poses           │
        │                         │                 │
        ▼                         ▼                 │
  frontier_detector_node    coordinator_node ◄──────┘
  /frontiers                  │  /robot_N/status (from FSM)
  /frontier_markers           │
                              │  /robot_N/exploration_target
                              ▼
                        robot_fsm_node (per robot)
                          Nav2 NavigateToPose
                          /robot_N/cmd_vel
```

---

## Docker / Containerization

```
swarm_ws/
├── Dockerfile      # single image: all nodes
└── compose.yaml    # one service per robot + sim
```

### Dockerfile

```dockerfile
FROM osrf/ros:jazzy-desktop AS base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz-bridge gz-harmonic \
    python3-pip && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir numpy scipy

WORKDIR /ws
COPY src/ src/
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install
```

### compose.yaml

```yaml
services:

  sim:
    build: .
    network_mode: host
    command: >
      bash -c ". /ws/install/setup.sh &&
               ros2 launch swarm_bringup swarm.launch.py"

  robot_0:
    build: .
    network_mode: host
    environment: {ROBOT_ID: robot_0, ROBOT_INIT_X: "0.6", ROBOT_INIT_Y: "0.6"}
    command: >
      bash -c ". /ws/install/setup.sh &&
               ros2 launch swarm_bringup robot_stack.launch.py
               robot_id:=$ROBOT_ID x:=$ROBOT_INIT_X y:=$ROBOT_INIT_Y"
    depends_on: [sim]

  robot_1:
    build: .
    network_mode: host
    environment: {ROBOT_ID: robot_1, ROBOT_INIT_X: "1.8", ROBOT_INIT_Y: "0.6"}
    command: >
      bash -c ". /ws/install/setup.sh &&
               ros2 launch swarm_bringup robot_stack.launch.py
               robot_id:=$ROBOT_ID x:=$ROBOT_INIT_X y:=$ROBOT_INIT_Y"
    depends_on: [sim]
```

`network_mode: host` gives all containers shared host networking so ROS 2 DDS multicast
discovery works with no extra configuration.

### Adding Robots

Copy any `robot_N` block in `compose.yaml`, increment the ID, and set new initial
coordinates. No rebuild required — same image.

---

## Verification

```bash
docker compose build && docker compose up

# Nodes alive
ros2 node list | grep -E "(map_merge|frontier|coordinator|robot_fsm)"

# Watch assignments
ros2 topic echo /robot_0/exploration_target
ros2 topic echo /robot_1/exploration_target

# Watch coverage grow
ros2 topic echo /merged_map --field info

# Foxglove panels:
#   3D:  /merged_map (OccupancyGrid) + /frontier_markers + /assignment_markers + TF
#   Plot: /robot_0/status.map_cells_known + /robot_1/status.map_cells_known vs time
```

## Implementation Phases

| Phase | Description | Test |
|---|---|---|
| 0 | Single robot on maze world ✅ | SLAM + Foxglove working; `single_robot.launch.py` for regression |
| 1 | `swarm_msgs` package | `colcon build --packages-select swarm_msgs` + `ros2 interface show swarm_msgs/msg/Frontier` |
| 2 | Multi-robot launch (Gazebo + N × robot_stack) | `ros2 topic list \| grep robot_` shows `/robot_0/map`, `/robot_1/map`; Nav2 goal via topic works |
| 3 | `map_merge_node` | Teleoperate robots to opposite sides; `/merged_map` shows combined area in Foxglove |
| 4 | `frontier_detector_node` | `/frontier_markers` shows spheres at unexplored openings; markers disappear as robot explores |
| 5 | `robot_fsm_node` | Manually publish to `/robot_0/exploration_target` → robot navigates autonomously |
| 6 | `coordinator_node` (capstone) | Both robots auto-explore maze; `/assignment_markers` shows robot→frontier lines; robots never share a frontier; 2-robot coverage ~2× faster than 1 |

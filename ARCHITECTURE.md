# Swarm Maze-Solving Architecture

## Package Structure

```
swarm_ws/
├── Dockerfile
├── compose.yaml
└── src/
    ├── swarm_msgs/          # ament_cmake — custom ROS interfaces only (no executables)
    ├── swarm_slam/          # ament_python — global map merge node
    ├── swarm_exploration/   # ament_python — frontier detector, robot FSM
    └── swarm_bringup/       # ament_python — launch files, worlds, configs
```

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
float32              info_score     # reserved (not used by current scorer)
string               assigned_to    # robot_id or "" if unassigned
```

**`FrontierArray.msg`**
```
std_msgs/Header header
swarm_msgs/Frontier[] frontiers
```

---

## Node Graph

### `global_node` — `swarm_slam` (global namespace, singleton)

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/map` | `nav_msgs/OccupancyGrid` — one per robot (dynamic) |
| Sub | `/robot_N/odom` | `nav_msgs/Odometry` — one per robot (dynamic) |
| Pub | `/merged_map` | `nav_msgs/OccupancyGrid` — max-confidence merge |
| Pub | `/robot_poses` | `geometry_msgs/PoseArray` — all robot positions in world frame |
| Pub | `/robot_ids` | `std_msgs/String` — comma-separated list of active robot IDs |

Merge rule: unknown(−1) < free(0) < occupied(100). All robots spawn at the same world origin
so map frames are co-located — direct numpy overlay, no ICP required. Publishes at 2 Hz.

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
    occupied = (arr == 100)

    # Frontier = free cell adjacent to at least one unknown cell,
    # far enough from obstacles
    unknown_dilated = ndimage.binary_dilation(unknown)
    safe_free = free & ~ndimage.binary_dilation(occupied)
    frontier_cells = safe_free & unknown_dilated

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

Publishes at 2 Hz. Frontier markers are rendered as cyan cylinders in Foxglove, height scaled by cluster size.

---

### `robot_fsm_node` — `swarm_exploration` (per robot, namespace `/robot_N`)

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/map` | `nav_msgs/OccupancyGrid` — for map_cells_known count |
| Sub | `/merged_map` | `nav_msgs/OccupancyGrid` — for BFS ground-distance computation |
| Sub | `/frontiers` | `swarm_msgs/FrontierArray` — global frontier list |
| Sub | `/robot_poses` | `geometry_msgs/PoseArray` — all robot positions in world frame |
| Sub | `/robot_ids` | `std_msgs/String` — peer discovery |
| Sub | `/{peer}/goal` | `geometry_msgs/PoseStamped` — peer current goals (subscribed dynamically) |
| Pub | `/robot_N/status` | `swarm_msgs/RobotStatus` — published at 2 Hz |
| Pub | `/robot_N/goal` | `geometry_msgs/PoseStamped` — this robot's current navigation goal |
| Pub | `/robot_N/goal_markers` | `visualization_msgs/MarkerArray` — arrow + sphere visualisation |
| Action | `/robot_N/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

**State machine:**

```
WAITING   → EXPLORING : frontier selected via local self-assignment
EXPLORING → WAITING   : Nav2 goal succeeded  OR  assigned frontier disappeared
EXPLORING → WAITING   : Nav2 goal failed (obstacle / unreachable)
WAITING   → DONE      : no frontiers remain after exploration
```

**Scoring function (decentralised self-assignment):**

Each robot independently scores all available frontiers and picks the best one:

```
score = (1 / effective_gd) * min_peer_goal_separation

effective_gd = gd / heading_factor
  where heading_factor ∈ [0.02, 1.0] based on how well the BFS path
  direction aligns with the robot's current heading (penalises U-turns)
gd = ground distance from BFS wavefront on /merged_map
     (falls back to Euclidean × 1.4 if BFS unavailable)
```

Peer goal separation: if a peer is already navigating to a nearby point, that frontier's score is reduced, preventing robots from converging on the same location.

Additional robustness mechanisms:
- **Blacklist with 120 s TTL** — failed navigation targets are ignored to prevent thrashing
- **Frontier vanishing detection** — if the assigned frontier disappears from `/frontiers` during navigation, the goal is cancelled and a new one selected
- **Goal timeout** — 60 s maximum per navigation goal

**Visualisation colours:** orange = robot_0, blue = robot_1, magenta = robot_2.

---

### Nav2 stack — per robot, namespace `/robot_N`

Standard Nav2 lifecycle nodes. Global and local costmaps consume `/robot_N/map`.
Controller publishes `/robot_N/cmd_vel`.

---

## Gazebo World

**`swarm_bringup/worlds/maze_world.sdf`** — pre-generated 12×12 m maze arena.

Robot spawns:

| Robot | x | y | z | yaw |
|---|---|---|---|---|
| robot_0 | 0.6 | 0.6 | 0.05 | 1.5708 |
| robot_1 | 1.8 | 0.6 | 0.05 | 1.5708 |
| robot_2 | 0.6 | 1.8 | 0.05 | 1.5708 |

All Gazebo plugin topics namespaced per robot: `/robot_N/cmd_vel`, `/robot_N/odom`,
`/robot_N/scan`, `/robot_N/tf`.

**TF frames:** `map_frame: robot_N/map` per SLAM instance. A static
`world → robot_N/map` identity transform is published at startup so all robots share
a common reference frame without ICP.

The maze SDF is generated from an ASCII specification via
`swarm_bringup/worlds/generate_maze.py`.

---

## Launch Architecture — `swarm_bringup`

```
launch/
  swarm.launch.py           # top-level: Gazebo + N × robot_stack + global nodes + Foxglove
  robot_stack.launch.py     # per-robot: RSP + gz_bridge + SLAM lifecycle + Nav2 + FSM
  global.launch.py          # global nodes: global_node + frontier_detector_node
  navigation.launch.py      # Nav2 bringup (called from robot_stack)
  single_robot.launch.py    # kept for single-robot regression testing
  gazebo.launch.py          # Gazebo sim + clock bridge (entry point for sim service)

config/
  nav2_params.yaml          # Nav2 config (robot_N namespace substituted at launch)
  slam_params.yaml          # slam_toolbox per-robot config (namespace substituted at launch)
```

**Startup sequence in `swarm.launch.py`:**

| Delay | Action |
|-------|--------|
| 0 s | Gazebo Harmonic headless (`gz sim -r -s`) + clock bridge |
| 0 s | Per-robot stacks (RSP, gz_bridge, SLAM, Nav2, FSM) |
| 5 s | Foxglove bridge on port 8765 |
| 15 s | `global_node` (waits for robots to have published their maps) |
| 25 s | `frontier_detector_node` (waits for `/merged_map` to be available) |

Robot identity is injected via environment variables (`SWARM_ROBOT_ID`, `SWARM_X`, `SWARM_Y`, `SWARM_YAW`) so the same image serves all robot services.

The Gazebo bridge config is generated programmatically per robot in `robot_stack.launch.py`
so that `/model/robot_N/cmd_vel` maps to `/robot_N/cmd_vel`.

---

## Key Architectural Trade-offs

| Decision | Choice | Rationale |
|---|---|---|
| Communication | Global ROS 2 topics, no radio simulation | Eliminates comm complexity; focus on exploration strategy |
| Exploration strategy | Decentralised local self-assignment (BFS ground distance + heading penalty + peer separation) | Scalable, no coordinator bottleneck, collision-free assignment |
| Coordination | No central coordinator — each robot picks its own frontier | More resilient; coordinator was a single point of failure with no added correctness benefit at this scale |
| Map sharing | Direct `/robot_N/map` subscriptions, numpy overlay | No compression overhead; bandwidth is unconstrained in sim |
| Frontier scoring | `1 / effective_gd` with heading factor and peer separation | Balances travel cost, direction continuity, and inter-robot spread |
| Frontier detection | `scipy.ndimage` label on merged map | Fast, dependency-free, well-tested |
| TF alignment | Identity static transform at spawn | Robots start at known offsets; no ICP needed |
| Container networking | `network_mode: host` + `/dev/shm` volume | Shared host network for DDS multicast; shared memory for low-latency IPC |
| DDS tuning | CycloneDDS `MaxAutoParticipantIndex: 200` | Prevents index collisions across 30+ nodes |

---

## Node Graph Summary

```
Gazebo Sim
  /robot_N/scan, /robot_N/odom, /robot_N/tf
        │
        ▼
  slam_toolbox (per robot, namespace /robot_N)
  /robot_N/map
        │
        ▼
  global_node (singleton) ──── /robot_poses, /robot_ids ───┐
  /merged_map                                              │
        │                                                  │
        ▼                                                  │
  frontier_detector_node (singleton)                       │
  /frontiers, /frontier_markers                            │
        │                                                  │
        └─────────────────────────────────┐                │
                                          ▼                ▼
                                    robot_fsm_node (per robot, /robot_N)
                                      ← /robot_N/map (local cells known)
                                      ← /{peer}/goal (peer goal positions)
                                      → /robot_N/goal, /robot_N/goal_markers
                                      → Nav2 NavigateToPose
                                      → /robot_N/cmd_vel (via Nav2)
                                      → /robot_N/status
```

---

## Docker / Containerization

```
swarm_ws/
├── Dockerfile      # production image: all nodes
└── compose.yaml    # 6 services: sim, robot_0, robot_1, robot_2, global, foxglove
```

### Dockerfile

Multi-stage build with per-package layers for fast incremental rebuilds:

```dockerfile
FROM osrf/ros:jazzy-desktop AS base

# System deps (Gazebo Harmonic + Nav2 + bridges)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz-bridge gz-harmonic \
    ros-jazzy-foxglove-bridge && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy scipy

WORKDIR /ws

# Per-package copy + build (fine-grained layer caching)
COPY src/swarm_msgs src/swarm_msgs
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install --packages-select swarm_msgs

COPY src/swarm_slam src/swarm_slam
RUN . /opt/ros/jazzy/setup.sh && . install/setup.sh && \
    colcon build --symlink-install --packages-select swarm_slam
# ... (same pattern for swarm_exploration and swarm_bringup)
```

### compose.yaml (abridged)

```yaml
services:

  sim:
    build: .
    network_mode: host
    volumes: [{type: tmpfs, target: /dev/shm}]
    command: ros2 launch swarm_bringup gazebo.launch.py world:=...

  robot_0:
    build: .
    network_mode: host
    volumes: [{type: tmpfs, target: /dev/shm}]
    environment:
      SWARM_ROBOT_ID: robot_0
      SWARM_X: "0.6"
      SWARM_Y: "0.6"
      SWARM_YAW: "1.5708"
      CYCLONEDDS_URI: "<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>...</Discovery></Domain></CycloneDDS>"
    command: ros2 launch swarm_bringup robot_stack.launch.py
    depends_on: [sim]

  # robot_1 and robot_2 follow the same pattern at different coordinates

  global:
    build: .
    network_mode: host
    command: ros2 launch swarm_bringup global.launch.py
    depends_on: [sim]

  foxglove:
    build: .
    network_mode: host
    command: ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
    depends_on: [sim]
```

`network_mode: host` gives all containers shared host networking so ROS 2 DDS multicast
discovery works with no extra configuration. `/dev/shm` is mounted as `tmpfs` for
low-latency intra-host IPC.

### Adding Robots

Copy any `robot_N` block in `compose.yaml`, increment the ID, and set new initial
coordinates. No rebuild required — same image serves all robots.

---

## Verification

```bash
docker compose build && docker compose up

# Nodes alive
ros2 node list | grep -E "(global_node|frontier_detector|robot_fsm)"

# Watch merged map grow
ros2 topic echo /merged_map --field info

# Watch frontier list
ros2 topic echo /frontiers

# Watch per-robot goal assignments
ros2 topic echo /robot_0/goal
ros2 topic echo /robot_1/goal
ros2 topic echo /robot_2/goal

# Watch coverage
ros2 topic echo /robot_0/status --field map_cells_known

# Foxglove panels:
#   3D:  /merged_map (OccupancyGrid) + /frontier_markers + /robot_N/goal_markers + TF
#   Plot: /robot_N/status.map_cells_known vs time
```

---

## Implementation Phases

| Phase | Description | Status |
|---|---|---|
| 0 | Single robot on maze world + SLAM + Foxglove | ✅ Complete — `single_robot.launch.py` for regression |
| 1 | `swarm_msgs` package | ✅ Complete |
| 2 | Multi-robot launch (Gazebo + N × robot_stack) | ✅ Complete |
| 3 | `global_node` (map merge + robot pose broadcast) | ✅ Complete |
| 4 | `frontier_detector_node` | ✅ Complete |
| 5 | `robot_fsm_node` with manual target publishing | ✅ Complete |
| 6 | Autonomous exploration with decentralised self-assignment | ✅ Complete |

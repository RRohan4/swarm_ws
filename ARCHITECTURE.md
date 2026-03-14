# Swarm Maze-Solving Architecture

## Package Structure

```
swarm_ws/src/
├── swarm_msgs/          # Custom ROS interfaces only (no executables)
├── swarm_radio/         # Mock radio "god" node — singleton, global namespace
├── swarm_perception/    # Per-robot: LIDAR processing, pseudo-3D world model
├── swarm_comms/         # Per-robot: TX/RX queuing, FEC, ARQ, heartbeat, peer registry
├── swarm_slam/          # Per-robot: slam_toolbox wrapper, map fragment I/O, peer map merge
├── swarm_exploration/   # Per-robot: NBV candidate sampling, info-gain scoring, submodular claim consensus, role FSM
├── swarm_coordinator/   # Per-robot: distributed coordinator, no central authority
└── swarm_bringup/       # Maze SDF, multi-robot launch, nav2/SLAM param YAMLs
```

Existing `src/swarm_exploration/` launch files → folded into `swarm_bringup/`.

---

## Custom Messages — `swarm_msgs`

### Messages

**`RadioEnvelope.msg`** — wrapper for all inter-robot comms
```
std_msgs/Header header
string   src_robot_id
string   dst_robot_id        # specific ID or "BROADCAST"
uint8    msg_type            # MAP_FRAGMENT=1, VIEWPOINT_CLAIM=2, PEER_STATUS=3,
                             # DISCOVERY=4, GOAL_FOUND=5, ACK=6, HEARTBEAT=7
uint32   seq_num
uint32   ack_num
uint8    fec_scheme          # 0=none, 1=Hamming(7,4), 2=RS(255,223)
uint8[]  fec_payload
uint16   crc16               # CRC-16/CCITT of raw payload before FEC
bool     is_fragment
uint8    fragment_index
uint8    fragment_total
```

**`MapFragment.msg`** — compressed occupancy grid delta from a peer
```
std_msgs/Header header
string                   origin_robot_id
uint32                   fragment_seq
nav_msgs/MapMetaData     map_info
uint8[]                  data_compressed       # zlib-compressed OccupancyGrid.data
geometry_msgs/Pose       robot_pose_in_map
uint64                   timestamp_ns
```

**`ViewpointClaim.msg`** — NBV candidate claim broadcast
```
std_msgs/Header header
string                 claimant_robot_id
uint32                 claim_id              # hash(robot_id XOR candidate_pose)
geometry_msgs/Pose     candidate_pose        # evaluated viewpoint in world frame
float32                expected_info_gain    # expected entropy reduction (nats)
float32                claim_radius_m        # exclusion zone radius around candidate
uint8                  claim_state           # CLAIMING=0, CONFIRMED=1, RELEASED=2, EXPIRED=3
uint32                 ttl_ms
```

**`PeerStatus.msg`**
```
std_msgs/Header header
string                 robot_id
uint8                  operational_state     # ACTIVE=0, RELAY=1, RECOVERING=2, FAILED=3
float32                battery_fraction      # 0.0–1.0 (proxy: 1 - distance_traveled / max_range)
uint32                 viewpoints_visited
uint32                 map_cells_known
geometry_msgs/Pose     current_pose
float32                comms_quality         # rolling RSSI proxy 0.0–1.0
```

**`SwarmDiscovery.msg`**
```
std_msgs/Header header
string                 announcing_robot_id
string[]               known_peer_ids
uint32                 swarm_epoch           # monotonically increasing; detects partitions
geometry_msgs/Pose[]   known_poses
```

**`GoalFound.msg`**
```
std_msgs/Header header
string                       discovering_robot_id
geometry_msgs/PoseStamped    goal_pose
float32                      confidence
bool                         confirmed             # true once two robots independently verify
```

### Services

**`RequestMapFragment.srv`**
```
string   requesting_robot_id
uint32   last_known_seq       # requester's highest received seq; peer replies with delta
---
swarm_msgs/MapFragment[]  fragments
bool                      success
```

**`ClaimViewpoint.srv`**
```
swarm_msgs/ViewpointClaim  claim
---
bool    granted
string  conflict_robot_id   # populated if granted=false
```

### Actions

**`ExploreViewpoint.action`**
```
# Goal
geometry_msgs/Pose  target_viewpoint
string              assigned_role      # "scout" | "mapper" | "relay"
---
# Result
bool     goal_found
uint32   cells_added_to_map
float32  distance_traveled_m
---
# Feedback
geometry_msgs/Pose  current_pose
float32             info_gain_so_far
uint32              candidates_remaining
```

---

## Mock Radio God Node — `swarm_radio`

**Node:** `radio_god_node` — single instance, global namespace, sole broker for all inter-robot communication.

**Subscribes:**
- `/robot_N/odom` (`nav_msgs/Odometry`) — one dynamic subscription per robot, caches positions
- `/radio/tx` (`RadioEnvelope`) — outbound messages from any robot

**Publishes:**
- `/robot_N/radio/rx` (`RadioEnvelope`) — delivered (possibly corrupted) messages per robot
- `/radio/channel_state` (`diagnostic_msgs/DiagnosticArray`) — per-pair RSSI/link quality

**Channel simulation pipeline (per message):**
1. Resolve `dst_robot_id` → recipient list (single robot or all except src)
2. For each recipient: compute Euclidean distance from cached odom
3. Drop if distance > `comm_radius_m`
4. Log-distance path loss model → RSSI; drop if RSSI < noise floor
5. Check per-pair dropout registry; drop if inside active dropout window
6. Roll `dropout_probability`; if hit, open dropout window (`dropout_duration_min/max_ms`), drop
7. Compute delivery delay: `latency_base_ms` + Gaussian(`latency_jitter_std_ms`); schedule via ROS timer
8. At delivery: BER-proportional bit-flipping on `fec_payload` bytes

**Key parameters (`radio_params.yaml`):**
```yaml
radio_god_node:
  ros__parameters:
    robot_ids: ["robot_0", "robot_1", "robot_2"]
    comm_radius_m: 4.0
    path_loss_exponent: 2.5
    reference_distance_m: 1.0
    reference_rssi_dbm: -40.0
    noise_floor_dbm: -90.0
    ber_at_sensitivity: 0.01
    dropout_probability: 0.05
    dropout_duration_min_ms: 500
    dropout_duration_max_ms: 3000
    latency_base_ms: 10.0
    latency_jitter_std_ms: 5.0
```

---

## Per-Robot Node Stack (namespace `/robot_N`)

All inter-robot comms go exclusively through the radio god node. Each robot namespace runs 6 nodes.

### 1. Perception Node — `swarm_perception/perception_node`

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/scan` | `sensor_msgs/LaserScan` |
| Pub | `/robot_N/obstacle_map` | `nav_msgs/OccupancyGrid` |
| Pub | `/robot_N/elevation_map` | `grid_map_msgs/GridMap` |
| Pub | `/robot_N/processed_scan` | `sensor_msgs/PointCloud2` |

**Pseudo-3D elevation inference:** accumulate consecutive 2D LIDAR scans over an odom-tracked motion window; when a scan beam's range changes relative to a stationary reference beam, infer height change; maintain per-cell height layer in a `grid_map`. Detects wall-top edges and partial occlusions. Used by nav2 costmap for 3D-aware obstacle inflation.

### 2. SLAM / Map Fusion Node — `swarm_slam/slam_node`

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/scan` | `sensor_msgs/LaserScan` |
| Sub | `/robot_N/odom` | `nav_msgs/Odometry` |
| Sub | `/robot_N/radio/rx` | `swarm_msgs/RadioEnvelope` (MAP_FRAGMENT type) |
| Pub | `/robot_N/map` | `nav_msgs/OccupancyGrid` (local) |
| Pub | `/robot_N/merged_map` | `nav_msgs/OccupancyGrid` (peer-fused global estimate) |

- Wraps `slam_toolbox` async SLAM per robot with `map_frame: robot_N/map`, `base_frame: robot_N/base_link`
- **Map merge:** decompress received `MapFragment.data_compressed` (zlib), transform to common world frame using sender's `robot_pose_in_map`, merge into `merged_map` via max-confidence voting (unknown < free < occupied)
- **Map export:** 5 Hz delta export — diff local map against last-sent snapshot, zlib-compress changed cells, publish as `MapFragment` via comms node TX request

### 3. Communication Manager Node — `swarm_comms/comms_node`

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/radio/rx` | `swarm_msgs/RadioEnvelope` |
| Sub | `/robot_N/comms/tx_request` | `swarm_msgs/RadioEnvelope` |
| Pub | `/radio/tx` | `swarm_msgs/RadioEnvelope` |
| Pub | `/robot_N/comms/rx_dispatch/map_fragment` | `swarm_msgs/MapFragment` |
| Pub | `/robot_N/comms/rx_dispatch/viewpoint_claim` | `swarm_msgs/ViewpointClaim` |
| Pub | `/robot_N/comms/rx_dispatch/peer_status` | `swarm_msgs/PeerStatus` |
| Pub | `/robot_N/comms/rx_dispatch/discovery` | `swarm_msgs/SwarmDiscovery` |
| Pub | `/robot_N/comms/rx_dispatch/goal_found` | `swarm_msgs/GoalFound` |

- **FEC:** Hamming(7,4) for control messages (low overhead); Reed-Solomon(255,223) for `MAP_FRAGMENT` (strong protection for large payloads)
- **CRC:** CRC-16/CCITT on raw payload before FEC; corrupt messages → NACK → ARQ retransmit
- **ARQ:** sliding window (size 8); retransmit unACK'd after `arq_timeout_ms=200`; max 3 retries then mark link degraded in peer registry
- **Heartbeat:** emit `HEARTBEAT` envelope every 500ms; peer marked `FAILED` after 3 missed heartbeats (1.5s)
- **Peer registry:** per-peer `{last_seen, rssi_proxy, operational_state, last_pose}`

### 4. Exploration Planner Node — `swarm_exploration/exploration_node`

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/merged_map` | `nav_msgs/OccupancyGrid` |
| Sub | `/robot_N/comms/rx_dispatch/viewpoint_claim` | `swarm_msgs/ViewpointClaim` |
| Sub | `/robot_N/comms/rx_dispatch/peer_status` | `swarm_msgs/PeerStatus` |
| Pub | `/robot_N/exploration_target` | `geometry_msgs/PoseStamped` |
| Pub | `/robot_N/comms/tx_request` | `swarm_msgs/RadioEnvelope` |

**NBV / Information-Gain Planning:**

1. **Candidate sampling:** uniformly sample poses from free-space cells adjacent to unknown cells on `merged_map`
2. **Info-gain scoring:** for each candidate pose, raytrace simulated LIDAR beams (Bresenham) through the current map; count cells that would transition from unknown → known; score = `expected_new_cells / nav2_path_cost_to_candidate`
3. **Submodular multi-robot coordination:** each robot broadcasts its top candidate with `expected_info_gain`; robot picks the candidate maximizing **marginal** gain — gain assuming peers already visit their claimed candidates first. This greedy submodular maximization yields ≥ (1 − 1/e) ≈ 63% of optimal joint coverage.
4. **Claim protocol:**
   - Broadcast `ViewpointClaim` with `claim_state=CLAIMING` and `claim_radius_m`
   - Wait `claim_window_ms=300ms`; if peer claim overlaps (candidate within `claim_radius_m`), higher robot_id yields, re-evaluates next-best candidate
   - Broadcast `CONFIRMED`; re-broadcast before TTL expiry to hold claim
   - Release (`RELEASED`) when viewpoint reached or robot transitions to RELAY

**Role FSM:**
- `SCOUT` → navigate to highest-gain viewpoint, sample map, repeat
- `MAPPER` → stay at current position, refine local map detail (triggered when gain < threshold)
- `RELAY` → suspend exploration, position to maximise connectivity between isolated peers

**Goal detection:** detect goal marker (unique LIDAR retroreflector signature) during scan processing; publish `GoalFound`; await independent confirmation from one other robot before broadcasting halt

### 5. Local Swarm Coordinator Node — `swarm_coordinator/coordinator_node`

| | Topic | Type |
|---|---|---|
| Sub | `/robot_N/comms/rx_dispatch/peer_status` | `swarm_msgs/PeerStatus` |
| Sub | `/robot_N/comms/rx_dispatch/discovery` | `swarm_msgs/SwarmDiscovery` |
| Sub | `/robot_N/comms/rx_dispatch/goal_found` | `swarm_msgs/GoalFound` |
| Pub | `/robot_N/coordinator/role_assignment` | `std_msgs/String` |
| Pub | `/robot_N/comms/tx_request` | `swarm_msgs/RadioEnvelope` |

- **Epoch-based peer view:** broadcast `SwarmDiscovery` every 2s with full known-peer list; on epoch mismatch merge via union of known peers
- **Relay election:** if comms_node reports robot A and C have no direct link but both link to B → coordinator transitions B to `RELAY`; B navigates to midpoint of A–C segment
- **Failure handling:** on peer `FAILED`, broadcast peer's last-known claimed viewpoint as `RELEASED`; remaining robots immediately re-evaluate candidates
- **Partition recovery:** isolated robot (no peers heard for 5s) navigates toward last-known peer position to restore comms, then resumes exploration

### 6. Navigation — `nav2` per robot

- All nav2 lifecycle nodes in `/robot_N` namespace
- Global/local costmaps consume `/robot_N/merged_map` and `/robot_N/obstacle_map`
- `/robot_N/elevation_map` fed into 3D-aware inflation layer
- Controller publishes `/robot_N/cmd_vel`

---

## Gazebo World

**`swarm_bringup/worlds/maze_world.sdf`** — 12×12m arena with internal maze walls and a goal marker (retroreflective box at maze end with distinct cross-section for LIDAR detection).

Multi-robot spawning: parameterized robot model `{robot_id}`, spawned N times via `gz service /world/swarm_world/create` calls from the launch file with offset initial poses.

All Gazebo plugin topics namespaced per robot: `/robot_N/cmd_vel`, `/robot_N/odom`, `/robot_N/scan`, `/robot_N/tf`.

---

## Launch Architecture — `swarm_bringup`

```
launch/
  swarm.launch.py           # top-level: Gazebo + radio god + N × robot_stack
  robot_stack.launch.py     # per-robot include: perception, comms, slam, exploration, coordinator, nav2
  radio_god.launch.py       # radio_god_node with channel params
  single_robot.launch.py    # kept for single-robot dev/debug

config/
  radio_params.yaml         # radio god channel simulation params
  nav2_params.yaml          # nav2 config (robot_N namespace substitution via LaunchConfiguration)
  slam_params.yaml          # slam_toolbox per-robot config
  swarm_params.yaml         # NBV scoring weights, ARQ timeouts, heartbeat intervals, relay thresholds
```

---

## Key Architectural Trade-offs

| Decision | Choice | Rationale |
|---|---|---|
| Exploration strategy | NBV / information-gain (Charrow et al. 2015) | Principled multi-robot via submodular maximization; no frontier degenerate cases in narrow corridors |
| Multi-robot coordination | Greedy submodular (marginal gain) | (1−1/e) optimality guarantee; fits naturally into distributed claim broadcast |
| Central vs. distributed | Fully distributed; radio god is infrastructure only | Resilient to any single robot failure |
| Map sharing | Delta compression (zlib) + 5 Hz cap | Bandwidth vs. freshness; prevents radio saturation |
| FEC scheme | Hamming(7,4) small msgs / RS(255,223) map fragments | Match overhead to payload size and error sensitivity |
| Viewpoint claim | Optimistic TTL + marginal-gain yield on conflict | Avoids distributed lock contention; higher robot_id breaks ties |
| Relay election | Connectivity-triggered, midpoint positioning | Greedy but effective for maze topology |
| 2D → pseudo-3D | Scan stacking over odom-tracked motion | No extra hardware; sufficient for maze obstacle detection |

---

## Node Graph Summary

```
                    ┌─────────────────────┐
                    │   radio_god_node     │  (global ns)
                    │  /radio/tx  →  route │
                    │  → /robot_N/radio/rx │
                    └──────────┬──────────┘
                               │ per robot
          ┌────────────────────┼────────────────────┐
          │                    │                    │
   /robot_0/...          /robot_1/...         /robot_2/...
          │
   ┌──────┴────────────────────────────────────┐
   │  perception_node                           │
   │    /scan → /obstacle_map, /elevation_map  │
   ├───────────────────────────────────────────┤
   │  slam_node                                │
   │    /scan + /odom + rx → /map, /merged_map │
   ├───────────────────────────────────────────┤
   │  comms_node                               │
   │    /radio/rx ↔ /radio/tx                 │
   │    → /comms/rx_dispatch/*                 │
   ├───────────────────────────────────────────┤
   │  exploration_node  (NBV planner)          │
   │    /merged_map → /exploration_target      │
   │    ↔ /comms/tx_request (ViewpointClaim)  │
   ├───────────────────────────────────────────┤
   │  coordinator_node                         │
   │    peer_status/discovery → role_assignment│
   ├───────────────────────────────────────────┤
   │  nav2 stack                               │
   │    /exploration_target → /cmd_vel         │
   └───────────────────────────────────────────┘
```

---

## Verification

```bash
ros2 launch swarm_bringup swarm.launch.py num_robots:=3 && \
ros2 topic echo /radio/channel_state --once && \
ros2 node list | grep -E "(radio_god|robot_[0-2]/(perception|comms|slam|exploration|coordinator))"
```

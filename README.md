# Coordinated Swarm Exploration with ROS 2

ROS 2 workspace for multi-robot swarm exploration. Four TurtleBot3 Waffle robots autonomously explore a complex maze using decentralized frontier-based navigation. Gazebo sim runs headless and Foxglove provides the visualisation UI.

Written in Python with ROS 2 Jazzy, the codebase includes SLAM, Nav2, a custom frontier detector, and a per-robot FSM. Robots coordinate via **geodesic Voronoi partitioning**: a simultaneous multi-source BFS divides the merged map so each robot owns the region it can reach fastest, then selects the nearest frontier within its partition. This eliminates duplicate work without any central coordinator. See [`ARCHITECTURE.md`](ARCHITECTURE.md) for a full description.

![ezgif-5dca8da404a8c63b](https://github.com/user-attachments/assets/3e7e5409-2bfa-445c-80f0-fc237b1a7c50)

<img width="769" height="480" alt="image" src="https://github.com/user-attachments/assets/2c9a1917-566a-4427-acec-bad63f26021a" />

## Dev flow overview

| Step | Tool | Purpose |
|------|------|---------|
| Write & edit code | Dev Container | Full ROS 2 + tooling environment in VS Code |
| Run the simulator | `docker compose` | Builds the sim image and launches the full stack |

The dev container forwards the host Docker socket, so `docker compose` commands work from inside it.

---

## 1. Open in Dev Container

### Requirements

- [Docker](https://docs.docker.com/get-docker/) (Desktop or Engine)
- [VS Code](https://code.visualstudio.com/) with the **Dev Containers** extension (`ms-vscode-remote.remote-containers`)

### Steps

1. Clone this repo and open the folder in VS Code.
2. When prompted, click **Reopen in Container** (or `F1` → **Dev Containers: Reopen in Container**).
3. VS Code builds the image from [`.devcontainer/Dockerfile.dev`](.devcontainer/Dockerfile.dev) and mounts the workspace at `/ws`.
4. `pre-commit install` runs automatically on first open.

The dev container includes ROS 2 Jazzy, `ruff`, `mypy`, `pre-commit`, and recommended VS Code extensions (Ruff, Pylance, ROS, XML, TOML, Claude Code).

---

## 2. Build & run the simulator

From the workspace root — either on the host or from a terminal inside the dev container:

```bash
make build    # Build the sim image (only needed after Dockerfile or dependency changes)
make up       # Start the full 4-robot stack
make down     # Stop and remove containers
make rebuild  # Build then start in one step
make logs     # Follow logs from all running services
```

This starts seven services:

| Service | What it runs |
|---------|-------------|
| `sim` | Gazebo Harmonic headless + clock bridge |
| `robot_0` | TurtleBot3 Waffle at (0.6, 0.6) — SLAM + Nav2 + FSM |
| `robot_1` | TurtleBot3 Waffle at (1.8, 0.6) — SLAM + Nav2 + FSM |
| `robot_2` | TurtleBot3 Waffle at (0.6, 1.8) — SLAM + Nav2 + FSM |
| `robot_3` | TurtleBot3 Waffle at (1.8, 1.8) — SLAM + Nav2 + FSM |
| `global` | Map merge node + frontier detector |
| `foxglove` | Foxglove bridge on `ws://localhost:8765` |

Stop with `Ctrl-C` or `make down`.

### Regression tests

**Fast (headless)** — validates launch file syntax and node imports in seconds, no display needed:

```bash
make test        # docker build --target test .
```

**Live single-robot** — starts Gazebo with one robot to verify the full exploration stack end-to-end:

```bash
make single      # docker compose --profile single up
```

### Bag recording

Run headless at maximum simulation speed and record a bag (auto-stops when all robots reach DONE):

```bash
make record           # headless only
make record FOXGLOVE=1  # also start the Foxglove bridge
make record EXPLORE=60  # stop at 60% exploration
make record TIMEOUT=300  # stop after 300 seconds
```

Bags are written to `./bags/`.

---

## 3. Connect Foxglove

Open [Foxglove](https://app.foxglove.dev/) (Desktop or web):

1. **New connection** → **Foxglove WebSocket** → `ws://localhost:8765`
2. Set fixed frame to `world`.

Useful topics:

| Panel | Topic |
|-------|-------|
| Map (merged) | `/merged_map` |
| LaserScan (robot 0) | `/robot_0/scan` |
| LaserScan (robot 1) | `/robot_1/scan` |
| LaserScan (robot 2) | `/robot_2/scan` |
| Frontier markers | `/frontier_markers` |
| Goal markers | `/robot_N/goal_markers` |
| Coverage plot | `/robot_N/status` → `map_cells_known` |

A pre-built dashboard layout is in [`config/foxglove/swarm_ws.json`](config/foxglove/swarm_ws.json) — import it via **File → Import layout**.

---

## Pre-commit hooks

Hooks run automatically before every commit. To run manually:

```bash
pre-commit run --all-files
```

| Hook | What it does |
|------|-------------|
| `check-json` | Validates JSON syntax |
| `check-yaml` | Validates YAML syntax |
| `check-toml` | Validates TOML syntax |
| `ruff` | Python linting (auto-fix) |
| `ruff-format` | Python formatting |
| `trailing-whitespace` | Strips trailing spaces |
| `end-of-file-fixer` | Ensures files end with a newline |
| `mixed-line-ending` | Normalises line endings to LF |
| `check-merge-conflict` | Blocks leftover merge-conflict markers |
| `check-added-large-files` | Blocks files > 500 KB |

Update hook versions:

```bash
pre-commit autoupdate
```

---

## Troubleshooting

- **Port 8765 in use**: stop any process on that port and rerun `docker compose up`.
- **No map updates**: SLAM and the global map merge start with timed delays — wait ~25 s after the stack is up before expecting `/merged_map` or `/frontiers`.
- **Robots not moving**: the FSM enters EXPLORING only after frontiers appear on `/frontiers`; allow time for SLAM to build initial maps.
- **Missing ROS packages in sim image**: rebuild with `docker compose build`.
- **DDS participant index errors**: CycloneDDS is configured with `MaxAutoParticipantIndex: 200` to handle 30+ nodes; if you see index collisions check the `CYCLONEDDS_URI` env var in `compose.yaml`.

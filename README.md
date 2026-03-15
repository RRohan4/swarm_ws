# Swarm Simulator — Headless Gazebo + Foxglove

ROS 2 workspace for single- (and eventually multi-) robot swarm exploration. Gazebo runs headless; Foxglove provides the visualisation UI.

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
2. When prompted, click **Reopen in Container** — or `F1` → **Dev Containers: Reopen in Container**.
3. VS Code builds the image from [`.devcontainer/Dockerfile.dev`](.devcontainer/Dockerfile.dev) and mounts the workspace at `/ws`.
4. `pre-commit install` runs automatically on first open.

The dev container includes ROS 2 Jazzy, `ruff`, `mypy`, `pre-commit`, and recommended VS Code extensions (Ruff, Pylance, ROS, XML, TOML, Claude Code).

---

## 2. Build & run the simulator

From the workspace root — either on the host or from a terminal inside the dev container:

```bash
# Build the sim image (only needed after Dockerfile or dependency changes)
docker compose build

# Start the sim stack
docker compose up
```

This starts the `sim` service which runs:
- Gazebo Harmonic in headless/server mode
- TurtleBot3 Waffle spawn
- ROS ↔ Gazebo topic bridges (`ros_gz_bridge`)
- SLAM Toolbox (lifecycle managed automatically)
- Foxglove bridge on `ws://localhost:8765`

Stop with `Ctrl-C` or `docker compose down`.

---

## 3. Connect Foxglove

Open [Foxglove](https://app.foxglove.dev/) (Desktop or web):

1. **New connection** → **Foxglove WebSocket** → `ws://localhost:8765`
2. Set fixed frame to `map`.

Useful topics:

| Panel | Topic |
|-------|-------|
| LaserScan | `/scan` |
| PointCloud | `/depth/points` |
| Map | `/map` |
| Image | `/depth/image` |

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
- **No map updates**: SLAM starts with timed delays — wait ~15 s after the stack is up.
- **Robot not moving in map**: send velocity commands via the Foxglove Teleop panel so SLAM receives motion and scan data.
- **Missing ROS packages in sim image**: rebuild with `docker compose build`.

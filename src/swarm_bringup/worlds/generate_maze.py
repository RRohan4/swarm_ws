#!/usr/bin/env python3
"""Generate a 29×29 orthogonal swarm-exploration maze.

Structure:
  - 5×5 open central staging area
  - Surrounding maze carved with Prim's randomised algorithm (grows outward)
  - Walls and corridors are 1 cell wide throughout

Output: maze.txt       ('#' = wall, ' ' = open)
        maze_world.sdf (Gazebo Harmonic world, centred at origin)
"""

import random

SEED = 42
W, H = 29, 29
CX, CY = W // 2, H // 2  # (14, 14)
HUB_HALF = 2  # hub covers rows/cols CY±2  →  5×5

random.seed(SEED)

grid = [["#"] * W for _ in range(H)]


def carve(r: int, c: int) -> None:
    grid[r][c] = " "


# ── Central hub (5×5) ─────────────────────────────────────────────────────────
for r in range(CY - HUB_HALF, CY + HUB_HALF + 1):
    for c in range(CX - HUB_HALF, CX + HUB_HALF + 1):
        carve(r, c)

# ── Prim's maze generation ────────────────────────────────────────────────────
# Cells in the logical maze grid sit at odd grid positions: 1, 3, …, 27  (14 each)
# Cell (i, j) in [0,13]² maps to grid position (2i+1, 2j+1).
# The hub occupies cell rows/cols 6-7 (grid rows/cols 12-16).

NUM_CELLS = 14  # cells per dimension


def cell_to_grid(i: int, j: int) -> tuple[int, int]:
    return 2 * i + 1, 2 * j + 1


def in_hub(i: int, j: int) -> bool:
    hub_min = (CY - HUB_HALF + 1) // 2  # first cell index inside hub  = 6
    hub_max = (CY + HUB_HALF - 1) // 2  # last  cell index inside hub  = 7
    return hub_min <= i <= hub_max and hub_min <= j <= hub_max


def cell_neighbours(i: int, j: int) -> list[tuple[int, int]]:
    result = []
    for di, dj in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        ni, nj = i + di, j + dj
        if 0 <= ni < NUM_CELLS and 0 <= nj < NUM_CELLS:
            result.append((ni, nj))
    return result


# Seed the maze with all hub cells
in_maze: set[tuple[int, int]] = set()
for i in range(NUM_CELLS):
    for j in range(NUM_CELLS):
        if in_hub(i, j):
            in_maze.add((i, j))

# Build initial frontier: cells adjacent to hub but outside it
frontier: set[tuple[int, int]] = set()
for i, j in in_maze:
    for ni, nj in cell_neighbours(i, j):
        if (ni, nj) not in in_maze:
            frontier.add((ni, nj))

# Prim's loop
while frontier:
    i, j = random.choice(sorted(frontier))
    frontier.discard((i, j))

    # Neighbours already in the maze
    maze_nbrs = [(ni, nj) for ni, nj in cell_neighbours(i, j) if (ni, nj) in in_maze]
    if not maze_nbrs:
        continue  # shouldn't happen, but guard anyway

    # Connect to a random maze neighbour by carving the wall between them
    ni, nj = random.choice(maze_nbrs)
    wr = (2 * i + 1 + 2 * ni + 1) // 2
    wc = (2 * j + 1 + 2 * nj + 1) // 2
    carve(wr, wc)  # wall between cells
    carve(2 * i + 1, 2 * j + 1)  # the cell itself

    in_maze.add((i, j))

    # Expand frontier
    for ni2, nj2 in cell_neighbours(i, j):
        if (ni2, nj2) not in in_maze and (ni2, nj2) not in frontier:
            frontier.add((ni2, nj2))

# ── Write maze.txt ────────────────────────────────────────────────────────────
lines = ["".join(row) for row in grid]
output = "\n".join(lines) + "\n"

with open("maze.txt", "w") as f:
    f.write(output)

open_cells = sum(c == " " for row in grid for c in row)
wall_cells = W * H - open_cells
print(f"Map written to maze.txt  ({W}×{H})")
print(f"Open cells : {open_cells}  ({100 * open_cells // (W * H)}%)")
print(f"Wall cells : {wall_cells}")

# ── Generate maze_world.sdf ───────────────────────────────────────────────────
CELL = 1.0  # metres per grid cell
WALL_H = 0.5  # wall height (m)
WORLD = W * CELL  # 17.4 m


def _cx(col_start: int, ncols: int) -> float:
    return (col_start + ncols / 2) * CELL - WORLD / 2


def _cy(row: int) -> float:
    return (H / 2 - 0.5 - row) * CELL


# Merge horizontal runs of '#' into single boxes to reduce link count
wall_runs: list[tuple[float, float, float, float]] = []
for r, row_cells in enumerate(grid):
    c = 0
    while c < W:
        if row_cells[c] == "#":
            start = c
            while c < W and row_cells[c] == "#":
                c += 1
            n = c - start
            wall_runs.append((_cx(start, n), _cy(r), n * CELL, CELL))
        else:
            c += 1

# Find up to 4 open cells in the hub for robot spawn comments
spawn_pts: list[tuple[float, float]] = []
for r in range(CY - HUB_HALF, CY + HUB_HALF + 1):
    for c in range(CX - HUB_HALF, CX + HUB_HALF + 1):
        if grid[r][c] == " ":
            spawn_pts.append((_cx(c, 1), _cy(r)))
        if len(spawn_pts) == 4:
            break
    if len(spawn_pts) == 4:
        break

sdf_lines: list[str] = []
_a = sdf_lines.append

_a('<?xml version="1.0" ?>')
_a("<!--")
_a("  Gazebo Harmonic maze world")
_a("  Generated from maze.txt by generate_maze.py (Prim's algorithm, seed 42)")
_a(f"  Maze:   {W}x{H} cells, cell size {CELL} m")
_a(f"  World:  {WORLD:.2f} m x {WORLD:.2f} m  (centred at origin)")
_a("  Hub:    5x5 open staging area at world (0, 0)")
_a("-->")
_a('<sdf version="1.9">')
_a('  <world name="maze_world">')
_a("")
_a("    <!-- Physics -->")
_a('    <physics name="1ms" type="ignored">')
_a("      <max_step_size>0.001</max_step_size>")
_a("      <real_time_factor>1.0</real_time_factor>")
_a("      <real_time_update_rate>1000</real_time_update_rate>")
_a("    </physics>")
_a("")
_a("    <!-- Required Gazebo Harmonic system plugins -->")
_a('    <plugin filename="gz-sim-physics-system"')
_a('            name="gz::sim::systems::Physics"/>')
_a('    <plugin filename="gz-sim-scene-broadcaster-system"')
_a('            name="gz::sim::systems::SceneBroadcaster"/>')
_a('    <plugin filename="gz-sim-user-commands-system"')
_a('            name="gz::sim::systems::UserCommands"/>')
_a('    <plugin filename="gz-sim-sensors-system"')
_a('            name="gz::sim::systems::Sensors">')
_a("      <render_engine>ogre2</render_engine>")
_a("    </plugin>")
_a('    <plugin filename="gz-sim-contact-system"')
_a('            name="gz::sim::systems::Contact"/>')
_a('    <plugin filename="gz-sim-imu-system"')
_a('            name="gz::sim::systems::Imu"/>')
_a("")
_a("    <!-- Scene -->")
_a("    <scene>")
_a("      <ambient>0.4 0.4 0.4 1</ambient>")
_a("      <background>0.15 0.15 0.15 1</background>")
_a("      <shadows>true</shadows>")
_a("      <grid>false</grid>")
_a("    </scene>")
_a("")
_a("    <!-- Sun -->")
_a('    <light type="directional" name="sun">')
_a("      <cast_shadows>true</cast_shadows>")
_a("      <pose>0 0 20 0 0 0</pose>")
_a("      <diffuse>0.9 0.9 0.9 1</diffuse>")
_a("      <specular>0.3 0.3 0.3 1</specular>")
_a("      <attenuation>")
_a("        <range>1000</range>")
_a("        <constant>0.9</constant>")
_a("        <linear>0.01</linear>")
_a("        <quadratic>0.001</quadratic>")
_a("      </attenuation>")
_a("      <direction>-0.5 0.2 -0.9</direction>")
_a("    </light>")
_a("")
_a("    <!-- Ground plane -->")
_a('    <model name="ground_plane">')
_a("      <static>true</static>")
_a('      <link name="link">')
_a('        <collision name="col">')
_a("          <geometry>")
_a("            <plane><normal>0 0 1</normal><size>500 500</size></plane>")
_a("          </geometry>")
_a("          <surface>")
_a("            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>")
_a("          </surface>")
_a("        </collision>")
_a('        <visual name="vis">')
_a("          <geometry>")
_a("            <plane><normal>0 0 1</normal><size>500 500</size></plane>")
_a("          </geometry>")
_a("          <material>")
_a("            <ambient>0.25 0.25 0.25 1</ambient>")
_a("            <diffuse>0.25 0.25 0.25 1</diffuse>")
_a("          </material>")
_a("        </visual>")
_a("      </link>")
_a("    </model>")
_a("")
_a("    <!-- Maze walls -->")
_a('    <model name="maze_walls">')
_a("      <static>true</static>")

for idx, (lx, ly, sx, sy) in enumerate(wall_runs):
    size = f"{sx:.5f} {sy:.5f} {WALL_H:.5f}"
    geom = f"          <geometry><box><size>{size}</size></box></geometry>"
    _a(f'      <link name="w{idx:04d}">')
    _a(f"        <pose>{lx:.5f} {ly:.5f} {WALL_H / 2:.5f} 0 0 0</pose>")
    _a('        <collision name="col">')
    _a(geom)
    _a("          <surface>")
    _a("            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>")
    _a("            <contact><ode/></contact>")
    _a("          </surface>")
    _a("        </collision>")
    _a('        <visual name="vis">')
    _a(geom)
    _a("          <material>")
    _a("            <ambient>0.55 0.55 0.55 1</ambient>")
    _a("            <diffuse>0.55 0.55 0.55 1</diffuse>")
    _a("            <specular>0.1 0.1 0.1 1</specular>")
    _a("          </material>")
    _a("        </visual>")
    _a("      </link>")

_a("    </model>")
_a("")
_a("    <!--")
_a("      Robot spawn points (inside central 5x5 hub, facing north yaw=pi/2):")
_a("")
for i, (sx2, sy2) in enumerate(spawn_pts):
    _a(f'      Robot {i}: x="{sx2:.4f}"  y="{sy2:.4f}"  z="0.05"  yaw="1.5708"')
_a("    -->")
_a("")
_a("  </world>")
_a("</sdf>")

with open("maze_world.sdf", "w") as f:
    f.write("\n".join(sdf_lines) + "\n")

print(f"SDF written to maze_world.sdf  ({len(wall_runs)} wall links)")

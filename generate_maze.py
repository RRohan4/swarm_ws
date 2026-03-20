#!/usr/bin/env python3
"""
generate_maze.py — Gazebo Harmonic maze world generator
─────────────────────────────────────────────────────────
Algorithm : Recursive backtracking (depth-first, random neighbour selection)
            → produces a *perfect* maze (exactly one path between any two cells).

Outputs:
  ~/swarm_ws/maze_world.sdf   — Gazebo Harmonic SDF world
  ~/swarm_ws/maze.txt         — ASCII map  (also printed to stdout)

Usage:
  python3 generate_maze.py                    # default 15×15
  python3 generate_maze.py --rows 20 --cols 20 --seed 42
  python3 generate_maze.py --cell-size 1.5 --output /tmp/my_maze.sdf
"""

import argparse
import random
import sys
from pathlib import Path

# ── Default configuration ──────────────────────────────────────────────────────
DEFAULT_ROWS  = 15        # maze cells tall  (15 × 1.2 m ≈ 18 m)
DEFAULT_COLS  = 15        # maze cells wide
CELL_SIZE     = 1.2       # metres per cell — TurtleBot3 is ~0.3 m diameter,
                          #   passage width ≈ CELL_SIZE − WALL_T ≈ 1.08 m
WALL_T        = 0.12      # wall thickness (m)
WALL_H        = 0.5       # wall height (m) — above 2-D lidar plane (~0.2 m)

WALL_RGBA     = "0.55 0.55 0.55 1"
FLOOR_RGBA    = "0.25 0.25 0.25 1"

OUTPUT_SDF    = Path.home() / "swarm_ws" / "maze_world.sdf"
OUTPUT_ASCII  = Path.home() / "swarm_ws" / "maze.txt"

# ── Direction helpers ──────────────────────────────────────────────────────────
DIRS  = {"N": (-1, 0), "S": (1, 0), "E": (0, 1), "W": (0, -1)}
OPP   = {"N": "S", "S": "N", "E": "W", "W": "E"}


# ══════════════════════════════════════════════════════════════════════════════
#  MAZE GENERATION
# ══════════════════════════════════════════════════════════════════════════════

def generate_maze(rows: int, cols: int, seed=None) -> list[list[set]]:
    """
    Recursive backtracking maze generation.

    Returns
    -------
    walls[r][c] : set of direction strings {"N","S","E","W"}
        Directions where a *passage* has been carved (wall removed).
        Row 0 = top of maze, Col 0 = left of maze.
        Outer boundary walls are never in this set (no passage out of bounds),
        *except* for the entrance and exit cells where we explicitly add them.
    """
    rng = random.Random(seed)
    sys.setrecursionlimit(rows * cols * 4 + 500)

    walls   = [[set() for _ in range(cols)] for _ in range(rows)]
    visited = [[False] * cols               for _ in range(rows)]

    def carve(r: int, c: int):
        visited[r][c] = True
        directions = list(DIRS.keys())
        rng.shuffle(directions)
        for d in directions:
            dr, dc = DIRS[d]
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                walls[r][c].add(d)           # open passage from (r,c)
                walls[nr][nc].add(OPP[d])    # open passage from neighbour
                carve(nr, nc)

    carve(0, 0)

    # Entrance: gap in south wall of bottom-left cell  (world y ≈ 0)
    walls[rows - 1][0].add("S")
    # Exit:     gap in north wall of top-right cell    (world y ≈ rows*W)
    walls[0][cols - 1].add("N")

    return walls


# ══════════════════════════════════════════════════════════════════════════════
#  ASCII RENDERING
# ══════════════════════════════════════════════════════════════════════════════

def render_ascii(walls: list[list[set]], rows: int, cols: int) -> str:
    """
    Classic box-drawing ASCII maze.  Example for a 3×3:

      +---+---+ EX
      |       |
      +   +---+
      |   |   |
      +   +   +
      |       |
    EN+---+---+

    Row numbers run down the left side, column numbers along the bottom,
    so you can cross-reference cells with world coordinates.
    """
    lines = []

    # ── Top border (row 0 north wall / exit) ──────────────────────────────
    top = "    +"
    for c in range(cols):
        top += ("   " if "N" in walls[0][c] else "---") + "+"
    lines.append(top + "  ← EXIT (col " + str(cols - 1) + ")")

    # ── Cell rows ─────────────────────────────────────────────────────────
    for r in range(rows):
        # Cell content row
        row_str = f"{r:3d} "
        for c in range(cols):
            west = " " if (c == 0 and "W" in walls[r][c]) else "|"
            row_str += west + "   "
        east = " " if "E" in walls[r][cols - 1] else "|"
        row_str += east
        lines.append(row_str)

        # South wall row
        south_str = "    +"
        for c in range(cols):
            south_str += ("   " if "S" in walls[r][c] else "---") + "+"
        if r == rows - 1:
            south_str += "  ← ENTRANCE (col 0)"
        lines.append(south_str)

    # Column index footer
    col_footer = "     "
    for c in range(cols):
        col_footer += f"{c:3d} "
    lines.append(col_footer)

    header = (
        f"Maze {rows}×{cols}  "
        f"cell={CELL_SIZE:.2f} m  "
        f"world={cols*CELL_SIZE:.1f} m × {rows*CELL_SIZE:.1f} m\n"
        f"Passage width ≈ {CELL_SIZE - WALL_T:.2f} m  "
        f"wall t={WALL_T:.2f} m  h={WALL_H:.2f} m\n"
    )
    return header + "\n".join(lines)


# ══════════════════════════════════════════════════════════════════════════════
#  SDF GENERATION
# ══════════════════════════════════════════════════════════════════════════════

def _link_box(name: str,
              cx: float, cy: float, cz: float,
              sx: float, sy: float, sz: float,
              rgba: str) -> str:
    """Single static box link (collision + visual)."""
    return f"""\
      <link name="{name}">
        <pose>{cx:.5f} {cy:.5f} {cz:.5f} 0 0 0</pose>
        <collision name="col">
          <geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <contact><ode/></contact>
          </surface>
        </collision>
        <visual name="vis">
          <geometry><box><size>{sx:.5f} {sy:.5f} {sz:.5f}</size></box></geometry>
          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>"""


def build_wall_links(walls: list[list[set]],
                     rows: int, cols: int,
                     W: float, t: float, h: float) -> list[str]:
    """
    Collect unique wall box links using a 4-rule deduplication scheme:

        Rule 1 — south wall  of every cell (r, c) if passage absent
        Rule 2 — east  wall  of every cell (r, c) if passage absent
        Rule 3 — north outer wall of row-0  cells if passage absent  (exit gap)
        Rule 4 — west  outer wall of col-0  cells if passage absent

    Coverage (no duplicates):
        Interior H-walls  → Rule 1 (south of upper cell only)
        Interior V-walls  → Rule 2 (east  of left  cell only)
        South boundary    → Rule 1 (row rows-1)
        North boundary    → Rule 3
        East  boundary    → Rule 2 (col cols-1)
        West  boundary    → Rule 4

    World frame: origin at SW corner of maze.
        +x = east  (increasing col)
        +y = north (decreasing row index)
    Cell (r, c) SW corner at world ( c*W, (rows-1-r)*W ).
    """
    links = []
    idx   = [0]

    def add(cx, cy, sx, sy):
        links.append(_link_box(f"w{idx[0]:04d}", cx, cy, h / 2,
                               sx, sy, h, WALL_RGBA))
        idx[0] += 1

    for r in range(rows):
        rw = rows - 1 - r          # world-y row index (0 = south edge of maze)
        for c in range(cols):
            # Rule 1: south wall → horizontal bar at y = rw * W
            if "S" not in walls[r][c]:
                add((c + 0.5) * W,  rw * W,          W, t)

            # Rule 2: east wall  → vertical bar at x = (c+1) * W
            if "E" not in walls[r][c]:
                add((c + 1) * W,    (rw + 0.5) * W,  t, W)

    # Rule 3: north outer boundary (y = rows * W)
    for c in range(cols):
        if "N" not in walls[0][c]:
            add((c + 0.5) * W,  rows * W,   W, t)

    # Rule 4: west outer boundary (x = 0)
    for r in range(rows):
        rw = rows - 1 - r
        if "W" not in walls[r][0]:
            add(0,  (rw + 0.5) * W,          t, W)

    return links


def generate_sdf(walls: list[list[set]],
                 rows: int, cols: int,
                 W: float = CELL_SIZE,
                 seed=None) -> str:
    """Assemble the full Gazebo Harmonic SDF world string."""
    t, h = WALL_T, WALL_H

    wall_links = build_wall_links(walls, rows, cols, W, t, h)
    wall_block  = "\n".join(wall_links)

    world_w = cols * W
    world_h = rows * W

    # Robot 1 spawn: centre of bottom-left cell, facing north (+y = into maze)
    r1_x   = 0.5 * W
    r1_y   = 0.5 * W
    r1_z   = 0.05
    r1_yaw = 1.5708   # π/2 rad → facing +y (north into maze)

    # Placeholder spawns for robots 2-N, offset east by one cell each
    extra_spawns = "\n".join(
        f"      <!-- Robot {i}: x=\"{r1_x + (i-1)*W:.4f}\" y=\"{r1_y:.4f}\" "
        f"z=\"{r1_z}\" yaw=\"{r1_yaw}\" -->"
        for i in range(2, 9)
    )

    seed_str = str(seed) if seed is not None else "random"

    return f"""\
<?xml version="1.0" ?>
<!--
  Gazebo Harmonic maze world
  Generated by generate_maze.py
  Maze:      {rows}×{cols} cells, cell size {W:.2f} m
  World:     {world_w:.2f} m × {world_h:.2f} m
  Passages:  ≈{W - t:.2f} m wide  (TurtleBot3 ≈ 0.30 m diameter)
  Seed:      {seed_str}
  Entrance:  south wall of (row={rows-1}, col=0)  → world x≈{r1_x:.2f}, y≈0
  Exit:      north wall of (row=0, col={cols-1})   → world x≈{(cols-0.5)*W:.2f}, y≈{world_h:.2f}
-->
<sdf version="1.9">
  <world name="maze_world">

    <!-- ── Physics ─────────────────────────────────────────────────────── -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- ── Required Gazebo Harmonic system plugins ──────────────────────── -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>

    <!-- ── Scene ────────────────────────────────────────────────────────── -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.15 0.15 0.15 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>

    <!-- ── Directional sunlight ─────────────────────────────────────────── -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <!-- ── Ground plane ─────────────────────────────────────────────────── -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry>
            <plane><normal>0 0 1</normal><size>500 500</size></plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="vis">
          <geometry>
            <plane><normal>0 0 1</normal><size>500 500</size></plane>
          </geometry>
          <material>
            <ambient>{FLOOR_RGBA}</ambient>
            <diffuse>{FLOOR_RGBA}</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ── Maze walls ({len(wall_links)} segments) ──────────────────────────────────── -->
    <!--
      All walls in one static model. One <link> per wall segment.
      Horizontal walls: size_x=W, size_y=t  (run along x-axis)
      Vertical   walls: size_x=t, size_y=W  (run along y-axis)
    -->
    <model name="maze_walls">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
{wall_block}
    </model>

    <!-- ── Robot spawn points ───────────────────────────────────────────── -->
    <!--
      Robots start near the entrance (south of maze, left side), facing
      north (yaw = π/2 ≈ 1.5708 rad) into the maze.

      Each additional robot is offset +{W:.2f} m along +x (one cell east),
      so robots spawn in separate corridor cells and don't overlap.

      Use these values in your launch file (x_pose, y_pose, z_pose, yaw):

      Robot 1 (active):  x="{r1_x:.4f}"  y="{r1_y:.4f}"  z="{r1_z}"  yaw="{r1_yaw}"
{extra_spawns}
    -->

  </world>
</sdf>
"""


# ══════════════════════════════════════════════════════════════════════════════
#  CLI
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Generate a recursive-backtracking maze SDF for Gazebo Harmonic"
    )
    parser.add_argument("--rows",      type=int,   default=DEFAULT_ROWS,
                        help=f"Maze height in cells (default {DEFAULT_ROWS})")
    parser.add_argument("--cols",      type=int,   default=DEFAULT_COLS,
                        help=f"Maze width  in cells (default {DEFAULT_COLS})")
    parser.add_argument("--cell-size", type=float, default=CELL_SIZE,
                        help=f"Cell size in metres (default {CELL_SIZE})")
    parser.add_argument("--seed",      type=int,   default=None,
                        help="RNG seed for reproducible mazes (default: random)")
    parser.add_argument("--output",    type=str,   default=str(OUTPUT_SDF),
                        help=f"SDF output path (default {OUTPUT_SDF})")
    parser.add_argument("--ascii",     type=str,   default=str(OUTPUT_ASCII),
                        help=f"ASCII map output path (default {OUTPUT_ASCII})")
    args = parser.parse_args()

    W = args.cell_size

    print(f"\nGenerating {args.rows}×{args.cols} maze  "
          f"(cell={W:.2f} m, seed={args.seed}) …")

    walls = generate_maze(args.rows, args.cols, seed=args.seed)

    # ── ASCII ──────────────────────────────────────────────────────────────
    ascii_map = render_ascii(walls, args.rows, args.cols)
    print("\n" + ascii_map)

    ascii_path = Path(args.ascii)
    ascii_path.parent.mkdir(parents=True, exist_ok=True)
    ascii_path.write_text(ascii_map)
    print(f"\nASCII map saved → {ascii_path}")

    # ── SDF ───────────────────────────────────────────────────────────────
    sdf = generate_sdf(walls, args.rows, args.cols, W=W, seed=args.seed)

    sdf_path = Path(args.output)
    sdf_path.parent.mkdir(parents=True, exist_ok=True)
    sdf_path.write_text(sdf)
    print(f"SDF world saved → {sdf_path}")

    # ── Summary ───────────────────────────────────────────────────────────
    r1_x, r1_y = 0.5 * W, 0.5 * W
    print(f"""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 World size  : {args.cols * W:.2f} m  ×  {args.rows * W:.2f} m
 Passage     : {W - WALL_T:.2f} m wide  (TurtleBot3 ≈ 0.30 m)
 Entrance    : south of (row={args.rows-1}, col=0)
 Exit        : north of (row=0, col={args.cols-1})

 Robot 1 spawn (paste into launch file):
   x_pose="{r1_x:.4f}"  y_pose="{r1_y:.4f}"  z_pose="0.05"  yaw="1.5708"

 To load world directly:
   gz sim {sdf_path}

 Robots 2-N spawn offsets are commented inside the SDF.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")


if __name__ == "__main__":
    main()

"""A* Path Planner — finds door-aware paths through the apartment.

This is the missing "brain stem" between the LLM's high-level goals
and the robot's low-level movement. It:

  1. Converts the wall layout into an occupancy grid
  2. Runs A* search to find the shortest obstacle-free path
  3. Simplifies the path into clean waypoints
  4. Ensures paths go THROUGH doors, not through walls

This is what makes the robot navigate like a human — it knows
you have to go through the hallway door to reach the kitchen.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Optional

from loguru import logger


@dataclass
class GridMap:
    """2D occupancy grid for path planning.

    Each cell is:
      0 = free (robot can pass)
      1 = occupied (wall)
    """
    grid: list[list[int]]
    resolution: float       # meters per cell
    width: int              # cells
    height: int             # cells
    origin_x: float = 0.0   # world X of grid[0][0]
    origin_y: float = 0.0   # world Y of grid[0][0]

    def is_free(self, gx: int, gy: int) -> bool:
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.grid[gy][gx] == 0
        return False

    def world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return max(0, min(self.width - 1, gx)), max(0, min(self.height - 1, gy))

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        wx = gx * self.resolution + self.origin_x + self.resolution / 2
        wy = gy * self.resolution + self.origin_y + self.resolution / 2
        return wx, wy


def build_grid_from_walls(
    walls: list,
    world_width: float,
    world_height: float,
    resolution: float = 0.1,
    robot_radius: float = 0.15,
) -> GridMap:
    """
    Convert wall segments into an occupancy grid.

    Inflates walls by robot_radius so the path planner keeps
    the robot clear of walls.
    """
    grid_w = int(world_width / resolution) + 1
    grid_h = int(world_height / resolution) + 1
    grid = [[0] * grid_w for _ in range(grid_h)]

    inflate_cells = int(robot_radius / resolution) + 1

    for wall in walls:
        x1, y1, x2, y2 = wall.x1, wall.y1, wall.x2, wall.y2

        # Rasterize the wall segment onto the grid
        length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        steps = max(int(length / (resolution * 0.5)), 2)

        for i in range(steps + 1):
            t = i / steps
            wx = x1 + t * (x2 - x1)
            wy = y1 + t * (y2 - y1)
            gx = int(wx / resolution)
            gy = int(wy / resolution)

            # Inflate: mark cells around the wall as occupied
            for dy in range(-inflate_cells, inflate_cells + 1):
                for dx in range(-inflate_cells, inflate_cells + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < grid_w and 0 <= ny < grid_h:
                        grid[ny][nx] = 1

    occupied = sum(cell for row in grid for cell in row)
    total = grid_w * grid_h
    logger.info(
        f"Grid map built: {grid_w}x{grid_h} cells, "
        f"resolution={resolution}m, "
        f"{occupied}/{total} occupied ({100*occupied/total:.1f}%)"
    )

    return GridMap(
        grid=grid,
        resolution=resolution,
        width=grid_w,
        height=grid_h,
    )


def astar(
    grid_map: GridMap,
    start: tuple[float, float],
    goal: tuple[float, float],
) -> Optional[list[tuple[float, float]]]:
    """
    A* pathfinding on the occupancy grid.

    Args:
        grid_map: The occupancy grid
        start: (x, y) world coordinates of start
        goal: (x, y) world coordinates of goal

    Returns:
        List of (x, y) world-coordinate waypoints, or None if no path found.
    """
    sx, sy = grid_map.world_to_grid(start[0], start[1])
    gx, gy = grid_map.world_to_grid(goal[0], goal[1])

    # If start or goal is inside a wall, find nearest free cell
    sx, sy = _find_nearest_free(grid_map, sx, sy)
    gx, gy = _find_nearest_free(grid_map, gx, gy)

    if sx is None or gx is None:
        logger.warning("A*: start or goal is unreachable (inside wall)")
        return None

    # A* search
    open_set: list[tuple[float, int, int]] = []
    heapq.heappush(open_set, (0.0, sx, sy))

    came_from: dict[tuple[int, int], Optional[tuple[int, int]]] = {(sx, sy): None}
    g_score: dict[tuple[int, int], float] = {(sx, sy): 0.0}
    closed: set[tuple[int, int]] = set()

    # 8-directional movement
    neighbors = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414),
    ]

    iterations = 0
    max_iterations = grid_map.width * grid_map.height * 2

    while open_set and iterations < max_iterations:
        iterations += 1
        _, cx, cy = heapq.heappop(open_set)

        if (cx, cy) in closed:
            continue
        closed.add((cx, cy))

        if cx == gx and cy == gy:
            # Reconstruct path
            path_grid = []
            node: Optional[tuple[int, int]] = (cx, cy)
            while node is not None:
                path_grid.append(node)
                node = came_from.get(node)
            path_grid.reverse()

            # Convert to world coordinates
            path_world = [grid_map.grid_to_world(px, py) for px, py in path_grid]

            # Simplify path (remove redundant waypoints)
            simplified = _simplify_path(path_world, tolerance=0.2)

            logger.info(
                f"A* path found: {len(path_grid)} cells → "
                f"{len(simplified)} waypoints, {iterations} iterations"
            )
            return simplified

        for dx, dy, cost in neighbors:
            nx, ny = cx + dx, cy + dy

            if (nx, ny) in closed:
                continue

            if not grid_map.is_free(nx, ny):
                continue

            # For diagonal moves, check that both adjacent cells are free
            # (prevents cutting corners through walls)
            if dx != 0 and dy != 0:
                if not grid_map.is_free(cx + dx, cy) or not grid_map.is_free(cx, cy + dy):
                    continue

            new_g = g_score.get((cx, cy), float('inf')) + cost
            if new_g < g_score.get((nx, ny), float('inf')):
                g_score[(nx, ny)] = new_g
                h = math.sqrt((nx - gx) ** 2 + (ny - gy) ** 2)
                f = new_g + h
                came_from[(nx, ny)] = (cx, cy)
                heapq.heappush(open_set, (f, nx, ny))

    logger.warning(f"A*: no path found after {iterations} iterations")
    return None


def _find_nearest_free(
    grid_map: GridMap, gx: int, gy: int, max_search: int = 20
) -> tuple[Optional[int], Optional[int]]:
    """Find the nearest free cell to (gx, gy)."""
    if grid_map.is_free(gx, gy):
        return gx, gy

    for radius in range(1, max_search):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if abs(dx) == radius or abs(dy) == radius:
                    nx, ny = gx + dx, gy + dy
                    if grid_map.is_free(nx, ny):
                        return nx, ny
    return None, None


def _simplify_path(
    path: list[tuple[float, float]], tolerance: float = 0.2
) -> list[tuple[float, float]]:
    """
    Simplify a path using the Ramer-Douglas-Peucker algorithm.
    Removes unnecessary waypoints while keeping the path shape.
    """
    if len(path) <= 2:
        return path

    # Find the point farthest from the line between first and last
    start = path[0]
    end = path[-1]
    max_dist = 0.0
    max_idx = 0

    for i in range(1, len(path) - 1):
        dist = _point_line_distance(path[i], start, end)
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    if max_dist > tolerance:
        left = _simplify_path(path[:max_idx + 1], tolerance)
        right = _simplify_path(path[max_idx:], tolerance)
        return left[:-1] + right
    else:
        return [start, end]


def _point_line_distance(
    point: tuple[float, float],
    line_start: tuple[float, float],
    line_end: tuple[float, float],
) -> float:
    """Distance from a point to a line segment."""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    dx, dy = x2 - x1, y2 - y1
    length_sq = dx * dx + dy * dy

    if length_sq == 0:
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / length_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy

    return math.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)

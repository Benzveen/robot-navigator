"""Map Manager — handles occupancy grid maps from SLAM."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np
import yaml
from loguru import logger


class MapManager:
    """
    Manages the occupancy grid map produced by SLAM Toolbox.

    The map is a 2D numpy array where:
      - 0   = free space
      - 100 = occupied (wall/obstacle)
      - -1  = unknown/unexplored
    """

    def __init__(self, map_dir: str = "maps"):
        self.map_dir = Path(map_dir)
        self.map_dir.mkdir(parents=True, exist_ok=True)

        self.grid: Optional[np.ndarray] = None
        self.resolution: float = 0.05       # meters per pixel
        self.origin_x: float = 0.0          # map origin in meters
        self.origin_y: float = 0.0
        self.width: int = 0
        self.height: int = 0

        # Room labels (populated by LLM or manual annotation)
        self.room_labels: dict[str, dict] = {}

    def load_map(self, map_name: str = "current_map") -> bool:
        """
        Load a map from PGM + YAML files (standard ROS 2 map format).

        Args:
            map_name: Base name of the map files (without extension)

        Returns:
            True if map loaded successfully
        """
        yaml_path = self.map_dir / f"{map_name}.yaml"
        pgm_path = self.map_dir / f"{map_name}.pgm"

        if not yaml_path.exists():
            logger.warning(f"Map YAML not found: {yaml_path}")
            return False

        # Load map metadata
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        self.resolution = meta.get("resolution", 0.05)
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        self.origin_x = origin[0]
        self.origin_y = origin[1]

        # Load map image
        image_file = self.map_dir / meta.get("image", f"{map_name}.pgm")
        if not image_file.exists():
            logger.warning(f"Map image not found: {image_file}")
            return False

        try:
            import cv2
            img = cv2.imread(str(image_file), cv2.IMREAD_GRAYSCALE)
        except ImportError:
            # Fallback: read PGM manually
            img = self._read_pgm(str(image_file))

        if img is None:
            logger.error("Failed to load map image")
            return False

        # Convert to occupancy grid (-1, 0, 100)
        self.grid = self._image_to_occupancy(img, meta)
        self.height, self.width = self.grid.shape

        logger.info(
            f"Map loaded: {self.width}x{self.height}, "
            f"resolution={self.resolution}m/px, "
            f"origin=({self.origin_x:.2f}, {self.origin_y:.2f})"
        )
        return True

    def save_map(self, map_name: str = "current_map") -> bool:
        """Save the current map to PGM + YAML files."""
        if self.grid is None:
            logger.warning("No map to save")
            return False

        yaml_path = self.map_dir / f"{map_name}.yaml"
        pgm_path = self.map_dir / f"{map_name}.pgm"

        # Save metadata
        meta = {
            "image": f"{map_name}.pgm",
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }
        with open(yaml_path, "w") as f:
            yaml.dump(meta, f, default_flow_style=False)

        # Save image
        img = self._occupancy_to_image(self.grid)
        try:
            import cv2
            cv2.imwrite(str(pgm_path), img)
        except ImportError:
            self._write_pgm(str(pgm_path), img)

        logger.info(f"Map saved: {yaml_path}")
        return True

    # ── Map Queries ─────────────────────────────────────────────────

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates (meters) to grid cell indices."""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        """Convert grid cell indices to world coordinates (meters)."""
        x = gx * self.resolution + self.origin_x
        y = gy * self.resolution + self.origin_y
        return x, y

    def is_free(self, x: float, y: float) -> bool:
        """Check if a world position is free space."""
        if self.grid is None:
            return True
        gx, gy = self.world_to_grid(x, y)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.grid[gy, gx] == 0
        return False

    def get_explored_percentage(self) -> float:
        """Calculate what percentage of the map has been explored."""
        if self.grid is None:
            return 0.0
        total = self.grid.size
        unknown = np.count_nonzero(self.grid == -1)
        return ((total - unknown) / total) * 100.0

    def get_frontier_cells(self) -> list[tuple[int, int]]:
        """
        Find frontier cells — free cells adjacent to unknown cells.
        These are the boundaries of explored space.
        """
        if self.grid is None:
            return []

        frontiers = []
        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                if self.grid[y, x] != 0:
                    continue
                # Check 4-connected neighbors for unknown cells
                neighbors = [
                    self.grid[y - 1, x],
                    self.grid[y + 1, x],
                    self.grid[y, x - 1],
                    self.grid[y, x + 1],
                ]
                if -1 in neighbors:
                    frontiers.append((x, y))

        return frontiers

    def get_map_info(self) -> dict:
        """Get map summary for LLM context."""
        if self.grid is None:
            return {
                "status": "no_map",
                "explored_percentage": 0.0,
                "rooms": [],
                "frontier_count": 0,
            }

        frontiers = self.get_frontier_cells()
        return {
            "status": "ready",
            "width_m": round(self.width * self.resolution, 2),
            "height_m": round(self.height * self.resolution, 2),
            "resolution": self.resolution,
            "explored_percentage": round(self.get_explored_percentage(), 1),
            "rooms": list(self.room_labels.keys()),
            "frontier_count": len(frontiers),
        }

    def label_room(self, name: str, x: float, y: float) -> None:
        """Label a position on the map as a named room."""
        self.room_labels[name] = {"x": x, "y": y}
        logger.info(f"Room labeled: '{name}' at ({x:.2f}, {y:.2f})")

    # ── Internal Helpers ────────────────────────────────────────────

    @staticmethod
    def _image_to_occupancy(img: np.ndarray, meta: dict) -> np.ndarray:
        """Convert grayscale image to occupancy grid values."""
        occupied_thresh = meta.get("occupied_thresh", 0.65)
        free_thresh = meta.get("free_thresh", 0.196)

        # Normalize to 0-1
        normalized = img.astype(float) / 255.0

        grid = np.full(img.shape, -1, dtype=np.int8)  # default unknown
        grid[normalized > (1 - free_thresh)] = 0       # free
        grid[normalized < (1 - occupied_thresh)] = 100  # occupied

        return grid

    @staticmethod
    def _occupancy_to_image(grid: np.ndarray) -> np.ndarray:
        """Convert occupancy grid back to grayscale image."""
        img = np.full(grid.shape, 205, dtype=np.uint8)  # unknown = gray
        img[grid == 0] = 254                              # free = white
        img[grid == 100] = 0                              # occupied = black
        return img

    @staticmethod
    def _read_pgm(filepath: str) -> Optional[np.ndarray]:
        """Read a PGM file without OpenCV."""
        try:
            with open(filepath, "rb") as f:
                header = f.readline().decode().strip()
                if header not in ("P5", "P2"):
                    return None
                # Skip comments
                line = f.readline().decode().strip()
                while line.startswith("#"):
                    line = f.readline().decode().strip()
                width, height = map(int, line.split())
                max_val = int(f.readline().decode().strip())
                data = np.frombuffer(f.read(), dtype=np.uint8)
                return data.reshape((height, width))
        except Exception as e:
            logger.error(f"PGM read error: {e}")
            return None

    @staticmethod
    def _write_pgm(filepath: str, img: np.ndarray) -> None:
        """Write a PGM file without OpenCV."""
        height, width = img.shape
        with open(filepath, "wb") as f:
            f.write(f"P5\n{width} {height}\n255\n".encode())
            f.write(img.tobytes())

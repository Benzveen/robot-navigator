"""Virtual World — a simple 2D simulated environment for testing.

Provides:
  - A room layout with walls
  - Simulated LiDAR scans based on robot position
  - Simulated odometry (position tracking)
  - Collision detection
  - Battery drain simulation

This lets you test the full navigation + LLM stack on your laptop
without any real hardware.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import Optional

from loguru import logger


@dataclass
class Wall:
    """A wall segment defined by two endpoints."""
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class RoomLayout:
    """A collection of walls defining the environment."""
    name: str
    walls: list[Wall] = field(default_factory=list)
    rooms: dict[str, tuple[float, float]] = field(default_factory=dict)  # name → (x, y)
    width: float = 10.0   # meters
    height: float = 8.0   # meters


def create_apartment_layout() -> RoomLayout:
    """
    Create a simple apartment layout for simulation.

    Layout (10m x 8m):
    ┌──────────────────────────────────┐
    │           Hallway                │
    │     ┌──────────┐                 │
    │     │          │                 │
    │     │ Bedroom  │   Living Room   │
    │     │          │                 │
    │     └────  ────┘                 │
    │                                  │
    │     ┌──────────┐                 │
    │     │          │                 │
    │     │ Kitchen  │                 │
    │     │          │                 │
    │     └──────────┘                 │
    └──────────────────────────────────┘
    """
    layout = RoomLayout(
        name="apartment",
        width=10.0,
        height=8.0,
    )

    # Outer walls
    layout.walls.extend([
        Wall(0, 0, 10, 0),    # bottom
        Wall(10, 0, 10, 8),   # right
        Wall(10, 8, 0, 8),    # top
        Wall(0, 8, 0, 0),     # left
    ])

    # Bedroom walls (bottom-left, 4m x 3m)
    layout.walls.extend([
        Wall(0, 3, 4, 3),     # top wall of bedroom
        Wall(4, 0, 4, 1.2),   # right wall (with door gap)
        Wall(4, 1.8, 4, 3),   # right wall (above door)
    ])

    # Kitchen walls (top-left, 4m x 3m)
    layout.walls.extend([
        Wall(0, 5, 4, 5),     # bottom wall of kitchen
        Wall(4, 5, 4, 6.2),   # right wall (with door gap)
        Wall(4, 6.8, 4, 8),   # right wall (above door)
    ])

    # Room center coordinates
    layout.rooms = {
        "bedroom": (2.0, 1.5),
        "kitchen": (2.0, 6.5),
        "living_room": (7.0, 4.0),
        "hallway": (4.5, 4.0),
    }

    return layout


@dataclass
class VirtualRobot:
    """Simulated robot state in the virtual world."""
    x: float = 5.0          # starting position (hallway)
    y: float = 4.0
    yaw: float = 0.0        # radians
    speed: float = 0.0      # m/s
    angular_speed: float = 0.0  # rad/s
    battery: float = 100.0  # percentage
    max_speed: float = 0.3  # m/s


class VirtualWorld:
    """
    A 2D simulated environment for testing the robot system.

    Simulates:
    - Robot movement with differential drive kinematics
    - LiDAR scans via raycasting against walls
    - Battery drain over time
    - Collision detection
    """

    def __init__(self, layout: Optional[RoomLayout] = None):
        self.layout = layout or create_apartment_layout()
        self.robot = VirtualRobot()
        self._last_update = time.time()
        self._running = False

        logger.info(
            f"🏠 Virtual world created: '{self.layout.name}' "
            f"({self.layout.width}m x {self.layout.height}m), "
            f"{len(self.layout.walls)} walls, "
            f"{len(self.layout.rooms)} rooms"
        )

    # ── Robot Movement ──────────────────────────────────────────

    def update(self, dt: Optional[float] = None) -> None:
        """Update robot position based on current velocity."""
        now = time.time()
        if dt is None:
            dt = now - self._last_update
        self._last_update = now

        if dt <= 0 or dt > 1.0:
            return

        # Update orientation
        self.robot.yaw += self.robot.angular_speed * dt
        # Normalize to [-pi, pi]
        self.robot.yaw = math.atan2(math.sin(self.robot.yaw), math.cos(self.robot.yaw))

        # Update position
        new_x = self.robot.x + self.robot.speed * math.cos(self.robot.yaw) * dt
        new_y = self.robot.y + self.robot.speed * math.sin(self.robot.yaw) * dt

        # Collision check — only move if no wall collision
        if not self._check_collision(new_x, new_y):
            self.robot.x = new_x
            self.robot.y = new_y
        else:
            self.robot.speed = 0  # Stop on collision
            logger.debug("💥 Collision detected — robot stopped")

        # Battery drain (very slow: ~0.1% per second when moving)
        if self.robot.speed != 0:
            self.robot.battery = max(0, self.robot.battery - 0.1 * dt)

    def set_velocity(self, linear: float, angular: float) -> None:
        """Set robot velocity (called by motor controller sim)."""
        self.robot.speed = max(-self.robot.max_speed, min(self.robot.max_speed, linear))
        self.robot.angular_speed = max(-1.0, min(1.0, angular))

    def move_to(self, target_x: float, target_y: float, speed: float = 0.15) -> bool:
        """
        Instantly teleport toward a target (simplified navigation).
        Returns True when target is reached.
        """
        dx = target_x - self.robot.x
        dy = target_y - self.robot.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 0.15:  # Close enough
            self.robot.speed = 0
            return True

        # Set heading toward target
        self.robot.yaw = math.atan2(dy, dx)
        self.robot.speed = min(speed, distance)

        # Step forward
        step = min(speed * 0.1, distance)
        new_x = self.robot.x + step * math.cos(self.robot.yaw)
        new_y = self.robot.y + step * math.sin(self.robot.yaw)

        if not self._check_collision(new_x, new_y):
            self.robot.x = new_x
            self.robot.y = new_y

        return distance < 0.15

    # ── LiDAR Simulation ───────────────────────────────────────

    def get_lidar_scan(self, num_rays: int = 360, max_range: float = 12.0) -> list[float]:
        """
        Simulate a 360° LiDAR scan via raycasting.

        Returns a list of distances (meters) for each degree.
        """
        ranges = []
        for i in range(num_rays):
            angle = self.robot.yaw + math.radians(i)
            distance = self._raycast(self.robot.x, self.robot.y, angle, max_range)
            # Add some noise (±2cm)
            distance += random.uniform(-0.02, 0.02)
            distance = max(0.05, min(max_range, distance))
            ranges.append(distance)
        return ranges

    def get_nearest_obstacle(self) -> tuple[float, float]:
        """Get distance and angle to nearest obstacle."""
        scan = self.get_lidar_scan(num_rays=72, max_range=5.0)  # 5° resolution
        min_dist = min(scan)
        min_idx = scan.index(min_dist)
        angle = math.radians(min_idx * 5)
        return min_dist, angle

    # ── State Queries ──────────────────────────────────────────

    def get_current_room(self) -> str:
        """Determine which room the robot is currently in."""
        for room_name, (cx, cy) in self.layout.rooms.items():
            # Simple distance-based room detection
            dist = math.sqrt((self.robot.x - cx) ** 2 + (self.robot.y - cy) ** 2)
            if dist < 2.5:
                return room_name
        return "hallway"

    def get_robot_state(self) -> dict:
        """Get complete robot state for the LLM agent."""
        nearest_dist, nearest_angle = self.get_nearest_obstacle()
        return {
            "x": round(self.robot.x, 3),
            "y": round(self.robot.y, 3),
            "yaw": round(self.robot.yaw, 3),
            "yaw_deg": round(math.degrees(self.robot.yaw), 1),
            "linear_speed": round(self.robot.speed, 3),
            "angular_speed": round(self.robot.angular_speed, 3),
            "max_speed": self.robot.max_speed,
            "battery_level": round(self.robot.battery, 1),
            "nearest_obstacle": {
                "distance": round(nearest_dist, 3),
                "angle_deg": round(math.degrees(nearest_angle), 1),
                "is_critical": nearest_dist < 0.3,
            },
            "obstacle_range": 0.3,
            "current_room": self.get_current_room(),
            "is_navigating": self.robot.speed != 0,
            "map_status": "ready",
            "explored_percentage": 85.0,
            "is_safe": nearest_dist > 0.3 and self.robot.battery > 5,
            "is_emergency_stopped": False,
            "sensors": {
                "imu": True,
                "lidar": True,
                "arduino": True,
            },
            "uptime": round(time.time() - self._last_update + 100, 1),
        }

    # ── Internal Helpers ───────────────────────────────────────

    def _raycast(self, ox: float, oy: float, angle: float, max_range: float) -> float:
        """Cast a ray from (ox, oy) at angle and return distance to nearest wall."""
        dx = math.cos(angle)
        dy = math.sin(angle)
        min_dist = max_range

        for wall in self.layout.walls:
            dist = self._ray_segment_intersection(
                ox, oy, dx, dy,
                wall.x1, wall.y1, wall.x2, wall.y2,
            )
            if dist is not None and dist < min_dist:
                min_dist = dist

        return min_dist

    @staticmethod
    def _ray_segment_intersection(
        ox: float, oy: float, dx: float, dy: float,
        x1: float, y1: float, x2: float, y2: float,
    ) -> Optional[float]:
        """
        Find intersection of ray (ox,oy)+t*(dx,dy) with segment (x1,y1)-(x2,y2).
        Returns distance t, or None if no intersection.
        """
        sx = x2 - x1
        sy = y2 - y1

        denom = dx * sy - dy * sx
        if abs(denom) < 1e-10:
            return None  # Parallel

        t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom

        if t > 0.01 and 0 <= u <= 1:
            return t
        return None

    def _check_collision(self, x: float, y: float, radius: float = 0.12) -> bool:
        """Check if position (x, y) collides with any wall."""
        # Simple: check if any raycast in 8 directions hits within radius
        for i in range(8):
            angle = i * math.pi / 4
            dist = self._raycast(x, y, angle, radius * 2)
            if dist < radius:
                return True

        # Also check bounds
        if x < radius or x > self.layout.width - radius:
            return True
        if y < radius or y > self.layout.height - radius:
            return True

        return False

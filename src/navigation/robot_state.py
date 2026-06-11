"""Robot state manager — aggregates data from all sensors and subsystems."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

from loguru import logger


@dataclass
class Pose2D:
    """2D pose in the map frame."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # radians

    def distance_to(self, other: Pose2D) -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "yaw_deg": math.degrees(self.yaw)}


@dataclass
class ObstacleInfo:
    """Nearest obstacle information from LiDAR."""
    distance: float = 999.0       # meters
    angle: float = 0.0            # radians (relative to robot front)
    is_critical: bool = False     # True if within stop range

    def to_dict(self) -> dict:
        return {
            "distance": round(self.distance, 3),
            "angle_deg": round(math.degrees(self.angle), 1),
            "is_critical": self.is_critical,
        }


@dataclass
class RobotState:
    """
    Complete robot state — updated continuously from sensor data.

    This is the single source of truth for the robot's current condition.
    """

    # Position & motion
    pose: Pose2D = field(default_factory=Pose2D)
    linear_speed: float = 0.0     # m/s
    angular_speed: float = 0.0    # rad/s
    max_speed: float = 0.3        # m/s (configurable)

    # Sensors
    nearest_obstacle: ObstacleInfo = field(default_factory=ObstacleInfo)
    battery_level: float = 100.0  # percentage
    imu_connected: bool = False
    lidar_connected: bool = False
    arduino_connected: bool = False

    # Navigation
    current_room: str = "unknown"
    is_navigating: bool = False
    navigation_goal: Optional[Pose2D] = None
    map_status: str = "not started"  # "not started" | "mapping" | "ready"
    explored_percentage: float = 0.0

    # Safety
    obstacle_range: float = 0.5   # meters — stop threshold
    is_emergency_stopped: bool = False

    # Timestamps
    last_updated: float = field(default_factory=time.time)
    uptime_start: float = field(default_factory=time.time)

    def update_pose(self, x: float, y: float, yaw: float) -> None:
        """Update robot position from localization."""
        self.pose.x = x
        self.pose.y = y
        self.pose.yaw = yaw
        self.last_updated = time.time()

    def update_obstacle(self, distance: float, angle: float) -> None:
        """Update nearest obstacle from LiDAR scan."""
        self.nearest_obstacle.distance = distance
        self.nearest_obstacle.angle = angle
        self.nearest_obstacle.is_critical = distance < self.obstacle_range

        if self.nearest_obstacle.is_critical:
            logger.warning(
                f"⚠️ Critical obstacle at {distance:.2f}m, "
                f"angle {math.degrees(angle):.1f}°"
            )

    def is_safe_to_move(self) -> bool:
        """Check if it's safe for the robot to move."""
        if self.is_emergency_stopped:
            return False
        if self.nearest_obstacle.is_critical:
            return False
        if self.battery_level < 5:
            return False
        return True

    def should_return_home(self) -> bool:
        """Check if the robot should return to charging station."""
        return self.battery_level < 20

    @property
    def uptime(self) -> float:
        """Robot uptime in seconds."""
        return time.time() - self.uptime_start

    def to_dict(self) -> dict:
        """Serialize state for LLM context and dashboard."""
        return {
            "x": round(self.pose.x, 3),
            "y": round(self.pose.y, 3),
            "yaw": round(self.pose.yaw, 3),
            "yaw_deg": round(math.degrees(self.pose.yaw), 1),
            "linear_speed": round(self.linear_speed, 3),
            "angular_speed": round(self.angular_speed, 3),
            "max_speed": self.max_speed,
            "battery_level": round(self.battery_level, 1),
            "nearest_obstacle": self.nearest_obstacle.to_dict(),
            "obstacle_range": self.obstacle_range,
            "current_room": self.current_room,
            "is_navigating": self.is_navigating,
            "map_status": self.map_status,
            "explored_percentage": round(self.explored_percentage, 1),
            "is_safe": self.is_safe_to_move(),
            "is_emergency_stopped": self.is_emergency_stopped,
            "sensors": {
                "imu": self.imu_connected,
                "lidar": self.lidar_connected,
                "arduino": self.arduino_connected,
            },
            "uptime": round(self.uptime, 1),
        }

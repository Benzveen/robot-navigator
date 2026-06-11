"""Waypoint Motion Controller — follows A* paths smoothly.

This is the "legs" of the robot. Given a list of waypoints from
the path planner, it:

  1. Steers toward the next waypoint
  2. Moves forward at appropriate speed
  3. Slows down near obstacles
  4. Advances to the next waypoint when close enough
  5. Reports when the full path is complete

This makes the robot navigate through doors and around walls
like a human would — following the hallway, turning through
doorways, etc.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from loguru import logger


@dataclass
class NavigationGoal:
    """A navigation goal with its planned path."""
    target_x: float
    target_y: float
    target_room: Optional[str] = None
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    current_waypoint_idx: int = 0
    is_complete: bool = False
    is_failed: bool = False

    @property
    def current_waypoint(self) -> Optional[tuple[float, float]]:
        if self.current_waypoint_idx < len(self.waypoints):
            return self.waypoints[self.current_waypoint_idx]
        return None

    @property
    def remaining_waypoints(self) -> int:
        return max(0, len(self.waypoints) - self.current_waypoint_idx)


class MotionController:
    """
    Follows waypoint paths with smooth steering.

    Simulates how a real differential-drive robot would follow
    a planned path — turning toward each waypoint, driving forward,
    and advancing to the next one.
    """

    def __init__(
        self,
        waypoint_reach_dist: float = 0.25,   # meters — "close enough" to waypoint
        max_speed: float = 0.15,              # m/s
        min_speed: float = 0.03,              # m/s (slow near obstacles)
        slowdown_distance: float = 0.5,       # slow down this far from waypoint
        obstacle_slowdown: float = 0.8,       # slow down when obstacle closer than this
        obstacle_stop: float = 0.25,          # stop when obstacle closer than this
    ):
        self.waypoint_reach_dist = waypoint_reach_dist
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.slowdown_distance = slowdown_distance
        self.obstacle_slowdown = obstacle_slowdown
        self.obstacle_stop = obstacle_stop

        self.goal: Optional[NavigationGoal] = None

    @property
    def is_navigating(self) -> bool:
        return self.goal is not None and not self.goal.is_complete and not self.goal.is_failed

    def set_goal(self, goal: NavigationGoal) -> None:
        """Set a new navigation goal with planned waypoints."""
        self.goal = goal
        logger.info(
            f"🧭 Navigation goal set: ({goal.target_x:.1f}, {goal.target_y:.1f})"
            f" via {len(goal.waypoints)} waypoints"
        )

    def cancel(self) -> None:
        """Cancel current navigation."""
        self.goal = None

    def update(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        nearest_obstacle_dist: float,
    ) -> tuple[float, float]:
        """
        Compute velocity commands to follow the current path.

        Args:
            robot_x, robot_y: Current robot position (meters)
            robot_yaw: Current heading (radians)
            nearest_obstacle_dist: Distance to nearest obstacle (meters)

        Returns:
            (linear_speed, angular_speed) — velocity commands
        """
        if not self.is_navigating:
            return 0.0, 0.0

        wp = self.goal.current_waypoint
        if wp is None:
            self.goal.is_complete = True
            return 0.0, 0.0

        wx, wy = wp
        dx = wx - robot_x
        dy = wy - robot_y
        distance = math.sqrt(dx * dx + dy * dy)

        # Check if we've reached the current waypoint
        if distance < self.waypoint_reach_dist:
            self.goal.current_waypoint_idx += 1
            remaining = self.goal.remaining_waypoints

            if remaining == 0:
                self.goal.is_complete = True
                room_str = f" ({self.goal.target_room})" if self.goal.target_room else ""
                logger.info(
                    f"✅ Reached destination"
                    f" ({self.goal.target_x:.1f}, {self.goal.target_y:.1f}){room_str}"
                )
                return 0.0, 0.0
            else:
                next_wp = self.goal.current_waypoint
                logger.debug(
                    f"📍 Waypoint {self.goal.current_waypoint_idx}/{len(self.goal.waypoints)}"
                    f" → ({next_wp[0]:.1f}, {next_wp[1]:.1f}), {remaining} remaining"
                )
                # Recalculate for next waypoint
                wp = next_wp
                wx, wy = wp
                dx = wx - robot_x
                dy = wy - robot_y
                distance = math.sqrt(dx * dx + dy * dy)

        # Compute desired heading
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(desired_yaw - robot_yaw)

        # Angular speed: proportional to heading error
        angular_speed = 2.0 * yaw_error  # P-controller
        angular_speed = max(-1.0, min(1.0, angular_speed))

        # Linear speed: reduce when turning, near obstacles, or near waypoint
        speed = self.max_speed

        # Slow down when turning sharply
        if abs(yaw_error) > 0.5:  # ~30°
            speed *= 0.3
        elif abs(yaw_error) > 0.2:  # ~12°
            speed *= 0.7

        # Slow down near waypoint
        if distance < self.slowdown_distance:
            speed *= max(0.3, distance / self.slowdown_distance)

        # Slow down / stop near obstacles
        if nearest_obstacle_dist < self.obstacle_stop:
            speed = 0.0
            # Try to turn away from obstacle
            angular_speed = 0.5 if yaw_error >= 0 else -0.5
        elif nearest_obstacle_dist < self.obstacle_slowdown:
            factor = (nearest_obstacle_dist - self.obstacle_stop) / (self.obstacle_slowdown - self.obstacle_stop)
            speed *= max(0.2, factor)

        speed = max(self.min_speed if speed > 0 else 0.0, min(self.max_speed, speed))

        return speed, angular_speed

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

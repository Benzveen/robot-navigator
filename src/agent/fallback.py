"""Offline fallback logic when the LLM API is unreachable."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from loguru import logger


class FallbackMode(str, Enum):
    WAYPOINT = "waypoint"       # Continue following queued waypoints
    RETURN_HOME = "return_home" # Navigate back to home station
    STOP = "stop"               # Stop and wait for connectivity


@dataclass
class FallbackController:
    """Handles robot behavior when the LLM API is unavailable."""

    mode: FallbackMode = FallbackMode.WAYPOINT
    waypoint_queue: list[tuple[float, float]] = field(default_factory=list)
    home_position: tuple[float, float] = (0.0, 0.0)
    _last_api_check: float = 0.0
    _api_check_interval: float = 10.0  # seconds between retry checks

    def is_api_available(self) -> bool:
        """Check if the Anthropic API is reachable."""
        now = time.time()
        if now - self._last_api_check < self._api_check_interval:
            return False  # Don't spam checks

        self._last_api_check = now

        try:
            import anthropic
            import os

            client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))
            # Minimal API call to check connectivity
            client.messages.create(
                model="claude-haiku-4-20250414",
                max_tokens=10,
                messages=[{"role": "user", "content": "ping"}],
            )
            logger.info("API connectivity restored!")
            return True
        except Exception as e:
            logger.warning(f"API still unreachable: {e}")
            return False

    def get_fallback_action(self, robot_state: dict) -> Optional[dict]:
        """
        Determine the next action based on fallback mode.

        Returns a dict with action type and parameters, or None if no action.
        """
        battery = robot_state.get("battery_level", 100)

        # Critical battery — always go home regardless of mode
        if battery < 15:
            logger.warning(f"Critical battery ({battery}%) — returning home")
            return {
                "action": "navigate_to",
                "params": {"x": self.home_position[0], "y": self.home_position[1]},
                "reason": "critical_battery",
            }

        if self.mode == FallbackMode.STOP:
            logger.info("Fallback mode: STOP — waiting for API")
            return {"action": "stop", "params": {}, "reason": "offline_stop_mode"}

        if self.mode == FallbackMode.RETURN_HOME:
            logger.info("Fallback mode: RETURN_HOME — navigating home")
            return {
                "action": "navigate_to",
                "params": {"x": self.home_position[0], "y": self.home_position[1]},
                "reason": "offline_return_home",
            }

        if self.mode == FallbackMode.WAYPOINT:
            if self.waypoint_queue:
                waypoint = self.waypoint_queue[0]
                logger.info(f"Fallback mode: WAYPOINT — heading to ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
                return {
                    "action": "navigate_to",
                    "params": {"x": waypoint[0], "y": waypoint[1]},
                    "reason": "offline_waypoint",
                }
            else:
                logger.info("Fallback mode: WAYPOINT — no waypoints queued, stopping")
                return {"action": "stop", "params": {}, "reason": "no_waypoints"}

        return None

    def waypoint_reached(self) -> None:
        """Remove the current waypoint from the queue after reaching it."""
        if self.waypoint_queue:
            reached = self.waypoint_queue.pop(0)
            logger.info(f"Waypoint reached: ({reached[0]:.2f}, {reached[1]:.2f}). "
                        f"{len(self.waypoint_queue)} remaining.")

    def add_waypoint(self, x: float, y: float) -> None:
        """Queue a waypoint for offline navigation."""
        self.waypoint_queue.append((x, y))
        logger.debug(f"Waypoint queued: ({x:.2f}, {y:.2f}). "
                      f"Queue size: {len(self.waypoint_queue)}")

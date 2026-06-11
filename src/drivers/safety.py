"""Safety Monitor — watches for dangerous conditions and triggers emergency stops."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from loguru import logger


@dataclass
class SafetyConfig:
    """Safety thresholds and parameters."""
    obstacle_stop_distance: float = 0.3     # meters — emergency stop
    obstacle_slow_distance: float = 0.8     # meters — reduce speed
    min_battery_voltage: float = 9.0        # volts — shutdown threshold
    low_battery_voltage: float = 10.0       # volts — return home
    max_tilt_angle: float = 30.0            # degrees — robot tipping over
    watchdog_timeout: float = 5.0           # seconds — no heartbeat = stop
    check_interval: float = 0.1            # seconds between safety checks


@dataclass
class SafetyStatus:
    """Current safety status."""
    is_safe: bool = True
    emergency_stopped: bool = False
    warnings: list[str] = field(default_factory=list)
    last_check: float = 0.0


class SafetyMonitor:
    """
    Monitors robot safety conditions and triggers emergency stops.

    Checks:
    - Obstacle proximity (from LiDAR)
    - Battery voltage (from Arduino)
    - Robot tilt angle (from IMU)
    - Communication watchdog (Arduino heartbeat)
    """

    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self._status = SafetyStatus()
        self._running = False
        self._estop_callbacks: list[Callable] = []
        self._warning_callbacks: list[Callable[[str], None]] = []

        # Data sources (set by main.py during initialization)
        self._get_obstacle_distance: Optional[Callable[[], float]] = None
        self._get_battery_voltage: Optional[Callable[[], float]] = None
        self._get_tilt_angle: Optional[Callable[[], float]] = None
        self._motor_stop: Optional[Callable] = None

    @property
    def status(self) -> SafetyStatus:
        return self._status

    def on_emergency_stop(self, callback: Callable) -> None:
        """Register callback for emergency stop events."""
        self._estop_callbacks.append(callback)

    def on_warning(self, callback: Callable[[str], None]) -> None:
        """Register callback for safety warnings."""
        self._warning_callbacks.append(callback)

    def set_data_sources(
        self,
        obstacle_fn: Optional[Callable] = None,
        battery_fn: Optional[Callable] = None,
        tilt_fn: Optional[Callable] = None,
        motor_stop_fn: Optional[Callable] = None,
    ) -> None:
        """Connect data source functions."""
        self._get_obstacle_distance = obstacle_fn
        self._get_battery_voltage = battery_fn
        self._get_tilt_angle = tilt_fn
        self._motor_stop = motor_stop_fn

    async def start(self) -> None:
        """Start the safety monitoring loop."""
        self._running = True
        logger.info("🛡️ Safety monitor started")

        while self._running:
            try:
                await self._check_safety()
                await asyncio.sleep(self.config.check_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Safety monitor error: {e}")
                # On error, trigger emergency stop as precaution
                await self._trigger_emergency_stop("Safety monitor error")
                await asyncio.sleep(1.0)

    def stop(self) -> None:
        """Stop the safety monitor."""
        self._running = False
        logger.info("Safety monitor stopped")

    async def _check_safety(self) -> None:
        """Run all safety checks."""
        warnings = []
        is_safe = True

        # 1. Check obstacle distance
        if self._get_obstacle_distance:
            distance = self._get_obstacle_distance()
            if distance < self.config.obstacle_stop_distance:
                await self._trigger_emergency_stop(
                    f"Obstacle too close: {distance:.2f}m"
                )
                return
            elif distance < self.config.obstacle_slow_distance:
                warnings.append(f"Obstacle nearby: {distance:.2f}m")

        # 2. Check battery voltage
        if self._get_battery_voltage:
            voltage = self._get_battery_voltage()
            if voltage > 0:  # 0 means no reading
                if voltage < self.config.min_battery_voltage:
                    await self._trigger_emergency_stop(
                        f"Critical battery: {voltage:.1f}V"
                    )
                    return
                elif voltage < self.config.low_battery_voltage:
                    warnings.append(f"Low battery: {voltage:.1f}V")

        # 3. Check tilt angle
        if self._get_tilt_angle:
            tilt = self._get_tilt_angle()
            if abs(tilt) > self.config.max_tilt_angle:
                await self._trigger_emergency_stop(
                    f"Robot tilting: {tilt:.1f}°"
                )
                return
            elif abs(tilt) > self.config.max_tilt_angle * 0.7:
                warnings.append(f"Robot tilting: {tilt:.1f}°")

        # Update status
        self._status.is_safe = is_safe
        self._status.warnings = warnings
        self._status.last_check = time.time()

        # Emit warnings
        for warning in warnings:
            for callback in self._warning_callbacks:
                try:
                    callback(warning)
                except Exception:
                    pass

    async def _trigger_emergency_stop(self, reason: str) -> None:
        """Trigger an emergency stop."""
        logger.critical(f"🚨 EMERGENCY STOP: {reason}")

        self._status.emergency_stopped = True
        self._status.is_safe = False

        # Stop motors immediately
        if self._motor_stop:
            try:
                await self._motor_stop()
            except Exception as e:
                logger.error(f"Failed to stop motors during e-stop: {e}")

        # Notify callbacks
        for callback in self._estop_callbacks:
            try:
                callback(reason)
            except Exception:
                pass

    async def reset_emergency_stop(self) -> dict:
        """Reset emergency stop (manual override)."""
        logger.warning("⚠️ Emergency stop reset by operator")
        self._status.emergency_stopped = False
        self._status.is_safe = True
        return {"success": True, "message": "Emergency stop reset"}

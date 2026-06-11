"""LiDAR Driver — reads scan data from RPLidar sensor.

Provides 360° distance measurements for obstacle detection and SLAM.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass, field
from typing import Callable, Optional

from loguru import logger

try:
    from rplidar import RPLidar
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False
    logger.warning("rplidar not available — LidarDriver in simulation mode")


@dataclass
class ScanPoint:
    """Single LiDAR measurement."""
    angle: float      # degrees (0-360)
    distance: float   # millimeters
    quality: int = 0


@dataclass
class LidarScan:
    """Complete 360° LiDAR scan."""
    points: list[ScanPoint] = field(default_factory=list)
    timestamp: float = 0.0

    @property
    def count(self) -> int:
        return len(self.points)

    def get_nearest(self, angle_min: float = 0, angle_max: float = 360) -> Optional[ScanPoint]:
        """Get the nearest point within an angle range."""
        filtered = [
            p for p in self.points
            if angle_min <= p.angle <= angle_max and p.distance > 0
        ]
        if not filtered:
            return None
        return min(filtered, key=lambda p: p.distance)

    def get_nearest_front(self, fov: float = 60) -> Optional[ScanPoint]:
        """Get nearest obstacle in front of the robot (±fov/2 degrees)."""
        half = fov / 2
        # Front is 0° (or 360°), so we check two ranges
        filtered = [
            p for p in self.points
            if (p.angle <= half or p.angle >= 360 - half) and p.distance > 0
        ]
        if not filtered:
            return None
        return min(filtered, key=lambda p: p.distance)

    def to_ranges(self, num_bins: int = 360) -> list[float]:
        """Convert to fixed-size range array (for ROS LaserScan message)."""
        ranges = [float("inf")] * num_bins
        for p in self.points:
            idx = int(p.angle) % num_bins
            dist_m = p.distance / 1000.0  # mm to meters
            if dist_m > 0 and dist_m < ranges[idx]:
                ranges[idx] = dist_m
        return ranges


class LidarDriver:
    """
    RPLidar driver — reads continuous scan data.

    Provides:
    - Continuous scan streaming
    - Nearest obstacle detection
    - ROS-compatible range data
    """

    def __init__(self, port: str = "/dev/ttyUSB1", baud_rate: int = 115200):
        self.port = port
        self.baud_rate = baud_rate
        self._lidar: Optional[RPLidar] = None
        self._connected = False
        self._scanning = False
        self._latest_scan = LidarScan()
        self._scan_callbacks: list[Callable[[LidarScan], None]] = []

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def latest_scan(self) -> LidarScan:
        return self._latest_scan

    def on_scan(self, callback: Callable[[LidarScan], None]) -> None:
        """Register a callback for new scan data."""
        self._scan_callbacks.append(callback)

    async def connect(self) -> bool:
        """Connect to the RPLidar sensor."""
        if not RPLIDAR_AVAILABLE:
            logger.info("[SIM] LiDAR connected (simulation)")
            self._connected = True
            return True

        try:
            self._lidar = RPLidar(self.port, baudrate=self.baud_rate)
            info = self._lidar.get_info()
            health = self._lidar.get_health()

            logger.info(
                f"LiDAR connected: model={info.get('model', 'unknown')}, "
                f"firmware={info.get('firmware', 'unknown')}, "
                f"health={health[0]}"
            )

            if health[0] != "Good":
                logger.warning(f"LiDAR health: {health}")

            self._connected = True
            return True

        except Exception as e:
            logger.error(f"Failed to connect LiDAR: {e}")
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Disconnect from the LiDAR sensor."""
        self._scanning = False

        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception as e:
                logger.error(f"LiDAR disconnect error: {e}")

        self._connected = False
        logger.info("LiDAR disconnected")

    async def start_scanning(self) -> None:
        """Start continuous scanning in a background task."""
        if not self._connected:
            logger.warning("Cannot start scanning — LiDAR not connected")
            return

        self._scanning = True
        logger.info("🔄 LiDAR scanning started")

        if not RPLIDAR_AVAILABLE or self._lidar is None:
            # Simulation: generate fake scan data
            asyncio.create_task(self._simulate_scans())
            return

        asyncio.create_task(self._scan_loop())

    async def stop_scanning(self) -> None:
        """Stop scanning."""
        self._scanning = False
        if self._lidar:
            try:
                self._lidar.stop()
            except Exception:
                pass
        logger.info("LiDAR scanning stopped")

    async def _scan_loop(self) -> None:
        """Main scan reading loop (runs in background)."""
        try:
            for scan in self._lidar.iter_scans():
                if not self._scanning:
                    break

                points = [
                    ScanPoint(angle=angle, distance=distance, quality=quality)
                    for quality, angle, distance in scan
                    if distance > 0
                ]

                self._latest_scan = LidarScan(points=points)

                # Notify callbacks
                for callback in self._scan_callbacks:
                    try:
                        callback(self._latest_scan)
                    except Exception as e:
                        logger.error(f"Scan callback error: {e}")

                # Yield to event loop
                await asyncio.sleep(0)

        except Exception as e:
            logger.error(f"LiDAR scan loop error: {e}")
            self._scanning = False

    async def _simulate_scans(self) -> None:
        """Generate simulated scan data for testing."""
        import random
        import time

        while self._scanning:
            points = []
            for angle in range(0, 360, 1):
                # Simulate a room with walls
                base_distance = 2000  # 2 meters
                noise = random.uniform(-50, 50)
                distance = base_distance + noise

                # Simulate some closer obstacles
                if 170 <= angle <= 190:
                    distance = random.uniform(500, 800)

                points.append(ScanPoint(angle=float(angle), distance=distance))

            self._latest_scan = LidarScan(points=points, timestamp=time.time())

            for callback in self._scan_callbacks:
                try:
                    callback(self._latest_scan)
                except Exception as e:
                    logger.error(f"Scan callback error: {e}")

            await asyncio.sleep(0.1)  # ~10 Hz

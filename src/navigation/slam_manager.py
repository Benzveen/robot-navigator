"""SLAM Manager — controls SLAM Toolbox for mapping and localization."""

from __future__ import annotations

from enum import Enum
from typing import Optional

from loguru import logger

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Empty
    from slam_toolbox.srv import SaveMap, SerializePoseGraph

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    logger.warning("ROS 2 not available — SlamManager will run in simulation mode")


class SlamMode(str, Enum):
    MAPPING = "mapping"           # Active SLAM — building the map
    LOCALIZATION = "localization" # Map is built, just localizing
    OFF = "off"


class SlamManager:
    """
    Controls SLAM Toolbox via ROS 2 service calls.

    Modes:
    - MAPPING: Robot is exploring and building the map
    - LOCALIZATION: Map is complete, robot is localizing within it
    - OFF: SLAM is disabled
    """

    def __init__(self, node: Optional[object] = None):
        self._node = node
        self._mode = SlamMode.OFF

        if ROS_AVAILABLE and node is not None:
            # Service clients for SLAM Toolbox
            self._save_map_client = node.create_client(SaveMap, "/slam_toolbox/save_map")
            self._serialize_client = node.create_client(
                SerializePoseGraph, "/slam_toolbox/serialize_map"
            )
            self._pause_client = node.create_client(Empty, "/slam_toolbox/pause_new_measurements")
            self._resume_client = node.create_client(Empty, "/slam_toolbox/resume_new_measurements")
            logger.info("SlamManager connected to SLAM Toolbox services")
        else:
            logger.info("SlamManager running in simulation mode")

    @property
    def mode(self) -> SlamMode:
        return self._mode

    async def start_mapping(self) -> dict:
        """Start SLAM in mapping mode."""
        logger.info("🗺️ Starting SLAM mapping mode")
        self._mode = SlamMode.MAPPING

        if ROS_AVAILABLE and self._resume_client is not None:
            try:
                req = Empty.Request()
                await self._resume_client.call_async(req)
                return {"success": True, "message": "SLAM mapping started"}
            except Exception as e:
                logger.error(f"Failed to start SLAM: {e}")
                return {"success": False, "message": str(e)}

        return {"success": True, "message": "SLAM mapping started [simulation]"}

    async def stop_mapping(self) -> dict:
        """Pause SLAM measurements (stop updating the map)."""
        logger.info("⏸️ Pausing SLAM mapping")

        if ROS_AVAILABLE and self._pause_client is not None:
            try:
                req = Empty.Request()
                await self._pause_client.call_async(req)
                self._mode = SlamMode.LOCALIZATION
                return {"success": True, "message": "SLAM paused, switched to localization"}
            except Exception as e:
                logger.error(f"Failed to pause SLAM: {e}")
                return {"success": False, "message": str(e)}

        self._mode = SlamMode.LOCALIZATION
        return {"success": True, "message": "SLAM paused [simulation]"}

    async def save_map(self, map_name: str = "current_map") -> dict:
        """Save the current SLAM map to disk."""
        logger.info(f"💾 Saving map as '{map_name}'")

        if ROS_AVAILABLE and self._save_map_client is not None:
            try:
                req = SaveMap.Request()
                req.name.data = f"maps/{map_name}"
                result = await self._save_map_client.call_async(req)
                return {"success": True, "message": f"Map saved as '{map_name}'"}
            except Exception as e:
                logger.error(f"Failed to save map: {e}")
                return {"success": False, "message": str(e)}

        return {"success": True, "message": f"Map saved as '{map_name}' [simulation]"}

    async def serialize_map(self, map_name: str = "current_map") -> dict:
        """Serialize the pose graph for later localization."""
        logger.info(f"📦 Serializing pose graph as '{map_name}'")

        if ROS_AVAILABLE and self._serialize_client is not None:
            try:
                req = SerializePoseGraph.Request()
                req.filename = f"maps/{map_name}"
                result = await self._serialize_client.call_async(req)
                return {"success": True, "message": f"Pose graph serialized as '{map_name}'"}
            except Exception as e:
                logger.error(f"Failed to serialize: {e}")
                return {"success": False, "message": str(e)}

        return {"success": True, "message": f"Pose graph serialized [simulation]"}

    def get_status(self) -> dict:
        """Get current SLAM status."""
        return {
            "mode": self._mode.value,
            "is_mapping": self._mode == SlamMode.MAPPING,
            "is_localizing": self._mode == SlamMode.LOCALIZATION,
        }

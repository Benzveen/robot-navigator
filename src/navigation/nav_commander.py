"""Nav2 Commander — sends navigation goals to the ROS 2 Nav2 stack."""

from __future__ import annotations

import math
from typing import Optional

from loguru import logger

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    logger.warning("ROS 2 not available — NavCommander will run in simulation mode")


class NavCommander:
    """
    Interface to ROS 2 Nav2 for sending navigation goals.

    Wraps the NavigateToPose action client and provides
    a clean async API for the agent layer.
    """

    def __init__(self, node: Optional[object] = None):
        self._node = node
        self._nav_client = None
        self._current_goal_handle = None
        self._is_navigating = False

        if ROS_AVAILABLE and node is not None:
            self._nav_client = ActionClient(
                node, NavigateToPose, "navigate_to_pose"
            )
            logger.info("NavCommander connected to Nav2 action server")
        else:
            logger.info("NavCommander running in simulation mode")

    @property
    def is_navigating(self) -> bool:
        return self._is_navigating

    async def navigate_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = "map",
    ) -> dict:
        """
        Send a navigation goal to Nav2.

        Args:
            x: Target X position in meters
            y: Target Y position in meters
            yaw: Target orientation in radians
            frame_id: Reference frame (default: "map")

        Returns:
            dict with success status and message
        """
        logger.info(f"🎯 Navigating to ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)")

        if not ROS_AVAILABLE or self._nav_client is None:
            # Simulation mode
            logger.info("[SIM] Navigation goal accepted (simulation)")
            self._is_navigating = True
            return {"success": True, "message": f"Navigating to ({x:.2f}, {y:.2f}) [simulation]"}

        # Build goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Wait for action server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            logger.error("Nav2 action server not available!")
            return {"success": False, "message": "Nav2 server unavailable"}

        # Send goal
        self._is_navigating = True
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )

        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self._is_navigating = False
            logger.warning("Navigation goal rejected by Nav2")
            return {"success": False, "message": "Goal rejected by Nav2"}

        self._current_goal_handle = goal_handle
        logger.info("Navigation goal accepted by Nav2")

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = await result_future

        self._is_navigating = False
        self._current_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            logger.info(f"✅ Reached goal ({x:.2f}, {y:.2f})")
            return {"success": True, "message": f"Reached ({x:.2f}, {y:.2f})"}
        else:
            logger.warning(f"Navigation failed with status: {result.status}")
            return {"success": False, "message": f"Navigation failed (status: {result.status})"}

    async def cancel_navigation(self) -> dict:
        """Cancel the current navigation goal."""
        if self._current_goal_handle is not None:
            logger.info("Cancelling current navigation goal")
            cancel_future = self._current_goal_handle.cancel_goal_async()
            await cancel_future
            self._is_navigating = False
            self._current_goal_handle = None
            return {"success": True, "message": "Navigation cancelled"}
        return {"success": True, "message": "No active navigation to cancel"}

    def _feedback_callback(self, feedback_msg) -> None:
        """Handle navigation feedback from Nav2."""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        distance = feedback.distance_remaining

        logger.debug(
            f"Nav feedback — pos: ({current_pose.position.x:.2f}, "
            f"{current_pose.position.y:.2f}), remaining: {distance:.2f}m"
        )

    async def navigate_to_room(self, room_name: str, room_coords: dict) -> dict:
        """
        Navigate to a named room using stored coordinates.

        Args:
            room_name: Name of the room (e.g., "kitchen")
            room_coords: Dict mapping room names to (x, y) coordinates

        Returns:
            dict with success status and message
        """
        if room_name not in room_coords:
            available = ", ".join(room_coords.keys())
            return {
                "success": False,
                "message": f"Unknown room '{room_name}'. Known rooms: {available}",
            }

        coords = room_coords[room_name]
        x, y = coords["x"], coords["y"]
        logger.info(f"🏠 Navigating to room '{room_name}' at ({x:.2f}, {y:.2f})")
        return await self.navigate_to(x, y)

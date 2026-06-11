"""Agent Bridge Node — connects the LLM agent to ROS 2 topics and services.

This node:
  - Subscribes to robot pose, LiDAR, and sensor topics
  - Publishes navigation goals from the LLM agent
  - Provides the state_provider and tool_executor for the agent
"""

from __future__ import annotations

import asyncio
import json
import math
from typing import Optional

from loguru import logger

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan, Imu
    from std_msgs.msg import String, Float32

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    logger.warning("ROS 2 not available — AgentBridgeNode in simulation mode")

from navigation.robot_state import RobotState
from navigation.nav_commander import NavCommander
from navigation.slam_manager import SlamManager
from navigation.map_manager import MapManager


class AgentBridgeNode:
    """
    Bridge between the LLM Agent and ROS 2.

    Aggregates sensor data into RobotState and translates
    agent tool calls into ROS 2 actions.
    """

    def __init__(self):
        self._node: Optional[Node] = None
        self.robot_state = RobotState()
        self.nav_commander: Optional[NavCommander] = None
        self.slam_manager: Optional[SlamManager] = None
        self.map_manager = MapManager()

        # Room coordinates (populated during/after mapping)
        self.room_coords: dict[str, dict] = {}

    async def initialize(self) -> None:
        """Initialize ROS 2 node and subscriptions."""
        if not ROS_AVAILABLE:
            logger.info("[SIM] AgentBridgeNode initialized (simulation)")
            self.nav_commander = NavCommander()
            self.slam_manager = SlamManager()
            return

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("agent_bridge")
        logger.info("AgentBridgeNode initialized")

        # Initialize navigation components
        self.nav_commander = NavCommander(self._node)
        self.slam_manager = SlamManager(self._node)

        # ── Subscribers ─────────────────────────────────────────

        # Robot pose from AMCL / robot_localization
        self._node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._pose_callback,
            10,
        )

        # Odometry
        self._node.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            10,
        )

        # LiDAR scan
        self._node.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            10,
        )

        # Map updates
        self._node.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            10,
        )

        # Battery level (from Arduino via motor_node)
        self._node.create_subscription(
            Float32,
            "/battery_voltage",
            self._battery_callback,
            10,
        )

        # ── Publishers ──────────────────────────────────────────

        # Velocity commands (for manual control / override)
        self._cmd_vel_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)

        # Agent status (for dashboard)
        self._status_pub = self._node.create_publisher(String, "/agent/status", 10)

        logger.info("AgentBridgeNode subscriptions and publishers ready")

    # ── ROS 2 Callbacks ─────────────────────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Update robot pose from AMCL."""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orient.w * orient.z + orient.x * orient.y),
            1.0 - 2.0 * (orient.y ** 2 + orient.z ** 2),
        )
        self.robot_state.update_pose(pos.x, pos.y, yaw)

    def _odom_callback(self, msg: Odometry) -> None:
        """Update velocity from odometry."""
        self.robot_state.linear_speed = msg.twist.twist.linear.x
        self.robot_state.angular_speed = msg.twist.twist.angular.z

    def _scan_callback(self, msg: LaserScan) -> None:
        """Update nearest obstacle from LiDAR scan."""
        self.robot_state.lidar_connected = True
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid_ranges:
                min_dist = min(valid_ranges)
                min_idx = msg.ranges.index(min_dist)
                angle = msg.angle_min + min_idx * msg.angle_increment
                self.robot_state.update_obstacle(min_dist, angle)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Update map status from SLAM."""
        self.robot_state.map_status = "mapping" if self.slam_manager.mode.value == "mapping" else "ready"

    def _battery_callback(self, msg: Float32) -> None:
        """Update battery level."""
        # Convert voltage to percentage (3S LiPo: 9.0V = 0%, 12.6V = 100%)
        voltage = msg.data
        percentage = max(0, min(100, (voltage - 9.0) / (12.6 - 9.0) * 100))
        self.robot_state.battery_level = percentage

    # ── Agent Interface ─────────────────────────────────────────

    async def get_state(self) -> dict:
        """State provider for the LLM agent."""
        # Spin ROS 2 node to process callbacks
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=0.01)

        return self.robot_state.to_dict()

    async def execute_tool(self, tool_name: str, params: dict) -> dict:
        """
        Tool executor for the LLM agent.

        Translates agent tool calls into ROS 2 actions.
        """
        logger.info(f"Executing tool: {tool_name}({json.dumps(params)})")

        if tool_name == "navigate_to":
            return await self._handle_navigate(params)
        elif tool_name == "explore_area":
            return await self._handle_explore(params)
        elif tool_name == "get_robot_state":
            return {"success": True, "data": self.robot_state.to_dict()}
        elif tool_name == "get_map_info":
            return {"success": True, "data": self.map_manager.get_map_info()}
        elif tool_name == "set_speed":
            speed = params.get("speed", 0.15)
            self.robot_state.max_speed = speed
            return {"success": True, "message": f"Speed set to {speed} m/s"}
        elif tool_name == "stop":
            return await self._handle_stop()
        elif tool_name == "rotate":
            return await self._handle_rotate(params)
        elif tool_name == "go_home":
            return await self.nav_commander.navigate_to(0.0, 0.0)
        else:
            return {"success": False, "message": f"Unknown tool: {tool_name}"}

    async def _handle_navigate(self, params: dict) -> dict:
        """Handle navigate_to tool call."""
        room_name = params.get("room_name")
        if room_name:
            return await self.nav_commander.navigate_to_room(room_name, self.room_coords)

        x = params.get("x")
        y = params.get("y")
        if x is not None and y is not None:
            return await self.nav_commander.navigate_to(x, y)

        return {"success": False, "message": "Provide room_name or x,y coordinates"}

    async def _handle_explore(self, params: dict) -> dict:
        """Handle explore_area tool call."""
        # Start SLAM if not already mapping
        if self.slam_manager.mode.value != "mapping":
            await self.slam_manager.start_mapping()

        # Find nearest frontier and navigate to it
        frontiers = self.map_manager.get_frontier_cells()
        if not frontiers:
            return {"success": True, "message": "No unexplored frontiers — map is complete!"}

        # Pick the nearest frontier
        robot_gx, robot_gy = self.map_manager.world_to_grid(
            self.robot_state.pose.x, self.robot_state.pose.y
        )

        nearest = min(
            frontiers,
            key=lambda f: (f[0] - robot_gx) ** 2 + (f[1] - robot_gy) ** 2,
        )

        target_x, target_y = self.map_manager.grid_to_world(nearest[0], nearest[1])
        return await self.nav_commander.navigate_to(target_x, target_y)

    async def _handle_stop(self) -> dict:
        """Handle stop tool call."""
        await self.nav_commander.cancel_navigation()

        if self._node:
            stop_msg = Twist()  # All zeros = stop
            self._cmd_vel_pub.publish(stop_msg)

        self.robot_state.is_navigating = False
        return {"success": True, "message": "Robot stopped"}

    async def _handle_rotate(self, params: dict) -> dict:
        """Handle rotate tool call."""
        angle_deg = params.get("angle", 0)
        speed = params.get("speed", 0.5)

        if self._node:
            twist = Twist()
            twist.angular.z = speed if angle_deg > 0 else -speed
            duration = abs(math.radians(angle_deg)) / speed

            self._cmd_vel_pub.publish(twist)
            await asyncio.sleep(duration)

            # Stop rotation
            stop_twist = Twist()
            self._cmd_vel_pub.publish(stop_twist)

        return {"success": True, "message": f"Rotated {angle_deg}°"}

    async def shutdown(self) -> None:
        """Clean shutdown."""
        if self._node:
            self._node.destroy_node()
        logger.info("AgentBridgeNode shut down")

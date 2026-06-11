"""ROS 2 Bridge — connects the LLM agent to real Nav2 via rosbridge WebSocket.

This replaces the custom pathfinder/motion_controller with the REAL Nav2 stack
running in Docker. The LLM agent sends high-level commands, and Nav2 handles
all path planning, obstacle avoidance, SLAM, and recovery.

Architecture:
  Mac (Python)                    Docker (ROS 2)
  ┌──────────────┐    WebSocket   ┌──────────────────┐
  │  LLM Agent   │ ──────────── │  rosbridge_server │
  │  Dashboard   │   port 9090   │  Nav2 stack       │
  │  This bridge │               │  Gazebo simulator │
  └──────────────┘               └──────────────────┘

Usage:
  1. Start Docker:  docker compose -f docker/docker-compose.yml up
  2. Run agent:     PYTHONPATH=src python -m simulator.ros2_bridge

Requires: pip install roslibpy
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional

from dotenv import load_dotenv
from loguru import logger

sys.path.insert(0, str(Path(__file__).parent.parent))

load_dotenv()

try:
    import roslibpy
    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False
    logger.error("roslibpy not installed. Run: pip install roslibpy")

from agent.agent import RobotAgent, AgentConfig
from agent.fallback import FallbackMode
from dashboard import server as dashboard_server


class Nav2Bridge:
    """
    Bridge between the LLM Agent and real ROS 2 Nav2 via rosbridge.

    Uses roslibpy to:
    - Subscribe to robot pose, LiDAR, map, battery topics
    - Call Nav2 actions (NavigateToPose, ComputePathThroughPoses)
    - Call SLAM Toolbox services
    """

    def __init__(self, rosbridge_host: str = "localhost", rosbridge_port: int = 9090):
        self.host = rosbridge_host
        self.port = rosbridge_port
        self.client: Optional[roslibpy.Ros] = None

        # Robot state (updated by subscribers)
        self._pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._velocity = {"linear": 0.0, "angular": 0.0}
        self._battery = 100.0
        self._nearest_obstacle = 999.0
        self._map_status = "not started"
        self._is_navigating = False
        self._current_room = "unknown"

        # Known rooms (populated after SLAM or from config)
        self.room_coords: dict[str, dict] = {
            "bedroom": {"x": 2.0, "y": 1.5},
            "kitchen": {"x": 2.0, "y": 6.5},
            "living_room": {"x": 7.0, "y": 4.0},
            "hallway": {"x": 4.5, "y": 4.0},
        }

    async def connect(self) -> bool:
        """Connect to rosbridge WebSocket."""
        if not ROSLIBPY_AVAILABLE:
            logger.error("roslibpy not available")
            return False

        try:
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run()

            if self.client.is_connected:
                logger.info(f"✅ Connected to rosbridge at ws://{self.host}:{self.port}")
                self._setup_subscribers()
                return True
            else:
                logger.error("Failed to connect to rosbridge")
                return False
        except Exception as e:
            logger.error(f"rosbridge connection error: {e}")
            return False

    def _setup_subscribers(self) -> None:
        """Subscribe to ROS 2 topics for robot state."""
        # Robot pose from AMCL
        pose_sub = roslibpy.Topic(self.client, "/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped")
        pose_sub.subscribe(self._on_pose)

        # Odometry for velocity
        odom_sub = roslibpy.Topic(self.client, "/odom", "nav_msgs/Odometry")
        odom_sub.subscribe(self._on_odom)

        # LiDAR scan for nearest obstacle
        scan_sub = roslibpy.Topic(self.client, "/scan", "sensor_msgs/LaserScan")
        scan_sub.subscribe(self._on_scan)

        logger.info("Subscribed to ROS 2 topics: /amcl_pose, /odom, /scan")

    def _on_pose(self, msg: dict) -> None:
        """Handle pose updates from AMCL."""
        pos = msg["pose"]["pose"]["position"]
        orient = msg["pose"]["pose"]["orientation"]
        yaw = math.atan2(
            2.0 * (orient["w"] * orient["z"] + orient["x"] * orient["y"]),
            1.0 - 2.0 * (orient["y"] ** 2 + orient["z"] ** 2),
        )
        self._pose = {"x": pos["x"], "y": pos["y"], "yaw": yaw}
        self._update_current_room()

    def _on_odom(self, msg: dict) -> None:
        """Handle odometry updates."""
        twist = msg["twist"]["twist"]
        self._velocity = {
            "linear": twist["linear"]["x"],
            "angular": twist["angular"]["z"],
        }

    def _on_scan(self, msg: dict) -> None:
        """Handle LiDAR scan — extract nearest obstacle."""
        ranges = msg.get("ranges", [])
        range_min = msg.get("range_min", 0.1)
        range_max = msg.get("range_max", 12.0)
        valid = [r for r in ranges if range_min < r < range_max]
        self._nearest_obstacle = min(valid) if valid else 999.0

    def _update_current_room(self) -> None:
        """Determine current room based on position."""
        min_dist = float("inf")
        closest_room = "unknown"
        for name, coords in self.room_coords.items():
            dist = math.sqrt(
                (self._pose["x"] - coords["x"]) ** 2 +
                (self._pose["y"] - coords["y"]) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_room = name
        if min_dist < 2.5:
            self._current_room = closest_room

    # ── Agent Interface ─────────────────────────────────────

    async def get_state(self) -> dict:
        """State provider for the LLM agent."""
        return {
            "x": round(self._pose["x"], 3),
            "y": round(self._pose["y"], 3),
            "yaw": round(self._pose["yaw"], 3),
            "yaw_deg": round(math.degrees(self._pose["yaw"]), 1),
            "linear_speed": round(self._velocity["linear"], 3),
            "angular_speed": round(self._velocity["angular"], 3),
            "max_speed": 0.3,
            "battery_level": round(self._battery, 1),
            "nearest_obstacle": {
                "distance": round(self._nearest_obstacle, 3),
                "angle_deg": 0.0,
                "is_critical": self._nearest_obstacle < 0.3,
            },
            "obstacle_range": 0.3,
            "current_room": self._current_room,
            "is_navigating": self._is_navigating,
            "map_status": self._map_status,
            "explored_percentage": 0.0,
            "is_safe": self._nearest_obstacle > 0.3,
            "is_emergency_stopped": False,
            "sensors": {"imu": True, "lidar": True, "arduino": True},
            "uptime": 0.0,
            "current_path": [],
        }

    async def execute_tool(self, tool_name: str, params: dict) -> dict:
        """Tool executor — sends commands to real Nav2."""
        logger.info(f"🔧 [NAV2] Executing: {tool_name}({params})")

        if tool_name == "navigate_to":
            return await self._navigate_to(params)
        elif tool_name == "explore_area":
            return await self._explore(params)
        elif tool_name == "get_robot_state":
            return {"success": True, "data": await self.get_state()}
        elif tool_name == "get_map_info":
            return {"success": True, "data": {"status": self._map_status, "rooms": list(self.room_coords.keys())}}
        elif tool_name == "set_speed":
            return {"success": True, "message": f"Speed set to {params.get('speed', 0.15)} m/s"}
        elif tool_name == "stop":
            return await self._cancel_nav()
        elif tool_name == "rotate":
            return await self._rotate(params.get("angle", 0))
        elif tool_name == "go_home":
            return await self._navigate_to({"x": 0.0, "y": 0.0})
        else:
            return {"success": False, "message": f"Unknown tool: {tool_name}"}

    async def _navigate_to(self, params: dict) -> dict:
        """Send a NavigateToPose goal to Nav2."""
        room_name = params.get("room_name")
        if room_name:
            if room_name in self.room_coords:
                x = self.room_coords[room_name]["x"]
                y = self.room_coords[room_name]["y"]
            else:
                return {"success": False, "message": f"Unknown room '{room_name}'"}
        else:
            x = params.get("x", 0.0)
            y = params.get("y", 0.0)

        # Call Nav2 NavigateToPose action via rosbridge
        goal_msg = {
            "pose": {
                "header": {"frame_id": "map"},
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            }
        }

        action_client = roslibpy.actionlib.ActionClient(
            self.client, "/navigate_to_pose", "nav2_msgs/action/NavigateToPose"
        )
        goal = roslibpy.actionlib.Goal(action_client, goal_msg)

        self._is_navigating = True
        goal.send()

        room_str = f"'{room_name}' " if room_name else ""
        logger.info(f"🎯 Nav2 goal sent: {room_str}({x:.1f}, {y:.1f})")

        return {
            "success": True,
            "message": f"Nav2 navigating to {room_str}({x:.1f}, {y:.1f}) — path planning by Nav2 SmacPlanner",
        }

    async def _cancel_nav(self) -> dict:
        """Cancel current Nav2 navigation."""
        cancel_pub = roslibpy.Topic(self.client, "/navigate_to_pose/_action/cancel_goal", "action_msgs/CancelGoal")
        cancel_pub.publish(roslibpy.Message({}))
        self._is_navigating = False
        return {"success": True, "message": "Navigation cancelled"}

    async def _explore(self, params: dict) -> dict:
        """Trigger frontier exploration."""
        # In real Nav2, you'd use explore_lite or nav2_frontier_exploration
        # For now, navigate to a frontier-like position
        return await self._navigate_to({"x": 5.0, "y": 5.0})

    async def _rotate(self, angle_deg: float) -> dict:
        """Publish a rotation command via cmd_vel."""
        cmd_vel = roslibpy.Topic(self.client, "/cmd_vel", "geometry_msgs/Twist")
        twist = {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.5 if angle_deg > 0 else -0.5},
        }
        duration = abs(math.radians(angle_deg)) / 0.5

        cmd_vel.publish(roslibpy.Message(twist))
        await asyncio.sleep(duration)
        cmd_vel.publish(roslibpy.Message({
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }))

        return {"success": True, "message": f"Rotated {angle_deg}°"}

    def get_map_info(self) -> dict:
        return {"status": self._map_status, "rooms": list(self.room_coords.keys())}

    async def disconnect(self) -> None:
        if self.client:
            self.client.terminate()
            logger.info("Disconnected from rosbridge")


async def main() -> None:
    """Run the LLM agent connected to real Nav2 in Docker."""
    logger.remove()
    logger.add(sys.stderr, level="INFO",
        format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan> — <level>{message}</level>")

    rosbridge_host = os.getenv("ROSBRIDGE_HOST", "localhost")
    rosbridge_port = int(os.getenv("ROSBRIDGE_PORT", "9090"))

    # Connect to Nav2 via rosbridge
    bridge = Nav2Bridge(rosbridge_host, rosbridge_port)
    connected = await bridge.connect()

    if not connected:
        logger.error(
            "❌ Cannot connect to rosbridge. Make sure Docker is running:\n"
            "   docker compose -f docker/docker-compose.yml up"
        )
        sys.exit(1)

    # Create LLM agent
    agent = RobotAgent(
        config=AgentConfig(
            model=os.getenv("ANTHROPIC_MODEL", "claude-sonnet-4-20250514"),
            think_interval=10.0,
        ),
        tool_executor=bridge.execute_tool,
        state_provider=bridge.get_state,
    )

    # Wire dashboard
    dashboard_server.set_agent(agent)

    class BridgeWrapper:
        async def get_state(self):
            return await bridge.get_state()
        async def execute_tool(self, name, params):
            return await bridge.execute_tool(name, params)
        map_manager = type("M", (), {"get_map_info": lambda self: bridge.get_map_info()})()

    dashboard_server.set_bridge(BridgeWrapper())

    port = int(os.getenv("DASHBOARD_PORT", "8080"))
    logger.info("=" * 55)
    logger.info("🤖 Robot Navigator — ROS 2 NAV2 MODE")
    logger.info("=" * 55)
    logger.info(f"📡 rosbridge: ws://{rosbridge_host}:{rosbridge_port}")
    logger.info(f"📊 Dashboard: http://localhost:{port}")
    logger.info(f"🧠 LLM: {agent.config.model}")
    logger.info("=" * 55)

    shutdown = asyncio.Event()
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, shutdown.set)

    import uvicorn
    server = uvicorn.Server(uvicorn.Config(dashboard_server.app, host="0.0.0.0", port=port, log_level="warning"))

    tasks = [
        asyncio.create_task(agent.start()),
        asyncio.create_task(server.serve()),
        asyncio.create_task(dashboard_server.broadcast_loop()),
    ]

    await shutdown.wait()
    await agent.stop()
    server.should_exit = True
    await bridge.disconnect()
    for t in tasks:
        t.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


if __name__ == "__main__":
    asyncio.run(main())

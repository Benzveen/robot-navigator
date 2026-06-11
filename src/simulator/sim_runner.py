"""Simulation Runner — runs the full robot system with a virtual world.

This is the main entry point for testing without hardware.
It wires up:
  - VirtualWorld (simulated room + LiDAR + physics)
  - LLM Agent (real Anthropic API calls)
  - Web Dashboard (real FastAPI server)
  - All connected via simulated state provider + tool executor

Usage:
    python -m simulator.sim_runner
    # or
    make run-sim
"""

from __future__ import annotations

import asyncio
import math
import os
import signal
import sys
from pathlib import Path

from dotenv import load_dotenv
from loguru import logger

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from agent.agent import RobotAgent, AgentConfig
from agent.fallback import FallbackMode
from dashboard import server as dashboard_server
from simulator.virtual_world import VirtualWorld, create_apartment_layout
from simulator.pathfinder import build_grid_from_walls, astar, GridMap
from simulator.motion_controller import MotionController, NavigationGoal

load_dotenv()


class SimulatedRobot:
    """
    Connects the LLM Agent to the Virtual World with SMART navigation.

    Key improvement: uses A* path planning + waypoint following so the
    robot navigates THROUGH DOORS and AROUND WALLS like a human.

    Architecture:
      LLM says "go to kitchen"
        → A* finds path: hallway → door → kitchen
        → Motion controller follows waypoints smoothly
        → Robot drives through the door correctly
    """

    def __init__(self, world: VirtualWorld):
        self.world = world
        self.motion = MotionController(max_speed=0.15)
        self.grid_map: GridMap | None = None
        self._current_path: list[tuple[float, float]] = []

        # Build the occupancy grid from walls (once at startup)
        self._build_grid()

    def _build_grid(self) -> None:
        """Convert apartment walls into an A* searchable grid."""
        self.grid_map = build_grid_from_walls(
            walls=self.world.layout.walls,
            world_width=self.world.layout.width,
            world_height=self.world.layout.height,
            resolution=0.1,       # 10cm cells
            robot_radius=0.14,    # inflate walls by robot radius
        )

    def _plan_path(self, target_x: float, target_y: float) -> list[tuple[float, float]] | None:
        """Plan an A* path from current position to target."""
        if self.grid_map is None:
            return None
        start = (self.world.robot.x, self.world.robot.y)
        goal = (target_x, target_y)
        return astar(self.grid_map, start, goal)

    async def get_state(self) -> dict:
        """State provider for the LLM agent."""
        # Run motion controller if navigating
        if self.motion.is_navigating:
            obs_dist, _ = self.world.get_nearest_obstacle()
            linear, angular = self.motion.update(
                self.world.robot.x,
                self.world.robot.y,
                self.world.robot.yaw,
                obs_dist / 1000.0 if obs_dist > 10 else obs_dist,  # handle mm vs m
            )
            self.world.set_velocity(linear, angular)

            # Check if navigation completed
            if self.motion.goal and self.motion.goal.is_complete:
                self.world.set_velocity(0, 0)
                self._current_path = []
        else:
            # Not navigating — make sure robot is stopped
            if abs(self.world.robot.speed) > 0.001:
                self.world.set_velocity(0, 0)

        self.world.update()
        state = self.world.get_robot_state()

        # Add path info for dashboard
        state["current_path"] = [(round(x, 2), round(y, 2)) for x, y in self._current_path]
        state["is_navigating"] = self.motion.is_navigating
        if self.motion.goal:
            state["nav_target"] = {
                "x": self.motion.goal.target_x,
                "y": self.motion.goal.target_y,
                "room": self.motion.goal.target_room,
                "waypoints_remaining": self.motion.goal.remaining_waypoints,
            }

        return state

    async def execute_tool(self, tool_name: str, params: dict) -> dict:
        """Tool executor — translates agent decisions into world actions."""
        logger.info(f"🔧 [SIM] Executing: {tool_name}({params})")

        if tool_name == "navigate_to":
            return await self._handle_navigate(params)
        elif tool_name == "explore_area":
            return await self._handle_explore(params)
        elif tool_name == "get_robot_state":
            state = self.world.get_robot_state()
            return {"success": True, "data": state}
        elif tool_name == "get_map_info":
            return {"success": True, "data": self._get_map_info()}
        elif tool_name == "set_speed":
            speed = params.get("speed", 0.15)
            self.motion.max_speed = speed
            self.world.robot.max_speed = speed
            return {"success": True, "message": f"Speed set to {speed} m/s"}
        elif tool_name == "stop":
            self.motion.cancel()
            self.world.set_velocity(0, 0)
            self._current_path = []
            return {"success": True, "message": "Robot stopped"}
        elif tool_name == "rotate":
            angle_deg = params.get("angle", 0)
            self.world.robot.yaw += math.radians(angle_deg)
            return {"success": True, "message": f"Rotated {angle_deg}°"}
        elif tool_name == "go_home":
            return await self._navigate_with_path(5.0, 4.0, room_name="home")
        else:
            return {"success": False, "message": f"Unknown tool: {tool_name}"}

    async def _handle_navigate(self, params: dict) -> dict:
        room_name = params.get("room_name")
        if room_name:
            rooms = self.world.layout.rooms
            if room_name in rooms:
                x, y = rooms[room_name]
                return await self._navigate_with_path(x, y, room_name=room_name)
            else:
                available = ", ".join(rooms.keys())
                return {"success": False, "message": f"Unknown room '{room_name}'. Available: {available}"}

        x = params.get("x")
        y = params.get("y")
        if x is not None and y is not None:
            return await self._navigate_with_path(x, y)

        return {"success": False, "message": "Provide room_name or x,y coordinates"}

    async def _navigate_with_path(
        self, target_x: float, target_y: float, room_name: str | None = None
    ) -> dict:
        """Plan an A* path and start following it."""
        path = self._plan_path(target_x, target_y)

        if path is None or len(path) < 2:
            return {
                "success": False,
                "message": f"No path found to ({target_x:.1f}, {target_y:.1f}). Area may be unreachable.",
            }

        # Store path for dashboard visualization
        self._current_path = path

        # Create navigation goal
        goal = NavigationGoal(
            target_x=target_x,
            target_y=target_y,
            target_room=room_name,
            waypoints=path,
        )
        self.motion.set_goal(goal)

        room_str = f"'{room_name}' " if room_name else ""
        return {
            "success": True,
            "message": (
                f"Path planned to {room_str}({target_x:.1f}, {target_y:.1f}): "
                f"{len(path)} waypoints, navigating through doors and around walls."
            ),
        }

    async def _handle_explore(self, params: dict) -> dict:
        """Explore by navigating to a reachable random location."""
        import random

        # Try a few random targets until we find one with a valid path
        for _ in range(10):
            target_x = random.uniform(1.0, self.world.layout.width - 1.0)
            target_y = random.uniform(1.0, self.world.layout.height - 1.0)
            path = self._plan_path(target_x, target_y)
            if path and len(path) >= 2:
                return await self._navigate_with_path(target_x, target_y)

        return {"success": False, "message": "Could not find a reachable exploration target"}

    def _get_map_info(self) -> dict:
        return {
            "status": "ready",
            "width_m": self.world.layout.width,
            "height_m": self.world.layout.height,
            "resolution": 0.1,
            "explored_percentage": 85.0,
            "rooms": list(self.world.layout.rooms.keys()),
            "frontier_count": 3,
            "path_planner": "A* on occupancy grid",
            "grid_size": f"{self.grid_map.width}x{self.grid_map.height}" if self.grid_map else "not built",
        }


async def main() -> None:
    """Run the full simulation."""
    # Setup logging
    logger.remove()
    logger.add(
        sys.stderr,
        level="INFO",
        format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan> — <level>{message}</level>",
    )

    # Check API key
    api_key = os.getenv("ANTHROPIC_API_KEY")
    if not api_key or api_key == "your-api-key-here":
        logger.warning(
            "⚠️  No ANTHROPIC_API_KEY set. The LLM agent will fail on API calls.\n"
            "   Set it in .env or export ANTHROPIC_API_KEY=sk-ant-...\n"
            "   The dashboard and simulation will still work for manual control."
        )

    # Create virtual world
    world = VirtualWorld(create_apartment_layout())

    # Create simulated robot bridge
    sim_robot = SimulatedRobot(world)

    # Create LLM agent
    agent = RobotAgent(
        config=AgentConfig(
            model=os.getenv("ANTHROPIC_MODEL", "claude-sonnet-4-20250514"),
            max_tokens=1024,
            temperature=0.3,
            think_interval=10.0,  # Slower in sim to save API costs
            max_retries=2,
            fallback_mode=FallbackMode.STOP,
        ),
        tool_executor=sim_robot.execute_tool,
        state_provider=sim_robot.get_state,
    )

    # Wire up dashboard
    dashboard_server.set_agent(agent)

    # Create a minimal bridge-like object for the dashboard
    class SimBridge:
        async def get_state(self):
            return await sim_robot.get_state()

        async def execute_tool(self, name, params):
            return await sim_robot.execute_tool(name, params)

        map_manager = type("MapManager", (), {"get_map_info": lambda self: sim_robot._get_map_info()})()

    dashboard_server.set_bridge(SimBridge())

    # Print startup info
    port = int(os.getenv("DASHBOARD_PORT", "8080"))
    logger.info("=" * 55)
    logger.info("🤖 Robot Navigator — SIMULATION MODE")
    logger.info("=" * 55)
    logger.info(f"🏠 World: {world.layout.name} ({world.layout.width}m x {world.layout.height}m)")
    logger.info(f"📍 Robot at: ({world.robot.x:.1f}, {world.robot.y:.1f})")
    logger.info(f"🧠 LLM: {agent.config.model}")
    logger.info(f"📊 Dashboard: http://localhost:{port}")
    logger.info(f"🏘️  Rooms: {', '.join(world.layout.rooms.keys())}")
    logger.info("")
    logger.info("Try these commands in the dashboard chat:")
    logger.info('  • "Go to the kitchen"')
    logger.info('  • "What room am I in?"')
    logger.info('  • "Explore the area"')
    logger.info('  • "Go to the bedroom"')
    logger.info("=" * 55)

    # Start everything
    shutdown_event = asyncio.Event()

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, shutdown_event.set)

    import uvicorn

    uvicorn_config = uvicorn.Config(
        dashboard_server.app,
        host="0.0.0.0",
        port=port,
        log_level="warning",
    )
    server = uvicorn.Server(uvicorn_config)

    tasks = [
        asyncio.create_task(agent.start(), name="agent"),
        asyncio.create_task(server.serve(), name="dashboard"),
        asyncio.create_task(dashboard_server.broadcast_loop(), name="broadcast"),
    ]

    # Wait for shutdown
    await shutdown_event.wait()
    logger.info("🛑 Shutting down simulation...")

    await agent.stop()
    server.should_exit = True

    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)

    logger.info("✅ Simulation stopped")


if __name__ == "__main__":
    asyncio.run(main())

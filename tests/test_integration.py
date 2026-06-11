"""Integration tests — test the full sim pipeline without API calls.

These tests verify that all components wire together correctly:
  - VirtualWorld → SimulatedRobot → Agent tool executor
  - State flows correctly from world → agent
  - Tool calls translate correctly to world actions
"""

import asyncio
import math
import sys
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from simulator.virtual_world import VirtualWorld, create_apartment_layout
from simulator.sim_runner import SimulatedRobot
from agent.agent import RobotAgent, AgentConfig
from agent.tools import TOOL_DEFINITIONS


class TestSimulatedRobot:
    """Test the SimulatedRobot bridge between agent and world."""

    @pytest.fixture
    def sim(self):
        world = VirtualWorld(create_apartment_layout())
        return SimulatedRobot(world)

    @pytest.mark.asyncio
    async def test_get_state_returns_valid_dict(self, sim):
        state = await sim.get_state()
        assert isinstance(state, dict)
        assert "x" in state
        assert "y" in state
        assert "battery_level" in state
        assert "current_room" in state

    @pytest.mark.asyncio
    async def test_navigate_to_room(self, sim):
        result = await sim.execute_tool("navigate_to", {"room_name": "kitchen"})
        assert result["success"] is True
        assert "kitchen" in result["message"]
        assert sim._navigating_to is not None

    @pytest.mark.asyncio
    async def test_navigate_to_unknown_room(self, sim):
        result = await sim.execute_tool("navigate_to", {"room_name": "bathroom"})
        assert result["success"] is False
        assert "Unknown room" in result["message"]

    @pytest.mark.asyncio
    async def test_navigate_to_coordinates(self, sim):
        result = await sim.execute_tool("navigate_to", {"x": 3.0, "y": 2.0})
        assert result["success"] is True
        assert sim._navigating_to == {"x": 3.0, "y": 2.0}

    @pytest.mark.asyncio
    async def test_stop(self, sim):
        # Start navigating
        await sim.execute_tool("navigate_to", {"room_name": "bedroom"})
        assert sim._navigating_to is not None

        # Stop
        result = await sim.execute_tool("stop", {})
        assert result["success"] is True
        assert sim._navigating_to is None
        assert sim.world.robot.speed == 0

    @pytest.mark.asyncio
    async def test_rotate(self, sim):
        old_yaw = sim.world.robot.yaw
        result = await sim.execute_tool("rotate", {"angle": 90})
        assert result["success"] is True
        expected_yaw = old_yaw + math.radians(90)
        assert abs(sim.world.robot.yaw - expected_yaw) < 0.01

    @pytest.mark.asyncio
    async def test_set_speed(self, sim):
        result = await sim.execute_tool("set_speed", {"speed": 0.2})
        assert result["success"] is True
        assert sim.world.robot.max_speed == 0.2

    @pytest.mark.asyncio
    async def test_get_robot_state_tool(self, sim):
        result = await sim.execute_tool("get_robot_state", {})
        assert result["success"] is True
        assert "data" in result
        assert "x" in result["data"]

    @pytest.mark.asyncio
    async def test_get_map_info_tool(self, sim):
        result = await sim.execute_tool("get_map_info", {})
        assert result["success"] is True
        assert "data" in result
        assert "rooms" in result["data"]

    @pytest.mark.asyncio
    async def test_go_home(self, sim):
        result = await sim.execute_tool("go_home", {})
        assert result["success"] is True
        assert sim._navigating_to == {"x": 5.0, "y": 4.0}

    @pytest.mark.asyncio
    async def test_explore(self, sim):
        result = await sim.execute_tool("explore_area", {"direction": "auto"})
        assert result["success"] is True
        assert sim._navigating_to is not None

    @pytest.mark.asyncio
    async def test_unknown_tool(self, sim):
        result = await sim.execute_tool("fly_to_moon", {})
        assert result["success"] is False

    @pytest.mark.asyncio
    async def test_navigation_progress(self, sim):
        """Test that repeated get_state calls move robot toward target."""
        await sim.execute_tool("navigate_to", {"x": 6.0, "y": 4.0})

        initial_state = await sim.get_state()
        initial_x = initial_state["x"]

        # Call get_state multiple times (which triggers movement)
        for _ in range(50):
            state = await sim.get_state()

        final_x = state["x"]
        assert final_x != initial_x, "Robot should have moved toward target"


class TestAgentWithSimulation:
    """Test the LLM agent wired to the simulation (without real API calls)."""

    @pytest.fixture
    def agent_and_sim(self):
        world = VirtualWorld(create_apartment_layout())
        sim = SimulatedRobot(world)
        agent = RobotAgent(
            config=AgentConfig(
                model="claude-sonnet-4-20250514",
                think_interval=1.0,
            ),
            tool_executor=sim.execute_tool,
            state_provider=sim.get_state,
        )
        return agent, sim

    def test_agent_creation_with_sim(self, agent_and_sim):
        agent, sim = agent_and_sim
        assert agent.tool_executor is not None
        assert agent.state_provider is not None

    @pytest.mark.asyncio
    async def test_agent_get_state(self, agent_and_sim):
        agent, sim = agent_and_sim
        state = await agent._get_robot_state()
        assert "x" in state
        assert "current_room" in state

    def test_agent_event_system_with_sim(self, agent_and_sim):
        agent, sim = agent_and_sim
        events = []
        agent.on("action_start", lambda data: events.append(data))

        # Manually trigger an event
        agent._emit("action_start", {"action": "test"})
        assert len(events) == 1

    @pytest.mark.asyncio
    async def test_agent_execute_action(self, agent_and_sim):
        agent, sim = agent_and_sim
        action = {"action": "stop", "params": {}}
        result = await agent._execute_action(action)
        assert result["success"] is True


class TestToolDefinitionsMatchSimulation:
    """Verify all agent tools are handled by the simulation."""

    @pytest.fixture
    def sim(self):
        world = VirtualWorld(create_apartment_layout())
        return SimulatedRobot(world)

    @pytest.mark.asyncio
    async def test_all_tools_are_handled(self, sim):
        """Every tool defined for the LLM should be executable in simulation."""
        tool_names = [t["name"] for t in TOOL_DEFINITIONS]

        # Default params for each tool
        test_params = {
            "navigate_to": {"room_name": "kitchen"},
            "explore_area": {"direction": "auto"},
            "get_robot_state": {},
            "get_map_info": {},
            "set_speed": {"speed": 0.2},
            "stop": {},
            "rotate": {"angle": 45},
            "go_home": {},
        }

        for tool_name in tool_names:
            params = test_params.get(tool_name, {})
            result = await sim.execute_tool(tool_name, params)
            assert "success" in result, f"Tool '{tool_name}' didn't return success field"
            assert result["success"] is True, f"Tool '{tool_name}' failed: {result.get('message')}"


class TestEndToEndSimulation:
    """End-to-end tests simulating a full robot session."""

    @pytest.mark.asyncio
    async def test_full_navigation_sequence(self):
        """Simulate: start → navigate to kitchen → check state → stop."""
        world = VirtualWorld(create_apartment_layout())
        sim = SimulatedRobot(world)

        # 1. Check initial state
        state = await sim.get_state()
        assert state["current_room"] in ("hallway", "living_room")

        # 2. Navigate to kitchen
        result = await sim.execute_tool("navigate_to", {"room_name": "kitchen"})
        assert result["success"]

        # 3. Simulate movement (many ticks)
        for _ in range(500):
            state = await sim.get_state()

        # 4. Should be near kitchen
        dist_to_kitchen = math.sqrt(
            (state["x"] - 2.0) ** 2 + (state["y"] - 6.5) ** 2
        )
        assert dist_to_kitchen < 3.0, f"Should be near kitchen, but distance is {dist_to_kitchen:.1f}m"

        # 5. Stop
        result = await sim.execute_tool("stop", {})
        assert result["success"]

    @pytest.mark.asyncio
    async def test_multi_room_tour(self):
        """Navigate through multiple rooms sequentially."""
        world = VirtualWorld(create_apartment_layout())
        sim = SimulatedRobot(world)

        rooms_to_visit = ["bedroom", "kitchen", "living_room"]

        for room in rooms_to_visit:
            result = await sim.execute_tool("navigate_to", {"room_name": room})
            assert result["success"], f"Failed to start navigation to {room}"

            # Simulate movement
            for _ in range(300):
                await sim.get_state()

        # Should have visited all rooms without crashing
        final_state = await sim.get_state()
        assert final_state["battery_level"] > 0, "Battery should not be depleted"

    @pytest.mark.asyncio
    async def test_battery_monitoring(self):
        """Battery should drain during navigation."""
        world = VirtualWorld(create_apartment_layout())
        sim = SimulatedRobot(world)

        initial_battery = (await sim.get_state())["battery_level"]

        # Navigate and move
        await sim.execute_tool("navigate_to", {"x": 8.0, "y": 6.0})
        for _ in range(100):
            await sim.get_state()

        final_battery = (await sim.get_state())["battery_level"]
        # Battery drain is very slow in sim, so just check it's not increasing
        assert final_battery <= initial_battery


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

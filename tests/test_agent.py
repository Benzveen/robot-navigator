"""Tests for the LLM agent layer."""

import asyncio
import os
import sys
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from agent.agent import RobotAgent, AgentConfig
from agent.fallback import FallbackController, FallbackMode
from agent.tools import TOOL_DEFINITIONS
from agent.prompts import build_system_prompt


class TestPrompts:
    """Test prompt building."""

    def test_build_system_prompt(self):
        state = {
            "max_speed": 0.3,
            "obstacle_range": 0.5,
            "battery_level": 85,
            "x": 1.5,
            "y": 2.3,
            "current_room": "kitchen",
            "map_status": "mapping",
        }
        prompt = build_system_prompt(state)
        assert "kitchen" in prompt
        assert "85" in prompt
        assert "1.50" in prompt

    def test_build_system_prompt_defaults(self):
        prompt = build_system_prompt({})
        assert "unknown" in prompt


class TestToolDefinitions:
    """Test tool definitions are valid."""

    def test_all_tools_have_required_fields(self):
        for tool in TOOL_DEFINITIONS:
            assert "name" in tool
            assert "description" in tool
            assert "input_schema" in tool
            assert tool["input_schema"]["type"] == "object"

    def test_tool_names_are_unique(self):
        names = [t["name"] for t in TOOL_DEFINITIONS]
        assert len(names) == len(set(names))

    def test_expected_tools_exist(self):
        names = {t["name"] for t in TOOL_DEFINITIONS}
        expected = {"navigate_to", "explore_area", "get_robot_state", "stop", "go_home"}
        assert expected.issubset(names)


class TestFallbackController:
    """Test offline fallback logic."""

    def test_stop_mode(self):
        fc = FallbackController(mode=FallbackMode.STOP)
        action = fc.get_fallback_action({"battery_level": 50})
        assert action["action"] == "stop"

    def test_return_home_mode(self):
        fc = FallbackController(mode=FallbackMode.RETURN_HOME, home_position=(1.0, 2.0))
        action = fc.get_fallback_action({"battery_level": 50})
        assert action["action"] == "navigate_to"
        assert action["params"]["x"] == 1.0

    def test_waypoint_mode_with_queue(self):
        fc = FallbackController(mode=FallbackMode.WAYPOINT)
        fc.add_waypoint(3.0, 4.0)
        action = fc.get_fallback_action({"battery_level": 50})
        assert action["action"] == "navigate_to"
        assert action["params"]["x"] == 3.0

    def test_waypoint_mode_empty_queue(self):
        fc = FallbackController(mode=FallbackMode.WAYPOINT)
        action = fc.get_fallback_action({"battery_level": 50})
        assert action["action"] == "stop"

    def test_critical_battery_overrides_mode(self):
        fc = FallbackController(mode=FallbackMode.STOP, home_position=(0.0, 0.0))
        action = fc.get_fallback_action({"battery_level": 10})
        assert action["action"] == "navigate_to"
        assert action["reason"] == "critical_battery"

    def test_waypoint_reached(self):
        fc = FallbackController(mode=FallbackMode.WAYPOINT)
        fc.add_waypoint(1.0, 1.0)
        fc.add_waypoint(2.0, 2.0)
        assert len(fc.waypoint_queue) == 2
        fc.waypoint_reached()
        assert len(fc.waypoint_queue) == 1


class TestRobotAgent:
    """Test the main agent (without API calls)."""

    def test_agent_creation(self):
        agent = RobotAgent()
        assert agent.config.model is not None
        assert not agent.is_running

    def test_agent_event_system(self):
        agent = RobotAgent()
        received = []
        agent.on("test_event", lambda data: received.append(data))
        agent._emit("test_event", {"key": "value"})
        assert len(received) == 1
        assert received[0]["key"] == "value"

    def test_reset_conversation(self):
        agent = RobotAgent()
        agent.messages.append({"role": "user", "content": "test"})
        agent.reset_conversation()
        assert len(agent.messages) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

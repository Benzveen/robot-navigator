"""Main LLM Agent — the brain of the robot.

Implements an agentic loop:
  1. Sense  — gather robot state + sensor data
  2. Think  — send state to Claude, get tool calls
  3. Act    — execute tool calls on the robot
  4. Report — log results, update dashboard
"""

from __future__ import annotations

import asyncio
import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import anthropic
from dotenv import load_dotenv
from loguru import logger

from agent.fallback import FallbackController, FallbackMode
from agent.prompts import build_system_prompt, build_exploration_prompt
from agent.tools import TOOL_DEFINITIONS

load_dotenv()


@dataclass
class AgentConfig:
    """Configuration for the LLM agent."""
    model: str = os.getenv("ANTHROPIC_MODEL", "claude-sonnet-4-20250514")
    max_tokens: int = 1024
    temperature: float = 0.3
    think_interval: float = 5.0
    max_retries: int = 3
    retry_delay: float = 2.0
    fallback_mode: FallbackMode = FallbackMode.WAYPOINT


class RobotAgent:
    """
    LLM-powered robot agent using Anthropic Claude.

    The agent runs a continuous sense-think-act loop, using Claude's
    tool-calling capability to make navigation decisions.
    """

    def __init__(
        self,
        config: Optional[AgentConfig] = None,
        tool_executor: Optional[Callable] = None,
        state_provider: Optional[Callable] = None,
    ):
        self.config = config or AgentConfig()
        self.client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))
        self.tool_executor = tool_executor
        self.state_provider = state_provider
        self.fallback = FallbackController(mode=self.config.fallback_mode)

        # Conversation history for multi-turn context
        self.messages: list[dict] = []
        self.max_history = 20  # Keep last N messages to manage token usage

        # State
        self.is_running = False
        self.is_online = True
        self._last_think_time = 0.0

        # Event callbacks for dashboard
        self._event_listeners: dict[str, list[Callable]] = {}

        logger.info(f"RobotAgent initialized with model: {self.config.model}")

    # ── Event System ────────────────────────────────────────────────

    def on(self, event: str, callback: Callable) -> None:
        """Register an event listener."""
        self._event_listeners.setdefault(event, []).append(callback)

    def _emit(self, event: str, data: Any = None) -> None:
        """Emit an event to all registered listeners."""
        for callback in self._event_listeners.get(event, []):
            try:
                callback(data)
            except Exception as e:
                logger.error(f"Event listener error ({event}): {e}")

    # ── Core Agent Loop ─────────────────────────────────────────────

    async def start(self) -> None:
        """Start the agent's sense-think-act loop."""
        self.is_running = True
        logger.info("🤖 Agent loop started")
        self._emit("agent_started")

        while self.is_running:
            try:
                await self._tick()
                await asyncio.sleep(self.config.think_interval)
            except asyncio.CancelledError:
                logger.info("Agent loop cancelled")
                break
            except Exception as e:
                logger.error(f"Agent loop error: {e}")
                await asyncio.sleep(self.config.retry_delay)

        logger.info("🤖 Agent loop stopped")
        self._emit("agent_stopped")

    async def stop(self) -> None:
        """Stop the agent loop."""
        self.is_running = False
        logger.info("Agent stop requested")

    async def _tick(self) -> None:
        """Single iteration of the sense-think-act loop."""
        # 1. SENSE — gather current state
        robot_state = await self._get_robot_state()
        self._emit("state_update", robot_state)

        # 2. Check if we should think (rate limiting)
        now = time.time()
        if now - self._last_think_time < self.config.think_interval:
            return
        self._last_think_time = now

        # 3. THINK — get decision from LLM (or fallback)
        try:
            actions = await self._think(robot_state)
            self.is_online = True
        except (anthropic.APIConnectionError, anthropic.APITimeoutError) as e:
            logger.warning(f"API unreachable: {e}. Switching to fallback.")
            self.is_online = False
            fallback_action = self.fallback.get_fallback_action(robot_state)
            actions = [fallback_action] if fallback_action else []
            self._emit("fallback_active", {"reason": str(e)})

        # 4. ACT — execute decisions
        for action in actions:
            await self._execute_action(action)

    async def _get_robot_state(self) -> dict:
        """Get current robot state from the state provider."""
        if self.state_provider:
            return await self.state_provider()
        # Default mock state for testing
        return {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "battery_level": 100,
            "speed": 0.0,
            "current_room": "unknown",
            "nearest_obstacle": 999.0,
            "map_status": "not started",
            "max_speed": 0.3,
            "obstacle_range": 0.5,
        }

    # ── LLM Thinking ───────────────────────────────────────────────

    async def _think(self, robot_state: dict) -> list[dict]:
        """
        Send robot state to Claude and get back tool-call actions.

        Returns a list of action dicts: [{"action": "navigate_to", "params": {...}}, ...]
        """
        system_prompt = build_system_prompt(robot_state)

        # Build the user message with current state summary
        # Extract nearest obstacle distance (handles both float and dict formats)
        obstacle_raw = robot_state.get('nearest_obstacle', 999.0)
        if isinstance(obstacle_raw, dict):
            obstacle_dist = obstacle_raw.get('distance', 999.0)
        else:
            obstacle_dist = float(obstacle_raw)

        state_summary = (
            f"Current state update:\n"
            f"- Position: ({robot_state['x']:.2f}, {robot_state['y']:.2f})\n"
            f"- Room: {robot_state['current_room']}\n"
            f"- Battery: {robot_state['battery_level']}%\n"
            f"- Nearest obstacle: {obstacle_dist:.2f}m\n"
            f"- Map: {robot_state['map_status']}\n\n"
            f"What should I do next?"
        )

        self.messages.append({"role": "user", "content": state_summary})

        # Trim history to prevent token overflow
        if len(self.messages) > self.max_history:
            self._trim_history()

        actions = []

        for attempt in range(self.config.max_retries):
            try:
                response = self.client.messages.create(
                    model=self.config.model,
                    max_tokens=self.config.max_tokens,
                    temperature=self.config.temperature,
                    system=system_prompt,
                    tools=TOOL_DEFINITIONS,
                    messages=self.messages,
                )

                # Process response
                assistant_content = response.content
                self.messages.append({"role": "assistant", "content": assistant_content})

                # Extract tool calls
                tool_results = []
                for block in assistant_content:
                    if block.type == "tool_use":
                        action = {
                            "action": block.name,
                            "params": block.input,
                            "tool_use_id": block.id,
                        }
                        actions.append(action)
                        logger.info(f"🧠 LLM decided: {block.name}({json.dumps(block.input)})")
                        self._emit("llm_decision", action)

                        # Queue waypoints for offline fallback
                        if block.name == "navigate_to" and "x" in block.input and "y" in block.input:
                            self.fallback.add_waypoint(block.input["x"], block.input["y"])

                    elif block.type == "text":
                        logger.info(f"🧠 LLM says: {block.text}")
                        self._emit("llm_message", block.text)

                break  # Success — exit retry loop

            except anthropic.RateLimitError:
                logger.warning(f"Rate limited. Retry {attempt + 1}/{self.config.max_retries}")
                await asyncio.sleep(self.config.retry_delay * (attempt + 1))
            except anthropic.APIStatusError as e:
                logger.error(f"API error: {e.status_code} — {e.message}")
                raise

        return actions

    # ── Action Execution ────────────────────────────────────────────

    async def _execute_action(self, action: dict) -> dict:
        """Execute a single action on the robot hardware."""
        action_name = action.get("action", "unknown")
        params = action.get("params", {})
        tool_use_id = action.get("tool_use_id")

        logger.info(f"⚡ Executing: {action_name}({json.dumps(params)})")
        self._emit("action_start", action)

        result = {"success": False, "message": "No executor configured"}

        if self.tool_executor:
            try:
                result = await self.tool_executor(action_name, params)
                logger.info(f"✅ {action_name} result: {result.get('message', 'ok')}")
            except Exception as e:
                result = {"success": False, "message": str(e)}
                logger.error(f"❌ {action_name} failed: {e}")

        self._emit("action_complete", {"action": action_name, "result": result})

        # Send tool result back to Claude for multi-turn awareness
        if tool_use_id:
            self.messages.append({
                "role": "user",
                "content": [
                    {
                        "type": "tool_result",
                        "tool_use_id": tool_use_id,
                        "content": json.dumps(result),
                    }
                ],
            })

        return result

    # ── History Management ──────────────────────────────────────────

    def _trim_history(self) -> None:
        """
        Trim conversation history while keeping it valid for the Anthropic API.

        Rules:
        - History must start with a "user" role message
        - A "tool_result" message must be preceded by an assistant message
          containing the matching "tool_use" block
        - We keep the most recent messages and discard old ones
        """
        trimmed = self.messages[-self.max_history:]

        # Find the first safe starting point:
        # Must be a plain "user" message (string content, not tool_result)
        safe_start = 0
        for i, msg in enumerate(trimmed):
            if msg["role"] == "user" and isinstance(msg.get("content"), str):
                safe_start = i
                break

        self.messages = trimmed[safe_start:]

        # If we ended up empty or still start with assistant, reset
        if not self.messages or self.messages[0]["role"] != "user":
            self.messages = []
            logger.debug("Conversation history reset (no safe trim point)")
        else:
            logger.debug(f"History trimmed to {len(self.messages)} messages")

    # ── User Commands ───────────────────────────────────────────────

    async def send_command(self, user_message: str) -> str:
        """
        Send a natural language command from the user to the agent.

        Returns the agent's text response.
        """
        logger.info(f"📨 User command: {user_message}")

        self.messages.append({"role": "user", "content": user_message})

        robot_state = await self._get_robot_state()
        system_prompt = build_system_prompt(robot_state)

        try:
            response = self.client.messages.create(
                model=self.config.model,
                max_tokens=self.config.max_tokens,
                temperature=self.config.temperature,
                system=system_prompt,
                tools=TOOL_DEFINITIONS,
                messages=self.messages,
            )

            self.messages.append({"role": "assistant", "content": response.content})

            text_response = ""
            for block in response.content:
                if block.type == "text":
                    text_response += block.text
                elif block.type == "tool_use":
                    action = {
                        "action": block.name,
                        "params": block.input,
                        "tool_use_id": block.id,
                    }
                    await self._execute_action(action)

            return text_response or "Command executed."

        except Exception as e:
            logger.error(f"Command processing error: {e}")
            return f"Error processing command: {e}"

    def reset_conversation(self) -> None:
        """Clear conversation history."""
        self.messages.clear()
        logger.info("Conversation history cleared")

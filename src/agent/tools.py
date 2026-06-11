"""Tool definitions for the LLM agent — these map to real robot actions."""

from __future__ import annotations

from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


# ── Tool Input Schemas ──────────────────────────────────────────────


class NavigateToInput(BaseModel):
    """Navigate the robot to a target location."""
    room_name: Optional[str] = Field(None, description="Name of the room to navigate to (e.g., 'kitchen', 'bedroom')")
    x: Optional[float] = Field(None, description="Target X coordinate in meters (map frame)")
    y: Optional[float] = Field(None, description="Target Y coordinate in meters (map frame)")


class ExploreAreaInput(BaseModel):
    """Explore an unknown area to expand the map."""
    direction: Optional[str] = Field(None, description="Preferred direction: 'north', 'south', 'east', 'west', or 'auto'")
    duration: float = Field(30.0, description="Maximum exploration time in seconds", ge=5.0, le=300.0)


class SetSpeedInput(BaseModel):
    """Set the robot's movement speed."""
    speed: float = Field(description="Speed in m/s (0.05 to 0.3)", ge=0.05, le=0.3)


class RotateInput(BaseModel):
    """Rotate the robot in place."""
    angle: float = Field(description="Rotation angle in degrees. Positive = counter-clockwise, negative = clockwise")
    speed: float = Field(0.5, description="Rotation speed in rad/s", ge=0.1, le=1.0)


# ── Anthropic Tool Definitions ──────────────────────────────────────

TOOL_DEFINITIONS = [
    {
        "name": "navigate_to",
        "description": (
            "Navigate the robot to a target location. Provide either a room_name "
            "(e.g., 'kitchen') or specific x,y coordinates in meters. "
            "The robot will plan a path and avoid obstacles automatically."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "room_name": {
                    "type": "string",
                    "description": "Name of the room to navigate to (e.g., 'kitchen', 'bedroom', 'living_room')",
                },
                "x": {
                    "type": "number",
                    "description": "Target X coordinate in meters (map frame). Use when room_name is not available.",
                },
                "y": {
                    "type": "number",
                    "description": "Target Y coordinate in meters (map frame). Use when room_name is not available.",
                },
            },
        },
    },
    {
        "name": "explore_area",
        "description": (
            "Explore an unknown/unmapped area to expand the robot's map. "
            "The robot will move toward the nearest unexplored frontier. "
            "Use this when there are areas that haven't been mapped yet."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "direction": {
                    "type": "string",
                    "enum": ["north", "south", "east", "west", "auto"],
                    "description": "Preferred exploration direction. Use 'auto' to let the robot choose.",
                },
                "duration": {
                    "type": "number",
                    "description": "Maximum exploration time in seconds (5-300). Default: 30",
                    "default": 30,
                },
            },
        },
    },
    {
        "name": "get_robot_state",
        "description": (
            "Get the robot's current state including position, battery level, "
            "speed, nearest obstacles, and current room. Call this before making decisions."
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "get_map_info",
        "description": (
            "Get information about the current map: explored percentage, "
            "known rooms, unexplored frontiers, and total area covered."
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "set_speed",
        "description": "Set the robot's maximum movement speed in meters per second.",
        "input_schema": {
            "type": "object",
            "properties": {
                "speed": {
                    "type": "number",
                    "description": "Speed in m/s. Range: 0.05 (slow/careful) to 0.3 (fast). Default: 0.15",
                    "minimum": 0.05,
                    "maximum": 0.3,
                },
            },
            "required": ["speed"],
        },
    },
    {
        "name": "stop",
        "description": (
            "Immediately stop the robot. Use in emergencies or when the current "
            "task should be cancelled."
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
        },
    },
    {
        "name": "rotate",
        "description": "Rotate the robot in place by a specified angle in degrees.",
        "input_schema": {
            "type": "object",
            "properties": {
                "angle": {
                    "type": "number",
                    "description": "Rotation angle in degrees. Positive = counter-clockwise, negative = clockwise.",
                },
                "speed": {
                    "type": "number",
                    "description": "Rotation speed in rad/s (0.1 to 1.0). Default: 0.5",
                    "default": 0.5,
                },
            },
            "required": ["angle"],
        },
    },
    {
        "name": "go_home",
        "description": (
            "Navigate the robot back to its home/charging station. "
            "Use when battery is low or when exploration is complete."
        ),
        "input_schema": {
            "type": "object",
            "properties": {},
        },
    },
]

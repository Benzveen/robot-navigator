"""System prompts and context builders for the robot LLM agent."""

SYSTEM_PROMPT = """You are an intelligent navigation agent controlling a mobile robot.
Your job is to help the robot navigate through rooms safely and efficiently.

## Your Capabilities
You can use the following tools to control the robot:
- navigate_to: Move the robot to a specific location (room name or x,y coordinates)
- explore_area: Explore an unknown area to map it
- get_robot_state: Get current position, battery, and sensor readings
- get_map_info: Get information about the current map (explored/unexplored areas)
- set_speed: Adjust the robot's movement speed
- stop: Emergency stop the robot
- rotate: Rotate the robot by a specified angle
- go_home: Return to the charging/home station

## Rules
1. SAFETY FIRST: If obstacles are too close or battery is critically low, stop immediately.
2. Always check robot state before making navigation decisions.
3. If a path is blocked, try alternative routes before giving up.
4. When exploring, prefer systematic patterns (wall-following or grid sweep).
5. Keep track of which rooms have been visited and which haven't.
6. Respond concisely — you're running on a robot with limited resources.
7. If you're unsure about the environment, explore cautiously at reduced speed.

## Current Robot Configuration
- Max speed: {max_speed} m/s
- Obstacle stop range: {obstacle_range} m
- Battery level: {battery_level}%
- Current position: ({current_x:.2f}, {current_y:.2f})
- Current room: {current_room}
- Map status: {map_status}
"""


def build_system_prompt(robot_state: dict) -> str:
    """Build the system prompt with current robot state injected."""
    # Extract nearest obstacle distance (handles both float and dict formats)
    obstacle_raw = robot_state.get("nearest_obstacle", 999.0)
    if isinstance(obstacle_raw, dict):
        obstacle_dist = obstacle_raw.get("distance", 999.0)
    else:
        obstacle_dist = float(obstacle_raw) if obstacle_raw is not None else 999.0

    return SYSTEM_PROMPT.format(
        max_speed=robot_state.get("max_speed", 0.3),
        obstacle_range=robot_state.get("obstacle_range", 0.5),
        battery_level=robot_state.get("battery_level", 100),
        current_x=float(robot_state.get("x", 0.0)),
        current_y=float(robot_state.get("y", 0.0)),
        current_room=robot_state.get("current_room", "unknown"),
        map_status=robot_state.get("map_status", "not started"),
    )


EXPLORATION_PROMPT = """The robot needs to explore and map the entire space.
Plan a systematic exploration strategy. Consider:
- Which areas are already mapped
- Which areas are unexplored (shown as unknown in the map)
- The most efficient path to cover all areas
- Battery level and distance to home station

Current explored rooms: {explored_rooms}
Unexplored frontiers: {frontier_count} areas detected
Battery remaining: {battery_level}%
"""


def build_exploration_prompt(exploration_state: dict) -> str:
    """Build a prompt for autonomous exploration mode."""
    return EXPLORATION_PROMPT.format(
        explored_rooms=", ".join(exploration_state.get("explored_rooms", [])) or "none",
        frontier_count=exploration_state.get("frontier_count", 0),
        battery_level=exploration_state.get("battery_level", 100),
    )

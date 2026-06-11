"""Tests for the virtual world simulation."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from simulator.virtual_world import (
    VirtualWorld,
    VirtualRobot,
    RoomLayout,
    Wall,
    create_apartment_layout,
)


class TestRoomLayout:
    """Test room layout creation."""

    def test_apartment_layout_has_walls(self):
        layout = create_apartment_layout()
        assert len(layout.walls) > 0
        assert layout.width == 10.0
        assert layout.height == 8.0

    def test_apartment_layout_has_rooms(self):
        layout = create_apartment_layout()
        assert "bedroom" in layout.rooms
        assert "kitchen" in layout.rooms
        assert "living_room" in layout.rooms
        assert "hallway" in layout.rooms

    def test_room_coordinates_within_bounds(self):
        layout = create_apartment_layout()
        for name, (x, y) in layout.rooms.items():
            assert 0 < x < layout.width, f"Room '{name}' x={x} out of bounds"
            assert 0 < y < layout.height, f"Room '{name}' y={y} out of bounds"


class TestVirtualWorld:
    """Test the virtual world simulation."""

    def test_world_creation(self):
        world = VirtualWorld()
        assert world.robot.x == 5.0
        assert world.robot.y == 4.0
        assert world.robot.battery == 100.0

    def test_robot_movement(self):
        world = VirtualWorld()
        world.robot.yaw = 0  # facing right
        world.robot.speed = 0.1
        old_x = world.robot.x
        world.update(dt=1.0)
        assert world.robot.x > old_x, "Robot should move right"

    def test_robot_rotation(self):
        world = VirtualWorld()
        world.robot.angular_speed = math.pi / 2  # 90°/s
        old_yaw = world.robot.yaw
        world.update(dt=1.0)
        assert abs(world.robot.yaw - old_yaw - math.pi / 2) < 0.01

    def test_collision_with_outer_wall(self):
        world = VirtualWorld()
        # Try to move outside the map
        collision = world._check_collision(-0.5, 4.0)
        assert collision, "Should detect collision outside map"

    def test_no_collision_in_open_space(self):
        world = VirtualWorld()
        # Center of living room should be clear
        collision = world._check_collision(7.0, 4.0)
        assert not collision, "Should not collide in open space"

    def test_set_velocity(self):
        world = VirtualWorld()
        world.set_velocity(0.2, 0.5)
        assert world.robot.speed == 0.2
        assert world.robot.angular_speed == 0.5

    def test_velocity_clamping(self):
        world = VirtualWorld()
        world.set_velocity(10.0, 10.0)  # Way too fast
        assert world.robot.speed == world.robot.max_speed
        assert world.robot.angular_speed == 1.0

    def test_battery_drain(self):
        world = VirtualWorld()
        world.robot.x = 5.0  # Center of map — no walls nearby
        world.robot.y = 4.0
        world.robot.yaw = 0  # Facing right (open space)
        world.robot.speed = 0.1  # Moving slowly
        initial_battery = world.robot.battery
        # Use small time steps to avoid collision-stop
        for _ in range(100):
            world.update(dt=0.1)
        assert world.robot.battery < initial_battery, "Battery should drain while moving"

    def test_battery_no_drain_when_stopped(self):
        world = VirtualWorld()
        world.robot.speed = 0.0  # Stopped
        initial_battery = world.robot.battery
        world.update(dt=10.0)
        assert world.robot.battery == initial_battery, "Battery should not drain when stopped"


class TestLidarSimulation:
    """Test simulated LiDAR scans."""

    def test_lidar_scan_returns_correct_count(self):
        world = VirtualWorld()
        scan = world.get_lidar_scan(num_rays=360)
        assert len(scan) == 360

    def test_lidar_scan_returns_positive_distances(self):
        world = VirtualWorld()
        scan = world.get_lidar_scan(num_rays=36)
        for dist in scan:
            assert dist > 0, "LiDAR distances should be positive"

    def test_lidar_detects_walls(self):
        world = VirtualWorld()
        # Place robot near a wall
        world.robot.x = 0.5
        world.robot.y = 4.0
        scan = world.get_lidar_scan(num_rays=360)
        min_dist = min(scan)
        assert min_dist < 1.0, "Should detect nearby wall"

    def test_nearest_obstacle(self):
        world = VirtualWorld()
        dist, angle = world.get_nearest_obstacle()
        assert dist > 0
        assert isinstance(angle, float)


class TestRoomDetection:
    """Test room identification."""

    def test_detect_hallway(self):
        world = VirtualWorld()
        world.robot.x = 4.5
        world.robot.y = 4.0
        assert world.get_current_room() == "hallway"

    def test_detect_bedroom(self):
        world = VirtualWorld()
        world.robot.x = 2.0
        world.robot.y = 1.5
        assert world.get_current_room() == "bedroom"

    def test_detect_kitchen(self):
        world = VirtualWorld()
        world.robot.x = 2.0
        world.robot.y = 6.5
        assert world.get_current_room() == "kitchen"

    def test_detect_living_room(self):
        world = VirtualWorld()
        world.robot.x = 7.0
        world.robot.y = 4.0
        assert world.get_current_room() == "living_room"


class TestRobotState:
    """Test state serialization."""

    def test_get_robot_state_has_required_fields(self):
        world = VirtualWorld()
        state = world.get_robot_state()

        required_fields = [
            "x", "y", "yaw", "battery_level", "current_room",
            "nearest_obstacle", "is_safe", "sensors", "map_status",
        ]
        for field in required_fields:
            assert field in state, f"Missing field: {field}"

    def test_state_sensors_all_connected(self):
        world = VirtualWorld()
        state = world.get_robot_state()
        assert state["sensors"]["lidar"] is True
        assert state["sensors"]["imu"] is True
        assert state["sensors"]["arduino"] is True

    def test_state_battery_in_range(self):
        world = VirtualWorld()
        state = world.get_robot_state()
        assert 0 <= state["battery_level"] <= 100


class TestNavigation:
    """Test simulated navigation."""

    def test_move_to_reaches_target(self):
        world = VirtualWorld()
        # Move to a nearby point
        for _ in range(200):
            reached = world.move_to(6.0, 4.0)
            if reached:
                break
        assert reached, "Should reach nearby target"

    def test_move_to_updates_position(self):
        world = VirtualWorld()
        old_x = world.robot.x
        world.move_to(8.0, 4.0)
        assert world.robot.x != old_x or world.robot.y != 4.0


class TestRaycasting:
    """Test ray-segment intersection math."""

    def test_ray_hits_horizontal_wall(self):
        world = VirtualWorld()
        # Ray going up from (5, 0.5) should hit top wall at y=8
        dist = world._raycast(5.0, 0.5, math.pi / 2, 20.0)
        assert 7.0 < dist < 8.0, f"Expected ~7.5m, got {dist}"

    def test_ray_hits_vertical_wall(self):
        world = VirtualWorld()
        # Ray going right from (0.5, 4) should hit right wall at x=10
        dist = world._raycast(0.5, 4.0, 0, 20.0)
        # Might hit internal wall first, but should be < 10m
        assert dist < 10.0

    def test_ray_misses_parallel_wall(self):
        world = VirtualWorld()
        # A ray parallel to a wall should not intersect it at close range
        # This is more of a sanity check
        dist = world._raycast(5.0, 4.0, 0, 0.01)
        assert dist == 0.01  # max_range returned (no wall within 1cm)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

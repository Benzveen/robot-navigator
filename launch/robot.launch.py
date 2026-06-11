"""Main ROS 2 launch file — starts all robot nodes."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_dir = Path(__file__).parent.parent
    config_dir = pkg_dir / "src" / "config"
    urdf_file = pkg_dir / "urdf" / "robot.urdf"

    # Launch arguments
    use_sim = DeclareLaunchArgument("use_sim", default_value="false", description="Use simulation mode")
    slam_mode = DeclareLaunchArgument("slam_mode", default_value="mapping", description="SLAM mode: mapping or localization")

    # Read URDF
    urdf_content = ""
    if urdf_file.exists():
        urdf_content = urdf_file.read_text()

    # ── Robot State Publisher ───────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf_content}],
        output="screen",
    )

    # ── RPLidar Driver ──────────────────────────────────────
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "serial_port": "/dev/ttyUSB1",
            "serial_baudrate": 115200,
            "frame_id": "laser_frame",
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
        output="screen",
    )

    # ── SLAM Toolbox ────────────────────────────────────────
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[str(config_dir / "slam_params.yaml")],
        output="screen",
    )

    # ── Nav2 Stack ──────────────────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.environ.get("AMENT_PREFIX_PATH", "").split(":")[0],
                "share", "nav2_bringup", "launch", "navigation_launch.py"
            )
        ),
        launch_arguments={
            "params_file": str(config_dir / "nav2_params.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim"),
        }.items(),
    )

    # ── Robot Localization (EKF) ────────────────────────────
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[{
            "frequency": 30.0,
            "sensor_timeout": 0.1,
            "two_d_mode": True,
            "map_frame": "map",
            "odom_frame": "odom",
            "base_link_frame": "base_link",
            "world_frame": "odom",
            "odom0": "/wheel_odom",
            "odom0_config": [True, True, False,
                             False, False, True,
                             True, False, False,
                             False, False, True,
                             False, False, False],
            "imu0": "/imu/data",
            "imu0_config": [False, False, False,
                            True, True, True,
                            False, False, False,
                            True, True, True,
                            True, False, False],
        }],
        output="screen",
    )

    return LaunchDescription([
        use_sim,
        slam_mode,
        robot_state_publisher,
        rplidar_node,
        slam_toolbox,
        robot_localization,
        nav2_bringup,
    ])

"""
Robot Navigator — Main Entry Point

Starts all subsystems:
  1. Hardware drivers (LiDAR, IMU, Motors)
  2. Safety monitor
  3. ROS 2 bridge (Nav2, SLAM)
  4. LLM Agent (Anthropic Claude)
  5. Web Dashboard (FastAPI)
"""

from __future__ import annotations

import asyncio
import os
import signal
import sys
from pathlib import Path

import yaml
from dotenv import load_dotenv
from loguru import logger

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from agent.agent import RobotAgent, AgentConfig
from agent.fallback import FallbackMode
from drivers.lidar import LidarDriver
from drivers.imu import IMUDriver
from drivers.motor_controller import MotorController
from drivers.safety import SafetyMonitor, SafetyConfig
from navigation.robot_state import RobotState
from ros_nodes.agent_bridge_node import AgentBridgeNode
from dashboard import server as dashboard_server

# Load environment
load_dotenv()


def load_config() -> dict:
    """Load robot configuration from YAML."""
    config_path = Path(__file__).parent / "config" / "robot_config.yaml"
    if config_path.exists():
        with open(config_path) as f:
            return yaml.safe_load(f)
    logger.warning("Config file not found, using defaults")
    return {}


def setup_logging(config: dict) -> None:
    """Configure loguru logging."""
    log_config = config.get("logging", {})
    log_level = log_config.get("level", os.getenv("LOG_LEVEL", "INFO"))
    log_file = log_config.get("file", "logs/robot.log")

    # Remove default handler
    logger.remove()

    # Console handler
    logger.add(
        sys.stderr,
        level=log_level,
        format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan> — <level>{message}</level>",
    )

    # File handler
    log_dir = Path(log_file).parent
    log_dir.mkdir(parents=True, exist_ok=True)
    logger.add(
        log_file,
        level="DEBUG",
        rotation=log_config.get("rotation", "10 MB"),
        retention=log_config.get("retention", "7 days"),
        compression="zip",
    )


class RobotSystem:
    """
    Main robot system — orchestrates all subsystems.
    """

    def __init__(self, config: dict):
        self.config = config
        self._shutdown_event = asyncio.Event()

        # Subsystems
        self.motor_controller: MotorController | None = None
        self.lidar: LidarDriver | None = None
        self.imu: IMUDriver | None = None
        self.safety: SafetyMonitor | None = None
        self.bridge: AgentBridgeNode | None = None
        self.agent: RobotAgent | None = None

    async def start(self) -> None:
        """Initialize and start all subsystems."""
        logger.info("🚀 Robot Navigator starting up...")

        hw_config = self.config.get("hardware", {})
        nav_config = self.config.get("navigation", {})
        agent_config = self.config.get("agent", {})

        # ── 1. Hardware Drivers ─────────────────────────────
        logger.info("Initializing hardware drivers...")

        # Motor controller (Arduino)
        arduino_cfg = hw_config.get("arduino", {})
        self.motor_controller = MotorController(
            port=arduino_cfg.get("port", "/dev/ttyUSB0"),
            baud_rate=arduino_cfg.get("baud_rate", 115200),
            heartbeat_interval=arduino_cfg.get("heartbeat_interval", 0.5),
        )
        await self.motor_controller.connect()

        # LiDAR
        lidar_cfg = hw_config.get("lidar", {})
        self.lidar = LidarDriver(
            port=lidar_cfg.get("port", "/dev/ttyUSB1"),
            baud_rate=lidar_cfg.get("baud_rate", 115200),
        )
        await self.lidar.connect()
        await self.lidar.start_scanning()

        # IMU
        imu_cfg = hw_config.get("imu", {})
        self.imu = IMUDriver(
            bus_number=imu_cfg.get("i2c_bus", 1),
            address=imu_cfg.get("address", 0x68),
        )
        await self.imu.connect()

        # ── 2. Safety Monitor ───────────────────────────────
        logger.info("Starting safety monitor...")

        safety_cfg = SafetyConfig(
            obstacle_stop_distance=nav_config.get("obstacle_range", 0.3),
            obstacle_slow_distance=nav_config.get("slowdown_range", 0.8),
        )
        self.safety = SafetyMonitor(config=safety_cfg)
        self.safety.set_data_sources(
            obstacle_fn=lambda: (
                self.lidar.latest_scan.get_nearest_front(fov=90).distance / 1000.0
                if self.lidar.latest_scan.get_nearest_front(fov=90)
                else 999.0
            ),
            battery_fn=lambda: self.motor_controller.get_battery_voltage(),
            tilt_fn=lambda: abs(self.imu.reading.roll) + abs(self.imu.reading.pitch),
            motor_stop_fn=self.motor_controller.stop,
        )
        self.safety.on_emergency_stop(
            lambda reason: logger.critical(f"🚨 E-STOP: {reason}")
        )

        # ── 3. ROS 2 Bridge ────────────────────────────────
        logger.info("Initializing ROS 2 bridge...")

        self.bridge = AgentBridgeNode()
        await self.bridge.initialize()

        # Load room coordinates from config
        rooms = self.config.get("rooms", [])
        for room in rooms:
            if room.get("center_x") is not None and room.get("center_y") is not None:
                self.bridge.room_coords[room["name"]] = {
                    "x": room["center_x"],
                    "y": room["center_y"],
                }

        # ── 4. LLM Agent ───────────────────────────────────
        logger.info("Starting LLM agent...")

        fallback_mode = FallbackMode(agent_config.get("fallback_mode", "waypoint"))
        self.agent = RobotAgent(
            config=AgentConfig(
                model=agent_config.get("model", os.getenv("ANTHROPIC_MODEL", "claude-sonnet-4-20250514")),
                max_tokens=agent_config.get("max_tokens", 1024),
                temperature=agent_config.get("temperature", 0.3),
                think_interval=agent_config.get("think_interval", 5.0),
                max_retries=agent_config.get("max_retries", 3),
                fallback_mode=fallback_mode,
            ),
            tool_executor=self.bridge.execute_tool,
            state_provider=self.bridge.get_state,
        )

        # ── 5. Web Dashboard ───────────────────────────────
        logger.info("Starting web dashboard...")

        dashboard_cfg = self.config.get("dashboard", {})
        dashboard_server.set_agent(self.agent)
        dashboard_server.set_bridge(self.bridge)

        # ── Start all async tasks ──────────────────────────
        logger.info("=" * 50)
        logger.info("🤖 Robot Navigator is READY!")
        logger.info(f"📊 Dashboard: http://localhost:{dashboard_cfg.get('port', 8080)}")
        logger.info(f"🧠 LLM Model: {self.agent.config.model}")
        logger.info("=" * 50)

        tasks = [
            asyncio.create_task(self.agent.start(), name="agent"),
            asyncio.create_task(self.safety.start(), name="safety"),
            asyncio.create_task(self.imu.start_reading(), name="imu"),
            asyncio.create_task(
                self._run_dashboard(
                    dashboard_cfg.get("host", "0.0.0.0"),
                    dashboard_cfg.get("port", 8080),
                ),
                name="dashboard",
            ),
            asyncio.create_task(dashboard_server.broadcast_loop(), name="broadcast"),
        ]

        # Wait for shutdown signal
        await self._shutdown_event.wait()

        # Cancel all tasks
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)

    async def _run_dashboard(self, host: str, port: int) -> None:
        """Run the FastAPI dashboard server."""
        import uvicorn

        config = uvicorn.Config(
            dashboard_server.app,
            host=host,
            port=port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        await server.serve()

    async def shutdown(self) -> None:
        """Gracefully shut down all subsystems."""
        logger.info("🛑 Shutting down Robot Navigator...")

        # Stop agent
        if self.agent:
            await self.agent.stop()

        # Stop safety monitor
        if self.safety:
            self.safety.stop()

        # Stop motors
        if self.motor_controller:
            await self.motor_controller.stop()
            await self.motor_controller.disconnect()

        # Stop LiDAR
        if self.lidar:
            await self.lidar.stop_scanning()
            await self.lidar.disconnect()

        # Stop IMU
        if self.imu:
            self.imu.stop_reading()
            await self.imu.disconnect()

        # Stop ROS 2 bridge
        if self.bridge:
            await self.bridge.shutdown()

        self._shutdown_event.set()
        logger.info("✅ Robot Navigator shut down complete")


async def main() -> None:
    """Main entry point."""
    config = load_config()
    setup_logging(config)

    system = RobotSystem(config)

    # Handle shutdown signals
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(system.shutdown()))

    try:
        await system.start()
    except KeyboardInterrupt:
        await system.shutdown()
    except Exception as e:
        logger.exception(f"Fatal error: {e}")
        await system.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())

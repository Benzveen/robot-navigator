"""Motor Controller — communicates with Arduino over serial for motor control.

The Arduino handles:
  - PWM motor control
  - Hardware watchdog (stops motors if no heartbeat)
  - Encoder counting
  - Emergency stop

Communication protocol (JSON over serial):
  Pi → Arduino:  {"cmd": "move", "left": 150, "right": 150}
  Arduino → Pi:  {"enc_l": 1234, "enc_r": 1230, "battery": 11.2}
"""

from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass
from typing import Optional

from loguru import logger

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    logger.warning("pyserial not available — MotorController in simulation mode")


@dataclass
class MotorState:
    """Current motor/encoder state from Arduino."""
    left_encoder: int = 0
    right_encoder: int = 0
    battery_voltage: float = 0.0
    is_emergency_stopped: bool = False
    last_update: float = 0.0


class MotorController:
    """
    Controls motors via Arduino serial interface.

    Sends velocity commands and receives encoder/battery data.
    Includes heartbeat mechanism — if heartbeat stops, Arduino
    will automatically stop the motors (safety feature).
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud_rate: int = 115200,
        heartbeat_interval: float = 0.5,
        timeout: float = 2.0,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self.heartbeat_interval = heartbeat_interval
        self.timeout = timeout

        self._serial: Optional[serial.Serial] = None
        self._connected = False
        self._state = MotorState()
        self._heartbeat_task: Optional[asyncio.Task] = None

        # Speed limits (PWM values 0-255)
        self.max_pwm = 200
        self.min_pwm = 40  # Below this, motors don't move

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def state(self) -> MotorState:
        return self._state

    async def connect(self) -> bool:
        """Open serial connection to Arduino."""
        if not SERIAL_AVAILABLE:
            logger.info("[SIM] Motor controller connected (simulation)")
            self._connected = True
            return True

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
            )
            # Wait for Arduino to reset after serial connection
            await asyncio.sleep(2.0)
            # Flush any startup messages
            self._serial.reset_input_buffer()

            self._connected = True
            logger.info(f"Motor controller connected on {self.port}")

            # Start heartbeat
            self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to Arduino: {e}")
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Close serial connection."""
        await self.stop()

        if self._heartbeat_task:
            self._heartbeat_task.cancel()

        if self._serial and self._serial.is_open:
            self._serial.close()

        self._connected = False
        logger.info("Motor controller disconnected")

    async def set_velocity(self, linear: float, angular: float) -> dict:
        """
        Set robot velocity (differential drive).

        Args:
            linear: Forward speed in m/s (-max_speed to +max_speed)
            angular: Rotation speed in rad/s (-max_angular to +max_angular)

        Returns:
            dict with success status
        """
        # Differential drive kinematics
        # v_left  = linear - angular * wheel_separation / 2
        # v_right = linear + angular * wheel_separation / 2
        wheel_separation = 0.16  # meters (from config)

        v_left = linear - angular * wheel_separation / 2.0
        v_right = linear + angular * wheel_separation / 2.0

        # Convert m/s to PWM (approximate linear mapping)
        max_speed = 0.3  # m/s at max_pwm
        left_pwm = int((v_left / max_speed) * self.max_pwm)
        right_pwm = int((v_right / max_speed) * self.max_pwm)

        # Clamp PWM values
        left_pwm = max(-self.max_pwm, min(self.max_pwm, left_pwm))
        right_pwm = max(-self.max_pwm, min(self.max_pwm, right_pwm))

        return await self._send_command("move", left=left_pwm, right=right_pwm)

    async def stop(self) -> dict:
        """Emergency stop — immediately stop both motors."""
        logger.info("🛑 Motor STOP")
        return await self._send_command("stop")

    async def _send_command(self, cmd: str, **kwargs) -> dict:
        """Send a JSON command to Arduino."""
        message = {"cmd": cmd, **kwargs}

        if not SERIAL_AVAILABLE or self._serial is None:
            logger.debug(f"[SIM] → Arduino: {message}")
            return {"success": True, "message": f"[sim] {cmd}"}

        if not self._connected:
            return {"success": False, "message": "Not connected to Arduino"}

        try:
            data = json.dumps(message) + "\n"
            self._serial.write(data.encode())
            self._serial.flush()

            # Read response
            response_line = self._serial.readline().decode().strip()
            if response_line:
                response = json.loads(response_line)
                self._update_state(response)
                return {"success": True, "data": response}

            return {"success": True, "message": "Command sent (no response)"}

        except (serial.SerialException, json.JSONDecodeError) as e:
            logger.error(f"Arduino communication error: {e}")
            return {"success": False, "message": str(e)}

    def _update_state(self, data: dict) -> None:
        """Update motor state from Arduino response."""
        if "enc_l" in data:
            self._state.left_encoder = data["enc_l"]
        if "enc_r" in data:
            self._state.right_encoder = data["enc_r"]
        if "battery" in data:
            self._state.battery_voltage = data["battery"]
        if "estop" in data:
            self._state.is_emergency_stopped = data["estop"]
        self._state.last_update = time.time()

    async def _heartbeat_loop(self) -> None:
        """Send periodic heartbeat to Arduino to prevent watchdog timeout."""
        while self._connected:
            try:
                await self._send_command("heartbeat")
                await asyncio.sleep(self.heartbeat_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
                await asyncio.sleep(1.0)

    def get_encoder_delta(self) -> tuple[int, int]:
        """Get encoder counts (useful for odometry calculation)."""
        return self._state.left_encoder, self._state.right_encoder

    def get_battery_voltage(self) -> float:
        """Get battery voltage from Arduino ADC."""
        return self._state.battery_voltage

"""IMU Driver — reads orientation and acceleration from MPU6050 via I2C."""

from __future__ import annotations

import asyncio
import math
import struct
import time
from dataclasses import dataclass
from typing import Optional

from loguru import logger

try:
    import smbus2
    I2C_AVAILABLE = True
except ImportError:
    I2C_AVAILABLE = False
    logger.warning("smbus2 not available — IMU driver in simulation mode")


# MPU6050 Register addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41

# Scale factors
ACCEL_SCALE = 16384.0  # ±2g mode
GYRO_SCALE = 131.0     # ±250°/s mode


@dataclass
class IMUReading:
    """Single IMU measurement."""
    # Accelerometer (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    # Gyroscope (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0

    # Temperature (°C)
    temperature: float = 0.0

    # Computed orientation (from complementary filter)
    roll: float = 0.0    # radians
    pitch: float = 0.0   # radians
    yaw: float = 0.0     # radians

    timestamp: float = 0.0

    def to_dict(self) -> dict:
        return {
            "accel": {"x": round(self.accel_x, 4), "y": round(self.accel_y, 4), "z": round(self.accel_z, 4)},
            "gyro": {"x": round(self.gyro_x, 4), "y": round(self.gyro_y, 4), "z": round(self.gyro_z, 4)},
            "orientation": {
                "roll_deg": round(math.degrees(self.roll), 2),
                "pitch_deg": round(math.degrees(self.pitch), 2),
                "yaw_deg": round(math.degrees(self.yaw), 2),
            },
            "temperature": round(self.temperature, 1),
        }


class IMUDriver:
    """
    MPU6050 IMU driver over I2C.

    Provides:
    - Raw accelerometer and gyroscope readings
    - Complementary filter for orientation estimation
    - Temperature reading
    """

    def __init__(self, bus_number: int = 1, address: int = MPU6050_ADDR):
        self.bus_number = bus_number
        self.address = address
        self._bus: Optional[smbus2.SMBus] = None
        self._connected = False
        self._reading = IMUReading()
        self._running = False

        # Complementary filter parameters
        self._alpha = 0.98  # Gyro weight (0-1)
        self._last_time = 0.0

        # Calibration offsets
        self._gyro_offset_x = 0.0
        self._gyro_offset_y = 0.0
        self._gyro_offset_z = 0.0

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def reading(self) -> IMUReading:
        return self._reading

    async def connect(self) -> bool:
        """Initialize the MPU6050 sensor."""
        if not I2C_AVAILABLE:
            logger.info("[SIM] IMU connected (simulation)")
            self._connected = True
            return True

        try:
            self._bus = smbus2.SMBus(self.bus_number)
            # Wake up MPU6050 (it starts in sleep mode)
            self._bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
            await asyncio.sleep(0.1)

            # Verify connection by reading WHO_AM_I register
            who_am_i = self._bus.read_byte_data(self.address, 0x75)
            if who_am_i != 0x68:
                logger.warning(f"Unexpected WHO_AM_I: 0x{who_am_i:02X} (expected 0x68)")

            self._connected = True
            logger.info("IMU (MPU6050) connected and initialized")

            # Calibrate gyroscope
            await self.calibrate()

            return True

        except Exception as e:
            logger.error(f"Failed to connect IMU: {e}")
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Close I2C connection."""
        self._running = False
        if self._bus:
            self._bus.close()
        self._connected = False
        logger.info("IMU disconnected")

    async def calibrate(self, samples: int = 100) -> None:
        """Calibrate gyroscope by averaging readings while stationary."""
        if not self._connected or not I2C_AVAILABLE:
            logger.info("[SIM] IMU calibrated")
            return

        logger.info(f"Calibrating IMU ({samples} samples)... keep robot still!")

        gx_sum, gy_sum, gz_sum = 0.0, 0.0, 0.0

        for _ in range(samples):
            raw = self._read_raw()
            gx_sum += raw["gyro_x"]
            gy_sum += raw["gyro_y"]
            gz_sum += raw["gyro_z"]
            await asyncio.sleep(0.01)

        self._gyro_offset_x = gx_sum / samples
        self._gyro_offset_y = gy_sum / samples
        self._gyro_offset_z = gz_sum / samples

        logger.info(
            f"IMU calibrated — gyro offsets: "
            f"({self._gyro_offset_x:.4f}, {self._gyro_offset_y:.4f}, {self._gyro_offset_z:.4f})"
        )

    async def start_reading(self, rate_hz: int = 100) -> None:
        """Start continuous IMU reading in background."""
        self._running = True
        self._last_time = time.time()
        interval = 1.0 / rate_hz

        logger.info(f"IMU reading started at {rate_hz} Hz")

        while self._running:
            try:
                self._update_reading()
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"IMU read error: {e}")
                await asyncio.sleep(0.1)

    def stop_reading(self) -> None:
        """Stop continuous reading."""
        self._running = False

    def _update_reading(self) -> None:
        """Read sensor and update orientation estimate."""
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        if dt <= 0 or dt > 1.0:
            return

        if not I2C_AVAILABLE or self._bus is None:
            # Simulation: return near-zero readings
            self._reading.timestamp = now
            return

        raw = self._read_raw()

        # Apply calibration
        gyro_x = (raw["gyro_x"] - self._gyro_offset_x) / GYRO_SCALE
        gyro_y = (raw["gyro_y"] - self._gyro_offset_y) / GYRO_SCALE
        gyro_z = (raw["gyro_z"] - self._gyro_offset_z) / GYRO_SCALE

        # Convert to rad/s
        gyro_x_rad = math.radians(gyro_x)
        gyro_y_rad = math.radians(gyro_y)
        gyro_z_rad = math.radians(gyro_z)

        accel_x = raw["accel_x"] / ACCEL_SCALE * 9.81
        accel_y = raw["accel_y"] / ACCEL_SCALE * 9.81
        accel_z = raw["accel_z"] / ACCEL_SCALE * 9.81

        # Accelerometer-based angles
        accel_roll = math.atan2(accel_y, accel_z)
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y ** 2 + accel_z ** 2))

        # Complementary filter
        self._reading.roll = self._alpha * (self._reading.roll + gyro_x_rad * dt) + (1 - self._alpha) * accel_roll
        self._reading.pitch = self._alpha * (self._reading.pitch + gyro_y_rad * dt) + (1 - self._alpha) * accel_pitch
        self._reading.yaw += gyro_z_rad * dt  # Yaw from gyro only (no magnetometer)

        # Store raw values
        self._reading.accel_x = accel_x
        self._reading.accel_y = accel_y
        self._reading.accel_z = accel_z
        self._reading.gyro_x = gyro_x_rad
        self._reading.gyro_y = gyro_y_rad
        self._reading.gyro_z = gyro_z_rad

        # Temperature
        temp_raw = raw.get("temp", 0)
        self._reading.temperature = temp_raw / 340.0 + 36.53

        self._reading.timestamp = now

    def _read_raw(self) -> dict:
        """Read raw sensor values from MPU6050 registers."""
        try:
            # Read 14 bytes starting from ACCEL_XOUT_H
            data = self._bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)

            return {
                "accel_x": self._bytes_to_int16(data[0], data[1]),
                "accel_y": self._bytes_to_int16(data[2], data[3]),
                "accel_z": self._bytes_to_int16(data[4], data[5]),
                "temp": self._bytes_to_int16(data[6], data[7]),
                "gyro_x": self._bytes_to_int16(data[8], data[9]),
                "gyro_y": self._bytes_to_int16(data[10], data[11]),
                "gyro_z": self._bytes_to_int16(data[12], data[13]),
            }
        except Exception as e:
            logger.error(f"I2C read error: {e}")
            return {"accel_x": 0, "accel_y": 0, "accel_z": 0, "temp": 0, "gyro_x": 0, "gyro_y": 0, "gyro_z": 0}

    @staticmethod
    def _bytes_to_int16(high: int, low: int) -> int:
        """Convert two bytes to signed 16-bit integer."""
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value

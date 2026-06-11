# 🤖 Robot Navigator

**LLM-powered autonomous room navigation robot** using Raspberry Pi 5, RPLidar, and Anthropic Claude.

The robot uses LiDAR SLAM to map indoor spaces and an LLM agent (Claude) to make intelligent navigation decisions based on natural language commands.

## Architecture

```
┌─────────────────────────────────────────────────┐
│          Anthropic Claude API (Cloud)            │
│          Tool-calling for robot commands          │
└────────────────────┬────────────────────────────┘
                     │ HTTPS (Wi-Fi)
┌────────────────────▼────────────────────────────┐
│          Raspberry Pi 5 (Python)                 │
│  ┌───────────────────────────────────────────┐  │
│  │  LLM Agent (anthropic SDK)                │  │
│  │  Web Dashboard (FastAPI + WebSocket)       │  │
│  └───────────────────┬───────────────────────┘  │
│  ┌───────────────────▼───────────────────────┐  │
│  │  ROS 2: SLAM Toolbox + Nav2 + AMCL        │  │
│  └───────────────────┬───────────────────────┘  │
│  ┌───────────────────▼───────────────────────┐  │
│  │  Drivers: LiDAR + IMU + Serial            │  │
│  └───────────────────────────────────────────┘  │
└────────────────────┬────────────────────────────┘
                     │ Serial/USB
┌────────────────────▼────────────────────────────┐
│  Arduino: Motor PWM + Watchdog + Encoders        │
└─────────────────────────────────────────────────┘
```

## Hardware Requirements

| Component | Model |
|---|---|
| SBC | Raspberry Pi 5 (8GB) |
| LiDAR | RPLidar A1 or A2 |
| Motor Driver | L298N dual H-bridge |
| Motors | DC gear motors with encoders |
| IMU | MPU6050 |
| Microcontroller | Arduino Nano (safety watchdog) |
| Chassis | 2WD robot chassis |
| Battery | 11.1V 3S LiPo |

## Quick Start

### 1. Clone & Install

```bash
git clone <repo-url>
cd robot

# Create virtual environment and install dependencies
make install

# Create environment file
make setup-env
# Edit .env with your Anthropic API key
```

### 2. Upload Arduino Firmware

```bash
# Install arduino-cli first: https://arduino.github.io/arduino-cli/
make upload-arduino
```

### 3. Install ROS 2 (on Raspberry Pi)

```bash
# Install ROS 2 Humble on Ubuntu 22.04
sudo apt install ros-humble-desktop
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
    ros-humble-robot-localization ros-humble-rplidar-ros

source /opt/ros/humble/setup.bash
```

### 4. Run the Robot

```bash
# Full system (on Raspberry Pi with hardware connected)
make run

# Simulation mode (no hardware needed, for development)
make run-sim

# Dashboard only
make dashboard
```

### 5. Open Dashboard

Navigate to `http://<raspberry-pi-ip>:8080` in your browser.

## Project Structure

```
robot/
├── src/
│   ├── agent/              # LLM agent (Anthropic Claude)
│   │   ├── agent.py        # Main agentic loop
│   │   ├── tools.py        # Tool definitions
│   │   ├── prompts.py      # System prompts
│   │   └── fallback.py     # Offline fallback logic
│   ├── navigation/          # ROS 2 navigation interface
│   │   ├── nav_commander.py # Nav2 goal sender
│   │   ├── map_manager.py  # Occupancy grid maps
│   │   ├── slam_manager.py # SLAM Toolbox control
│   │   └── robot_state.py  # Robot state aggregator
│   ├── drivers/             # Hardware interfaces
│   │   ├── motor_controller.py  # Arduino serial
│   │   ├── lidar.py        # RPLidar driver
│   │   ├── imu.py          # MPU6050 I2C
│   │   └── safety.py       # Safety monitor
│   ├── ros_nodes/           # Custom ROS 2 nodes
│   │   └── agent_bridge_node.py
│   ├── dashboard/           # Web UI
│   │   ├── server.py       # FastAPI server
│   │   └── static/         # HTML/JS/CSS
│   ├── config/              # Configuration files
│   └── main.py             # Entry point
├── arduino/                 # Arduino firmware
├── launch/                  # ROS 2 launch files
├── urdf/                    # Robot description
├── maps/                    # Saved maps
├── requirements.txt
├── Makefile
└── README.md
```

## Usage Examples

### Chat Commands (via Dashboard)

- "Go to the kitchen"
- "Explore the area to the left"
- "What room am I in?"
- "Map the entire house"
- "Return to home base"
- "Stop"

### Makefile Commands

```bash
make help          # Show all commands
make run           # Start full system
make run-sim       # Simulation mode
make dashboard     # Web dashboard only
make slam          # Start SLAM mapping
make save-map      # Save current map
make upload-arduino # Flash Arduino firmware
make logs          # Tail log file
```

## Configuration

Edit `src/config/robot_config.yaml` for:
- Hardware pin assignments
- Navigation parameters
- LLM model and behavior
- Safety thresholds
- Room definitions

Edit `.env` for API keys and secrets.

## Safety Features

- **Arduino watchdog**: Motors stop if Pi stops sending heartbeats
- **Emergency stop button**: Physical button on the robot
- **Obstacle detection**: LiDAR-based emergency stop at close range
- **Battery monitoring**: Auto return-home on low battery
- **Tilt detection**: IMU-based tip-over protection
- **Offline fallback**: Continues with waypoints if API is unreachable

## License

MIT

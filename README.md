# 🤖 Robot Navigator

**LLM-powered autonomous room navigation robot** using Raspberry Pi 5, RPLidar, and Anthropic Claude.

The robot uses LiDAR SLAM to map indoor spaces and an LLM agent (Claude) to make intelligent navigation decisions based on natural language commands like *"go to the kitchen"* or *"explore the house"*.

## Architecture

```
┌──────────────────────────────────────────────────┐
│          Anthropic Claude API (Cloud)             │
│          Tool-calling for robot commands           │
└─────────────────────┬────────────────────────────┘
                      │ HTTPS (Wi-Fi)
┌─────────────────────▼────────────────────────────┐
│          Raspberry Pi 5 (Python)                  │
│  ┌────────────────────────────────────────────┐  │
│  │  LLM Agent (anthropic SDK)                 │  │
│  │  Web Dashboard (FastAPI + WebSocket)        │  │
│  └─────────────────┬──────────────────────────┘  │
│  ┌─────────────────▼──────────────────────────┐  │
│  │  ROS 2 Jazzy: Nav2 + SLAM Toolbox + AMCL   │  │
│  └─────────────────┬──────────────────────────┘  │
│  ┌─────────────────▼──────────────────────────┐  │
│  │  Drivers: RPLidar + MPU6050 IMU + Serial    │  │
│  └────────────────────────────────────────────┘  │
└─────────────────────┬────────────────────────────┘
                      │ Serial/USB
┌─────────────────────▼────────────────────────────┐
│  Arduino Nano: Motor PWM + Watchdog + Encoders    │
└──────────────────────────────────────────────────┘
```

---

## Quick Start

### macOS (Lightweight Simulation)

No hardware or ROS 2 needed. Runs a virtual 2D apartment with A* path planning, LiDAR raycasting, and the full LLM agent.

```bash
git clone https://github.com/Benzveen/robot-navigator.git
cd robot-navigator

# Setup
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Configure API key
cp .env.example .env
# Edit .env → set ANTHROPIC_API_KEY=sk-ant-...

# Run simulation
make run-sim

# Open dashboard → http://localhost:8080

# Run tests (no API key needed)
make test
```

### Ubuntu (Full ROS 2 + Gazebo Simulation)

Runs the real Nav2 stack with Gazebo 3D simulator. Use an Ubuntu 24.04 machine or a [UTM](https://mac.getutm.app/) VM on Mac.

```bash
# One-command setup (~20 min, installs ROS 2 + Nav2 + Gazebo + project)
curl -sSL https://raw.githubusercontent.com/Benzveen/robot-navigator/main/scripts/setup_ubuntu_vm.sh | bash

# Set your API key
cd ~/Development/robot
nano .env
# → Set ANTHROPIC_API_KEY=sk-ant-...

# Run the app (LLM agent + dashboard)
PYTHONPATH=src python3 -m simulator.sim_runner
# → Dashboard: http://localhost:8080

# Or launch full Nav2 + Gazebo (separate terminal)
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

### Raspberry Pi (Real Robot)

Deploy to physical hardware with real LiDAR, motors, and navigation.

```bash
git clone https://github.com/Benzveen/robot-navigator.git ~/robot
cd ~/robot

# Install ROS 2 + Nav2 + dependencies
chmod +x scripts/setup_ubuntu_vm.sh
./scripts/setup_ubuntu_vm.sh

# Flash Arduino motor controller
make upload-arduino

# Configure
cp .env.example .env
nano .env  # Set ANTHROPIC_API_KEY

# Run
make run
# → Dashboard: http://<pi-ip>:8080
```

---

## Hardware Requirements (for real robot)

| Component | Model |
|---|---|
| SBC | Raspberry Pi 5 (8GB) |
| LiDAR | RPLidar A1 or A2 |
| Motor Driver | L298N dual H-bridge |
| Motors | DC gear motors with encoders |
| IMU | MPU6050 |
| Microcontroller | Arduino Nano (safety watchdog) |
| Chassis | 2WD robot chassis |
| Battery | 11.1V 3S LiPo + 5V buck converter |

---

## Dashboard

The web dashboard at `http://localhost:8080` provides:

- **Live Map** — real-time apartment view with robot position, heading arrow, movement trail, and A* planned path
- **Chat** — natural language commands to the LLM agent
- **Controls** — Explore, Go Home, Emergency Stop buttons
- **Robot State** — position, speed, battery, nearest obstacle, current room
- **Sensor Status** — LiDAR, IMU, Arduino connection indicators

### Chat Commands

| Command | What happens |
|---|---|
| "Go to the kitchen" | Plans A* path through doors, navigates there |
| "Explore the area" | Moves toward unmapped regions |
| "What room am I in?" | Reports current room |
| "Go home" | Returns to starting position |
| "Stop" | Emergency stop |

---

## Project Structure

```
robot-navigator/
├── src/
│   ├── agent/                  # LLM Intelligence Layer
│   │   ├── agent.py            # Agentic loop (sense → think → act)
│   │   ├── tools.py            # 8 tool definitions for Claude
│   │   ├── prompts.py          # System prompts with robot context
│   │   └── fallback.py         # Offline fallback (waypoint/stop/home)
│   ├── simulator/              # Simulation (Mac — no hardware needed)
│   │   ├── sim_runner.py       # Simulation entry point
│   │   ├── virtual_world.py    # 2D apartment with walls, rooms, physics
│   │   ├── pathfinder.py       # A* path planning on occupancy grid
│   │   ├── motion_controller.py # Waypoint following with steering
│   │   └── ros2_bridge.py      # Bridge to real Nav2 via rosbridge
│   ├── navigation/             # ROS 2 Nav2 Interface
│   │   ├── nav_commander.py    # Nav2 NavigateToPose action client
│   │   ├── map_manager.py      # Occupancy grid map I/O
│   │   ├── slam_manager.py     # SLAM Toolbox service calls
│   │   └── robot_state.py      # Robot state aggregator
│   ├── drivers/                # Hardware Interfaces
│   │   ├── motor_controller.py # Arduino serial (JSON protocol)
│   │   ├── lidar.py            # RPLidar scan driver
│   │   ├── imu.py              # MPU6050 I2C + complementary filter
│   │   └── safety.py           # Safety monitor (e-stop, watchdog)
│   ├── ros_nodes/              # Custom ROS 2 Nodes
│   │   └── agent_bridge_node.py
│   ├── dashboard/              # Web Dashboard
│   │   ├── server.py           # FastAPI + WebSocket server
│   │   └── static/index.html   # Live map, chat, controls
│   ├── config/                 # Configuration
│   │   ├── robot_config.yaml   # Hardware + nav + agent settings
│   │   ├── nav2_params.yaml    # Nav2 stack parameters
│   │   └── slam_params.yaml    # SLAM Toolbox parameters
│   └── main.py                 # Production entry point (Raspberry Pi)
├── arduino/                    # Arduino motor controller firmware
├── launch/                     # ROS 2 launch files
├── urdf/                       # Robot URDF description
├── scripts/
│   └── setup_ubuntu_vm.sh      # One-command Ubuntu/Pi setup
├── tests/                      # 63 tests (unit + integration)
├── requirements.txt
├── Makefile
└── .env.example
```

---

## Makefile Commands

| Command | Description | Platform |
|---|---|---|
| `make run-sim` | Lightweight simulation + LLM agent + dashboard | macOS / Ubuntu |
| `make run` | Full system with real hardware | Raspberry Pi |
| `make run-nav2` | Connect LLM agent to Nav2 via rosbridge | macOS → Ubuntu/Pi |
| `make test` | Run 63 tests | All |
| `make dashboard` | Dashboard server only | All |
| `make upload-arduino` | Flash Arduino firmware | Pi |
| `make setup-env` | Create .env from template | All |
| `make setup-vm` | Show Ubuntu VM setup instructions | macOS |
| `make help` | Show all commands | All |

---

## Configuration

**`.env`** — API keys (not committed to git):
```
ANTHROPIC_API_KEY=sk-ant-...
ANTHROPIC_MODEL=claude-sonnet-4-20250514
DASHBOARD_PORT=8080
```

**`src/config/robot_config.yaml`** — Robot settings:
- Hardware pin assignments and serial ports
- Navigation parameters (speed, tolerances, safety distances)
- LLM agent behavior (model, temperature, think interval)
- Room definitions (names and coordinates)

---

## Safety Features

| Feature | How it works |
|---|---|
| **Arduino watchdog** | Motors stop if Pi stops sending heartbeats |
| **Emergency stop** | Physical button + software stop via dashboard |
| **Obstacle detection** | LiDAR-based emergency stop at close range |
| **Battery monitoring** | Auto return-home on low battery |
| **Tilt detection** | IMU-based tip-over protection |
| **Offline fallback** | Continues with waypoints if API is unreachable |

---

## Tech Stack

| Layer | Technology |
|---|---|
| LLM | Anthropic Claude SDK (tool-calling) |
| Web | FastAPI + WebSocket + Jinja2 |
| Navigation | ROS 2 Jazzy + Nav2 (production) / A* (simulation) |
| SLAM | SLAM Toolbox |
| Simulation | Custom 2D world with raycasting LiDAR |
| Hardware | RPLidar, MPU6050, Arduino Nano, L298N |
| Language | Python 3.11+ |

## License

MIT

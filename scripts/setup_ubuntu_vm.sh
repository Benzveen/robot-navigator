#!/bin/bash
# ============================================
# ROS 2 Jazzy + Nav2 + Gazebo Setup Script
# Run this inside the Ubuntu 24.04 VM (UTM)
# ============================================
#
# Usage:
#   chmod +x setup_ubuntu_vm.sh
#   ./setup_ubuntu_vm.sh
#
# This installs everything needed for full Nav2 simulation:
#   - ROS 2 Jazzy
#   - Nav2 (path planning, obstacle avoidance, recovery)
#   - SLAM Toolbox (mapping)
#   - Gazebo (3D robot simulator)
#   - Turtlebot3 simulation packages
#   - rosbridge (WebSocket bridge for LLM agent)
#   - Python packages for the LLM agent

set -e

echo "============================================"
echo "🤖 Robot Navigator — Ubuntu VM Setup"
echo "============================================"
echo ""

# ── 1. System Update ───────────────────────────────────────
echo "📦 Step 1/6: Updating system..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg lsb-release software-properties-common git python3-pip

# ── 2. Install ROS 2 Jazzy ────────────────────────────────
echo ""
echo "🔧 Step 2/6: Installing ROS 2 Jazzy..."
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop

# ── 3. Install Nav2 + SLAM + Gazebo ───────────────────────
echo ""
echo "🧭 Step 3/6: Installing Nav2, SLAM Toolbox, Gazebo..."
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz \
  ros-jazzy-nav2-minimal-tb3-sim \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools

# ── 4. Setup ROS 2 Environment ────────────────────────────
echo ""
echo "⚙️ Step 4/6: Configuring ROS 2 environment..."
echo "" >> ~/.bashrc
echo "# ROS 2 Jazzy" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

# ── 5. Install Python Packages ────────────────────────────
echo ""
echo "🐍 Step 5/6: Installing Python packages for LLM agent..."
pip3 install --break-system-packages \
  anthropic \
  fastapi \
  "uvicorn[standard]" \
  websockets \
  pydantic \
  python-dotenv \
  loguru \
  pyyaml \
  roslibpy \
  numpy \
  scipy

# ── 6. Clone Robot Project ────────────────────────────────
echo ""
echo "📂 Step 6/6: Setting up robot project..."
mkdir -p ~/Development
if [ ! -d ~/Development/robot ]; then
  echo "Clone your robot repo here:"
  echo "  cd ~/Development && git clone <your-repo-url> robot"
  echo ""
  echo "Or copy from Mac shared folder if available."
else
  echo "Robot project already exists at ~/Development/robot"
fi

# ── Done! ─────────────────────────────────────────────────
echo ""
echo "============================================"
echo "✅ Setup Complete!"
echo "============================================"
echo ""
echo "Test ROS 2:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 --version"
echo ""
echo "Launch Nav2 + Gazebo simulation:"
echo "  ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False"
echo ""
echo "Launch SLAM mode (for mapping new environments):"
echo "  ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False slam:=True"
echo ""
echo "Start rosbridge (for LLM agent connection from Mac):"
echo "  ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090"
echo ""
echo "Find VM IP (for Mac connection):"
echo "  ip addr show | grep 'inet '"
echo ""
echo "============================================"

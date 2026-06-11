# ============================================
# ROS 2 Jazzy + Nav2 + Gazebo Simulation
# ============================================
# This runs the REAL Nav2 stack in Docker on your Mac.
# Same stack that will run on the Raspberry Pi.
#
# Build:  docker build -t robot-nav2-sim docker/
# Run:    docker compose -f docker/docker-compose.yml up

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install Nav2, SLAM, Gazebo, and all navigation packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Nav2 full stack
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-nav2-minimal-tb3-sim \
    ros-${ROS_DISTRO}-nav2-minimal-tb4-sim \
    # SLAM
    ros-${ROS_DISTRO}-slam-toolbox \
    # Localization & sensor fusion
    ros-${ROS_DISTRO}-robot-localization \
    # Gazebo simulator
    ros-${ROS_DISTRO}-ros-gz \
    # Robot description
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    # ROS bridge (for connecting Python agent outside Docker)
    ros-${ROS_DISTRO}-rosbridge-suite \
    # Visualization
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    # TF2
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    # Utilities
    ros-${ROS_DISTRO}-rqt* \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for the LLM agent bridge
# Note: --ignore-installed to avoid conflicts with system packages (e.g. cryptography)
RUN pip3 install --break-system-packages --ignore-installed \
    anthropic \
    fastapi \
    "uvicorn[standard]" \
    websockets \
    pydantic \
    python-dotenv \
    loguru \
    pyyaml \
    roslibpy

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Copy our robot package
COPY ../src/ /ros2_ws/src/robot_navigator/src/
COPY ../launch/ /ros2_ws/src/robot_navigator/launch/
COPY ../urdf/ /ros2_ws/src/robot_navigator/urdf/
COPY ../package.xml /ros2_ws/src/robot_navigator/
COPY ../setup.py /ros2_ws/src/robot_navigator/

# Source ROS 2 in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# Expose ports
# 9090: rosbridge WebSocket (for LLM agent connection)
# 8080: Dashboard
# 11345: Gazebo
EXPOSE 9090 8080 11345

# Default: launch the full simulation
CMD ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True"]

# ros2-llm-safety-verifier – reproducible container
# Works on x86_64 (laptop) AND arm64 (Jetson Orin Nano/NX)
# Build with:
#   docker buildx build --platform linux/amd64,linux/arm64 -t munawarkazmi/ros2-llm-safety-verifier .

FROM ros:humble-ros-base AS base

# Prevent interactive prompts + timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Core system + ROS build tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    curl \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# rosdep (ignore if already initialized)
RUN rosdep init || true && rosdep update

# Create workspace
WORKDIR /ros2_ws

# 1. Copy only Python requirements first → maximum cache efficiency
COPY src/ros2-llm-safety-verifier/requirements.txt* /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || echo "No requirements.txt yet"

# 2. Copy source code
COPY src/ros2-llm-safety-verifier /ros2_ws/src/ros2-llm-safety-verifier

# 3. Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS 2 + workspace in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

WORKDIR /ros2_ws
CMD ["bash"]

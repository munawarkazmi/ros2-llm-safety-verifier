# ros2-llm-safety-verifier - development environment
# ROS 2 Humble + Nav2 on x86_64 (laptop) and arm64 (Jetson Orin Nano/NX).
#
# This image is the environment only. docker-compose.yml mounts the
# repository into /ros2_ws/src, so the verifier package is colcon-built
# inside the running container as it is developed:
#   docker compose up --build
#   # inside the container:
#   colcon build --symlink-install

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

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

RUN rosdep init || true && rosdep update

WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

CMD ["bash"]

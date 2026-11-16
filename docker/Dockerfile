FROM ros:jazzy-ros-base AS dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-robot-state-publisher ros-jazzy-tf2-geometry-msgs \
    python3-numpy python3-yaml python3-pytest python3-pil python3-pip python3-venv \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
FROM dependencies AS runtime
WORKDIR /ws
COPY . /ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
ENV ROS_DOMAIN_ID=77
CMD ["bash"]

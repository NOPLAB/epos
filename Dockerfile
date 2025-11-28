# syntax=docker/dockerfile:1
# ROS Humble base image (Ubuntu 22.04)
FROM ros:humble-ros-base

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies and ros2_control packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    udev \
    usbutils \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    libopencv-dev

# Create workspace directory
WORKDIR /opt

# Copy EPOS Linux Library for installation
COPY EPOS_Linux_Library /opt/EPOS_Linux_Library

# Install libEposCmd library (x86_64)
RUN mkdir -p /opt/EposCmdLib_6.8.1.0/lib && \
    cp -rf /opt/EPOS_Linux_Library/include /opt/EposCmdLib_6.8.1.0/ && \
    cp -rf /opt/EPOS_Linux_Library/lib/intel/x86_64 /opt/EposCmdLib_6.8.1.0/lib/ && \
    cp -rf /opt/EPOS_Linux_Library/misc /opt/EposCmdLib_6.8.1.0/ && \
    ln -sf /opt/EposCmdLib_6.8.1.0/lib/x86_64/libEposCmd.so.6.8.1.0 /usr/lib/libEposCmd.so && \
    ln -sf /opt/EposCmdLib_6.8.1.0/lib/x86_64/libftd2xx.so.1.4.8 /usr/lib/libftd2xx.so && \
    ldconfig && \
    rm -rf /opt/EPOS_Linux_Library

# Create ROS workspace
WORKDIR /ros2_ws/src

# Copy project sources
COPY epos /ros2_ws/src/epos
COPY shooter /ros2_ws/src/shooter

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]

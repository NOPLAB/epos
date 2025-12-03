# syntax=docker/dockerfile:1
# ROS Humble base image (Ubuntu 22.04)
FROM ros:humble-ros-base

# Target architecture for multi-platform builds
ARG TARGETARCH

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies and ros2_control packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    udev \
    usbutils \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-cv-bridge \
    libopencv-dev

# Create workspace directory
WORKDIR /opt

# Copy EPOS Linux Library for installation
COPY EPOS_Linux_Library /opt/EPOS_Linux_Library

# Install libEposCmd library (multi-architecture)
# TARGETARCH: amd64 -> intel/x86_64, arm64 -> arm/v8, arm -> arm/v7
RUN mkdir -p /opt/EposCmdLib_6.8.1.0/lib && \
    cp -rf /opt/EPOS_Linux_Library/include /opt/EposCmdLib_6.8.1.0/ && \
    cp -rf /opt/EPOS_Linux_Library/misc /opt/EposCmdLib_6.8.1.0/ && \
    case "${TARGETARCH}" in \
        amd64) LIB_PATH="intel/x86_64" ;; \
        arm64) LIB_PATH="arm/v8" ;; \
        arm)   LIB_PATH="arm/v7" ;; \
        *)     echo "Unsupported architecture: ${TARGETARCH}" && exit 1 ;; \
    esac && \
    ARCH_DIR=$(basename "${LIB_PATH}") && \
    cp -rf /opt/EPOS_Linux_Library/lib/${LIB_PATH} /opt/EposCmdLib_6.8.1.0/lib/ && \
    ln -sf /opt/EposCmdLib_6.8.1.0/lib/${ARCH_DIR}/libEposCmd.so.6.8.1.0 /usr/lib/libEposCmd.so && \
    ln -sf /opt/EposCmdLib_6.8.1.0/lib/${ARCH_DIR}/libftd2xx.so.1.4.8 /usr/lib/libftd2xx.so && \
    ldconfig && \
    rm -rf /opt/EPOS_Linux_Library

# Create ROS workspace
WORKDIR /ros2_ws/src

# Copy project sources
COPY epos /ros2_ws/src/epos
COPY shooter_description /ros2_ws/src/shooter_description
COPY shooter_bringup /ros2_ws/src/shooter_bringup
COPY shooter_control /ros2_ws/src/shooter_control

# Download YOLO models
RUN /ros2_ws/src/shooter_control/scripts/download_models.sh

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Setup entrypoint
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]

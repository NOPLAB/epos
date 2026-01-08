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
    curl \
    udev \
    usbutils \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    python3-pip && \
    pip3 install --no-cache-dir rpi-hardware-pwm

# Install OpenCV build dependencies and cv_bridge dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libatlas-base-dev \
    gfortran \
    python3-numpy \
    libboost-python-dev

# Build OpenCV with contrib modules (for TrackerNano)
ARG OPENCV_VERSION=4.8.0
WORKDIR /tmp
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
    mkdir -p opencv/build && cd opencv/build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_opencv_python3=OFF \
          -D WITH_GTK=ON \
          -D WITH_V4L=ON \
          -D WITH_FFMPEG=ON \
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/opencv /tmp/opencv_contrib

# Build cv_bridge from source to use our OpenCV
WORKDIR /opt/cv_bridge_ws/src
RUN git clone --depth 1 --branch humble https://github.com/ros-perception/vision_opencv.git
WORKDIR /opt/cv_bridge_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select cv_bridge --cmake-args -DOpenCV_DIR=/usr/local/lib/cmake/opencv4

# Set environment for custom cv_bridge
ENV AMENT_PREFIX_PATH=/opt/cv_bridge_ws/install/cv_bridge:$AMENT_PREFIX_PATH
ENV CMAKE_PREFIX_PATH=/opt/cv_bridge_ws/install/cv_bridge:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=/opt/cv_bridge_ws/install/cv_bridge/lib:/usr/local/lib:$LD_LIBRARY_PATH

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
COPY chikurin_description /ros2_ws/src/chikurin_description
COPY chikurin_control /ros2_ws/src/chikurin_control
COPY teleop_udp /ros2_ws/src/teleop_udp

# Download YOLO models
RUN /ros2_ws/src/chikurin_control/scripts/download_models.sh

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    . /opt/cv_bridge_ws/install/setup.sh && \
    colcon build --symlink-install --cmake-args -DOpenCV_DIR=/usr/local/lib/cmake/opencv4

# Setup entrypoint
COPY docker/docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]

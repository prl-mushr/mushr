#!/bin/bash
# Install workspace dependencies: extra ROS 2 packages, system libs, Python
# packages, range_libc, and basic dev tools.

set -e

# Source ROS 2 for tools that need it
source /opt/ros/jazzy/setup.bash

# Basic dev tools, networking, build tooling
apt-get install -y --no-install-recommends \
    git wget build-essential cmake python3-pip tmux libasio-dev \
    vim nano less tree htop \
    net-tools iputils-ping iproute2 dnsutils \
    openssh-client bash-completion sudo usbutils \
    ipython3 xterm x11-apps

# Extra ROS 2 packages used by the MuSHR stack
apt-get install -y --no-install-recommends \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf-transformations \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-fastrtps-cpp \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-camera-info-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joy \
    ros-jazzy-joy-teleop \
    ros-jazzy-xacro \
    ros-jazzy-plotjuggler \
    ros-jazzy-topic-tools \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-launch-pytest

# Python deps: numpy<2 to keep ABI compatible with apt-installed scipy etc.
pip install --no-cache-dir -U \
    "numpy<2" \
    transforms3d scipy matplotlib pandas networkx Cython sympy rosbags \
    tqdm requests \
    ros2-ndarray-msg-utils

# range_libc (custom C++ build, runtime dep of localization sensor_model)
git clone https://github.com/RTIS-Lab/range_libc.git /tmp/range_libc
cd /tmp/range_libc/pywrapper
python3 setup.py install
cd /
rm -rf /tmp/range_libc

# NanoSAM (hound_core sam_refine_node when sam_backend=nanosam).
# Python package: colcon_ws/src/nanosam-src (bind-mounted with workspace).
# TensorRT engines: colcon_ws/src/nanosam/data/*.engine (not installed here).
pip install --no-cache-dir git+https://github.com/NVIDIA-AI-IOT/torch2trt.git
NANOSAM_SRC=/root/colcon_ws/src/nanosam-src
if [ ! -d "${NANOSAM_SRC}/nanosam" ]; then
    git clone --depth 1 https://github.com/NVIDIA-AI-IOT/nanosam.git "${NANOSAM_SRC}"
fi
pip install --no-cache-dir -e "${NANOSAM_SRC}"


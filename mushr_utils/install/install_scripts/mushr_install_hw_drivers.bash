#!/bin/bash
# Install hardware-only drivers: librealsense, YDLidar SDK, Jetson.GPIO,
# ros-jazzy-librealsense2. Only runs when REAL=1.

set -e

# librealsense (Intel realsenseai apt repo)
apt-get install -y --no-install-recommends gnupg apt-transport-https pkg-config swig
mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
    | gpg --dearmor > /etc/apt/keyrings/librealsenseai.gpg
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/librealsense.list
apt-get update
apt-get install -y --no-install-recommends \
    librealsense2-dev librealsense2-utils \
    ros-jazzy-librealsense2

# Jetson GPIO (pip; apt package only in NVIDIA repos)
pip install --no-cache-dir Jetson.GPIO

# YDLidar SDK (system-wide install so find_package(ydlidar_sdk) works)
git clone https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/ydlidar-sdk
cd /tmp/ydlidar-sdk
mkdir build && cd build
cmake .. && make -j"$(nproc)" && make install
ldconfig
cd /
rm -rf /tmp/ydlidar-sdk

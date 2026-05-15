#!/bin/bash
# Install ROS 2 Humble base + rosdep.

set -e

# Setup software sources from packages.ros.org (already present in ros:humble
# base image, but kept here for completeness if base image is swapped).
apt-get update
apt-get install -y --no-install-recommends \
    curl gnupg lsb-release ca-certificates software-properties-common

# Install ROS 2 Humble desktop (rviz, demo nodes, common msgs, etc.)
apt-get install -y --no-install-recommends ros-humble-desktop

# Auto source ROS on terminal startup
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# DDS / domain configuration
echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
echo "export CYCLONEDDS_URI=file:///root/cyclone_dds.xml" >> /root/.bashrc

# Source it for the rest of this script
source /opt/ros/humble/setup.bash

# Custom rosdep keys: librealsense2 / jetson-gpio / ament_python /
# push_button_utils aren't in stock Ubuntu rosdep rules, so map them to
# no-op here. Lets `rosdep install` run clean for any workspace that lists
# these as deps.
mkdir -p /etc/ros/rosdep/sources.list.d
printf '%s\n' \
    'librealsense2:' \
    '  ubuntu: []' \
    'jetson-gpio:' \
    '  ubuntu: []' \
    'python3-jetson-gpio:' \
    '  ubuntu: []' \
    'ament_python:' \
    '  ubuntu: []' \
    'push_button_utils:' \
    '  ubuntu: []' \
    > /etc/ros/rosdep/mushr-extras.yaml
echo "yaml file:///etc/ros/rosdep/mushr-extras.yaml" \
    > /etc/ros/rosdep/sources.list.d/50-mushr.list

# Initialize / update rosdep (ros:humble already runs rosdep init; ignore failure)
rosdep init || true
rosdep update

#!/bin/bash
set -e

# ROS already installed in Isaac image (Jazzy)
echo "Sourcing ROS Jazzy..."

echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
source /opt/ros/jazzy/setup.bash

# DDS / domain configuration
echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
echo "export CYCLONEDDS_URI=file:///root/cyclone_dds.xml" >> /root/.bashrc

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

# Initialize / update rosdep (ros:jazzy already runs rosdep init; ignore failure)
rosdep init || true
rosdep update

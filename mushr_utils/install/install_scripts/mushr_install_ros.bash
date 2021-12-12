#!/bin/bash

# Setup software sources from packages.ros.org
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package index
apt-get update

# Install ROS libraries
apt-get install ros-noetic-desktop -y

# Auto source ROS on terminal startup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Source it for this terminal
source /opt/ros/noetic/setup.bash

# Install rosdep
apt-get install python3-rosdep -y

# Initialize rosdep
rosdep init

# Update rosdep
rosdep update

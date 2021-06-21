#!/bin/bash

# Setup software sources from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package index
sudo apt update

# Install ROS libraries
sudo apt install ros-melodic-desktop -y

# Auto source ROS on terminal startup
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Source it for this terminal
source /opt/ros/melodic/setup.bash

# Install rosdep
sudo apt install python-rosdep -y

# Initialize rosdep
sudo rosdep init

# Update rosdep
rosdep update

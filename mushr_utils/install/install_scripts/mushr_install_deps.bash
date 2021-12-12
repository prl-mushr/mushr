#!/bin/bash

# Install git, tkinter, wget, g++, vim, tmux, networking stuff
apt-get install -y git-all python3-tk wget g++ vim tmux net-tools iputils-ping

# Install vcstool, pip
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
apt-get update
apt-get install -y python3-vcstool python3-pip

# Install extra ROS packages
apt-get install -y ros-noetic-ackermann-msgs ros-noetic-map-server ros-noetic-urg-node ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-joy

# Install catkin tools
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get update
apt-get install -y python3-catkin-tools

# Install Cython
pip install Cython

# Create OpenCV symbolic link
ln -s /usr/include/opencv4 /usr/include/opencv

# Create workspace
source /opt/ros/noetic/setup.bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin build

# Auto source this workspace on terminal startup
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo "export ROS_IP=10.42.0.1" >> ~/.bashrc

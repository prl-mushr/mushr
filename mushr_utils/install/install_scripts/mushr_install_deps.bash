#!/bin/bash

# Install git, tkinter, wget, g++, vim, tmux, networking stuff
apt-get install -y git-all python3-tk wget g++ vim tmux net-tools iputils-ping

# Install vcstool, pip
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
apt-get update
apt-get install -y python3-vcstool python3-pip

# Install extra ROS packages
apt-get install -y ros-noetic-ackermann-msgs ros-noetic-map-server ros-noetic-urg-node ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-joy ros-noetic-ddynamic-reconfigure ros-noetic-fake-localization ros-noetic-gmapping

# Install catkin tools
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get update
apt-get install -y python3-catkin-tools

# Install Cython, PyTorch 1.10 at least!
pip3 install Cython torch torchvision torchaudio numpy scipy progress --upgrade

# Create OpenCV symbolic link
ln -s /usr/include/opencv4 /usr/include/opencv

# Auto source this workspace on terminal startup
echo "source ~/dependencies_ws/devel/setup.bash" >> ~/.bashrc

# Install rangelibc
cd ~/dependencies_ws/src/range_libc/pywrapper
python3 setup.py install
cd ~/dependencies_ws/src
rm -rf range_libc

# Create default RVIZ setup
mkdir ~/.rviz
cp ~/dependencies_ws/src/mushr/mushr_utils/rviz/default.rviz ~/.rviz/

# Set ROS_IP
export ROS_IP=$(ifconfig wlan0 | grep "inet " | awk '{print $2}')

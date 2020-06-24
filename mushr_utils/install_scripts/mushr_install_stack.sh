#!/bin/bash

# Install git
sudo apt install git-all

# Install tkinter
sudo apt install python-tk

# Install vcstool
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
sudo apt-get update
sudo apt-get install python3-vcstool

# Install pip
sudo apt install python-pip

# Install extra ROS packages
sudo apt install -y ros-melodic-ackermann-msgs ros-melodic-map-server ros-melodic-serial ros-melodic-urg-node ros-melodic-robot-state-publisher ros-melodic-xacro ros-melodic-joy

# Create workspace
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace

source /opt/ros/melodic/setup.bash
cd ~/catkin_ws
catkin_make

# Auto source this workspace on terminal startup
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo "export ROS_IP=10.42.0.1" >> ~/.bashrc

# Get repo info
sudo apt install wget
cd ~/catkin_ws/src
wget https://mushr.io/tutorials/quickstart/repos.yaml

# Clone the repos
vcs import < repos.yaml

# Install dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# Install Cython
sudo pip install Cython

# Install rangelibc
cd ~/catkin_ws/src/range_libc/pywrapper
sudo python setup.py install
cd ~/catkin_ws/src
sudo rm -rf range_libc

# Create OpenCV symbolic link
sudo ln -s /usr/include/opencv4 /usr/include/opencv

# Compile
cd ~/catkin_ws
catkin_make -j 2

# Create default RVIZ setup
mkdir ~/.rviz
cp ~/catkin_ws/src/mushr/mushr_utils/rviz/default.rviz ~/.rviz/

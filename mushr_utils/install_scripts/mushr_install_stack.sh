#!/bin/bash

# Install git
sudo apt install git-all -y

# Install tkinter
sudo apt install python-tk -y

# Install vcstool
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
sudo apt-get update
sudo apt-get install python3-vcstool -y

# Install pip
sudo apt install python-pip -y

# Install extra ROS packages
sudo apt install -y ros-melodic-ackermann-msgs ros-melodic-map-server ros-melodic-serial ros-melodic-urg-node ros-melodic-robot-state-publisher ros-melodic-xacro ros-melodic-joy -y

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
sudo apt install wget -y
cd ~/catkin_ws/src
wget https://raw.githubusercontent.com/prl-mushr/mushr/master/repos.yaml

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

# Install librealsense. Following commands copied from
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt-get install apt-utils -y
sudo apt-get install librealsense2-utils librealsense2-dev -y

# Compile
cd ~/catkin_ws
catkin_make -j 2

# Create default RVIZ setup
mkdir ~/.rviz
cp ~/catkin_ws/src/mushr/mushr_utils/rviz/default.rviz ~/.rviz/

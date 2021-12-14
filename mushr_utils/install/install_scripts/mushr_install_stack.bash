#!/bin/bash

# Install dependencies
#cd ~/catkin_ws
#export ROS_DISTRO="noetic"
#rosdep install --from-paths src --ignore-src -r -y

# Install rangelibc
cd ~/catkin_ws/src/range_libc/pywrapper
python3 setup.py install
cd ~/catkin_ws/src
rm -rf range_libc

# Create default RVIZ setup
mkdir ~/.rviz
cp ~/catkin_ws/src/mushr/mushr_utils/rviz/default.rviz ~/.rviz/

# Set ROS_IP
export ROS_IP=$(ifconfig wlan0 | grep "inet " | awk '{print $2}')

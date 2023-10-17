#!/bin/bash

# Install git, tkinter, wget, g++, vim, tmux, networking stuff, apt-add-repository
apt-get install -y git-all python3-tk wget g++ vim tmux net-tools iputils-ping software-properties-common

# Install vcstool, pip
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
apt-get update
apt-get install -y python3-vcstool python3-pip

# Install extra ROS packages
apt-get install -y ros-noetic-ackermann-msgs ros-noetic-map-server ros-noetic-urg-node ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-joy ros-noetic-ddynamic-reconfigure ros-noetic-fake-localization ros-noetic-gmapping ros-noetic-rosbridge-suite ros-noetic-sbpl

# Install catkin tools
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get update
apt-get install -y python3-catkin-tools


OS = "$1"
if [[ "$OS" == "x86_64" ]]; then
	## this is to make sure we get the correct opencv version
	pip install opencv-python==4.2.0.32
	pip uninstall -y opencv-python
	pip install opencv-python==4.2.0.32
else
	pip install cupy-cuda11x -f https://pip.cupy.dev/aarch64
fi
pip install empy
pip install catkin-tools ## because apparently python3-catkin-tools != catkin-tools?
apt-get install -y qtbase5-dev ros-noetic-pybind11-catkin

# Create OpenCV symbolic link
# ln -s /usr/include/opencv4 /usr/include/opencv

# Auto source this workspace on terminal startup
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install rangelibc
cd ~/catkin_ws/src/range_libc/pywrapper
python3 setup.py install
cd ~/catkin_ws/src
rm -rf range_libc

# Create default RVIZ setup
mkdir ~/.rviz
cp ~/catkin_ws/src/mushr/mushr_utils/rviz/default.rviz ~/.rviz/
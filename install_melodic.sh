#!/usr/bin/env sh
#For Ubuntu 18.04 with ROS Melodic

# If your workspace is not in ~/catkin_ws change this variable
ROS_WS="$HOME"/catkin_ws

#Install vctool
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y python3-vcstool

#Get Repos
vcs import < repos.yaml

#Joystick connection stuff
sudo apt-get install -y joystick
sudo apt-get install -y dkms
sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4
sudo dkms install -m xpad -v 0.4

sudo apt-get install -y software-properties-common
sudo apt-get install -y ros-melodic-joy
sudo apt-get install -y ros-melodic-map-server
sudo apt-get install -y ros-melodic-serial

# Packages needed for realsense
sudo apt-get install -y ros-melodic-cv-bridge
sudo apt-get install -y ros-melodic-ddynamic-reconfigure

sudo apt-get install -y ros-melodic-ackermann-msgs
sudo apt-get install -y ros-melodic-image-transport

#Needed for old car laser
sudo apt-get install -y ros-melodic-urg-node

#Install librealsense
git clone https://github.com/IntelRealSense/librealsense.git && cd librealsense
sudo apt-get install -y udev
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
#Ubuntu 18 specific
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make uninstall && make clean && make && sudo make install
cd "$ROS_WS"/src/mushr && rm -rf librealsense

#Install yaml-cpp (for ackermann_cmd_mux)
cd "$ROS_WS"/src/mushr/yaml-cpp/
sed -i '49s/OFF/ON/g' CMakeLists.txt
mkdir build && cd build
cmake .. && make && make install
cd "$ROS_WS"/src/mushr && rm -rf yaml-cpp

#udev rules to connect to devices
cp "$ROS_WS"/src/mushr/mushr_utils/udev_rules/* /etc/udev/rules.d

#Source and remake
. /opt/ros/melodic/setup.bash
. "$ROS_WS"/devel/setup.bash
cd "$ROS_WS" && catkin_make && cd "$ROS_WS"/src/mushr

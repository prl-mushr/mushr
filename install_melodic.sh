#!/usr/bin/env sh
#For Ubuntu 18.04 with ROS Melodic

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

#Get apt-add-repository
sudo apt-get install -y software-properties-common

#Install joy
sudo apt-get install -y ros-melodic-joy

#Install map_server
sudo apt-get install -y ros-melodic-map-server

#Install serial
sudo apt-get install -y ros-melodic-serial

#Install cv_bridge (for realsense)
sudo apt-get install -y ros-melodic-cv-bridge

#Install ackermann_msgs
sudo apt-get install -y ros-melodic-ackermann-msgs

#Install image_transport
sudo apt-get install -y ros-melodic-image-transport

#Install urg_node (for old car laser)
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
cd "$HOME"/catkin_ws/src/mushr && rm -rf librealsense

#Install yaml-cpp (for ackermann_cmd_mux)
cd "$HOME"/catkin_ws/src/mushr/yaml-cpp/
sed -i '49s/OFF/ON/g' CMakeLists.txt
mkdir build && cd build
cmake .. && make && make install
cd "$HOME"/catkin_ws/src/mushr && rm -rf yaml-cpp

#udev rules to connect to devices
cp "$HOME"/catkin_ws/src/mushr/mushr_utils/udev_rules/* /etc/udev/rules.d

#Source and remake
. /opt/ros/melodic/setup.bash
. "$HOME"/catkin_ws/devel/setup.bash
cd "$HOME"/catkin_ws/ && catkin_make && cd "$HOME"/catkin_ws/src/mushr

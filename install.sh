#!/usr/bin/env sh

#Install vctool
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y python3-vcstool

#Get Repos
vcs import < repos.yaml

#Get apt-add-repository
sudo apt-get install -y software-properties-common

#Install joy
sudo apt-get install -y ros-kinetic-joy

#Install serial
sudo apt-get install -y ros-kinetic-serial

#Install cv_bridge (for realsense)
sudo apt-get install -y ros-kinetic-cv-bridge

#Install ackermann_msgs
sudo apt-get install -y ros-kinetic-ackermann-msgs

#Install image_transport
sudo apt-get install -y ros-kinetic-image-transport

#Install urg_node (for old car laser)
sudo apt-get install -y ros-kinetic-urg-node

#Install IMU package
sudo apt-get install -y ros-kinetic-razor-imu-9dof

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

#Install yaml-cpp (for ackermann_cmd_mux)
cd $HOME/catkin_ws/src/mushr/yaml-cpp/
sed -i '49s/OFF/ON/g' CMakeLists.txt
mkdir build && cd build
cmake .. && make && make install
cd $HOME/catkin_ws/src/mushr && rm -rf yaml-cpp

#udev rules to connect to devices
cp $HOME/catkin_ws/src/mushr/mushr_utils/udev_rules/* /etc/udev/rules.d

#Source and remake
source /opt/ros/kinetic/setup.bash
source $HOME/catkin_ws/devel/setup.bash
cd $HOME/catkin_ws/ && catkin_make && cd $HOME/catkin_ws/src/mushr

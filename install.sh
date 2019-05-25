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

#Install librealsense verson 2.15 
#newer versions give make issue https://github.com/IntelRealSense/librealsense/issues/2314
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
ver="2.15.0-0~realsense0.83"
sudo apt-get install -y librealsense2=${ver}
sudo apt-get install -y librealsense2-utils=${ver}
sudo apt-get install -y librealsense2-dev=${ver}
sudo apt-get install -y librealsense2-dbg=${ver}

#Install yaml-cpp (for ackermann_cmd_mux)
cd $HOME/catkin_ws/src/mushr/yaml-cpp/
sed -i '49s/OFF/ON/g' CMakeLists.txt
mkdir build && cd build
cmake .. && make && make install
#cd $HOME/catkin_ws/src/mushr && rm -rf yaml-cpp

#Source and remake
source $HOME/catkin_ws/devel/setup.bash
cd $HOME/catkin_ws/ && catkin_make && cd $HOME/catkin_ws/src/mushr

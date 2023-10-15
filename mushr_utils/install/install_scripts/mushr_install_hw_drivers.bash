#!/bin/bash

# Need to reboot after running this script

# Install librealsense. Following commands copied from
# https://github.com/JetsonHacksNano/installLibrealsense/blob/master/installLibrealsense.sh
apt-key adv --keyserver keyserver.ubuntu.com  --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key

# NOTE hardcoded "bionic" b.c. focal arm64 release not found
add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u

apt-get install rsync -y
apt-get install librealsense2-utils librealsense2-dev -y
apt-get install ros-noetic-realsense2-camera ros-noetic-realsense2-description -y

# Set ROS_IP
OS = "$1"
HOUND = "$2"
if [[ "$OS" == "aarch64" ]]; then
    echo "export ROS_IP=\$(ifconfig wlan0 | grep 'inet ' | awk '{print \$2}')" >> ~/.bashrc
else
    echo "export ROS_IP=\$(ifconfig \$(ifconfig | awk '/^[a-zA-Z]/{interface=$1} END{print interface}' | sed 's/://') | grep 'inet ' | awk '{print \$2}')" >> ~/.bashrc
    conda install -c conda-forge rospkg
    pip uninstall numpy
    pip install numpy==1.22.3 ## reinstall the correct version
fi

if [[ "$HOUND" == 1 ]]; then
	echo "export Torch_DIR=\$(pip show torch | grep -i 'location' | awk '{print \$2}')/torch/share/cmake/Torch" >> ~/.bashrc
	echo 'export PYTHONPATH="${PYTHONPATH}:/root/catkin_ws/src/BeamNGRL"' >> ~/.bashrc
	echo 'export BNG_HOME="/root/BeamNG/BeamNG/"' >> ~/.bashrc
fi
echo "ldconfig" >> ~/.bashrc

# Install BLDC tool. Following commands adapted from:
# https://github.com/jetsonhacks/installBLDC/blob/master/installBLDC.sh
# cd $ROOT
# apt-get install -y qtcreator libudev-dev libqt5serialport5-dev 
# git clone https://github.com/vedderb/bldc-tool
# cd bldc-tool
# qmake -qt=qt5
# make clean & make

# Install push-button drivers
# pip install Jetson.GPIO -y
# rm /usr/bin/python
# ln -s /usr/bin/python3 /usr/bin/python
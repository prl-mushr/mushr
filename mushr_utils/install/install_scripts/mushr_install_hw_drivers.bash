#!/bin/bash

# Need to reboot after running this script

# Install librealsense. Following commands copied from
# https://github.com/JetsonHacksNano/installLibrealsense/blob/master/installLibrealsense.sh
apt-key adv --keyserver keyserver.ubuntu.com  --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key

# NOTE hardcoded "bionic" b.c. focal arm64 release not found
add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u

apt-get install rsync -y
apt-get install librealsense2-utils librealsense2-dev -y
apt-get install ros-noetic-realsense2-camera -y

# Install BLDC tool. Following commands adapted from:
# https://github.com/jetsonhacks/installBLDC/blob/master/installBLDC.sh
cd $ROOT
apt-get install -y qtcreator libudev-dev libqt5serialport5-dev 
git clone https://github.com/vedderb/bldc-tool
cd bldc-tool
qmake -qt=qt5
make clean & make

# Install push-button drivers
pip install Jetson.GPIO

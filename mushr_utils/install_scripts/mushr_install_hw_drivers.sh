#!/bin/bash

# Need to reboot after running this script

# Install librealsense. Following commands copied from
# https://github.com/JetsonHacksNano/installLibrealsense/blob/master/installLibrealsense.sh
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get install apt-utils -y
sudo apt-get install librealsense2-utils librealsense2-dev -y

# Install YDLidar drivers
cd ~/catkin_ws/src/mushr/mushr_hardware
git clone https://github.com/prl-mushr/ydlidar
cd ydlidar
sudo startup/initenv.sh

# Install BLDC tool. Following commands adapted from:
# https://github.com/jetsonhacks/installBLDC/blob/master/installBLDC.sh
cd $ROOT
sudo apt-get install qtcreator libqt4-dev libudev-dev libqt5serialport5-dev 
git clone https://github.com/vedderb/bldc-tool
cd bldc-tool
qmake -qt=qt5
make clean & make
cd $ROOT

# Add user to dialout to communicate with VESC
sudo adduser $USER dialout

# Setup VESC udev rules. Following commands adapted from:
# https://github.com/RacecarJ/installRACECARUdev
echo "ACTION==\"add\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", SYMLINK+=\"vesc\"" > /etc/udev/rules.d/10-vesc.rules
sudo udevadm control --reload-rules 
sudo udevadm trigger

# Install push-button drivers
cd ~/catkin_ws/src/mushr/mushr_hardware
git clone https://github.com/prl-mushr/push_button_utils

echo "Hardware drivers installed, please reboot for changes to take effect"
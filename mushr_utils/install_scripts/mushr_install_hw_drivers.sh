#!/bin/bash

# Need to reboot after running this script

# Install YDLidar drivers
cd ~/catkin_ws/src/mushr/mushr_hardware
git clone https://github.com/prl-mushr/ydlidar
cd ydlidar
sudo startup/initenv.sh

# Install BLDC tool. Following commands adapted from:
# https://github.com/jetsonhacks/installBLDC/blob/master/installBLDC.sh
cd $ROOT
sudo apt-get install qtcreator libqt4-dev libudev-dev libqt5serialport5-dev -y 
git clone https://github.com/vedderb/bldc-tool
cd bldc-tool
qmake -qt=qt5
make clean & make
cd $ROOT

# Add user to dialout to communicate with VESC
sudo adduser $USER dialout

# Setup VESC udev rules. Following commands adapted from:
# https://github.com/RacecarJ/installRACECARUdev
echo "ACTION==\"add\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", SYMLINK+=\"vesc\"" > /tmp/10-vesc.rules
sudo mv /tmp/10-vesc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules 
sudo udevadm trigger

# Install GPIO library
sudo pip install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo wget https://raw.githubusercontent.com/NVIDIA/jetson-gpio/master/lib/python/Jetson/GPIO/99-gpio.rules -O /etc/udev/rules.d/99-gpio.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Install push-button drivers
cd ~/catkin_ws/src/mushr/mushr_hardware
git clone https://github.com/prl-mushr/push_button_utils

# Setup rc.local
echo '#!/bin/sh -e' > /tmp/rc.local
echo "echo 200 > sys/class/gpio/export" >> /tmp/rc.local
echo "chmod go+w /sys/class/gpio/gpio200/direction" >> /tmp/rc.local
echo "chmod go+rw /sys/class/gpio/gpio200/value" >> /tmp/rc.local
echo "echo \"in\" > /sys/class/gpio/gpio200/direction" >> /tmp/rc.local
echo "rs-enumerate-devices &> /dev/null" >> /tmp/rc.local
echo "nvpmodel -m 0" >> /tmp/rc.local
echo "sleep 60 && jetson_clocks" >> /tmp/rc.local
sudo mv /tmp/rc.local /etc/rc.local

source /opt/ros/melodic/setup.bash

# Compile
cd ~/catkin_ws
catkin_make

# Install timed roslaunch
sudo apt install ros-melodic-timed-roslaunch -y

echo "Hardware drivers installed, please reboot for changes to take effect"

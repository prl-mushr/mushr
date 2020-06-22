#!/bin/bash

echo "This script will install all libraries necessary for the MuSHR racecar. It assumes that this SD card has freshly been flashed with the NVIDIA stock image, and that there is an internet connection"
read -p "Would you like to proceed with installation? " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

# Install ROS
source mushr_install_ros.sh

# Install MuSHR stack
source mushr_install_stack.sh

# Install hardware drivers
source mushr_install_hw_drivers.sh

echo "Installation complete. Please continue with the software setup at https://mushr.io/hardware/build_instructions/#software-setup"

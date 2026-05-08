#!/bin/bash

#
# Script to run teleop (or any command on startup).
# just add the line:
#   su nvidia -c '/path/to/teleop-on-startup.sh 2>&1 > /path/to/teleop-on-startup.log &'
# to /etc/rc.local (after seting up GPIO for the bumper).
#
# From there, when powering on, hold the bumper button.
# Voila, teleop running without needing to ssh in.
#

ROS_DISTRO=melodic
WORKSPACE=/home/robot/catkin_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source $WORKSPACE/devel/setup.bash

GPIO_BUTTON_FILE=/sys/class/gpio/gpio298/value

if [[ -f "$GPIO_BUTTON_FILE" ]]; then
        VALUE=$(cat $GPIO_BUTTON_FILE)
        if [[ "$VALUE" -eq 0 ]]; then
                roslaunch mushr_base teleop.launch &
                exit 0;
        fi
fi

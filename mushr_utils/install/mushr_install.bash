#!/bin/bash

# Detect OS 
export OS_TYPE="$(uname -s)"
if [[ $OS_TYPE == "Darwin" ]]; then
  export SHELL_PROFILE=".zshrc"
else
  export SHELL_PROFILE=".bashrc"
fi

# Are we in the right place to be running this?
if [[ ! -f mushr_install.bash ]]; then
  echo Wrong directory! Change directory to the one containing mushr_install.bash
  exit 1
fi
export INSTALL_PATH=$(pwd)

# Real robot or on a laptop?
read -p "Are you installing on robot and need all the sensor drivers? (y/n) " -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    export REAL_ROBOT=1
else
    export REAL_ROBOT=0
fi

# NVIDIA GPU? 
read -p "Do you have a nvidia gpu with installed drivers? (y/n) " -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    if [[ $REAL_ROBOT == 1 ]]; then
      export COMPOSE_FILE=docker-compose-robot.yml
    else
      export COMPOSE_FILE=docker-compose-gpu.yml
    fi
else
    export COMPOSE_FILE=docker-compose-cpu.yml
fi

# Build from scratch (assumes GPU)?
read -p "Build from scratch? (Not recommended, takes much longer than pulling ready-made image) (y/n) " -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
  export COMPOSE_FILE=docker-compose-build.yml
fi

# curl and dep keys
if [[ $OS_TYPE != "Darwin" ]]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-get update
  sudo apt-get install -y curl
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

# Robot specific settings
if [[ $REAL_ROBOT == 1 ]]; then
    echo Running robot specific commands
    
    # Don't need sudo for docker
    sudo usermod -aG docker $USER
    
    # docker-compose
    sudo curl -L "https://github.com/docker/compose/releases/download/v2.2.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose

    # Need to connect to ydlidar
    git clone https://github.com/prl-mushr/ydlidar
    cd ydlidar
    sudo sh startup/initenv.sh
    cd .. && rm -rf ydlidar

    # Setup VESC udev rules. Following commands adapted from:
    # https://github.com/RacecarJ/installRACECARUdev
    echo "ACTION==\"add\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", SYMLINK+=\"vesc\"" > /tmp/10-vesc.rules
    sudo mv /tmp/10-vesc.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && udevadm trigger

    # Install GPIO library
    pip install Jetson.GPIO
    sudo groupadd -f -r gpio
    sudo usermod -a -G gpio $USER
    sudo wget https://raw.githubusercontent.com/NVIDIA/jetson-gpio/master/lib/python/Jetson/GPIO/99-gpio.rules -O /etc/udev/rules.d/99-gpio.rules
    sudo udevadm control --reload-rules && udevadm trigger

    # Setup rc.local
    echo '#!/bin/sh -e' > /tmp/rc.local
    echo "echo 200 > /sys/class/gpio/export" >> /tmp/rc.local
    echo "chmod go+w /sys/class/gpio/gpio200/direction" >> /tmp/rc.local
    echo "chmod go+rw /sys/class/gpio/gpio200/value" >> /tmp/rc.local
    echo "echo \"in\" > /sys/class/gpio/gpio200/direction" >> /tmp/rc.local
    echo "rs-enumerate-devices &> /dev/null" >> /tmp/rc.local
    echo "nvpmodel -m 0" >> /tmp/rc.local
    echo "sleep 60 && jetson_clocks" >> /tmp/rc.local
    sudo mv /tmp/rc.local /etc/rc.local
fi

# vcstool https://github.com/dirk-thomas/vcstool 
if [[ $OS_TYPE != "Darwin" ]]; then
  sudo apt-get update && sudo apt install -y python3-vcstool
else
  sudo pip install vcstool
fi

# Make catkin_ws outside container for easy editing
if [[ ! -d "../../../../../catkin_ws" ]]; then
  mkdir -p ../../../catkin_ws/src
  cd ../../../ && mv mushr catkin_ws/src/mushr
fi

# Pull repos
export WS_PATH=$(pwd | sed 's:/catkin_ws.*::')
cd $WS_PATH/catkin_ws/src/ && vcs import < mushr/base-repos.yaml && vcs import < mushr/nav-repos.yaml
cd mushr/mushr_utils/install/ && export INSTALL_PATH=$(pwd)

# Make sure environment Variables are always set
if ! grep -Fq "export INSTALL_PATH=" ~/$SHELL_PROFILE ; then
  echo "export INSTALL_PATH=${INSTALL_PATH}" >> ~/$SHELL_PROFILE
fi
if ! grep -Fq "export REAL_ROBOT=" ~/$SHELL_PROFILE ; then
  echo "export REAL_ROBOT=${REAL_ROBOT}" >> ~/$SHELL_PROFILE
fi
if ! grep -Fq "export WS_PATH=" ~/$SHELL_PROFILE ; then
  echo "export WS_PATH=${WS_PATH}" >> ~/$SHELL_PROFILE
fi
if ! grep -Fq "export COMPOSE_FILE=" ~/$SHELL_PROFILE ; then
  echo "export COMPOSE_FILE=${COMPOSE_FILE}" >> ~/$SHELL_PROFILE
fi

# If laptop, don't build realsense2_camera, ydlidar, or push_button_utils
if [[ $REAL_ROBOT == 0 ]]; then
  touch $WS_PATH/catkin_ws/src/mushr/mushr_hardware/push_button_utils/CATKIN_IGNORE
  touch $WS_PATH/catkin_ws/src/mushr/mushr_hardware/ydlidar/CATKIN_IGNORE
  touch $WS_PATH/catkin_ws/src/mushr/mushr_hardware/realsense/realsense2_camera/CATKIN_IGNORE
fi

# Shortcuts
if ! grep -Fq "alias mushr_noetic=" ~/$SHELL_PROFILE ; then
  echo "alias mushr_noetic=\"docker-compose -f $INSTALL_PATH/$COMPOSE_FILE run -p 9090:9090 mushr_noetic bash\"" >> ~/$SHELL_PROFILE
fi
# TODO these don't work
#echo "alias mushr_build=\"docker-compose -f $INSTALL_PATH/$COMPOSE_FILE run mushr_noetic bash -c 'cd /root/catkin_ws && catkin_build'\" ">> ~/$SHELL_PROFILE
#echo "alias mushr_teleop=\"docker-compose -f $INSTALL_PATH/$COMPOSE_FILE run mushr_noetic roslaunch mushr_base teleop.launch\" ">> ~/$SHELL_PROFILE

# Make sure all devices are visible
if [[ $REAL_ROBOT == 1 ]]; then
  sudo udevadm control --reload-rules && sudo udevadm trigger
fi

# Display permissions
if ! grep -Fxq "xhost + >> /dev/null" ~/$SHELL_PROFILE ; then
  read -p $'Add "xhost +" to $SHELL_PROFILE? This enables GUI from docker but is a security risk.\nIf no, each time you run the docker container you will need to execute this command.\nAdd xhost + $SHELL_PROFILE? (y/n) ' -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo WARNING: Adding "xhost +" to $SHELL_PROFILE 
    echo "xhost + >> /dev/null" >> ~/$SHELL_PROFILE && source ~/$SHELL_PROFILE
  fi
fi

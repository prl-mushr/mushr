FROM ubuntu:focal

LABEL maintainer="Matt Schmittle <schmttle@cs.washington.edu>"
ARG DEBIAN_FRONTEND=noninteractive
# 1 = real robot, 0 = sim (laptop)
ARG REAL
WORKDIR /root

# Install docker deps for mushr_install 
RUN apt-get update -y \
    && apt-get install keyboard-configuration apt-utils gnupg2 curl -y \
    && apt-get update -y

# Adding/Running files separately allows us to cache
RUN mkdir install_scripts

# ROS Noetic
ADD src/mushr/mushr_utils/install/install_scripts/mushr_install_ros.bash install_scripts/mushr_install_ros.bash
RUN chmod +x install_scripts/mushr_install_ros.bash && install_scripts/mushr_install_ros.bash 

# Catkin build and other deps
ADD . catkin_ws
ADD src/mushr/mushr_utils/install/install_scripts/mushr_install_deps.bash install_scripts/mushr_install_deps.bash
RUN chmod +x install_scripts/mushr_install_deps.bash && install_scripts/mushr_install_deps.bash 

# Hardware drivers
ADD src/mushr/mushr_utils/install/install_scripts/mushr_install_hw_drivers.bash install_scripts/mushr_install_hw_drivers.bash
RUN if [ "$REAL" = 1 ] ; then chmod +x install_scripts/mushr_install_hw_drivers.bash && install_scripts/mushr_install_hw_drivers.bash ; fi

# Set ROS_IP
RUN if [ "$REAL" = 1 ] ; then \
    echo "export ROS_IP=\$(ifconfig wlan0 | grep 'inet ' | awk '{print \$2}')" >> ~/.bashrc ; \
    else \
    echo "export ROS_IP=\$(ifconfig eth0 | grep 'inet ' | awk '{print \$2}')" >> ~/.bashrc ; \
    fi

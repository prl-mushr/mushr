FROM timongentzsch/l4t-ubuntu20-pytorch:latest

MAINTAINER Matt Schmittle <schmttle@cs.washington.edu>

# Install docker deps for mushr_install 
RUN apt-get update -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install keyboard-configuration apt-utils gnupg2 curl -y \
    && apt-get update -y

# Adding/Running files separately allows us to cache
RUN mkdir install_scripts

# ROS Noetic
#ADD install_scripts/mushr_install_ros.sh install_scripts/mushr_install_ros.sh
#RUN chmod +x install_scripts/mushr_install_ros.sh && install_scripts/mushr_install_ros.sh 

# Catkin build and other deps
#ADD install_scripts/mushr_install_deps.sh install_scripts/mushr_install_deps.sh
#RUN chmod +x install_scripts/mushr_install_deps.sh && install_scripts/mushr_install_deps.sh 

# MuSHR base repos 
#ADD mushr /root/catkin_ws/src/mushr
#ADD mushr_sim /root/catkin_ws/src/mushr_sim
#ADD serial /root/catkin_ws/src/serial
#ADD range_libc /root/catkin_ws/src/range_libc

#ADD install_scripts/mushr_install_stack.sh install_scripts/mushr_install_stack.sh
#RUN chmod +x install_scripts/mushr_install_stack.sh && install_scripts/mushr_install_stack.sh 

# Hardware drivers
# 1 = real robot, 0 = sim (laptop)
ARG REAL=1 
ADD install_scripts/mushr_install_hw_drivers.sh install_scripts/mushr_install_hw_drivers.sh
RUN if [ "$REAL" = 1 ] ; then chmod +x install_scripts/mushr_install_hw_drivers.sh && install_scripts/mushr_install_hw_drivers.sh ; fi

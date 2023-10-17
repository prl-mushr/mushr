ARG BASE_IMAGE
FROM "$BASE_IMAGE"

MAINTAINER Matt Schmittle <schmttle@cs.washington.edu>
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /root

RUN apt-get update

RUN apt-get install software-properties-common -y

# Install docker deps for mushr_install 
RUN apt-get update -y \
    && apt-get install keyboard-configuration apt-utils gnupg2 curl -y \
    && apt-get update -y

# Adding/Running files separately allows us to cache
RUN mkdir install_scripts

# ROS Noetic
ADD src/mushr/mushr_utils/install/install_scripts/mushr_install_ros.bash install_scripts/mushr_install_ros.bash
RUN chmod +x install_scripts/mushr_install_ros.bash && install_scripts/mushr_install_ros.bash 

# Hardware drivers
# 1 = real robot, 0 = sim (laptop)
ARG REAL
ARG HOUND
ARG OS
# Catkin build and other deps
ADD . catkin_ws
ADD src/mushr/mushr_utils/install/install_scripts/mushr_install_deps.bash install_scripts/mushr_install_deps.bash
RUN chmod +x install_scripts/mushr_install_deps.bash && install_scripts/mushr_install_deps.bash "$OS"

ADD src/mushr/mushr_utils/install/install_scripts/mushr_install_hw_drivers.bash install_scripts/mushr_install_hw_drivers.bash
RUN if [ "$REAL" = 1 ] ; then chmod +x install_scripts/mushr_install_hw_drivers.bash && install_scripts/mushr_install_hw_drivers.bash "$OS" "$HOUND" ; fi

ADD src/mushr/mushr_utils/install/install_scripts/hound_install.bash install_scripts/hound_install.bash
RUN if [ "$HOUND" = 1 ] ; then chmod +x install_scripts/hound_install.bash && install_scripts/hound_install.bash ; fi
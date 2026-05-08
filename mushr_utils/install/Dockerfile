FROM ros:humble

LABEL maintainer="Ethan Kogan <dbzfan@uw.edu>"
ARG DEBIAN_FRONTEND=noninteractive
# 1 = real robot (install hardware drivers), 0 = sim
ARG REAL
WORKDIR /root

# Initial apt deps for the install scripts themselves
RUN apt-get update -y \
    && apt-get install -y --no-install-recommends \
        keyboard-configuration apt-utils gnupg2 curl \
    && apt-get update -y

# Adding/Running scripts separately allows us to cache each layer
RUN mkdir install_scripts

# Cyclone DDS unicast peer config (multicast-free network discovery)
ADD cyclone_dds.xml /root/cyclone_dds.xml

# ROS 2 Humble
ADD install_scripts/mushr_install_ros.bash install_scripts/mushr_install_ros.bash
RUN chmod +x install_scripts/mushr_install_ros.bash && install_scripts/mushr_install_ros.bash

# Workspace deps (apt + pip + range_libc + dev tools)
ADD install_scripts/mushr_install_deps.bash install_scripts/mushr_install_deps.bash
RUN chmod +x install_scripts/mushr_install_deps.bash && install_scripts/mushr_install_deps.bash

# Hardware drivers (librealsense, YDLidar SDK, Jetson.GPIO) — robot only
ADD install_scripts/mushr_install_hw_drivers.bash install_scripts/mushr_install_hw_drivers.bash
RUN if [ "$REAL" = "1" ]; then \
        chmod +x install_scripts/mushr_install_hw_drivers.bash \
        && install_scripts/mushr_install_hw_drivers.bash; \
    fi

WORKDIR /root/colcon_ws
CMD ["bash"]

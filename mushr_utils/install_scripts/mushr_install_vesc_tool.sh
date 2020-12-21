#!/bin/bash

# These install instructions were adapted from a post by
# subodh-malgonde here: https://github.com/vedderb/vesc_tool/issues/23

cd $HOME

git clone https://github.com/vedderb/vesc_tool

sudo apt-get install qt5-default libudev-dev libqt5serialport5-dev qtconnectivity5-dev qtpositioning5-dev qtquickcontrols2-5-dev -y

cd vesc_tool

qmake -qt=qt5

make clean && make -j4

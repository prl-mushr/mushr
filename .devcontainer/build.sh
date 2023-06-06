#!/bin/bash

cd /home/vscode/catkin_ws/src;
./mushr/mushr_utils/install/mushr_install.bash --trivial-only;
mushr_noetic run 'cd catkin_ws && catkin build'

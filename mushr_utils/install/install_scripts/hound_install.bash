pip install -r ~/catkin_ws/src/mushr/requirements_hound.txt ## this provides the mppi

apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-nmea-msgs ros-noetic-control-toolbox usbutils ros-noetic-grid-map ros-noetic-pcl-ros ros-noetic-depth-image-proc
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
apt-get remove -y ros-noetic-mavros ros-noetic-mavros-extras
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
## unit test: docker install --> catkin build --> BeamNG check --> BeamNG HITL test
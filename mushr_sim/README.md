[![Build Status](https://dev.azure.com/prl-mushr/mushr_sim/_apis/build/status/prl-mushr.mushr_sim?branchName=master)](https://dev.azure.com/prl-mushr/mushr_sim/_build/latest?definitionId=5&branchName=master)

# MuSHR Simulator
The MuSHR simulator is the easiest way to get started with MuSHR. The simulated car can be either controlled via keyboard teleoperation, or by a control algorithm. This simulator is designed to emulate the car's configuration as close as possible to reduce sim-to-real overhead. To do so, many of the hardware components are programmatically simulated (camera is not simulated).

### Tutorial
To install/run the simulator see the following [tutorial](https://prl-mushr.github.io/tutorials/quickstart/).

### API
For adjusting params see `config` it has the simulated vesc params and also the sensor params for each car. If you don't find the publishers or subscribers you were looking for, they are likely in [mushr_base](https://github.com/prl-mushr/mushr_base).

#### Publishers
Topic | Type | Description
------|------|------------
`/mux/ackermann_cmd_mux/input/teleop`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Publish teleop controls from keyboard
`/map` | [nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html) | Map from map server
`/map_metadata` | [nav_msgs/MapMetaData](http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html) | Map metadata
`/car/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Simulated laser scan topic

#### Subscribers
Topic | Type | Description
------|------|------------
`/tf` | [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html) | Transforms for the laserscan
`/tf_static` | [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html) | Static transforms for the laserscan
`/car/mux/ackermann_cmd_mux/input/teleop`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Publish teleop controls from keyboard
`/car/mux/ackermann_cmd_mux/input/navigation` | [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) | Programatic controller input control

# mushr_X
[![build-test](https://github.com/prl-mushr/mushr_x/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/prl-mushr/mushr_x/actions) 
<!-- must have ci badge -->
mushr_x does x and is a super awesome package. Below is install and run instructions, but the best way to get started is to checkout the [this tutorial](https://mushr.io/tutorials/). Detailed run instructions should be in a tutorial, this serves as a quick reference

**Authors:** <!-- give people credit! -->  
John Doe

### Install <!-- good to have with repo -->  

Clone repo:
`cd ~/catkin_ws/src/ && git clone git@github.com:prl-mushr/mushr_x.git`

### Run <!-- good to have with repo -->  
For real car: `roslaunch mushr_x real.launch`  
For sim: `roslaunch mushr_x sim.launch`

See [this tutorial](https://mushr.io/tutorials/) for more information. <!-- in case they didn't see it in the description, good to have here -->  

### API <!-- must have! -->  
Parameters can be changed in `config/config.yaml`
#### Parameters
Topic | Type | Description
------|------|------------
`param_1` | float (static) | description 3
`param_2` | int (static| description 2
`param` | str (dynamic)| If you have dynamic parameters that use [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure)

#### Publishers
Topic | Type | Description
------|------|------------
`/car/pf/inferred_pose` | [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) | Particle filter pose estimate
`/car/pf/viz/particles` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Partilcle array. Good for debugging
`/car/pf/viz/laserpose` | [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)| Pose fo the laser

#### Subscribers
Topic | Type | Description
------|------|------------
`/map` | [nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) | Map the robot is in
`/car/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Current laserscan
`/car/vesc/sensors/servo_position_command` | [std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html) | Current steering angle
`/car/vesc/sensors/core` | [vesc_msgs/VescStateStamped](https://github.com/prl-mushr/vesc/blob/master/vesc_msgs/msg/VescStateStamped.msg)| Current speed

#### Services
Topic | Type | Description
------|------|------------
`/map` | [nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) | Map the robot is in
`/car/scan` | [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html) | Current laserscan

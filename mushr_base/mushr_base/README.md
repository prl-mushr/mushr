[![Build Status](https://dev.azure.com/prl-mushr/mushr_base/_apis/build/status/prl-mushr.mushr_base?branchName=master)](https://dev.azure.com/prl-mushr/mushr_base/_build/latest?definitionId=3&branchName=master)

# MuSHR Base
MuSHR repo for base packages necessary for running the car in sim and on the robot.

### Mushr_base API
Joystick params can be set in `mushr_base/config/joy_teleop.yaml`

#### Publishers
Topic | Type | Description
------|------|------------
`/car_pose`| [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) |Pose of robot. Published when map &rarr; base_footprint transforms exist (sim).
`/joint_states`| [sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html) |Joint states from robot model and position
`/mux/ackermann_cmd_mux/input/teleop`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) |Publish teleop controls from joystick (real robot)
`/tf`| [tf2_msgs/TFMessage](http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html) |All transforms

#### Subscribers
Topic | Type | Description
------|------|------------
`/initialpose`| [geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) |Initial pose of robot, usually provided from rviz.  
`/vesc/sensors/core`| [vesc_msgs/VescStateStamped](https://github.com/prl-mushr/vesc/blob/master/vesc_msgs/msg/VescStateStamped.msg) |Vesc state. Speed param used to get controls.  
`/vesc/sensors/servo_position_command`| [std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html) |steering servo state

### Ackermann_cmd_mux API
Parameters can be set in ackermann_cmd_mux/param/mux.yaml

#### Subscribers
Topic | Type | Description
------|------|------------
`/mux/ackermann_cmd_mux/input/default`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) |Default input to car if not input control  
`/mux/ackermann_cmd_mux/input/navigation`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) |Controller's input channel to drive car  
`/mux/ackermann_cmd_mux/input/safety`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) |Safety controller's input channel. Currently null 
`/mux/ackermann_cmd_mux/input/teleop`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) |Teleop controller's input channel  

#### Publishers
Topic | Type | Description
------|------|------------
`/mux/ackermann_cmd_mux/output`| [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) |Output of muxed inputs topics
`/mux/ackermann_cmd_mux/active`| [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) |Which input is the current output 

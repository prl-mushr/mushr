# Ackermann Command Mux 
Multiplexer for choosing from autonomy, teleop, safety, or no control.

### API
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

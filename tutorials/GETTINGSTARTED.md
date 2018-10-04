# Getting Started with RACECAR Tutorial
This tutorial will get you started with RACECAR by connecting to the car and running the trajectory library file baseline which allows the car to navigate a known map. We are assuming you have ros and rviz installed locally. If you do not, follow these instructions and they should install both on the desktop-full version.

### Local Computer Setup:
  In .bashrc set the following environment variables:
```sh
export ROS_IP = $(hostname -I | cut -f2 -d ` `)
export ROS_MASTER_URI = http://172.16.77.#:11311
```
This sets your local ROS_IP and where to look for roscore on the car. Replace ‘#’ with your car number

### Connect to Car:
Make sure the battery tray on the bottom is connected. Also, connect the blue and green wires to the energizer battery pack (usb hub & tx2 power). Power on by pressing the red button labelled POWERBTN on the left side of the car. To turn off, hold that same button until the green PWR light turns off.
```sh
local$ ssh nvidia@172.16.77.#
password: nvidia#
```
###### Optional: Setup alias and ssh keys to speed up workflow
```sh
alias CAR_NAME='ssh nvidia@172.16.77.#'
```
If you don’t already have a private ssh key
```sh
local$ ssh-keygen
```
and follow the steps (you don’t need a paraphrase)
Share your public key with the car by:
```sh
local$ ssh-copy-id -i ~/.ssh/PUBLIC_KEY_NAME nvidia@172.16.77.#
```
Then by typing your CAR_NAME you should securely connect to your car

### Update Vesc
We need to update our low-level vesc controller to the UW modified version.
```sh
$ git remote add upstream https://github.com/johannesburg/uw-vesc.git
$ git fetch upstream
$ git commit -m “switching to upstream”
$ git checkout upstream
```

### Start Car with Teleop:
On the car run:
```sh
$ roslaunch racecar teleop.launch
```
To tune parameters edit:
~/catkin_ws/src/racecar_base/racecar/config/racecar-v2/vesc.yaml
And rerun.

### Run Particle Filter:
Clone MIT PF from into ~/catkin_ws/src/
Then run:
```sh
$ roslaunch particle_filter localize.launch
```
While teleop.launch is running.

To visualize:
```sh
local$ rosrun rviz rviz
```
On your local computer. You won’t see anything at first unless you subscribe to a topic in the bottom left of the screen by clicking the ‘Add’ button. To see all relevant information copy the pf.rviz file like so.
```sh
local$ cd ~/.rviz
```
```sh
local$ sftp nvidia@172.16.77.#
local$ get ~/catkin_ws/src/particle_filter/rviz/pf.rviz
```
On rviz go to file → Open Config → pf.rviz
If everything is working you should see white laser scan dots and red arrows representing location hypotheses.

Try tuning parameters to get better localization. They are located in the ~/catkin_ws/src/particle_filter/launch/localize.launch file.

### Run Trajectory Library Baseline
Baseline, MPPI, m3pi are all implemented in the MPPIController class.
You can use this package, get it from here. If there is a 404, you need permission to clone. Check out to the trajectory_library branch. On line 26 of the launch file, update the name and address to your car’s address.
~/catkin_ws/src/m3pi/launch/tl_mini_loop_experiment.launch

Get the new rviz file the same way as in the last step. It is located here:
~/catkin_ws/src/m3pi/rviz/experimental.rviz

Remake packages
```sh
$ cd ~/catkin_ws/
$ catkin make
```

Note: if you get a error about already havinng a build directory. Just run:
```sh
$ catkin clean
```
Run the experiment
```sh
$ roslaunch m3pi tl_mini_loop_experiment.launch sim:=false
local$ rosrun rviz rviz
```
With the rviz settings loaded, you should see a map of the building basement. By adding a “2D Pose Estimate” (located on the top of the rviz screen) to the map, the localization hypotheses should form around the your chosen location. Then, by adding a “2D Nav Goal” the robot should now have a start pose and a target pose. To run, hold down RB on the handheld controller.

### Troubleshooting
- ###### Robot drives crazy (usually a hard left)
    Likely your state estimate is off. Try resetting the trajectory library controller
    ```sh
	$ rostopic pub /mppi/reset std_msgs/Empty "{}"
	```
	And then redoing the “2D Pose Estimate”

- ###### It goes straight for the goal even though there is a wall
    Try re-computing the permissible regions. TODO



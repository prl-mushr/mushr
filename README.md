![no image](https://github.com/prl-mushr/mushr/blob/master/header.jpg)
# MuSHR
MuShR: Multi-agent System for non-Holonomic Racing

This is the main repository for MuSHR. For a full directory structure see [here](https://github.com/prl-mushr/mushr/blob/master/mushr_docs/structure.png). Below is a overview of each component:
- `mushr_demos`: launch files for all demos on the mushr platform
- `mushr_description`: meshes, stl files, and urdf's for each mushr platform. Also contains kinematic car model
- `mushr_docs`: tutorials, assignments, and readme directory
- `mushr_hardware`: launchfiles for running the car and location for installed hardware packages
- `mushr_utils`: mapfiles, rviz setup files, and other utils for running various tasks
- `mushr_base`: found [here](https://github.com/prl-mushr/mushr_base), provides core launchfiles and teleop scripts. Used for both real world and simulation

## Install
There are 2 ways of getting mushr running on a car. We strongly recommend option 1 as it is relatively plug and play. Option 2 is best used only when option 1 fails. Note, you will likely run into more problems with option 2.

For all installs know what type of car you have. To identify, see below:
- v1 (MIT Racecar): leftmost car in above image (jetson tx2)
- v2: middle car (jetson tx2)
- v3: rightmost car (jetson nano)

### 1: Install Jetson Image
Because there are a lot of small steps in the install, we decided to make it much easier for you by providing a base image that works right out of the box. 

**v1/v2** follow [these](https://github.com/prl-mushr/mushr/blob/master/mushr_docs/install/robosetup.md) instructions with [this]() image.

**v3** follow Nvidia's [instructions](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) with [this]() image.

### 2: Install Scripts
We again strongly recommend using our image as it is significantly less error prone and we will be much quicker to fix. To run the install scripts, make sure you have a ubuntu 16/18.04 with ROS kinetic/melodic installed.  
1. Clone `mushr` into your `catkin_ws/src`  
2. Then `cd .../catkin_ws/src/mushr/`
3. `sudo ./install_melodic.sh` or `sudo ./install_kinetic.sh` if you are using kinetic. The script will take ~1 hour to run.
4. If something fails, please contact us and we will do our best to respond to your issue. Most of the time, failures happend when compiling librealsense as it is not fully supported for ARM devices.

## Getting Started
To test your install and get the car running teleop make sure everything is plugged in and follow these steps
1. `cd ../catkin_ws/ && catkin_make` Make sure you see no failures.
2. `. ~/.bashrc` 
3. `roslaunch mushr_base teleop.launch`

For more detailed instructions on getting started see [this](todo) tutorial.

## FAQ

- **Can I install mushr on my desktop?**  
Yes, only a subset of mushr is required to run the simulator. You will need ROS kinetic/melodic and ubuntu 16/18.04. 
1. Clone this repository into your catkin workspace `/src` directory.
2. Install [vcstool](https://github.com/dirk-thomas/vcstool).  
3. Run `vcs import < repos.yaml` from inside the mushr directory and clone the [sim](todo) into mushr. 
4. In your catkin workspace run `catkin_make`. 
5. You should now be able to launch the sim `roslaunch mushr_sim teleop.launch`
## Acknowledgements
This project is from the Personal Robotics Lab at the University of Washington Paul G. Allen School of Computer Science. It wouldn't be possible without the hard work and dedication of the following people:

**Advisor:** [Sidd Srinivasa](https://goodrobot.ai/)  
**Post Doctorial Advisor:** [Sanjiban Choudhury](http://www.sanjibanchoudhury.com/)  
**Project Lead:** [Matt Schmittle](https://schmittlema.github.io/)  
**Our AMAZING Development Team:** Matthew Rockett, [Colin Summers](https://colinxsummers.com/), [Johan Michalove](http://www.johanam.com/), [Patrick Lancaster](https://homes.cs.washington.edu/~planc509/), [Max Thompson](https://www.linkedin.com/in/max-thompson-aa242310a/), and [AJ Kruse](https://www.linkedin.com/in/ajkruse/)

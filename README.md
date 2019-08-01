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

## Component Documentation
- [mushr_base](https://github.com/prl-mushr/mushr_base): Scripts that ties all mushr components together.
- [vesc](https://github.com/prl-mushr/vesc): Code for interfacing high-level commands with the low-level vesc controller.
- [ydlidar](https://github.com/prl-mushr/ydlidar): Package that contains all code for the laser scanner.
- [push_button_utils](https://github.com/prl-mushr/push_button_utils): Package containing code for the push button.
- [Realsense](https://github.com/IntelRealSense/realsense-ros): External package for interfacing withe realsense camera.
- [mushr_sim](https://github.com/prl-mushr/mushr_sim): The simulator package.
- [mushr_pf](https://github.com/prl-mushr/mushr_pf): The particle filter used for localization in a map from laser scans.
- [mushr_rhc](https://github.com/prl-mushr/mushr_rhc): The receding horizon controller for navigating to goal points in a map.

The directory structure after install is [here](https://raw.githubusercontent.com/prl-mushr/mushr/master/mushr_docs/structure.png?token=ADLMJKEM2OEXW5XMU5WQBOC5JNYLA).
## FAQ

- **How do I get started?!**
Visit our [website](https://prl-mushr.github.io/)! We have an ever-growing list of tutorials and build instructions there.

- **Who can use MuSHR?**  
This project is intended for students and researchers at the undergraduate and graduate level, but that doesn't mean you can't do it! We welcome motivated high school students, and makers to try out the platform too. You will need familiarity with the linux terminal, python, and general building skills. No soldering skills are required to build the platform.

- **Can I install mushr_sim on my desktop?**  
Yes, the easiest way is through following our [quickstart tutorial](https://prl-mushr.github.io/tutorials/quickstart/). Otherwise, you can follow these instructions on a linux machine (you may need additional dependencies!). Only a subset of mushr is required to run the simulator. You will need ROS kinetic/melodic and ubuntu 16/18.04. 
1. Clone this repository into your catkin workspace `/src` directory.
2. Install [vcstool](https://github.com/dirk-thomas/vcstool).  
3. Run `vcs import < repos.yaml` from inside the mushr directory and clone the [sim](https://github.com/prl-mushr/mushr_sim) into `catkin_ws/src`. 
4. In your catkin workspace run `catkin_make`. 
5. You should now be able to launch the sim `roslaunch mushr_sim teleop.launch`

## Acknowledgements
This project is from the Personal Robotics Lab at the University of Washington Paul G. Allen School of Computer Science. It wouldn't be possible without the hard work and dedication of the following people:

**Advisor:** [Sidd Srinivasa](https://goodrobot.ai/)  
**Post Doctorial Advisor:** [Sanjiban Choudhury](http://www.sanjibanchoudhury.com/)  
**Project Lead:** [Matt Schmittle](https://schmittlema.github.io/)  
**Our AMAZING Development Team:** Matthew Rockett, [Colin Summers](https://colinxsummers.com/), [Johan Michalove](http://www.johanam.com/), [Patrick Lancaster](https://homes.cs.washington.edu/~planc509/), [Max Thompson](https://www.linkedin.com/in/max-thompson-aa242310a/), and [AJ Kruse](https://www.linkedin.com/in/ajkruse/)

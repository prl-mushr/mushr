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


### 1: Install Jetson Image
Because there are a lot of small steps in the install, we decided to make it much easier for you by providing a base image that works right out of the box. 

### 2: Install Scripts

![no image](https://github.com/prl-mushr/mushr/blob/master/header.jpg)
# MuSHR
Multi-agent System for non-Holonomic Racing

This is the main repository for MuSHR.
- [About MuSHR](https://mushr.io/about/)
- [System Overview](https://mushr.io/tutorials/overview/)
- [Build Instructions](https://mushr.io/hardware/build_instructions/)
- [Simulation Quickstart](https://mushr.io/tutorials/quickstart/)
- [Community Page](https://spectrum.chat/mushr?tab=posts)

Components are listed below. Note not all components are installed by default. For install and running various components we recommend following our [tutorials](https://mushr.io/tutorials/) also individually linked with each component:

## Basic
- [`mushr_docs`](https://github.com/prl-mushr/mushr/tree/master/mushr_docs) : Install and hardware related docs. Component documentation found with each component
- [`mushr_base`](https://github.com/prl-mushr/mushr_base): Scripts that ties all mushr components together.

## Simulation
- [`mushr_sim`](https://github.com/prl-mushr/mushr_sim): MuSHR's main simulator
- [`mushr_mujoco_ros`](https://github.com/prl-mushr/mushr_mujoco_ros): MuSHR's mujoco simulator.
- [`gym-donkeycar`](https://github.com/prl-mushr/gym-donkeycar): MuSHR version of the Donkeycar simulator

## Hardware
- [`mushr_hardware`](https://github.com/prl-mushr/mushr/tree/master/mushr_hardware/mushr_hardware): launchfiles for running the car and location for installed hardware packages
- [`mushr_description`](https://github.com/prl-mushr/mushr/tree/master/mushr_description): Official meshes, stl files, and urdf's for each mushr platform. Also contains description of kinematic car model
- [`mushr_cad`](https://github.com/prl-mushr/mushr_cad): CAD files for all versions of the MuSHR car
- [`push_button_utils`](https://github.com/prl-mushr/push_button_utils): ROS node interface for front bumper
- [`vesc`](https://github.com/prl-mushr/vesc): Code for communicating with the MuSHR car's VESC
- [`ydlidar`](https://github.com/prl-mushr/ydlidar): Package that contains all code for the laser scanner
- [`Realsense`](https://github.com/IntelRealSense/realsense-ros): External package for interfacing withe realsense camera

## Utils and Development Tools
- [`devtools`](https://github.com/prl-mushr/devtools): Development tools for linting and contributing to MuSHR
- [`mushr_utils`]( https://github.com/prl-mushr/mushr/tree/master/mushr_utils
): install scripts, rviz setup files, and other utils for running various tasks

## Autonomous Navigation
- [`mushr_pf`](https://github.com/prl-mushr/mushr_pf): The MuSHR particle filter used for localization in a known map.
- [`mushr_pf.jl`](https://github.com/prl-mushr/mushr_pf.jl): Another MuSHR particle filter written in julia
- [`mushr_rhc`](https://github.com/prl-mushr/mushr_rhc): The MuSHR receding horizon controller for navigation in a known map.
- [`mushr_gp`](https://github.com/prl-mushr/mushr_gp): The MuSHR global planner, for planning in a known map.

## Machine Learning
- [`mushr-dl`](https://github.com/prl-mushr/MUSHR-DL): A deep learning stack for reinforcement learning in the donkey sim

## FAQ

- **How do I get started?!**  
Visit our [website](https://mushr.io)! We have an ever-growing list of tutorials and build instructions there.

- **Who can use MuSHR?**  
This project is intended for students and researchers at the undergraduate and graduate level, but that doesn't mean you can't do it! We welcome motivated high school students, and makers to try out the platform too. You will need familiarity with the linux terminal, python, and general building skills. No soldering skills are required to build the platform.

## Acknowledgements
This project is from the [Personal Robotics Lab](https://personalrobotics.cs.washington.edu/) at the [University of Washington Paul G. Allen School of Computer Science](https://www.cs.washington.edu/). 

**Advisor:** [Sidd Srinivasa](https://goodrobot.ai/)  
**[PRL Team](https://personalrobotics.cs.washington.edu/people/)**

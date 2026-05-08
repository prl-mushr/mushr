# Teleop on startup

This script will allow the user to specify a command to run at startup if the bumper button is being held. To use it, two files need to be modified.

First, copy `teleop-on-startup.sh` somewhere on the car, preferably `/home/robot/teleop-on-startup.sh`.

Then, edit `/etc/rc.local`, adding the contents of the `rc.local` file in this directory:
```
su robot -c '/home/robot/teleop-on-startup.sh' 2>&1 > /home/robot/teleop-on-startup.log &'
```
Which will run the `teleop-on-startup.sh` script on startup as the `robot` user.

The `$ROS_DISTRO` variable is used to source the `setup.bash` for the current ros distribution.
The `$WORKSPACE` variable sets the current catkin workspace, so set it accorndingly, defaults to `~/catkin_ws`.

## Stoping teleop via ssh

If you want to kill the teleop process, ssh into the car and run `pkill roslaunch`.

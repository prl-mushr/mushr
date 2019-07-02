# MuSHR Hardware Setup Tutorial For V2
In this tutorial, you will learn how to load the OS onto version 2 (`racecar-uw`) of the car and install firmware onto the VESC. At the end of this tutorial, you should have a car able to be teleoperated and ready to run your ROS packages.

### Items Needed
- MuSHR robot fully built
- Linux host computer with at least 75 GB of free disk space (a VM should work if all the ports are accessible to the VM)
- Micro USB cable
- HDMI cable
- HDMI compatible monitor
- Mouse
- Keyboard

### Procedure
1. Download the car's image and Jetpack from [here](https://drive.google.com/file/d/1M3vloiBwu_n52nJ-2ysJPitrKtezwaBG/view?usp=sharing) onto the Linux host. Uncompress the file.
2. Connect the Jetson to the Linux host with a micro USB cable. Plug the Jetson into the battery or the power adapter that it came with.
3. Boot the Jetson into recovery mode by holding down the recovery button (second from the right) and then pressing the power button (rightmost). Release the recovery button after five seconds.
4. Check that the Jetson has successfully booted into recovery mode by executing the `lsusb` command on the Linux host. If successful,the output will have an entry that contains the string ’NVidia Corp.’
5. Navigate to the Linux_for_Tegra directory in a terminal on the Linux host.
6. Begin flashing the image onto the Jetson using the following command:
```
$ sudo ./flash.sh -r jetson-tx2 mmcblk0p1
```
**Note:** it will take a few minutes
7. Once the Jetson has finished flashing, plug the mouse and keyboard into the Jetson, and connect the Jetson and monitor with the HDMI cable. Shutdown the Jetson by holding the power button, and then press it again to reboot it.
8. On the Jetson, change the password as appropriate using the `passwd` command. The default password of the robot user is `prl_robot`
9. **Optional:** Change the hostname to the name of your car. Open the following files and replace *nugget* with your name:
	- `sudo nano /etc/hostname`
	- `sudo nano /etc/hosts`
	- `sudo reboot`
10. Configure the SSD
	- Open the **Disks** utility
	- Click on the icon corresponding to the SSD in the left hand column
	- Delete any existing partitions in the Volumes table by clicking on the minus icon and following the prompts.
	- Click the plus icon and enter the name of the partition. Name the partition "JetsonSSD", use the `ext4` format, and utilize the full space on the SSD.
	- Open the GParted utility and select the SSD (usually `/dev/sda`) device from the GUI’s upper-right drop down list.
	- Right-click on the new partition again and choose **Format to -> ext4**, click the green checkmark.
	- Right-click on the new partition and choose **Name Partition**, using "JetsonSSD", and click the gree checkmark.
    - Add the following line to `/etc/fstab` (with root permissions)
    ```
    /dev/sda1  /media/robot/JetsonSSD  ext4  defaults  0  2
    ```
	- Reboot the computer`sudo reboot`
    - Change ownership of the device to your account:
    ```
    $ sudo chown robot:nvidia /media/robot/JetsonSSD
    ```
	- Now you should be able to access it through `/media/robot/JetsonSSD`
		
11. Setup the robot’s network according to [this](https://drive.google.com/open?id=11qcVyFoVtKxxCiVlF4_E_a74uTdJ4bOa) document. Note that if you setup a static IP on an existing network, your subnet may be different from the addresses used in the document
12. Setup the VESC. Note that this step is optional if firmware has previously been loaded onto the VESC. Follow the instructions [here](http://www.jetsonhacks.com/2017/06/01/get-your-motor-running-vesc-jetson-racecar-build/)
13. 
14. Navigate to the `mushr_base` package: `roscd mushr_base/launch`
15. Edit the teleop.launch file so that the value of the racecar_version argument is either racecar-uw if using UW hardware, or racecar-mit if using MIT hardware
16. Test the car. After checking that everything is powered properly, execute the following command to launch the robot’s drivers:
```
$ roslaunch mushr_base teleop.launch
```
17. If you would like to save your own image of whatever is currently on the Jetson, you can again connect it to the host computer, put it in recovery mode, and then execute the following command to copy the current image to the host computer:
```
$ sudo ./flash.sh -r -k APP -G my_backup.img jetson-tx2 mmcblk0p1
```
If you then wanted to push this image on to a different Jetson, you should move `my_backup.img` and `my_backup.img.raw` to `bootloader/system.img` and `bootloader/system.img.raw` repectively. Then follow the above procedure for flashing.

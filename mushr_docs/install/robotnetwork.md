# Robot Network Configuration

Introduction
============

The robot can operate in two different networking modes:

1. One in which it broadcasts its own network.
+ another in which it obtains a static IP on an existing network.

This document provides instructions for configuration of both modes.


**For all of the steps in this document, let Z be the number on the front of the
car (for V2). Or the number you've assigned to the cars.**

Static IP On Existing Network
=============================

You will have to connect a monitor, keyboard, and mouse to your robot
for the following procedure.

1.  Open `/etc/modprobe.d/bcmdhd.conf` with sudo privileges. If
    the second line (`options bcmdhd op_mode=2`) is not commented out then
    comment it out (`#options bcmdhd op_mode=2`), save the file and reboot
    the Jetson. Otherwise, just close the file.

+   Connect to the network that the robot should have a static IP on. Do
    this by clicking on the opposing arrows symbol in the top right hand
    corner of the Ubuntu GUI and inspecting the Wi-Fi network list. If
    your robot is already connected to the correct network, then proceed
    to the next step. If it is already connected but to the wrong
    network, click *Disconnect* and then allow the Wi-Fi network list to
    refresh. Once the desired network appears in the Wi-Fi network list,
    click on it to connect. If the network does not appear in the Wi-Fi
    list, try using the **Connect to Hidden Wi-Fi Network\...** option.

+   Open the network connection editor: `sudo nm-connection-editor`.

+   Highlight the connection that corresponds to the network the robot
    should be connected to and click **Edit\...**. The connection should
    have the same name as the network if the robot has never connected
    to this network before. If the robot has connected to this network
    before, it might have the same name but post-fixed with a number. If
    this is the case, choose the one with the highest post-fixed number.

+   Click on the **General** tab and check that **Automatically connect to
    this network when it is available** and **All users may connect to this
    network** are enabled.

+   Click on the **Wi-Fi** tab. Make sure that the **SSID:** field is set to
    be the name of the network that the robot will have a static IP on.
    For example, it could be *University of Washington*. Make sure that
    the **Mode:** dropdown is set to **Client**. In the **BSSID:** dropdown,
    choose the last option. In the **Device:** dropdown, choose the last
    option.

+   Click on the **Wi-Fi Security** tab. Check that the **Security:**
    dropdown is set to **None**.

+   Click on the **IPv4 Settings** tab. Set the following fields:
    +  **Method:** dropdown to be **Manual**.
    +  Click **Add**. Under **Address**, enter:
        - The Static IP: for example `172.16.77.Z`.
        - Netmask: for example `255.255.255.0`.
        - Gateway: for example `172.16.77.100`.
    + **DNS servers:** field, enter `8.8.8.8`.

+   Click the **Save** button.

+   Close the connection editor.

+   Edit `/home/robot/.bashrc` editing the line that sets the `ROS_IP`.
    Alter this line so that the `ROS_IP` is set to the static ip, for example
    `172.16.77.Z`. Alternatively if you know the network interface, you can use
    the command (should be `wlan0` interface on the Jetson):
    ```
    export ROS_IP=$(ifconfig wlan0 | awk /inet\ /'{print $2}')
    ```

+   Reboot the Jetson: `sudo reboot`.

+   After the Jetson reboots, use the `ifconfig` command to verify that
    the robot has obtained the expected static ip.

Logging In
----------

1.  Once the Jetson has fully booted, it will connect to the existing
    network at the specified static ip. You should then be able to ssh
    into it, for example: `ssh robot@172.16.77.Z`.

2.  You are now connected to the robot through the existing network. If
    this is your first time logging in since setting up the connection,
    you should check that `mushr_base/launch/teleop.launch` still works.
    If it doesn't, check that the robot's `ROS_IP` is correctly being set
    to the robot's static IP.

Broadcast Own Network Configuration
===================================

**Note: messing up one or more of these steps could cause you to be
unable to connect to the robot at all. In that case, you will likely
need to plug the Jetson into a monitor and repair the connection.**

Setup
-----

1.  Either connect a monitor, mouse, and keyboard to the Jetson, or ssh
    in to your robot with screen forwarding: `ssh -X robot@<robot ip>`.

+   Open the network connection editor: `sudo nm-connection-editor`.

+   If there are any network connections that start with the name
    `Jetson`, delete them.

+   Click the **Add** button.

+   From the dropdown menu, choose **Wi-Fi**. Then click the **Create\...**
    button.

+   Under **Connection name:**, enter `JetsonZ` (Or any desired name).

+   Click on the **General** tab and check that **Automatically connect to
    this network when it is available** and **All users may connect to this
    network** are enabled.

+   Click on the **Wi-Fi** tab. Set the **SSID:** field to be `JetsonZ`. In
    the **Mode:** dropdown, choose **Hotspot**. In the **Device:** dropdown,
    choose the last option.

+   Click on the **Wi-Fi Security** tab. In the **Security:** dropdown,
    choose **WPA & WPA2 Personal**. Enter an adequate password into the
    **Password:** field. Note that the password must be at least eight
    characters long.

+   Click on the **IPv4 Settings** tab and check that the **Method:** field
    is set to **Shared to other computers**.

+   Click the **Save** button.

+   Close the connection editor.

+   Edit `/home/robot/.bashrc` editing the line that sets the `ROS_IP`.
    Alter this line so that the `ROS_IP` is set to the static IP, for example
    `10.42.0.1`.

+   Open `/etc/modprobe.d/bcmdhd.conf` with sudo privileges. If
    the second line is commented out (`#options bcmdhd op_mode=2`), then
    uncomment it out (`options bcmdhd op_mode=2`). Otherwise, just close the
    file.

+   Reboot the Jetson: `sudo reboot`.

Logging In
----------

1.  Once the Jetson has fully booted, it will begin broadcasting its own
    network named `JetsonZ`. Look for this network in your Wi-Fi list
    and connect to it. When attempting to connect, if your computer asks
    you to **Enter the PIN from the router label**, click **Connect using a
    security key instead**. Enter the password that was set when setting
    up this connection. If asked, allow your computer to be discovered
    by other computers on the network.

+   After connecting to the network and if you are using a VM, you may
    need to reset your VM's network connection. Do this by clicking on
    the opposing arrows symbol in the upper right of the VM, and then
    clicking on **Wired connection 1**. When complete, a pop-up saying
    **Connection Established** should appear.

+   SSH into the robot: `ssh robot@10.42.0.1`.

+   You are now connected to the robot through its own network. If this
    is your first time logging in since setting up the connection, you
    should check that `mushr_base/launch/teleop.launch` still works. If it
    doesn't, check that the robot's `ROS_IP` is correctly being set
    to `10.42.0.1`.


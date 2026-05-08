#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

from __future__ import absolute_import, division, print_function

import atexit
import os
import signal
from threading import Lock, Thread

try:
    from tkinter import Frame, Label, Tk
except ImportError:
    from tkinter import Frame, Label, Tk

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

# Keycodes for WASD on a standard keyboard with querty labels
# We use keycodes instead of Tkinter's convenience symbol
# because it makes us more robust to accidental caps locks
# or alternative keyboard layouts.
keycodes = [25, 38 ,39 ,40]
# Q key on a standard keyboard with qwerty labels
quit_keycode = 24


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("speed", 2.0)
        self.declare_parameter("max_steering_angle", 0.34)
        self.max_velocity = self.get_parameter("speed").value
        self.max_steering_angle = self.get_parameter("max_steering_angle").value
        
        self.quit_key = quit_keycode
        self.keycodes = keycodes
        self.state = [False] * 4  # matching keys
        self.state_lock = Lock()


        self.state_pub = self.create_publisher(
            AckermannDriveStamped, "mux/input/teleop", 1
        )
        self.root = self.setup_tk()
        self.timer = self.create_timer(0.1, self.publish_cb)
        self.get_logger().info("created timer")
        self.tk_timer = self.create_timer(0.02, self.tk_update)

    def tk_update(self):
        self.root.update_idletasks()
        self.root.update()


    def setup_tk(self):
        root = Tk()
        frame = Frame(root, width=100, height=100)
        frame.bind("<KeyPress>", self.keydown)
        frame.bind("<KeyRelease>", self.keyup)
        frame.pack()
        frame.focus_set()
        lab_text = [
            "Focus on this window",
            "and use the WASD keys",
            "to drive the car.",
            "",
            "Press Q to quit",
        ]
        lab = Label(
            frame,
            height=10,
            width=30,
            text="\n".join(lab_text),
        )
        lab.pack()
        return root

    def shutdown(self, signum=None, frame=None):
        self.root.quit()
        self.root.update()

    @property
    def control(self):
        return any(self.state)

    def keyeq(self, ev, code):
        return ev.keycode == code

    def keydown(self, ev):
        with self.state_lock:
            if self.keyeq(ev, self.quit_key):
                self.shutdown()
            for i, k in enumerate(self.keycodes):
                if self.keyeq(ev, k):
                    self.state[i] = True
                    # is this next line really necessary?
                    self.state[(i+2) % len(self.state)] = False

    def keyup(self, ev):
        with self.state_lock:
            for i, k in enumerate(self.keycodes):
                if self.keyeq(ev, k):
                    self.state[i] = False

    def publish_cb(self):
        self.get_logger().debug("Publishing teleop command")
        with self.state_lock:
            self.get_logger().debug("Publishing teleop command. Past state_lock")

            cmd_up, cmd_left, cmd_down, cmd_right = self.state
            ack = AckermannDriveStamped()

            ack.drive.speed = 0.0
            ack.drive.steering_angle = 0.0

            if cmd_up:
                ack.drive.speed = self.max_velocity
            elif cmd_down:
                ack.drive.speed = -self.max_velocity

            if cmd_left:
                ack.drive.steering_angle = self.max_steering_angle
            elif cmd_right:
                ack.drive.steering_angle = -self.max_steering_angle

            # self.get_logger().info(f"cmds: {self.state}")
            if self.state_pub is not None:
                self.state_pub.publish(ack)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # Reenable key repeats on exit
    atexit.register(lambda: os.system("xset r on"))

    print("Press Q to quit")

if __name__ == '__main__':
    main()
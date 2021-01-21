#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

#this is just a quick scrip I chopped together for testing my setup
#please note that I am having a 40:1 gearbox on top my BLDCs
#so please change the scale() ratio to your needs...
#
#start with the left an right buttons to move axis1 to the desired position
#and press the up button to commit
#moving axis0 is mapped to left analog stick horizontal
#moving axis1 is mapped to left analog stick vertical

import odrive
from odrive.enums import *
import time
import os
import pprint
import pygame

left = True
right = True
old_value_x = 0
new_value_x = 0
old_value_y = 0
new_value_y = 0
start_moving = False
calibration = True

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
my_drive = odrive.find_any()

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#my_drive = odrive.find_any("serial:/dev/ttyUSB0")

# Calibrate motor and wait for it to finish
print("starting calibration...")
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while my_drive.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

def add(value):
    value = value + .5
    return value
    # if value < 40:
    #     return value
    # else:
    #     return 40

def sub(value):
    value = value -.5
    return value
    # if value > 0:
    #     return value
    # else:
    #     return 0

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""

        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        """Listen for events to happen"""

        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value, 2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                # Insert your code on what you would like to happen for each event here!
                # In the current setup, I have the state simply printing out to the screen.

                os.system('clear')
                #pprint.pprint(self.button_data)
                #pprint.pprint(self.axis_data)
                #pprint.pprint(self.hat_data)
                global start_moving
                global calibration
                global new_value_x

                if 0 in self.axis_data:
                    if start_moving == True:
                        global old_value_x
                        value = (self.axis_data[0])
                        value = int(scale(value, (-1.0, +1.0), (0.0, 40.0)))
                        if old_value_x != value:
                            print("changed")
                            old_value_x = value
                            my_drive.axis0.controller.input_pos = value

                if 1 in self.axis_data:
                    if start_moving == True:
                        global old_value_y
                        value = (self.axis_data[1])
                        value = new_value_x + int(scale(value, (-1.0, +1.0), (0.0, 40.0)))
                        if old_value_y != value:
                            print("changed")
                            old_value_y = value
                            my_drive.axis1.controller.input_pos = value

                if self.button_data[1] == True:
                    if calibration == True:
                        new_value_x = add(new_value_x)
                        my_drive.axis1.controller.input_pos = new_value_x
                        print("button pressed")
                if self.button_data[3] == True:
                    if calibration == True:
                        new_value_x = sub(new_value_x)
                        my_drive.axis1.controller.input_pos = new_value_x
                    print("button pressed")
                if self.button_data[2] == True:
                    start_moving = True
                    calibration = False
                    print("button pressed")

if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()

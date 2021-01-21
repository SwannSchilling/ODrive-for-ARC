# ODrive-for-ARC
ODrive for ARC running on Arduino Uno

**THIS SETUP WILL ONLY WORK WHEN RUNNING
ODRIVE FIRMWARE v0.4.12**

This is the setup for my Tarot 6S 4108 KV:380 BLDC motor to be used like any other servo in ARC...

It is just MY PERSONAL setup and meant to get you started, please READ THE manual over at the official Odrive website!
https://docs.odriverobotics.com/

First you will need to install and configure your Odrive Software...
I am running Linux, please check the installation guide for Windows found at the official Odrive website if you are running Windows!
This will just be needed for the first time setup. After the setup you will use an Arduino connected to a Windows PC running ARC!

https://github.com/SwannSchilling/ODrive-for-ARC/blob/main/setup

You should see your BLDC motor moving, after being put to closed loop and receiving the odrv0.axis0.controller.pos_setpoint command...

Next you will have to upload this code to your Arduino Uno
https://github.com/SwannSchilling/ODrive-for-ARC/blob/main/ODriveArcArduinoTest.ino

In ARC just add a servo control to your project, and connect to the Arduino!
https://synthiam.com/Community/Tutorials/Connecting-Arduino-to-ARC-17526
The Arduino sketch will receive those servo positions and send them to the ODrive...

All non-power I/O is 3.3V output and 5V tolerant on input, on ODrive v3.3 and newer.

This tutorial is just meant to be a starting point, please mess around with my settings and share your experience, so we will get a better understanding on how this whole setup can be utilized!!:)

![alt text](https://github.com/SwannSchilling/ODrive-for-ARC/blob/main/ODrive.png)

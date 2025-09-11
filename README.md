## Mecanumbot Project

The Mecanumbot projects goal is to build a Turtlebot3 friend with mecanum wheel drive and fit into the Turtlebot3 family.

The projct includes 3 repositories:
- [Mecanum_PC](https://github.com/Fortuz/Mecanum_PC): Includes the main documentation and ROS2 packages to be installed on an external computer.
- [Mecanum_Pi](https://github.com/Fortuz/Mecanum_Pi): Includes the ROS2 packages to be installed on the onboard Raspberry Pi on the Mecanumbot itself.
- [Mecanum_Arduino](https://github.com/Fortuz/Mecanum_Arduino): Includes the Arduino C codes for the micro controllers mounted on the Mecanumbot (OpenCR and Arduino Nano).

The project is made with Ubuntu 22.04 and ROS2 Humble. <br>
As a main source of information, documentation, codes and more the original [Turtlebot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) can be found here.
Lot of the setup and codes came from the original Turtlebot3 project but a huge chunk of the codebase have been modified to accomodate the new motors, setup, and work with the mecanum wheel drive system. Altough the setups steps are basicaly the same the important steps can be read below.

## Mecanumbot introduction

(image of the robot) <br>
(image of the arhitecture) <br>

(include STL files for 3D printing) <br>
(Motor setup)

## Setup

### PC Setup

Download and Install Ubuntu 22.04 LTS Desktop (3.1.1. at the Turtlebot3 e-Manual) <br>
Install ROS2 on the PC (3.1.2. at the Turtlebot3 e-Manual) <br>
Install dependent ROS2 Packages (3.1.3. at the Turtlebot3 e-Manual) <br>

3.1.4. coming <br> Install packages
3.1.5 coming <br> Environment Configuration

### SBC Setup - Raspberry Pi

Install the Raspberry Pi with an Ubuntu and make some neccesary settings (3.2.1, 3.2.2., 3.2.3., 3.2.4. at the Turtlebot e-Manual) <br>
After ssh is available the Mecanum Pi will be available.

```
$ ssh ubuntu@192.168.0.240.
```

At this point it is important to set up a network, where the PC and the PI can see each other.

Install ROS2 Humble (3.2.5./1 at the Turtlebot3 e-Manual) <br>
3.2.5./2 there will be modifications <br>
After that the udev rule settings (mainly the same but the Nano sould be included)

ROS domain ID setup (probably not 30 to avoid collision)

Lidar setup

Raspberry Pi cam setup

### Open CR setup

Connect the OpenCR to an external PC which is installed with Arduino IDE <br>
navigate into the sketch folder and download the Mecanumbot_Arduino repository. <br>
The Nano_Led code should be uploaded to the Arduino Nano (settings comming soon) <br>
The OpenCR_core should be uploaded to the OpenCR board (settings coming soon). <br>

### Test

Test with RC100 controller to move the robot. <br>
The LEd controlls can be tested with a ROS2 srv.




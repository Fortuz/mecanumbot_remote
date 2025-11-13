## Mecanumbot Project - Remote PC codes

The Mecanumbot projects goal is to build a Turtlebot3 friend with mecanum wheel drive and fit into the Turtlebot3 family.<br>
This repository contains the Remote PC related packages.

## Project Repositories

[mecanumbot_microcontrollers](https://github.com/Fortuz/mecanumbot_microcontrollers) - Contains the microcontroller codes for the project  <br>
[mecanumbot_remote](https://github.com/Fortuz/mecanumbot_remote) - [This repository] Contains ROS2 packages for used on an external computer connected to the robot <br>
[mecanumbot](https://github.com/Fortuz/mecanumbot) -  Contains ROS2 packages run on the Raspberry Pi on the robot <br>
[mecanumbot_python](https://github.com/fegyobeno/mecanumbot_python.git) - Contains the native python scripts for manipulating the motors. Can be found on the robot locally in the ~/Sandbox folder <br>

Building and using a robot with additional motors with different protocols, using a mecanum wheel drive sysetem instead of a differential drive system requires some changes in the original architecture so this repository is intend to provide a full functionality similar to the original Turtlebot3 repositories. <br>

As a main source of information, documentation, codes and more the original [Turtlebot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) can be found here.
Lot of the setup and codes came from the original Turtlebot3 project but a huge chunk of the codebase have been modified to accomodate the new motors, setup, and work with the mecanum wheel drive system. Altough the setups steps are basicaly the same the important steps can be read below. <br>

The project is made with Ubuntu 22.04 and ROS2 Humble. <br>

## Mecanumbot introduction

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/mecanumbot.jpg" width="600" alt="Mecanumbot">
</p>

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot/blob/main/docs/images/turtlebot_architecture.drawio.png" width="600" alt="Architecture">
</p>


## Install

```
$ mkdir -p ~/dev_ws/src && cd ~/dev_ws/src
$ git clone https://github.com/Fortuz/mecanumbot_remote.git
$ cd ~/dev_ws/
$ echo "alias source_ros = 'source /opt/ros/humble/setup.bash'" >> ~/.bashrc
$ echo "alias source_ws = 'source /install/local_setup.bash'" >> ~/.bashrc
$ source ~/.bashrc
$ source_ros
$ colcon build --symlink-install
$ source_ws
$ echo 'export OPENCR_MODEL=mecanumbot' >> ~/.bashrc
$ echo 'export ROS_DOMAIN_ID=19' >> ~/.bashrc
$ echo 'export TURTLEBOT3_MODEL=mecanumbot' >> ~/.bashrc
$ echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
$ source ~/.bashrc
```

## Bringup

[On Robot] - Start the robot
```
$ ssh ubuntu@192.168.1.240
$ start_robot_with_led
```

[Remote PC]
```
$ cd ~/dev_ws
$ source_ros
$ source_ws
$ ros2 run mecanumbot_teleop mecanumbot_keyboard
$ ros2 launch mecanumbot_ledgui mecanumbot_ledgui.launch.py
```




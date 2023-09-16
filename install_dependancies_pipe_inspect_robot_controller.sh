#!/bin/bash

sudo apt install ros-noetic-velocity-controllers
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-position-controllers
sudo apt-get install ros-noetic-position-controllers
sudo apt install ros-noetic-joy
sudo chmod a+rw /dev/input/js0
rosparam set joy_node/dev "/dev/input/jsX"

#!/bin/bash

cd /home/pi/Documents/capstone-slam/RPIScannerWorkspace/
source ./devel/setup.bash
catkin_make
roslaunch rpi_stitching.launch

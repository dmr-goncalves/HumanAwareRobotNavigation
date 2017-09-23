#!/bin/bash

clear

echo "  --- Installing packages dependencies ---"

echo " --- Installing libsvm ---"

sudo apt-get install libsvm-*

echo "  --- Installing laser lms1xx plugin ---"

sudo apt-get install ros-kinetic-lms1xx

echo "  --- Installing multimaster  ---"

sudo apt-get install ros-kinetic-multimaster-*

echo "  --- Installing interactive-marker ---"

sudo apt-get install ros-kinetic-interactive-marker-*

echo "  --- Installing twist-mux ---"

sudo apt-get install ros-kinetic-twist-mux

echo "  --- Installing robot-pose-ekf ---"

sudo apt-get install ros-kinetic-robot-pose-ekf

echo "  --- Installing amcl ---"

sudo apt-get install ros-kinetic-amcl

echo "  --- Installing gmapping ---"

sudo apt-get install ros-kinetic-gmapping

echo "  --- Installing navigation stack ---"

sudo apt-get install ros-kinetic-navigation

echo "  --- Installing gridmap ---"

sudo apt-get install ros-kinetic-grid-map

sudo apt-get install ros-kinetic-grid-map-*

echo "  --- Installing hector-gazebo-plugins ---"

sudo apt-get install ros-kinetic-hector-gazebo-plugins

echo "  --- Installing tinyxml2 ---"

sudo apt-get install libtinyxml2-dev

echo "  --- Installing opencv libraries ---"

sudo apt-get install libopencv*

echo " --- Installing urg_c ---"

sudo apt-get install ros-kinetic-urg-c

echo " --- Installing laser_proc ---"

sudo apt-get install ros-kinetic-laser-proc

echo " --- Adding hokuyo cfg file permissions ---"

sudo chmod 777 /thesis/hokuyo/cfg/Hokuyo.cfg

echo " --- Add laser serial permissions ---" 

sudo chmod 777 /dev/ttyAMCO

echo " --- Update packages to be sure ---"

rospack profile

echo " --- Rerun bashrc variables to be sure ---"

source ~/.bashrc

echo " ****************************************************** "

echo " * --- 		    You're good to go        	    --- * "

echo " * ---   Any question add an issue in github 		--- * "

echo " * --- Or contact me in dmr.goncalves@hotmail.com --- * "

echo " ****************************************************** "
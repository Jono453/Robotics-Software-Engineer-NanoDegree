#!/bin/sh

xterm -e "roslaunch my_robot world.launch" & 
sleep 5
xterm -e "roslaunch my_robot amcl.launch" #contains the rviz launch config already



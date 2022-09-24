#!/bin/sh

xterm -e "roscore" &
sleep 5
xterm -e "gazebo" &
sleep 5
xterm -e "roslaunch my_robot world.launch"  

#!/bin/sh

xterm -e "source devel/setup.bash; roslaunch my_robot world.launch"  &
sleep 5
xterm -e "source devel/setup.bash; rosrun rviz rviz" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "source devel/setup.bash; rosrun pick_objects pick_objects_node" &
sleep 5
xterm -e "source devel/setup.bash; roslaunch my_robot my_robot_slam_gmapping.launch"

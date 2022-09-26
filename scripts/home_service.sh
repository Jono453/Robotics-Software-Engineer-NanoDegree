#!/bin/sh

xterm -e "source devel/setup.bash; roslaunch my_robot world.launch"  &
sleep 5
xterm -e "source devel/setup.bash; roslaunch my_robot amcl.launch" & #amcl launch file also launces rviz with required config and gmapping_slam node
sleep 5
xterm -e "source devel/setup.bash; rosrun add_markers add_markers_node" &
sleep 5
xterm -e "source devel/setup.bash; rosrun pick_objects pick_objects_node" &
sleep 5


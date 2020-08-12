#!/bin/sh
xterm  -e  " roslaunch my_robot world.launch " &
sleep 5
xterm  -e  " roslaunch my_robot rviz_config.launch " &
sleep 5
xterm  -e  " roslaunch my_robot amcl.launch " & 
sleep 5
xterm  -e  " roslaunch pick_objects pick_objects.launch " & 
sleep 5
xterm  -e  " roslaunch add_markers add_marker.launch "

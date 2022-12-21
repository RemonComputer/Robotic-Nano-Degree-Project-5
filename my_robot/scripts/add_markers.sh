#!/bin/bash

xterm -e "roslaunch my_robot world.launch" &
sleep 5
xterm -e "roslaunch my_robot amcl.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers"

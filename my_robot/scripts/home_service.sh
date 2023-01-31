#!/bin/bash

#xterm -e "GAZEBO_MODEL_PATH=$(pwd)/src/Robotic-Nano-Degree-Project-5/my_robot/meshes/ roslaunch my_robot world.launch" &
#sleep 5
#xterm -e "roslaunch my_robot amcl.launch" &
#sleep 5
#xterm -e "rosrun pick_objects pick_objects" &
#xterm -e "rosrun add_markers add_markers"

roslaunch my_robot home_service.launch

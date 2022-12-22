# Robotic-Nano-Degree-Project-5

## Description
- This is the Final Project of the Robotics nano degree.
- It performs occupency grid mapping using slam_gmapping package.
- You can save the resulted map using the map_server package.
- It performs localization within the map using adaptive monte-carlo localization.
- It plan a path and navigate to a goal using the move_base navigation package which contains a global and local path planners with many options.
- It contains [test_slam.sh](my_robot/scripts/test_slam.sh) which would be used to run a SLAM algorithm to create a map of the environment.
- It contains [test_navigation.sh](my_robot/scripts/test_navigation.sh) which you would use rviz to make the robot navigate to a certain point.
- It contains [home_service.sh](my_robot/scripts/home_service.sh) which would make the robot perform 2 delivery cycles for a virtual object and then park itself.

## Building the project
```
# Creating the ROS Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
# Cloning and building the prerequisites
sudo apt-get update
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/turtlebot/turtlebot
git clone https://github.com/turtlebot/turtlebot_interactions
git clone https://github.com/turtlebot/turtlebot_simulator
git clone https://github.com/ros-teleop/teleop_twist_keyboard
cd ~/catkin_ws/
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash
# Cloning and building the project
cd src
git clone https://github.com/RemonComputer/Robotic-Nano-Degree-Project-5
cd ..
catkin_make
source devel/setup.bash 
```

## Applying the slam algorithm and saving a map
```
./src/Robotic-Nano-Degree-Project-5/my_robot/scripts/test_slam.sh
# walk your robot to cover the whole environment, watch RVIZ to determine the covarage
# when your are done save the map in the maps folder from another terminal
cd src/Robotic-Nano-Degree-Project-5/my_robot/maps/
rosrun map_server map_saver -f map
cd -
```

## Testing the navigation
```
./src/Robotic-Nano-Degree-Project-5/my_robot/scripts/test_navigation.sh
# use the 2DNav goal button and point to a goal, you will notice that the robot talks a walk to that goal
```

## Testing the home robot service delivery
```
./src/Robotic-Nano-Degree-Project-5/my_robot/scripts/home_service.sh
# You will notice that the robot will take 2 delivery cycles of a virtual blue box then it parks itself to the final position
```

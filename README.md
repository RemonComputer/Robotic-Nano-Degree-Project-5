# Robotic-Nano-Degree-Project-5

## Description
- This is the Final Project of the Robotics nano degree.
- It performs occupency grid mapping using slam_gmapping package.
- You can save the resulted map using the map_server package.
- It performs localization within the map using adaptive monte-carlo localization.
- It plan a path and navigate to a goal using the move_base navigation package which contains a global and local path planners with many options.
- For more info about the technical parts of the project please review [Packages_Description.md](./Packages_Description.md)
- It contains [test_slam.sh](my_robot/scripts/test_slam.sh) which would be used to run a SLAM algorithm to create a map of the environment.
- It contains [test_navigation.sh](my_robot/scripts/test_navigation.sh) which you would use rviz to make the robot navigate to a certain point.
- It contains [home_service.sh](my_robot/scripts/home_service.sh) which would make the robot perform 2 delivery cycles for a virtual object and then park itself.


## Demos
|[![Home Delivery](https://img.youtube.com/vi/rmFFGfkkE1U/0.jpg)](https://youtu.be/rmFFGfkkE1U)|
|:--:|
|<b>Video 1: Robot delivering objects demo</b>|
|:--:|
|[![Mapping using SLAM](https://img.youtube.com/vi/fy5jT41a8Fk/0.jpg)](https://youtu.be/fy5jT41a8Fk)|
|<b>Video 2: Mapping the environment using the SLAM Script</b>|
|:--:|
|[![Goal Navigation using RVIZ](https://img.youtube.com/vi/GjIxSsRoNbs/0.jpg)](https://youtu.be/GjIxSsRoNbs)|
|<b>Video 3: Performing Navigation using RVIZ</b>|

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
cd ~/catkin_ws/
source devel/setup.bash
rosdep -i install gmapping amcl move_base teleop_twist_keyboard
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
roslaunch my_robot test_slam.launch
# walk your robot to cover the whole environment, watch RVIZ to determine the covarage
# You can walk your robot using: i, j, k, l buttons but the terminal window should be active
# when your are done save the map in the maps folder from another terminal
cd src/Robotic-Nano-Degree-Project-5/my_robot/maps/
rosrun map_server map_saver -f map
cd -
```

## Testing the navigation
```
roslaunch my_robot test_navigation.launch
# use the 2DNav goal button and point to a goal, you will notice that the robot talks a walk to that goal
```

## Testing the home robot service delivery
```
roslaunch my_robot home_service.launch
# You will notice that the robot will take 2 delivery cycles of a virtual blue box then it parks itself to the final position
```

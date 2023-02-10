# Robotic-Nano-Degree-Project-5

## Description
- This is the Final Project of the Robotics nano degree.
- It performs occupency grid mapping using slam_gmapping package.
- You can save the resulted map using the map_server package.
- It performs localization within the map using adaptive monte-carlo localization.
- It plan a path and navigate to a goal using the move_base navigation package which contains a global and local path planners with many options.
- For more info about the technical parts of the project please review [Packages_Description.md](./Packages_Description.md)
- It contains [test_slam.launch](my_robot/launch/test_slam.launch) which would be used to run a SLAM algorithm to create a map of the environment.
- It contains [test_navigation.launch](my_robot/launch/test_navigation.launch) which you would use rviz to make the robot navigate to a certain point.
- It contains [home_service.launch](my_robot/scripts/home_service.launch) which would make the robot to  perform 2 delivery cycles for a virtual object and then park itself.


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
# Updating apt
sudo apt-get update
# Creating the ROS Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Cloning and building the project
git clone https://github.com/RemonComputer/Robotic-Nano-Degree-Project-5
cd ..
# Install the packages dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# Building the project
catkin_make
# Sourcing the workspace
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
# use the 2DNav goal button and click on the goal location and drag, you will see a big arrow and after releasing you will notice that the robot walks to the goal position.
```

## Testing the home robot service delivery
```
roslaunch my_robot home_service.launch
# You will notice that the robot will take 2 delivery cycles of a virtual blue box then it parks itself to the final position
```

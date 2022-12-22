# Packages Descriptions

## Description
This file entails the technical description of each package within the project

The following are the used packages within the project:
- rviz
- teleop_twist_keyboard
- xacro
- gazebo_ros
- libgazebo_ros_skid_steer_drive.so
- libgazebo_ros_openni_kinect.so
- libgazebo_ros_laser.so
- slam_gmapping
- map_server
- amcl
- move_base
- add_markers
- pick_objects
- my_robot

## rviz
Used to visualize the robot during its motion and the topics while it evolves in realtime.

## teleop_twist_keyboard
Used to steer and move the robot in the test_slam.sh script.

## xacro
Used to preprocess the custom robot urdf and its output is exported to Gazebo for simulation and for RVIZ for visualization.

## gazebo_ros
This is the Gazebo ROS bridge that provides a communication between ROS and Gazebo, it also used to push the robot in the simulation and starts the gazebo simulator.

## libgazebo_ros_skid_steer_drive.so
A Gazebo plugin used to steer and move the custom robot, it supports a robot with 4 wheels that can rotate in place like Pioneer 3-AT robot.

## libgazebo_ros_openni_kinect.so
A Gazebo plugin used to provide an interface to the robot component and make the RGBD Camera works and sends transmission using ROS topics.

## libgazebo_ros_laser.so
A Gazebo plugin used to provide an interface from the custom robot lidar and ROS, it sends the laser readings using ROS topics.

## slam_gmapping
- This package provides simultanious localization and mapping within the environment.
- Using this package we are able to map the environment while traversing it.

## map_server
- Using this package we are able to save the resulted map from the slam_gmapping in pgm image along with its metadata.
- We are also able to load the saved map for the navigation.

## amcl
- This package provides localization of the robot during its motion within the environment using the a map and sensor readings.
- It uses adaptive monte-carlo localization.
- It estimates the odometry with each motion.

## move_base
- This package works as a navigation stack for the project.
- It has an action server that you will provide with a goal and it will plan a path and navigate the robot to it.
- It has alot of options to use.
- It has a local and global planners, it has global and local planners.
- It uses the map and the laser sensors to detect objects and plan a path.
- It used the updated odometry.

## add_markers
- This package adds virtual markers to be picked by the robot.
- The markers act as a payload the will be picked and delivered by the robot.
- It listens for the robot odometry when it is near to it, it will wait for 5 seconds then simulate a pickup by hiding itself.
- When the robot reaches the delivery location, it will wait for 5 seconds and appear at the delivery loaction, to simulate the delivery.
- After another 10 seconds, it will reappear at the pickup location, to simulate another delivery.

## pick_objects
- This package uses the move_base package to direct the robot to go to certain goal location.
- It simulates 2 cargo delivery cicles, then it guides the robot to its final parking location.

## my_robot
- This is the main package that contains the scripts, world_description and launch files.
- It has the following structure:
  - CMakeLists.txt: The CMake file for compiing the package.
  - package.xml: The package meta-data xml file.
  - meshes: Contain meshes used in the project.
    - hokuyo.dae: The used Hokuyo Lidar sensor mesh.
  - maps: Contains generated maps from the slam script.
    - map.pgm: The  generated map pgm image.
    - map.yaml: The generated map meta-data.
  - worlds: Contains gazebo worlds used within the project.
    - empty.world: An empty world used within the project.
    - Remon_Kamal: The used gazebo world within the project.
  - urdf: The urdf files needed to generate and control the custom robot.
    - my_robot.xacro: The xacro urdf file used to generate the robot nodes, links and hinges.
    - my_robot.gazebo: A file used to interface gazebo ros plugins between the urdf file, ROS and Gazebo.
  - config: A folder to store various configurations used within the project.
    - Rviz_config.rviz: The visualization configuration that will be loaded by gazebo for topic visualizations.
    - base_local_planner_params.yaml: A local planner configuration used by the move_base package of the navigation stack.
    - local_costmap_params.yaml: A local costmap parameters used by the move_base package of the navigation stack.
    - costmap_common_params.yaml: A common costmap parameters used by the move_base package of the navigation stack.
    - global_costmap_params.yaml: A global costmap parameters used by the move_base package of the navigation stack.
  - scripts: A directory for various scripts used during the project phase.
    - launch.sh: A script to test running gazebo and sourcing ROS within the script in separate xterm windows.
    - test_slam.sh:  A script that is used  to launch and map the environment.
      - It launches the world, RVIZ, mapping node and the teleop node each in separeate xterm windows.
    - test_navigation.sh: A script that is used to launch RVIZ and the world, you can use the 2D Nav goal button to choose a location for the robot navigation.
      - It launches the world, RVIZ, the localization node, the map server and the navigation stack.
    - pick_objects.sh: Same as the previous script but it adds pick_objects node that instructs the robot to do the pickup cycles.
    - add_markers.sh: Same as test_navigation.sh but it adds add_markers node which add nodes at the pickup location and hides them if the robot approached it.
    - home_service.sh: The final script of the project that makes the final robot delivery cycles and parks the robot at the end.
  - launch: contains various launch files that is used to launch nodes across the project.
    - teleop.launch: Used to launch the keyboard teleop node that is used to steer and drive the robot.
    - robot_description.launch:
      - preprocesses the robot xacro urdf description.
      - Launches the robot_state_publisher node, which publishes the robot transforms with each frame of reference of its joints.
      - Launches the joint_state_publisher node, which publishes fake joint values.
    - world.launch: The main launch file that launches the world.
      - sets default attributes on the default robot positions and robot description file.
      - launches the robot_description.launch file.
      - Loads the gazebo simulator and pass the world file to it.
      - spawn the robot into the gazebo world.
      - Start the rviz using its stored configuration.
    - gmapping_demo.launch: Launches the slam_gmapping node to perform the grid occupency mapping algorithm.
    - mapping.launch: An old file not used in this project, it used to make 3d + 2d mapping and visualization using rtabmap ros package.
    - amcl.launch: Used to make localization and navigation of the robot.
      - Launches the map server that will load the stored map from previous node.
      - Launches the adaptive monte-carlo localization node.
      - Launches the move_base navigation stack used to drive the robot to a given path.




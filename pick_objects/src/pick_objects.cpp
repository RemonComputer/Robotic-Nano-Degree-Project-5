#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// #include <Quaternion.h>
#include <string>
#include "ros/time.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* ac;

void go_to_goal(float x, float y, float z, float w, std::string display_message) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "robot_footprint";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  // First_Position: (x=-4.2, y=-14.5, yaw=1.5707 --> z=0.7070727, w=0.7071408)
  // Second_Position: (x=-4.3, y=10.1, yaw=-1.5707)
  // original robot position (x=-0.31, y=-2.76, yaw=-1.5707)
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  // float yaw = 1.5707;
  //goal.target_pose.pose.orientation.setEuler(yaw, 0, 0); 
  goal.target_pose.pose.orientation.w = w;
  goal.target_pose.pose.orientation.z = z;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal: %.2f, %.2f", x, y);
  ac->sendGoal(goal);

  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached its goal
  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(display_message.c_str());
  else
    ROS_INFO("The base failed to target");
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
   ac = new MoveBaseClient("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 ros::Duration(2, 0).sleep();
 go_to_goal(-1, -5, 0, 1, "The Robot Reached the first location");
 ros::Duration(1, 0).sleep();
 go_to_goal(-1, -5, 0, 1, "The Robot Reached the second location");
 
 //go_to_goal(4, 4, 0, 1, "The Robot Reached the second location"); 
 ros::Duration(20, 0).sleep();

  return 0;
}

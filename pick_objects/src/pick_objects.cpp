#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// #include <Quaternion.h>
#include <string>
#include "ros/time.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient* ac;

void go_to_goal(float x, float y, std::string display_message) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = 0;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0; 
  goal.target_pose.pose.orientation.w = 1;

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
  ros::NodeHandle n;
  float x_pickup;
  float y_pickup;
  float x_deliver;
  float y_deliver;
  float x_parking;
  float y_parking;
  
  if (!n.getParam("x_pickup", x_pickup) || !n.getParam("y_pickup", y_pickup) || !n.getParam("x_delivery", x_deliver) || !n.getParam("y_delivery", y_deliver) || !n.getParam("x_parking", x_parking) || !n.getParam("y_parking", y_parking)){
   	ROS_ERROR("Failed to get parameters - pick_objects");
   }

  //tell the action client that we want to spin a thread by default
   ac = new MoveBaseClient("move_base", true);

  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
 for(int i = 0; i < 2; i++) {
 	go_to_goal(x_pickup, y_pickup, "The Robot Reached the first pickup location");
 	ros::Duration(1, 0).sleep();
 	go_to_goal(x_deliver, y_deliver, "The Robot Reached the delivery location");
 	ros::Duration(1, 0).sleep();
 }
 go_to_goal(x_parking, y_parking, "The Robot went to a parking position");
  return 0;
}

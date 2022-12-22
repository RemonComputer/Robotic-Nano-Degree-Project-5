#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

ros::Publisher marker_pub;

float x_pickup = -4;
float y_pickup = -10;
float x_deliver = -10;
float y_deliver = 10;

float epsilon = 0.1;

void publish_marker(float x, float y, float z, float r, float g, float b, ros::Publisher& marker_pub, bool add = true) {
  // Set our initial shape type to be a cube
   uint32_t shape = visualization_msgs::Marker::CUBE;

     visualization_msgs::Marker marker;
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
     marker.header.frame_id = "/map";
     marker.header.stamp = ros::Time::now();
 
     // Set the namespace and id for this marker.  This serves to create a unique ID
     // Any marker sent with the same namespace and id will overwrite the old one
     marker.ns = "my_marker";
     marker.id = 0;
 
     // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
     marker.type = shape;
 
     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
     marker.action = visualization_msgs::Marker::ADD;
     if(add == false) {
      marker.action = visualization_msgs::Marker::DELETE;
     }
 
     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
     marker.pose.position.x = x;
     marker.pose.position.y = y;
     marker.pose.position.z = z;
     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
 
     // Set the scale of the marker -- 1x1x1 here means 1m on a side
     marker.scale.x = 0.3;
     marker.scale.y = 0.3;
     marker.scale.z = 0.3;
 
     // Set the color -- be sure to set alpha to something non-zero!
     marker.color.r = r;
     marker.color.g = g;
     marker.color.b = b;
     marker.color.a = 1.0;
 
     marker.lifetime = ros::Duration();
 
     // Publish the marker
     while (marker_pub.getNumSubscribers() < 1)
     {
       if (!ros::ok())
       {
         return ;
       }
       ROS_WARN_ONCE("Please create a subscriber to the marker");
       sleep(1);
     }
     marker_pub.publish(marker);
} 

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  float x_robot = msg->pose.pose.position.x;
  float y_robot = msg->pose.pose.position.y;
  float delta_x_pickup = abs(x_robot - x_pickup);
  float delta_y_pickup = abs(y_robot - y_pickup);
  if(delta_x_pickup < epsilon && delta_y_pickup < epsilon) {
    ros::Duration(5, 0).sleep();
    publish_marker(x_pickup, y_pickup, 0, 0.0f, 0.0f, 1.0f, marker_pub, false);
    return;
  }
  float delta_x_deliver = abs(x_robot - x_deliver);
  float delta_y_deliver = abs(y_robot - y_deliver);
  if (delta_x_deliver < epsilon && delta_y_deliver < epsilon) {
    ros::Duration(5, 0).sleep();
    publish_marker(x_deliver, y_deliver, 0, 0.0f, 0.0f, 1.0f, marker_pub, true);
    ros::Duration(10, 0).sleep();
    publish_marker(x_pickup, y_pickup, 0, 0.0f, 0.0f, 1.0f, marker_pub, true);
    return;    
  }
}
 
int main( int argc, char** argv )
 {
   ros::init(argc, argv, "add_markers");
   ros::NodeHandle n;
   marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
   
   publish_marker(x_pickup, y_pickup, 0, 0.0f, 0.0f, 1.0f, marker_pub, true);
   ros::spin();
   return 0;
 }

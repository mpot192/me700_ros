#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <cmath>


int main(int argc, char **argv){
  ros::init(argc, argv, "hover_origin");
  int count = 0;
  ros::NodeHandle n;
  geometry_msgs::PoseStamped msg;

  // Publisher to send target position
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);

  ros::Rate loop_rate(5);

  while (ros::ok()){
    // Add time and frame_id to message
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    
    // Add current path point to message
    if(count > 50){
      msg.pose.position.x = -2.65; 
      msg.pose.position.y = -3.77; 
      msg.pose.position.z = 5;
    } else {
      msg.pose.position.x = 0; 
      msg.pose.position.y = 0; 
      msg.pose.position.z = 5;
    }

    // Add desired orientation quarternion to message
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0; 
    msg.pose.orientation.w = 1.0;

    // Publish message to desired trajectory topic
    chatter_pub.publish(msg);

    // Run callbacks
    ros::spinOnce();

    // Wait
    loop_rate.sleep();
    count++;
  }
  return 0;
}

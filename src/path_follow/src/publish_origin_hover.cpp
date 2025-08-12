#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <sstream>
#include <cmath>

#define HOVER_HEIGHT 5 // height to hover above ground [m]

int main(int argc, char **argv){
  ros::init(argc, argv, "hover_origin");
  int count = 0;
  ros::NodeHandle n;
  geometry_msgs::PoseStamped msg;

  // Publisher to send target position
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);

  ros::Rate loop_rate(5);

  int x_offset = 0;
  int y_offset = 0;

  // if three arguments, given x y hover coordinates so hover at those coordinates
  if(argc == 3){
    std::string arg1 = argv[1];
    std::string arg2 = argv[2];
    x_offset = std::stof(arg1);
    y_offset = std::stof(arg2);
    ROS_INFO("Hovering above [%f, %f].", std::stof(arg1), std::stof(arg2));
  } else if(argc == 2){ // if two arguments, going to one of the three presets (somewhat deprecated)
    std::string arg1 = argv[1];
    if(arg1 == "bottom"){
      y_offset = -5;
      ROS_INFO("GOING BOTTOMj");
    } else if(arg1 == "left"){
      x_offset = -5;
      ROS_INFO("GOING LEFT");
    } else if(arg1 == "right"){
      x_offset = 5;
      ROS_INFO("GOING RIGHT");
    }
  } 
  while (ros::ok()){
    // Add time and frame_id to message
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    
    // Add current hover point to message
    // hover at takeoff position for 10 sec, then move to actual position to get nicer takeoff
    if(count > 50){
      msg.pose.position.x = -2.65 + x_offset; 
      msg.pose.position.y = -3.77 + y_offset; 
      msg.pose.position.z = HOVER_HEIGHT;
    } else {
      msg.pose.position.x = 0; 
      msg.pose.position.y = 0; 
      msg.pose.position.z = HOVER_HEIGHT;
    }

    // Add desired orientation quarternion to message (in this case, no rotation)
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

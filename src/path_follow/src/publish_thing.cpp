#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

int main(int argc, char **argv){
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
  ros::Rate loop_rate(5);
  long count = 0;
  float step = 0.0;
  while (ros::ok()){
    geometry_msgs::PoseStamped msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

   if (count % 10 == 0 && count > 50){
      step += 0.01;
    }

    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 3.0;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = step; 
    msg.pose.orientation.w = 0.0;

    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }
  return 0;
}

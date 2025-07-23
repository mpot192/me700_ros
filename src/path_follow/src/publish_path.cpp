#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <sstream>
#include <cmath>

// I know this is not ideal but im just making it work for now
nav_msgs::Odometry drone_odom;
nav_msgs::Path path;
bool run = false;
bool got_path = false;
int cols;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // ROS_INFO("yo: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  drone_odom = *msg;
  run = true;
}
void pathCallback(const nav_msgs::Path::ConstPtr& msg){
  path = *msg;
  cols = path.poses.size();
  got_path = true;
}
float GetRMSE(float des_pos[3]){
  return sqrt((pow(drone_odom.pose.pose.position.x - des_pos[0], 2) + pow(drone_odom.pose.pose.position.y - des_pos[1], 2) + pow(drone_odom.pose.pose.position.z - des_pos[2], 2))/3);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "path_talker");

  ros::NodeHandle n;
  geometry_msgs::PoseStamped msg;

  // Publisher to send target position
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
  ros::Subscriber path_sub = n.subscribe("/path_gen/planned_path", 1000, pathCallback);
  // Subscriber to currrent odom
  ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 1000, odomCallback);
  ros::Publisher followed_pub = n.advertise<nav_msgs::Path>("/followed_path", 1000);
  nav_msgs::Path follow;

  ros::Rate loop_rate(5);

  long count = 0;
  int idx = 0;
  float RMSE;

  // Path info
  const int rows = 3;
  const int cols = 182;
  // Desired path 
  float current_path[3]; 
  nav_msgs::Path planned;
  geometry_msgs::PoseStamped p;
  while (ros::ok()){
    
    if(run) ROS_INFO("yo: [%f, %f, %f]", drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z);
    if(got_path){
      // Get target point
      current_path[0] = path.poses[idx].pose.position.x;
      current_path[1] = path.poses[idx].pose.position.y;
      current_path[2] = path.poses[idx].pose.position.z;

    
      RMSE = GetRMSE(current_path);

      ROS_INFO("RMSE: %.2f",RMSE);

      // Move to next index in desired path
      if (RMSE <= 0.2 && idx < cols){
          idx++;
          current_path[0] = path.poses[idx].pose.position.x;
          current_path[1] = path.poses[idx].pose.position.y;
          current_path[2] = path.poses[idx].pose.position.z;
      } else if (idx == cols - 1){
          idx = 0;
          count = 0;
      }

      // Add time and frame_id to message
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";
      
      // Add current path point to message
      msg.pose.position.x = current_path[0];
      msg.pose.position.y = current_path[1];
      msg.pose.position.z = current_path[2];

      // Add desired orientation quarternion to message
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0; 
      msg.pose.orientation.w = 1.0;

      // Publish message to desired trajectory topic
      chatter_pub.publish(msg);
    }
    for (int i = 0; i < cols; i++){
      geometry_msgs::PoseStamped p2;
      p2.header.stamp = ros::Time::now();
      p2.header.frame_id = "map";
      
      p2.pose.position.x = current_path[0];
      p2.pose.position.y = current_path[1];
      p2.pose.position.z = current_path[2];

      p2.pose.orientation.x = 0.0;
      p2.pose.orientation.y = 0.0;
      p2.pose.orientation.z = 0.0; 
      p2.pose.orientation.w = 1.0;
      planned.poses.push_back(p2);
    }

    planned.header.frame_id = "map";
    planned.header.stamp = ros::Time::now();
    // path_pub.publish(planned);
    // Track path
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "map";
    
    p.pose.position.x = drone_odom.pose.pose.position.x;
    p.pose.position.y = drone_odom.pose.pose.position.y;
    p.pose.position.z = drone_odom.pose.pose.position.z;

    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0; 
    p.pose.orientation.w = 1.0;
    follow.poses.push_back(p);
    follow.header.frame_id = "map";
    follow.header.stamp = ros::Time::now();

    followed_pub.publish(follow);    

    // Run callbacks
    ros::spinOnce();

    // Wait
    loop_rate.sleep();
    count++;
  }
  return 0;
}

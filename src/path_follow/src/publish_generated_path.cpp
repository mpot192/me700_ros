#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
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
  geometry_msgs::TwistStamped vel_msg;

  // Publisher to send target position
  // ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);
  
  // Subscriber to currrent odom (publishes the drones local position by MAVROS)
  ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 1000, odomCallback);
  // Subscriber to generated path
  ros::Subscriber path_sub = n.subscribe("/planned_path", 1000, pathCallback);

  // for publishing the actual path followed
  ros::Publisher followed_pub = n.advertise<nav_msgs::Path>("/followed_path", 1000);
  nav_msgs::Path follow;

  ros::Rate loop_rate(5);

  long count = 0;
  int idx = 0;
  float RMSE;
  float dx;
  float dy;
  float dz;
  
  float vx;
  float vy;
  float vz;
  
  // Path info
  // [x1 x2 .. ]
  // [y1 y2 .. ]
  // [z1 z2 .. ]]
  // const int rows = 3;
  // const int cols = 182;
  // // Desired path 
  // double path[rows][cols] = {{3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.12041, 3.22457, 3.29839, 3.33191, 3.32061, 3.26601, 3.17548, 3.06125, 2.93875, 2.82452, 2.73399, 2.67939, 2.66809, 2.70161, 2.77543, 2.87959, 3, 3, 3.06021, 3.11228, 3.14919, 3.16596, 3.1603, 3.133, 3.08774, 3.03062, 2.96938, 2.91226, 2.867, 2.8397, 2.83404, 2.85081, 2.88772, 2.93979, 3, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3},{3, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.33333, 3.31082, 3.24634, 3.14858, 3.03076, 2.90878, 2.79912, 2.71659, 2.67234, 2.67234, 2.71659, 2.79912, 2.90878, 3.03076, 3.14858, 3.24634, 3.31082, 3.33333, 3.16667, 3.15541, 3.12317, 3.07429, 3.01538, 2.95439, 2.89956, 2.8583, 2.83617, 2.83617, 2.8583, 2.89956, 2.95439, 3.01538, 3.07429, 3.12317, 3.15541, 3.16667, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5},{3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25}};
  
  float current_path[3]; 
  geometry_msgs::PoseStamped p;
  while (ros::ok()){
    
    // if(run) ROS_INFO("yo: [%f, %f, %f]", drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z);
    if(got_path){ // if have recieved a path message at least once
      // Get target point
      current_path[0] = path.poses[idx].pose.position.x;
      current_path[1] = path.poses[idx].pose.position.y;
      current_path[2] = path.poses[idx].pose.position.z;
      
      // calcualte error from path
      dx = current_path[0] - drone_odom.pose.pose.position.x;
      dy = current_path[1] - drone_odom.pose.pose.position.y;
      dz = current_path[2] - drone_odom.pose.pose.position.z;
      

      RMSE = GetRMSE(current_path);
      ROS_INFO("RMSE: %.2f",RMSE);

      // Move to next index in desired path
      if (RMSE <= 0.05 && idx < cols){
          idx++;
      } else if (idx == cols - 1){
          idx = 0;
          count = 0;
      }
      
      float norm = sqrt(dx*dx + dy*dy + dz*dz);
      float set_speed = 0.05; // (m/s)
      
      // Uses the drones current position and setpoint to calculate the velocity vector but scaled to set_speed
      vx = (dx/norm) * set_speed;
      vy = (dy/norm) * set_speed;
      vz = (dz/norm) * set_speed;
      

      // Add time and frame_id to message
      vel_msg.header.stamp = ros::Time::now();
      vel_msg.header.frame_id = "map";
      
      // Filling twiststamped velocity message
      vel_msg.twist.linear.x = vx;
      vel_msg.twist.linear.y = vy;
      vel_msg.twist.linear.z = vz;
      
      // Just setting angular velocity to 0 for now...
      vel_msg.twist.angular.x = 0.0;
      vel_msg.twist.angular.y = 0.0;
      vel_msg.twist.angular.z = 0.0;

      // Publish message to desired trajectory topic
      // chatter_pub.publish(msg);
      velocity_pub.publish(vel_msg);
    }

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

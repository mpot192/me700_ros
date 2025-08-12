#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <sstream>
#include <cmath>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <vector>

// I know this is not ideal but im just making it work for now
nav_msgs::Odometry drone_odom;
nav_msgs::Path path;
bool run = false;
bool got_path = false;
int path_sz;

// stuff related to finding crosstrack error
Eigen::Vector3f current_pos(0,0,0);
#define STEP_BACK 5 // how far back to look from current waypoint for closest path point
#define STEP_FWD 10 // same as above except forward

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // ROS_INFO("yo: [%f, %f, %f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  drone_odom = *msg;
  current_pos << drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z;
  run = true;
}
void pathCallback(const nav_msgs::Path::ConstPtr& msg){
  path = *msg;
  path_sz = path.poses.size();
  got_path = true;
}
float GetRMSE(Eigen::Vector3f des_pos){
  // return sqrt((pow(drone_odom.pose.pose.position.x - des_pos[0], 2) + pow(drone_odom.pose.pose.position.y - des_pos.(x), 2) + pow(drone_odom.pose.pose.position.z - des_pos[2], 2))/3);
  return (current_pos - des_pos).norm();
}

float GetCrosstrack(Eigen::Vector3f des_pos, int idx){
  
  float min_d = 99999;
  int min_i = -99999;
  int search_start = STEP_BACK;
  int search_stop = STEP_FWD;

  Eigen::Vector3f check_point(0,0,0); 

  // ROS_INFO("1");
  // find waypoint closest to drone currently within set window
  if(idx < STEP_BACK){
    search_start = idx;
  }
  if(idx + STEP_FWD > path_sz){
    search_stop = path_sz;
  }
  for(int i = -1*search_start; i < search_stop; i++){
    check_point << path.poses[idx + i].pose.position.x, path.poses[idx + i].pose.position.y, path.poses[idx + i].pose.position.z;

    if((current_pos - check_point).norm() < min_d){
      min_d = (current_pos - check_point).norm();
      min_i = i;
    }

  }
  // ROS_INFO("2");
  int offset = 1;
  if(idx + min_i <= 0 || idx + min_i >= path_sz - 1 ){
    offset = 0;
  }
  Eigen::Vector3f back(path.poses[idx + min_i - offset].pose.position.x, path.poses[idx + min_i - offset].pose.position.y, path.poses[idx + min_i - offset].pose.position.z);
  Eigen::Vector3f fwd(path.poses[idx + min_i + offset].pose.position.x, path.poses[idx + min_i + offset].pose.position.y, path.poses[idx + min_i + offset].pose.position.z);
  Eigen::Vector3f pt(path.poses[idx + min_i].pose.position.x, path.poses[idx + min_i].pose.position.y, path.poses[idx + min_i].pose.position.z);
  Eigen::Vector3f start_pt(0,0,0);
  float seg_d;
  // find next closest waypoint to get line segment of interest
  Eigen::Vector3f seg(0,0,0);
  if((current_pos - fwd).norm() < (current_pos - back).norm()){
    // find line segment
    seg = fwd - pt;
    start_pt = pt;
  } else{
    seg = pt - back;
    start_pt = back;
  }
  // ROS_INFO("3");
  // find distance along segment where closest point lies
  seg_d = (current_pos - start_pt).dot(seg) / seg.dot(seg);
  // normalise
  if(seg_d > 1){
    seg_d = 1;
  } else if(seg_d < 0){
    seg_d = 0;
  }
  // ROS_INFO("4");
  // find point where min line meets seg
  Eigen::Vector3f min_pt = start_pt + seg_d * seg;
  // find length of line (crosstrack error)
  return (current_pos - min_pt).norm();
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
  float XTE;

  // Path info
  // const int rows = 3;
  // const int path_sz = 182;
  // Desired path 
  // float current_path[3]; 
  Eigen::Vector3f current_path(0, 0, 0);
  nav_msgs::Path planned;
  geometry_msgs::PoseStamped p;

  std::ofstream logfile;
  logfile.open("/home/matt/catkin_ws/bag/path_follow_pos_log.txt");
  logfile << "pos = {[";
  
  std::ofstream logfile_xte;
  logfile_xte.open("/home/matt/catkin_ws/bag/path_follow_xte_log.txt");
  logfile_xte << "xte = [";

  while (ros::ok()){
    
    // if(run) ROS_INFO("yo: [%f, %f, %f]", drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z);
    if(got_path){
      // Get target point
      // current_path[0] = path.poses[idx].pose.position.x;
      // current_path[1] = path.poses[idx].pose.position.y;
      // current_path[2] = path.poses[idx].pose.position.z;
      current_path <<  path.poses[idx].pose.position.x, path.poses[idx].pose.position.y, path.poses[idx].pose.position.z;
    
      RMSE = GetRMSE(current_path);
      XTE = GetCrosstrack(current_path, idx);
      ROS_INFO("RMSE: %.2f",RMSE);
      ROS_INFO("XTE: %.2f", XTE);
      ROS_INFO("(%d/%d): [%.2f, %.2f, %.2f]", idx, path_sz, current_path.x(), current_path.y(), current_path.z());

      logfile << RMSE;
      if(!std::isnan(XTE)) logfile_xte << XTE; 
      if (idx >= path_sz - 1){
        idx = 0;
        count = 0;
        logfile << "]};" << std::endl << std::endl << "{";
        logfile_xte << "];" << std::endl << std::endl << "[";

      } else if (RMSE <= 0.2 && idx < path_sz){// Move to next index in desired path
          idx++;
          // current_path[0] = path.poses[idx].pose.position.x;
          // current_path[1] = path.poses[idx].pose.position.y;
          // current_path[2] = path.poses[idx].pose.position.z;
          
          current_path <<  path.poses[idx].pose.position.x, path.poses[idx].pose.position.y, path.poses[idx].pose.position.z;
          logfile << "],[";
          if(!std::isnan(XTE)) logfile_xte << ",";
      } else{
          logfile << ",";
          if(!std::isnan(XTE)) logfile_xte << ",";
      }


      // Add time and frame_id to message
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "map";
      
      // Add current path point to message
      msg.pose.position.x = current_path.x();
      msg.pose.position.y = current_path.y();
      msg.pose.position.z = current_path.z();

      // Add desired orientation quarternion to message
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = 0.0; 
      msg.pose.orientation.w = 1.0;

      // Publish message to desired trajectory topic
      chatter_pub.publish(msg);
    }
    for (int i = 0; i < path_sz; i++){
      geometry_msgs::PoseStamped p2;
      p2.header.stamp = ros::Time::now();
      p2.header.frame_id = "map";
      
      p2.pose.position.x = current_path.x();
      p2.pose.position.y = current_path.y();
      p2.pose.position.z = current_path.z();

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
  logfile.close();
  return 0;
}

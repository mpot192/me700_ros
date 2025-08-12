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

// message objects for getting current drone odom and desired path
nav_msgs::Odometry drone_odom;
nav_msgs::Path path;

bool got_path = false; // flag to know if path has been recieved before attempting to do path following
int path_sz; // number of waypoints in path being sent 

// get drone odometry and store current position in vector
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  drone_odom = *msg;
  current_pos << drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z;
}

// get path and store size
void pathCallback(const nav_msgs::Path::ConstPtr& msg){
  path = *msg;
  path_sz = path.poses.size();
  got_path = true;
}

// Calculate RMSE from current position to current desired position waypoint 
float GetRMSE(Eigen::Vector3f des_pos){
  return (current_pos - des_pos).norm();
}

// Calculate cross track error (closest distance to path from current position within seletected window)
float GetCrosstrack(Eigen::Vector3f des_pos, int idx){
  
  float min_d = 99999; // minimum distance to waypoint
  int min_i = -99999; // index of minimum point
  int search_start = STEP_BACK;
  int search_stop = STEP_FWD;

  Eigen::Vector3f check_point(0,0,0); // current waypoint distance is being check against

  // find waypoint closest to drone currently within set window
  // saturate to ensure checking within path limits
  if(idx < STEP_BACK){
    search_start = idx;
  }
  if(idx + STEP_FWD > path_sz){
    search_stop = path_sz;
  }
  // search through window and find waypoint closest to drone
  for(int i = -1*search_start; i < search_stop; i++){
    check_point << path.poses[idx + i].pose.position.x, path.poses[idx + i].pose.position.y, path.poses[idx + i].pose.position.z;

    if((current_pos - check_point).norm() < min_d){
      min_d = (current_pos - check_point).norm();
      min_i = i;
    }
  }

  int offset = 1; // step back/fwd from closest point by 1 to find closest line segment
  // check not out of range
  if(idx + min_i <= 0 || idx + min_i >= path_sz - 1 ){
    offset = 0;
  }
  Eigen::Vector3f back(path.poses[idx + min_i - offset].pose.position.x, path.poses[idx + min_i - offset].pose.position.y, path.poses[idx + min_i - offset].pose.position.z); // point behind closest point
  Eigen::Vector3f fwd(path.poses[idx + min_i + offset].pose.position.x, path.poses[idx + min_i + offset].pose.position.y, path.poses[idx + min_i + offset].pose.position.z); // point in front of closest point
  Eigen::Vector3f pt(path.poses[idx + min_i].pose.position.x, path.poses[idx + min_i].pose.position.y, path.poses[idx + min_i].pose.position.z); // closest point 
  Eigen::Vector3f start_pt(0,0,0); // store point to start line segment at

  float seg_d; // point along line segment closest to drone position (normalised)
  Eigen::Vector3f seg(0,0,0); // line segment

  // find next closest waypoint to get line segment of interest
  if((current_pos - fwd).norm() < (current_pos - back).norm()){
    // find line segment
    seg = fwd - pt;
    start_pt = pt;
  } else{
    seg = pt - back;
    start_pt = back;
  }

  // find distance along segment where closest point lies and normalise 
  seg_d = (current_pos - start_pt).dot(seg) / seg.dot(seg);

  // saturate 
  if(seg_d > 1){
    seg_d = 1;
  } else if(seg_d < 0){
    seg_d = 0;
  }

  // find absolute point where min line meets seg
  Eigen::Vector3f min_pt = start_pt + seg_d * seg;

  // find length of line between drone and closest point (crosstrack error)
  return (current_pos - min_pt).norm();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "follow_rmse_vel");

  ros::NodeHandle n;
  geometry_msgs::PoseStamped msg;
  geometry_msgs::TwistStamped vel_msg;

  // Publish to send target velocity
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);
  // Subscriber to currrent odom (publishes the drones local position by MAVROS)
  ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 1000, odomCallback);
  // Subscriber to generated path
  ros::Subscriber path_sub = n.subscribe("/path_gen/planned_path", 1000, pathCallback);

  // for publishing the actual path followed
  ros::Publisher followed_pub = n.advertise<nav_msgs::Path>("/followed_path", 1000);
  nav_msgs::Path follow;

  // loop at 5 Hz
  ros::Rate loop_rate(5);

  int idx = 0; // current index in path
  float RMSE; // store root mean squared error
  float XTE; // store cross track error

  // for generating velocity command
  float dx;
  float dy;
  float dz;
  
  float vx;
  float vy;
  float vz;
  
  Eigen::Vector3f current_target(0, 0, 0); // current target waypoint
  nav_msgs::Path planned; // store planned path
  geometry_msgs::PoseStamped p; 


  std::ofstream logfile; // logfile for rmse output in matlab cell array format
  logfile.open("/home/matt/catkin_ws/bag/path_follow_pos_log.txt");
  logfile << "pos = {[";
  
  std::ofstream logfile_xte; // logfile for crosstrack error in regular matlab array format
  logfile_xte.open("/home/matt/catkin_ws/bag/path_follow_xte_log.txt");
  logfile_xte << "xte = [";

  while (ros::ok()){
    
    if(got_path){ // if have recieved a path message at least once
      // Get target point
      current_target <<  path.poses[idx].pose.position.x, path.poses[idx].pose.position.y, path.poses[idx].pose.position.z;
    
      // calcualte error from path
      dx = current_target[0] - drone_odom.pose.pose.position.x;
      dy = current_target[1] - drone_odom.pose.pose.position.y;
      dz = current_target[2] - drone_odom.pose.pose.position.z;

      RMSE = GetRMSE(current_target);
      XTE = GetCrosstrack(current_target, idx);
      ROS_INFO("RMSE: %.2f",RMSE);
      ROS_INFO("XTE: %.2f", XTE);
      ROS_INFO("(%d/%d): [%.2f, %.2f, %.2f]", idx, path_sz, current_target.x(), current_target.y(), current_target.z());

logfile << RMSE;
      if(!std::isnan(XTE)) logfile_xte << XTE; // crosstrack can return nan in some cases when too far away, so just in case

      // if reached end of path, just return to start of path and continue
      if (idx >= path_sz - 1){
        idx = 0;
        logfile << "]};" << std::endl << std::endl << "{";
        logfile_xte << "];" << std::endl << std::endl << "[";

      } else if (RMSE <= 0.2 && idx < path_sz){ // Move to next index in desired path
          idx++;
          current_target <<  path.poses[idx].pose.position.x, path.poses[idx].pose.position.y, path.poses[idx].pose.position.z;
          logfile << "],[";
          if(!std::isnan(XTE)) logfile_xte << ",";
      } else{ // else just output data as normal
          logfile << ",";
          if(!std::isnan(XTE)) logfile_xte << ",";
      }

      float norm = sqrt(dx*dx + dy*dy + dz*dz);
      float set_speed = 0.2; // (m/s)
      
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
  logfile.close();
  return 0;

}

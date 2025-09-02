#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <cmath>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <vector>
#include <chrono>

// ---- CROSS TRACK ERROR ----
Eigen::Vector3f current_pos(0,0,0);
#define STEP_BACK 5 // how far back to look from current waypoint for closest path point
#define STEP_FWD 10 // same as above except forward

// ---- CARROT FOLLOWING ----
#define LOOKAHEAD_DIST 0.125            // distance from closest point on path to carrot [m]
#define MAX_CARROT_JUMP 100         // limit single carrot movement to this distance [m]
#define START_THRESHOLD 0.2             // error threshold for knowing if start of path has been reached [m]
#define SEARCH_WIN_START -3             // how far to step back when searching for closest point in path
#define SEARCH_WIN_STOP 3               // ^^ but forward

#define SET_SPEED 0.18                  // speed for general carrot following [m/s]
#define SET_SPEED_NEW_LAYER 0.13        // speed for moving to new layer [m/s]

// message objects for getting current drone odom and desired path
nav_msgs::Odometry drone_odom;
nav_msgs::Path path;

bool got_path = false; // flag to know if path has been recieved before attempting to do path following
int path_sz; // number of waypoints in path being sent
int layer_size;

int nlayers;
bool run = false;
int current_layer_idx = 0;
bool reached_first_point = false;
int col_size;
bool near_end;
int prev_closest_idx = 0;
bool first_carrot_set = false;
bool at_last_point = false;
Eigen::Vector3f prev_carrot(0,0,0);
Eigen::Vector3f velocity;


// get drone odometry and store current position in vector
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  drone_odom = *msg;
  current_pos << drone_odom.pose.pose.position.x, drone_odom.pose.pose.position.y, drone_odom.pose.pose.position.z;
  run = true;
}

// get path and store size
void pathCallback(const nav_msgs::Path::ConstPtr& msg){
  path = *msg;
  path_sz = path.poses.size();
  got_path = true;
}

void nlayerCallback(const std_msgs::Int32::ConstPtr& msg){
    nlayers = msg->data;
}

// Calculate DistErr from current position to current desired position waypoint 
float GetDistErr(Eigen::Vector3f des_pos){
  return (current_pos - des_pos).norm();
}

// calculate distance between two points (basically DistErr)
float Dist3D(float x1, float y1, float z1, float x2, float y2, float z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
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
  search_start = std::min(idx, STEP_BACK);
  search_stop = std::min(idx + STEP_FWD, path_sz);

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

bool FollowCarrotLayer(int current_layer_idx, int layer_size, float set_speed, const Eigen::Vector3f& drone_pos, Eigen::Vector3f& carrot, bool& layer_completed, bool& reached_first_point, int& nearest_waypoint){

    // PHASE 1: Go to first point
    if (!reached_first_point) {
        carrot = Eigen::Vector3f(path.poses[current_layer_idx*layer_size].pose.position.x, path.poses[current_layer_idx*layer_size].pose.position.y, path.poses[current_layer_idx*layer_size].pose.position.z);
        float start_error = (carrot - drone_pos).norm();

        if (start_error< START_THRESHOLD) { 
            reached_first_point = true;
            ROS_INFO("Reached first waypoint of layer. Switching to carrot-following.");
        }

        layer_completed = false;

        // Initialize carrot memory
        if (!first_carrot_set) {
            prev_carrot = carrot;
            first_carrot_set = true;
        }

        prev_closest_idx = 0; // Reset index when starting the layer

        return true;
    }

    // PHASE 2: Arc-length carrot following with interpolation
    else {

        // 1. Find closest point on path within set window
        int search_start = std::max(0, prev_closest_idx + SEARCH_WIN_START);   // Allow small backward correction
        int search_end   = std::min(layer_size - 1, prev_closest_idx + SEARCH_WIN_STOP); // Lookahead window 
        float min_dist = 10000; // Just a large number for now
        int closest_idx = 0;
        for (int i = search_start; i < search_end; ++i) {
            // Finding distance between drone and waypoints
            float dist = Dist3D(
                drone_odom.pose.pose.position.x,
                drone_odom.pose.pose.position.y,
                drone_odom.pose.pose.position.z,
                path.poses[current_layer_idx*layer_size + i].pose.position.x,
                path.poses[current_layer_idx*layer_size + i].pose.position.y, 
                path.poses[current_layer_idx*layer_size + i].pose.position.z
            );

            // Finding the index of the point thats closest to the drone
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
                nearest_waypoint = current_layer_idx*layer_size + closest_idx;
            }
        }
        prev_closest_idx = closest_idx; // Update memory for next iteration

        // ROS_INFO_THROTTLE(2,"Closest point: [%.2f, %.2f, %.2f]", path.poses[current_layer_idx*layer_size + closest_idx].pose.position.x, path.poses[current_layer_idx*layer_size + closest_idx].pose.position.y, path.poses[current_layer_idx*layer_size + closest_idx].pose.position.z);

        // 2. Walk forward along path to find carrot using interpolation
        float remaining_lookahead = LOOKAHEAD_DIST;
        int j = closest_idx;

        while (j < layer_size - 1) {
            Eigen::Vector3f p1(path.poses[current_layer_idx*layer_size + j].pose.position.x,
                    path.poses[current_layer_idx*layer_size + j].pose.position.y,
                    path.poses[current_layer_idx*layer_size + j].pose.position.z);
            Eigen::Vector3f p2(path.poses[current_layer_idx*layer_size + j + 1].pose.position.x,
                    path.poses[current_layer_idx*layer_size + j + 1].pose.position.y,
                    path.poses[current_layer_idx*layer_size + j + 1].pose.position.z);

            // draws a line between two points 
            float segment_len = (p2 - p1).norm();

            if (segment_len >= remaining_lookahead) {
                // Calculates how far along the line the carrot is if the lookahead intersects the path between two points
                float t = remaining_lookahead / segment_len;

                // Placing the carrot there
                carrot = p1 + t * (p2 - p1);
                // ROS_INFO("Moving carrot by %f", t);
                layer_completed = false;
                break;

            } else {
                remaining_lookahead -= segment_len;
                j++; 
            }
        }

        // Force carrot to last point if we are beyond the last segment
        Eigen::Vector3f last_point(path.poses[current_layer_idx*layer_size + layer_size - 1].pose.position.x,
                                path.poses[current_layer_idx*layer_size + layer_size - 1].pose.position.y,
                                path.poses[current_layer_idx*layer_size + layer_size - 1].pose.position.z);
        if (j >= layer_size - 1) {
            carrot = last_point;
            at_last_point = true;
        }

        if (!first_carrot_set) {
            prev_carrot = carrot;
            first_carrot_set = true;
        }else{

            // Memory element
            float jump_dist = (carrot - prev_carrot).norm();
            // ROS_INFO_THROTTLE(1.0, "Carrot in same Pos - Jump dist too high!");
            // if Carrot jumps too much:
            if (jump_dist > MAX_CARROT_JUMP){
                // Keep carrot in same position
                carrot = prev_carrot;
                ROS_INFO_THROTTLE(1.0, "Carrot in same Pos - Jump dist too high!");
            }
        }
        prev_carrot = carrot; // Store current carrot into prev for next iteration

        // Check if near the last point AND near the end of the path
        float dist_to_last_point = (last_point - drone_pos).norm();

        // Condition 1: We're very close to the last waypoint
        bool close_to_last_point = (dist_to_last_point < 1*set_speed); //vary with set speed to avoid getting stuck at final point
        // Condition 2: We've progressed far along the path
        if (j >= layer_size - 2){
            near_end = true;
        }

        // Final check
        layer_completed = (near_end && close_to_last_point);

        return true;
    }
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "follow_path");
    ros::NodeHandle n;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pos_msg;
    geometry_msgs::TwistStamped vel_msg;
    nav_msgs::Path follow;

    // publish target position
    ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
    // publish target velocity
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);
    // publish actual flown path
    ros::Publisher followed_pub = n.advertise<nav_msgs::Path>("/followed_path", 1000);
    // subscribe to planned path
    ros::Subscriber path_sub = n.subscribe("/path_gen/planned_path", 1000, pathCallback);
    ros::Subscriber nlayer_sub = n.subscribe("/path_gen/layer_count", 1000, nlayerCallback);
    // subscribe to currrent odom
    ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 1000, odomCallback);
    // Publisher to send position of carrot
    ros::Publisher carrot_marker_pub = n.advertise<visualization_msgs::Marker>("carrot_marker", 1);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";

    // loop at 5 Hz
    ros::Rate loop_rate(5);

    // Carrot Following Parameters
    bool reached_first_point = false;
    int num_of_layers = 0;
    int current_layer = 0;

    // DistErr Following
    int idx = 0; // current index in path
    float distErr; // store root mean squared error
    float XTE; // store cross track error
    
    // for generating velocity command
    float dx;
    float dy;
    float dz;
    
    float vx;
    float vy;
    float vz;

    float dist_err_thresh;
    float set_speed;
    float variable_speed;
    bool complete_carrot = false;
    bool complete_path = false;
    bool got_start = false;
    Eigen::Vector3f current_target(0, 0, 0); // current target waypoint
    nav_msgs::Path planned; // store planned path
    geometry_msgs::PoseStamped p; 



    if (argc < 2){
        ROS_ERROR("Missing required command line arguments <command_mode> (pos/vel/carrot) <thresh> <speed (if required)>");
        ros::shutdown();
        return -1;
    }

    std::string follow_mode = argv[1];
    
    if (follow_mode not_eq "vel" and follow_mode not_eq "pos" and follow_mode not_eq "carrot"){ // never knew you could write logical ops like this
        ROS_ERROR("Missing required command line arguments <command_mode> (pos/vel/carrot) <<thresh>> <<speed>>");
        ros::shutdown();
        return  -1;
    }

    if(follow_mode == "pos" || follow_mode == "vel"){
        dist_err_thresh = std::stof(argv[2]);
    } 
    if(follow_mode == "vel"){
        set_speed = std::stof(argv[3]);
    }
    if(follow_mode == "carrot"){
        set_speed = std::stof(argv[2]);
    }


    // LOG FILE INIT
    auto now = std::chrono::system_clock::now();
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()
               ).count();

    std::string filename_dist_err =  "/home/matt/catkin_ws/bag/path_follow_dist_err_log_" + std::to_string(ms) + ".txt";
    std::string filename_xte = "/home/matt/catkin_ws/bag/path_follow_xte_log_" + std::to_string(ms) + ".txt";

    std::ofstream logfile_dist_err; // logfile for dist_err output in matlab cell array format
    logfile_dist_err.open(filename_dist_err);
    
    std::ofstream logfile_xte; // logfile for crosstrack error in regular matlab array format
    logfile_xte.open(filename_xte);

    if(follow_mode == "pos"){
        logfile_dist_err << "dist_err_pos = {[";
        logfile_xte << "xte_pos = [";
    } else if(follow_mode == "vel"){
        logfile_dist_err << "dist_err_vel = {[";
        logfile_xte << "xte_vel = [";
    } else if(follow_mode == "carrot"){
        logfile_dist_err << "dist_err_carrot = {[";
        logfile_xte << "xte_carrot = [";
    }

    ros::Time follow_start;

    while(ros::ok()){

        if(got_path){
            if(!got_start){
                follow_start = ros::Time::now();
                got_start = true;
            }
            
            layer_size = path_sz/nlayers;

            // Get target point
            current_target <<  path.poses[idx].pose.position.x, path.poses[idx].pose.position.y, path.poses[idx].pose.position.z;
            
            // calcualte error from path
            dx = current_target.x() - drone_odom.pose.pose.position.x;
            dy = current_target.y() - drone_odom.pose.pose.position.y;
            dz = current_target.z() - drone_odom.pose.pose.position.z;

            distErr = GetDistErr(current_target);
            XTE = GetCrosstrack(current_target, idx);
            if(!complete_path){
                ROS_INFO_THROTTLE(1,"DistErr: %.2f",distErr);
                ROS_INFO_THROTTLE(1,"XTE: %.2f", XTE);
                ROS_INFO_THROTTLE(1,"(%d/%d): [%.2f, %.2f, %.2f]", idx, path_sz, current_target.x(), current_target.y(), current_target.z());
            }


            if(!complete_path) {
                logfile_dist_err << distErr;
                if(!std::isnan(XTE)) logfile_xte << XTE; // crosstrack can return nan in some cases when too far away, so just in case
            }

            // if reached end of path, just return to start of path and continue
            if (idx >= path_sz - 1 || complete_carrot){
                if(!complete_path){
                    ros::Time follow_end = ros::Time::now();
                    ros::Duration elapsed = follow_end - follow_start;
                    logfile_dist_err << "]};" << std::endl << "time = " << elapsed.toSec() << "s" << std::endl;
                    logfile_dist_err << "size =  " << path_sz << std::endl;
                    logfile_xte << "];" << std::endl<< "time = " << elapsed.toSec() << "s" << std::endl;

                    if(follow_mode == "pos"){
                        logfile_dist_err << "thresh = " << dist_err_thresh << std::endl;
                        logfile_xte << "thresh = " << dist_err_thresh << std::endl;
                    } else if(follow_mode == "vel"){
                        logfile_dist_err << "thresh = " << dist_err_thresh << std::endl;
                        logfile_dist_err << "set_speed = " << set_speed << std::endl;
                        logfile_xte << "thresh = " << dist_err_thresh << std::endl;
                        logfile_xte << "set_speed = " << set_speed << std::endl;
                    } else if(follow_mode == "carrot"){
                        logfile_dist_err << "set_speed = " << set_speed << std::endl;
                        logfile_xte << "set_speed = " << set_speed << std::endl;
                    }
                    

                    complete_path = true;
                }

            } else if (distErr <= dist_err_thresh & idx < path_sz){ // Move to next index in desired path
                idx++;
                current_target <<  path.poses[idx].pose.position.x, path.poses[idx].pose.position.y, path.poses[idx].pose.position.z;
                logfile_dist_err << "],[";
                if(!std::isnan(XTE)) logfile_xte << ",";
            } else{ // else just output data as normal
                logfile_dist_err << ",";
                if(!std::isnan(XTE)) logfile_xte << ",";
            }
            if(complete_path){
                // hold position above path after
                // Add time and frame_id to message
                pos_msg.header.stamp = ros::Time::now();
                pos_msg.header.frame_id = "map";
                    
                // Add current path point to message
                pos_msg.pose.position.x = path.poses[0].pose.position.x;
                pos_msg.pose.position.y = path.poses[0].pose.position.y;
                pos_msg.pose.position.z = path.poses[0].pose.position.z + 3;

                // Add desired orientation quarternion to message
                pos_msg.pose.orientation.x = 0.0;
                pos_msg.pose.orientation.y = 0.0;
                pos_msg.pose.orientation.z = 0.0; 
                pos_msg.pose.orientation.w = 1.0;

                // Publish message to desired trajectory topic
                pos_pub.publish(pos_msg);
            }else{
                if(follow_mode == "pos"){
                    // position
                    // Add time and frame_id to message
                    pos_msg.header.stamp = ros::Time::now();
                    pos_msg.header.frame_id = "map";
                    
                    // Add current path point to message
                    pos_msg.pose.position.x = current_target.x();
                    pos_msg.pose.position.y = current_target.y();
                    pos_msg.pose.position.z = current_target.z();

                    // Add desired orientation quarternion to message
                    pos_msg.pose.orientation.x = 0.0;
                    pos_msg.pose.orientation.y = 0.0;
                    pos_msg.pose.orientation.z = 0.0; 
                    pos_msg.pose.orientation.w = 1.0;

                    // Publish message to desired trajectory topic
                    pos_pub.publish(pos_msg);
                } else if(follow_mode == "vel"){
                    // velocity
                    float norm = sqrt(dx*dx + dy*dy + dz*dz);
                    // float set_speed = SET_VEL; // (m/s)
                    
                    // Uses the drones current position and setpoint to calculate the velocity vector but scaled to set_speed
                    velocity.x() = (dx/norm) * set_speed;
                    velocity.y() = (dy/norm) * set_speed;
                    velocity.z() = (dz/norm) * set_speed;

                } else if (follow_mode == "carrot"){
                    Eigen::Vector3f carrot(0,0,0);
                    bool layer_complete = false;
                    

                    Eigen::Vector3f drone_pos(
                        drone_odom.pose.pose.position.x,
                        drone_odom.pose.pose.position.y,
                        drone_odom.pose.pose.position.z
                    );

                    if (current_layer_idx < nlayers){
                        FollowCarrotLayer(current_layer_idx,
                            layer_size,
                            set_speed,
                            drone_pos, carrot,
                            layer_complete,
                            reached_first_point, idx);

                        if (layer_complete == true){
                            if (current_layer_idx < nlayers-1){
                                current_layer_idx++;
                                reached_first_point = false;
                                near_end = false; 
                                first_carrot_set = false;  // Reset memory for new layer
                                ROS_INFO("Switching to next layer: %d", current_layer_idx);
                            }else{
                                ROS_INFO("All layers completed.");
                                complete_carrot = true;
                                // Just hovering now
                            }

                        }
                    }

                    // Display carrot!!!
                    marker.header.stamp = ros::Time::now();
                    // Need to define namespaace and id so that carrot can be replaced
                    marker.ns = "carrot";
                    marker.id = 0;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    // Tells RViz to update marker
                    marker.action = visualization_msgs::Marker::ADD;

                    marker.pose.position.x = carrot.x();
                    marker.pose.position.y = carrot.y();
                    marker.pose.position.z = carrot.z();
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;

                    marker.scale.x = 0.025;
                    marker.scale.y = 0.025;
                    marker.scale.z = 0.025;

                    marker.color.r = 1.0f;
                    marker.color.g = 0.5f;
                    marker.color.b = 0.0f;
                    marker.color.a = 1.0f;

                    carrot_marker_pub.publish(marker);

                    if(!complete_carrot){
                        // === VELOCITY COMMAND ===
                        Eigen::Vector3f direction_to_carrot = carrot - drone_pos;
                        float dist_to_carrot = direction_to_carrot.norm();

                        ROS_INFO_THROTTLE(1,"(%d/%d) Carrot position: [%.2f, %.2f, %.2f]", current_layer_idx, nlayers - 1, carrot.x(), carrot.y(), carrot.z());
                        
                        if(dist_to_carrot > 0.01){ // balls 0.1 0.01
                            variable_speed = std::min(set_speed, (float)((set_speed/2) + (set_speed/2)*(dist_to_carrot/LOOKAHEAD_DIST - LOOKAHEAD_DIST))); // vary speed between set_speed/2 and set_speed
                            // normalized gets the unit vector
                            // could set speed to be different for moving to first point in layer if desired
                            if (!reached_first_point){
                                velocity = direction_to_carrot.normalized() * set_speed;
                            }else{
                                velocity = direction_to_carrot.normalized() * set_speed; // balls
                            }
                        } else{
                            velocity = Eigen::Vector3f(0, 0, 0);
                            // If close enough, set velocity to 0 - realistically shouldnt happen but
                        }
                        ROS_INFO_THROTTLE(1,"v: (%f) [%f, %f, %f]", variable_speed, velocity.x(),velocity.y(), velocity.z());
                    }

                }

                if(follow_mode == "vel" || follow_mode == "carrot"){
                    // Fill and publish velocity message
                    vel_msg.header.stamp = ros::Time::now();
                    vel_msg.header.frame_id = "map";
                    vel_msg.twist.linear.x = velocity.x();
                    vel_msg.twist.linear.y = velocity.y();
                    vel_msg.twist.linear.z = velocity.z();
                    vel_msg.twist.angular.x = 0.0;
                    vel_msg.twist.angular.y = 0.0;
                    vel_msg.twist.angular.z = 0.0;

                    velocity_pub.publish(vel_msg);
                }
            }
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
    }
    logfile_dist_err.close();
    logfile_xte.close();
    return 0;

}
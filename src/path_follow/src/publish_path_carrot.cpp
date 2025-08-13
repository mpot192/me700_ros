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
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <vector>


#define LOOKAHEAD_DIST 0.125            // distance from closest point on path to carrot [m]
#define MAX_CARROT_JUMP 0.225           // limit single carrot movement to this distance [m]
#define START_THRESHOLD 0.2             // error threshold for knowing if start of path has been reached [m]
#define SEARCH_WIN_START -3             // how far to step back when searching for closest point in path
#define SEARCH_WIN_STOP 3               // ^^ but forward

#define SET_SPEED 0.18                  // speed for general carrot following [m/s]
#define SET_SPEED_NEW_LAYER 0.13        // speed for moving to new layer [m/s]

nav_msgs::Odometry drone_odom;
nav_msgs::Path path;

bool got_path = false;                  // flag to know if path has been recieved before attempting to do path following
int path_sz;                            // number of waypoints in path being sent
int nlayers;
bool run = false;
int current_layer_idx = 0;
bool reached_first_point = false;
int col_size;
bool near_end;
int prev_closest_idx = 0;
bool first_carrot_set = false;
Eigen::Vector3f prev_carrot(0,0,0);

// for getting drone position
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    drone_odom = *msg;
    run = true;
}

// calculate root mean squared error between target point and drone position
float GetRMSE(float des_pos[3]){
  return sqrt((pow(drone_odom.pose.pose.position.x - des_pos[0], 2) + pow(drone_odom.pose.pose.position.y - des_pos[1], 2) + pow(drone_odom.pose.pose.position.z - des_pos[2], 2))/3);
}

// calculate distance between two points (basically rmse)
float Dist3D(float x1, float y1, float z1, float x2, float y2, float z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
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

bool FollowCarrotLayer(int current_layer_idx, int layer_size, const Eigen::Vector3f& drone_pos, Eigen::Vector3f& carrot, bool& layer_completed, bool& reached_first_point){

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
        int search_start = std::max(0, prev_closest_idx - SEARCH_WIN_START);   // Allow small backward correction
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
            }
        }
        prev_closest_idx = closest_idx; // Update memory for next iteration

        ROS_INFO_THROTTLE(1,"Closest point: [%.2f, %.2f, %.2f]", path.poses[current_layer_idx*layer_size + closest_idx].pose.position.x, path.poses[current_layer_idx*layer_size + closest_idx].pose.position.y, path.poses[current_layer_idx*layer_size + closest_idx].pose.position.z);

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
        }

        if (!first_carrot_set) {
            prev_carrot = carrot;
            first_carrot_set = true;
        }else{

            // Memory element
            float jump_dist = (carrot - prev_carrot).norm();
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
        bool close_to_last_point = (dist_to_last_point < 0.05f); // 0.05 hi
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "carrot_follower");
    ros::NodeHandle n;

    geometry_msgs::TwistStamped vel_msg;
    geometry_msgs::PoseStamped pose;

    // Publisher to send target position
    ros::Publisher velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);

    // Subscriber to current odom for drone position
    ros::Subscriber pos_sub = n.subscribe("/mavros/global_position/local", 1000, odomCallback);
    ros::Rate loop_rate(5);

    // Publisher to send planned path (to RViz)
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/planned_path", 1, true);
    nav_msgs::Path planned_path;
    planned_path.header.frame_id = "map";
    
    // Publisher to send actual drone positions to RViz
    ros::Publisher actual_path_pub = n.advertise<nav_msgs::Path>("/actual_path", 10);
    nav_msgs::Path actual_path;
    actual_path.header.frame_id = "map";

    // Publisher to send position of carrot
    ros::Publisher carrot_marker_pub = n.advertise<visualization_msgs::Marker>("carrot_marker", 1);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";

    // Subscribers for path and layer count
    ros::Subscriber path_sub = n.subscribe("/path_gen/planned_path", 1000, pathCallback);
    ros::Subscriber nlayer_sub = n.subscribe("/path_gen/layer_count", 1000, nlayerCallback);
    // Parameters
    bool reached_first_point = false;
    int num_of_layers = 0;
    int current_layer = 0;

    while(!got_path){
        ros::spinOnce();
        loop_rate.sleep();
    }

    int layer_size = path_sz/nlayers;

    while (ros::ok()) {
        if (!run) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        Eigen::Vector3f drone_pos(
            drone_odom.pose.pose.position.x,
            drone_odom.pose.pose.position.y,
            drone_odom.pose.pose.position.z
        );

        // Publishing current drone positions to topic
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id = "map";
        current_pose.pose.position.x = drone_pos.x();
        current_pose.pose.position.y = drone_pos.y();
        current_pose.pose.position.z = drone_pos.z();

        current_pose.pose.orientation.x = 0.0;
        current_pose.pose.orientation.y = 0.0;
        current_pose.pose.orientation.z = 0.0;
        current_pose.pose.orientation.w = 1.0;
        actual_path.poses.push_back(current_pose);
        actual_path.header.frame_id = "map";
        actual_path.header.stamp = ros::Time::now();
        actual_path_pub.publish(actual_path);

        Eigen::Vector3f carrot(0,0,0);
        bool layer_complete = false;
        Eigen::Vector3f velocity;

        if (current_layer_idx < nlayers){
            FollowCarrotLayer(current_layer_idx,
                layer_size,
                drone_pos, carrot,
                layer_complete,
                reached_first_point);

            if (layer_complete == true){
                if (current_layer_idx < nlayers){
                    current_layer_idx++;
                    reached_first_point = false;
                    near_end = false; 
                    first_carrot_set = false;  // Reset memory for new layer
                    ROS_INFO("Switching to next layer: %d", current_layer_idx);
                }else{
                    ROS_INFO("All layers completed.");
                    carrot = drone_pos;
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

        // === VELOCITY COMMAND ===
        Eigen::Vector3f direction_to_carrot = carrot - drone_pos;
        float dist_to_carrot = direction_to_carrot.norm();

        ROS_INFO_THROTTLE(1,"Carrot position: [%.2f, %.2f, %.2f]", carrot.x(), carrot.y(), carrot.z());

        if(dist_to_carrot > 0.01){ // balls 0.1 0.01
            // normalized gets the unit vector
            if (!reached_first_point){
                velocity = direction_to_carrot.normalized() * SET_SPEED_NEW_LAYER;
            }else{
                velocity = direction_to_carrot.normalized() * SET_SPEED; // balls
            }
        } else{
            velocity = Eigen::Vector3f(0, 0, 0);
            // If close enough, set velocity to 0 - realistically shouldnt happen but
        }
        // Stop if near end
        if (dist_to_carrot < 0.05 && reached_first_point) { // 0.05 balls 0.005
            velocity = Eigen::Vector3f(0, 0, 0);
            // ROS_INFO("Reached final target. Hovering.");
            ROS_INFO_THROTTLE(1.0, "Reached final target. Hovering.");
        }

        // Fill and publish velocity message
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "map";
        vel_msg.twist.linear.x = velocity[0];
        vel_msg.twist.linear.y = velocity[1];
        vel_msg.twist.linear.z = velocity[2];
        vel_msg.twist.angular.x = 0.0;
        vel_msg.twist.angular.y = 0.0;
        vel_msg.twist.angular.z = 0.0;

        velocity_pub.publish(vel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
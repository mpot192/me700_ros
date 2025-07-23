include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <vector>

nav_msgs::Odometry drone_odom;
bool run = false;
float lookahead_distance = 0.3f; // meters
float max_carrot_jump = 0.325f; // Max carrot movement per iteration (meters)
int current_layer_idx = 0;
bool reached_first_point = false;
int col_size;
bool near_end;
int last_closest_idx = 0;

bool first_carrot_set = false;
Eigen::Vector3f prev_carrot(0,0,0);

float dist3D(float x1, float y1, float z1, float x2, float y2, float z2);

// enum DroneNavState{ IDLE, FOLLOW_LAYER, NEXT_LAYER, END};
// DroneNavState current_state = IDLE;

bool FollowCarrotLayer(const double path_layer[3][182], int layer_size, const Eigen::Vector3f& drone_pos, Eigen::Vector3f& carrot, bool& layer_completed, bool& reached_first_point){
        

    // Eigen::Vector3f prev_carrot(0, 0, 0);
    // bool first_carrot_set = false;
    

    // PHASE 1: Go to first point
    if (!reached_first_point) {
        carrot = Eigen::Vector3f(path_layer[0][0], path_layer[1][0], path_layer[2][0]);
        float dist = (carrot - drone_pos).norm();

        if (dist < 0.15) {
            reached_first_point = true;
            ROS_INFO("Reached first waypoint of layer. Switching to carrot-following.");
        }

        layer_completed = false;

        // Initialize carrot memory
        if (!first_carrot_set) {
            prev_carrot = carrot;
            first_carrot_set = true;
        }

        last_closest_idx = 0; // Reset index when starting the layer ///////////////

        return true;
    }

        // PHASE 2: Arc-length carrot following with interpolation
    else {

        // 1. Find closest point on path (but it is limited with a window MAX 5 index jump)
        int search_start = std::max(0, last_closest_idx - 5);   // Allow small backward correction
        int search_end   = std::min(layer_size - 1, last_closest_idx + 20); // Lookahead window

        float min_dist = 10000; // Just a large number for now
        int closest_idx = 0;

        for (int i = search_start; i < search_end; ++i) {
        // for (int i = 0; i < layer_size; ++i) {

            // Finding distance between drone and waypoints
            float dist = dist3D(
                drone_odom.pose.pose.position.x,
                drone_odom.pose.pose.position.y,
                drone_odom.pose.pose.position.z,
                path_layer[0][i], path_layer[1][i], path_layer[2][i]
            );

            // Finding the index of the point thats closest to the drone
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        last_closest_idx = closest_idx; // Update memory for next iteration

        ROS_INFO_THROTTLE(1,"Closest point: [%.2f, %.2f, %.2f]", path_layer[0][closest_idx], path_layer[1][closest_idx], path_layer[2][closest_idx]);

        // 2. Walk forward along path to find carrot using interpolation
        float remaining_lookahead = lookahead_distance;
        int j = closest_idx;

        while (j < layer_size - 1) {
            Eigen::Vector3f p1(path_layer[0][j], path_layer[1][j], path_layer[2][j]);
            Eigen::Vector3f p2(path_layer[0][j+1], path_layer[1][j+1], path_layer[2][j+1]);

            // draws a linear line between two points 
            float segment_len = (p2 - p1).norm();

            if (segment_len >= remaining_lookahead) {
                // Calculates how far along the line the carrot is ff the lookahead intersects the path between two points
                float t = remaining_lookahead / segment_len;

                // Placing the carrot there
                carrot = p1 + t * (p2 - p1);
                layer_completed = false;
                break;

            } else {
                remaining_lookahead -= segment_len;
                j++; // ++i;
            }
        }

        // Force carrot to last point if we are beyond the last segment
        Eigen::Vector3f last_point(path_layer[0][layer_size - 1],
                                path_layer[1][layer_size - 1],
                                path_layer[2][layer_size - 1]);
        if (j >= layer_size - 1) {
            carrot = last_point;
        }

        // // If the closest point is at the last point, just place the carrot there
        // if (j >= layer_size - 1) { //cols
        //     carrot = Eigen::Vector3f(path_layer[0][layer_size  - 1], path_layer[1][layer_size  - 1], path_layer[2][layer_size  - 1]);
        // }

        if (!first_carrot_set) {
            prev_carrot = carrot;
            first_carrot_set = true;
        }else{

            // Memory element
            float jump_dist = (carrot - prev_carrot).norm();
            // if Carrot jumps too much:
            if (jump_dist > max_carrot_jump){
                // Keep carrot in same position
                carrot = prev_carrot;
            }
        }
        prev_carrot = carrot; // Store current carrot into prev for next iteration

        // Check if near the last point AND near the end of the path
        float dist_to_last_point = (last_point - drone_pos).norm();

        // Condition 1: We're very close to the last waypoint
        bool close_to_last_point = (dist_to_last_point < 0.10f);

        // Condition 2: We've progressed far along the path
        if (j >= layer_size - 2){
            near_end = true;
        }
        // near_end = (j >= layer_size - 2 || closest_idx > layer_size * 0.8);

        // Final check
        layer_completed = (near_end && close_to_last_point);

        return true;

        // Check if layer is complete
        // float dist_to_last_point = (carrot - drone_pos).norm(); // carrot   

        // float dist_to_last_point = (last_point - drone_pos).norm();

        // if (dist_to_last_point < 0.10){
        //     layer_completed = true;
        // }else {
        //     layer_completed = false;
        // }
        // return true;
    }
    return true;
}



void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    drone_odom = *msg;
    run = true;
}

float GetRMSE(float des_pos[3]){
  return sqrt((pow(drone_odom.pose.pose.position.x - des_pos[0], 2) + pow(drone_odom.pose.pose.position.y - des_pos[1], 2) + pow(drone_odom.pose.pose.position.z - des_pos[2], 2))/3);
}

float dist3D(float x1, float y1, float z1, float x2, float y2, float z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
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

    // Parameters
    // float lookahead_distance = 0.6f; // meters
    float set_speed = 0.25;  // m/s
    bool reached_first_point = false;

    // Path setup
    const int rows = 3;
    const int cols = 182;
    col_size = cols;
    // Path info
    // [x1 x2 .. ]
    // [y1 y2 .. ]
    // [z1 z2 .. ]]
    double path[rows][cols] = {{3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.12041, 3.22457, 3.29839, 3.33191, 3.32061, 3.26601, 3.17548, 3.06125, 2.93875, 2.82452, 2.73399, 2.67939, 2.66809, 2.70161, 2.77543, 2.87959, 3, 3, 3.06021, 3.11228, 3.14919, 3.16596, 3.1603, 3.133, 3.08774, 3.03062, 2.96938, 2.91226, 2.867, 2.8397, 2.83404, 2.85081, 2.88772, 2.93979, 3, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3}
    ,{3, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.33333, 3.31082, 3.24634, 3.14858, 3.03076, 2.90878, 2.79912, 2.71659, 2.67234, 2.67234, 2.71659, 2.79912, 2.90878, 3.03076, 3.14858, 3.24634, 3.31082, 3.33333, 3.16667, 3.15541, 3.12317, 3.07429, 3.01538, 2.95439, 2.89956, 2.8583, 2.83617, 2.83617, 2.8583, 2.89956, 2.95439, 3.01538, 3.07429, 3.12317, 3.15541, 3.16667, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5}
    ,{3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25}};

    int num_of_layers = 0;
    int current_layer = 0;
    int layer_index[cols]; // Stores the index of the first value of a new layer
    double unique_z[cols]; // Worst case

    for (int i = 0; i < cols; i++){
        double current_z = path[2][i];
        bool found = false;

        for (int j = 0 ; j < num_of_layers; j++){
            if(fabs(unique_z[j] - current_z) < 0.000001){
                found = true;
                break;
            }
        }

        if (!found){
            unique_z[num_of_layers] = current_z;
            num_of_layers++;
        }
    }

    double path_layer[num_of_layers][rows][cols];
    int layer_sizes[100] = {0}; // Max 100 numbers in 1 layer

    for (int i = 0; i < cols; i++){
        double current_z = path[2][i];
        int layer_idx = -1;

        for (int j = 0; j < num_of_layers; j++){
            if(fabs(unique_z[j] - current_z) < 0.000001){
                layer_idx = j;
                break;
            }
        }

        if (layer_idx != -1 && layer_sizes[layer_idx] < 100){ // MAX_POINTS_PER_LAYER
            int pos = layer_sizes[layer_idx];
            path_layer[layer_idx][0][pos] = path[0][i];  // X
            path_layer[layer_idx][1][pos] = path[1][i];  // Y
            path_layer[layer_idx][2][pos] = path[2][i];  // Z
            layer_sizes[layer_idx]++;
        }
    }

    // Print results
    for (int i = 0; i < num_of_layers; i++) {
        printf("Layer %d (Z = %.2f) has %d points:\n", i+1, unique_z[i], layer_sizes[i]);
        for (int j = 0; j < layer_sizes[i]; j++) {
            printf("(%f, %f, %f)\n", 
                   path_layer[i][0][j], 
                   path_layer[i][1][j], 
                   path_layer[i][2][j]);
        }
        printf("--------\n");
    }


    // Publish the planned path to RViz (only need to do once)
    // nav_msgs::Path planned_path;
    // planned_path.header.frame_id = "map";
    for (int i = 0; i < cols; ++i) {
        // geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = path[0][i];
        pose.pose.position.y = path[1][i];
        pose.pose.position.z = path[2][i];
        pose.pose.orientation.w = 1.0;
        planned_path.poses.push_back(pose);
    }
    // Publishing planned_path positions to topic
    path_pub.publish(planned_path);

    // To accumulate drone's actual path
    // nav_msgs::Path actual_path;
    // actual_path.header.frame_id = "map";


    while (ros::ok()) {
        if (!run) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        // ROS_INFO_THROTTLE(0.75,"Drone_pos: [%f, %f, %f]", 
        //     drone_odom.pose.pose.position.x, 
        //     drone_odom.pose.pose.position.y, 
        //     drone_odom.pose.pose.position.z
        // );

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

        // current_pose.pose.orientation.x = 0.0;
        // current_pose.pose.orientation.y = 0.0;
        // current_pose.pose.orientation.z = 0.0;
        current_pose.pose.orientation.w = 1.0;
        actual_path.poses.push_back(current_pose);
        // if (actual_path.poses.size() > 1000) actual_path.poses.erase(actual_path.poses.begin());
        actual_path_pub.publish(actual_path);



        Eigen::Vector3f carrot(0,0,0);
        bool layer_complete = false;
        Eigen::Vector3f velocity;

        if (current_layer_idx < num_of_layers){
            FollowCarrotLayer(path_layer[current_layer_idx],
                layer_sizes[current_layer_idx],
                drone_pos, carrot,
                layer_complete,
                reached_first_point);

            if (layer_complete == true){
                if (current_layer_idx < num_of_layers){
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

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        carrot_marker_pub.publish(marker);



        // === VELOCITY COMMAND ===
        Eigen::Vector3f direction_to_carrot = carrot - drone_pos;
        float dist_to_carrot = direction_to_carrot.norm();

        ROS_INFO_THROTTLE(1,"Carrot position: [%.2f, %.2f, %.2f]", carrot.x(), carrot.y(), carrot.z());

        if(dist_to_carrot > 0.01){
            // normalized gets the unit vector
            velocity = direction_to_carrot.normalized() * set_speed;
        } else{
            velocity = Eigen::Vector3f(0, 0, 0);
            // If close enough, set velocity to 0 - realistically shouldnt happen but
        }
        // Stop if near end
        if (dist_to_carrot < 0.05 && reached_first_point) {
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

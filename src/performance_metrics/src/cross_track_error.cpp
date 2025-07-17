#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

// Globals
Eigen::Vector3d drone_pos(0, 0, 0);
Eigen::Vector3d carrot_pos(0, 0, 0);
bool drone_received = false;
bool carrot_received = false;

double lookahead_distance = 0.3; // Adjust based on controller
std::ofstream log_file;

void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  // Store as a vector instead of msg format
  drone_pos = Eigen::Vector3d(msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z);
  drone_received = true;
}

void carrotCallback(const visualization_msgs::Marker::ConstPtr& carrot) { //msg
  carrot_pos = Eigen::Vector3d(carrot->pose.position.x,
                               carrot->pose.position.y,
                               carrot->pose.position.z);
  carrot_received = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_carrot_distance_node");
    ros::NodeHandle nh;

    // Open CSV file for logging
    log_file.open("/tmp/drone_carrot_distance.csv");
    log_file << "time,distance_error,approx_cross_track_error\n";

    // Subscribers
    ros::Subscriber drone_sub = nh.subscribe("/mavros/local_position/pose", 10, dronePoseCallback);
    ros::Subscriber carrot_sub = nh.subscribe("/carrot_marker", 10, carrotCallback);

    // Publishers
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float64>("distance_to_carrot", 10);
    ros::Publisher approx_xte_pub = nh.advertise<std_msgs::Float64>("approx_cross_track_error", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        if (drone_received && carrot_received) {
            double distance_error = (drone_pos - carrot_pos).norm();

            // Approximate Cross Track Error (just the difference between carrot and drone minus lookahead)
            double approx_xte = distance_error - lookahead_distance;

            // Publish distance error
            std_msgs::Float64 dist_msg;
            dist_msg.data = distance_error;
            dist_pub.publish(dist_msg);

            // Publish approximate cross-track error
            std_msgs::Float64 xte_msg;
            xte_msg.data = approx_xte;
            approx_xte_pub.publish(xte_msg);

            // Log to file
            double time_sec = ros::Time::now().toSec();
            log_file << time_sec << "," << distance_error << "," << approx_xte << "\n";

            ROS_INFO_THROTTLE(1.0, "Distance to carrot: %.3f m | Approx XTE: %.3f m",
                              distance_error, approx_xte);
        }

        rate.sleep();
    }

    log_file.close();
    return 0;
}






// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>
// #include <std_msgs/Float64.h>
// #include <fstream>
// #include <vector>
// #include <eigen3/Eigen/Dense>

// std::vector<Eigen::Vector3d> planned_path;
// bool path_received = false;
// std::ofstream log_file;

// // Compute shortest distance from point P to segment AB
// double pointToSegmentDistance(const Eigen::Vector3d& P,
//                                const Eigen::Vector3d& A,
//                                const Eigen::Vector3d& B) {
//     Eigen::Vector3d AP = P - A;
//     Eigen::Vector3d AB = B - A;
//     double t = AP.dot(AB) / AB.squaredNorm();
//     t = std::max(0.0, std::min(1.0, t)); // Clamp to [0,1]
//     Eigen::Vector3d proj = A + t * AB;
//     return (P - proj).norm();
// }

// void plannedPathCallback(const nav_msgs::Path::ConstPtr& msg) {
//     planned_path.clear();
//     for (const auto& pose : msg->poses) {
//         planned_path.push_back(Eigen::Vector3d(
//             pose.pose.position.x,
//             pose.pose.position.y,
//             pose.pose.position.z
//         ));
//     }
//     path_received = true;
//     ROS_INFO("Planned path received with %zu points", planned_path.size());
// }

// void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//     if (!path_received || planned_path.size() < 2) return;

//     Eigen::Vector3d drone_pos(msg->pose.position.x,
//                                msg->pose.position.y,
//                                msg->pose.position.z);

//     // Compute cross-track error (minimum distance to path segments)
//     double min_dist = 1e9;
//     for (size_t i = 0; i < planned_path.size() - 1; ++i) {
//         double dist = pointToSegmentDistance(drone_pos, planned_path[i], planned_path[i + 1]);
//         if (dist < min_dist) min_dist = dist;
//     }

//     // Publish XTE
//     static ros::NodeHandle nh;
//     static ros::Publisher error_pub = nh.advertise<std_msgs::Float64>("cross_track_error", 10);
//     std_msgs::Float64 error_msg;
//     error_msg.data = min_dist;


//     error_pub.publish(error_msg);

//     // Log to file
//     double time_sec = msg->header.stamp.toSec();
//     log_file << time_sec << "," << min_dist << "\n";

//     ROS_INFO_THROTTLE(1.0, "Cross-track error: %.3f m", min_dist);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "cross_track_error_node");
//     ros::NodeHandle nh;

//     // Open CSV file
//     log_file.open("/tmp/cross_track_error.csv");
//     log_file << "time,cross_track_error\n";

//     // Subscribers
//     ros::Subscriber path_sub = nh.subscribe("/planned_path", 1, plannedPathCallback);
//     ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 10, dronePoseCallback);

//     ROS_INFO("Cross-track error node started.");
//     ros::spin();

//     log_file.close();
//     return 0;
// }
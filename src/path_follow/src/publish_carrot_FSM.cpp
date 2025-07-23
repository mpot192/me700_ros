#include "ros/ros.h"
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
float lookahead_distance = 0.125f; // meters /0.3f balls 0.06
float max_carrot_jump = 0.225f; // Max carrot movement per iteration (meters) /0.325f 0.0625
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
// 182 204 84 153
bool FollowCarrotLayer(const double path_layer[3][153], int layer_size, const Eigen::Vector3f& drone_pos, Eigen::Vector3f& carrot, bool& layer_completed, bool& reached_first_point){
        

    // Eigen::Vector3f prev_carrot(0, 0, 0);
    // bool first_carrot_set = false;
    

    // PHASE 1: Go to first point
    if (!reached_first_point) {
        carrot = Eigen::Vector3f(path_layer[0][0], path_layer[1][0], path_layer[2][0]);
        float dist = (carrot - drone_pos).norm();

        if (dist < 0.05) { // 0.15 balls 0.1 
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
        int search_start = std::max(0, last_closest_idx - 3);   // Allow small backward correction
        int search_end   = std::min(layer_size - 1, last_closest_idx + 3); // Lookahead window //20

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
                ROS_INFO_THROTTLE(1.0, "Carrot in same Pos - Jump dist too high!");
            }
        }
        prev_carrot = carrot; // Store current carrot into prev for next iteration

        // Check if near the last point AND near the end of the path
        float dist_to_last_point = (last_point - drone_pos).norm();

        // Condition 1: We're very close to the last waypoint
        bool close_to_last_point = (dist_to_last_point < 0.05f); // 0.1 balls , 0.025

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
    float set_speed = 0.18;  // m/s // 0.25 balls 0.1 0.13
    float set_speed_fast = 0.13; //0.2
    bool reached_first_point = false;

    // Path setup
    const int rows = 3;
    const int cols = 153; // 182 204 84
    col_size = cols;
    // Path info
    // [x1 x2 .. ]
    // [y1 y2 .. ]
    // [z1 z2 .. ]]

    // 153 removed top layer
    double path[rows][cols] = {{-2.29077,-2.37393,-2.49407,-2.63397,-2.7743,-2.89625,-2.98409,-3.02714,-3.02112,-2.96845,-2.87779,-2.76256,-2.63906,-2.52412,-2.43282,-2.3765,-2.36139,-2.38793,-2.45093,-2.54051,-2.64358,-2.74579,-2.83346,-2.8955,-2.92472,-2.91869,-2.87987,-2.81499,-2.73404,-2.64867,-2.57056,-2.50982,-2.47366,-2.46558,-2.48509,-2.52793,-2.58696,-2.6532,-2.71727,-2.77067,-2.80691,-2.8223,-2.81627,-2.79128,-2.7522,-2.70553,-2.65829,-2.61701,-2.58682,-2.57081,-2.56978,-2.22189,-2.32044,-2.46246,-2.62756,-2.7929,-2.93632,-3.03935,-3.08958,-3.0821,-3.01995,-2.9134,-2.77833,-2.63388,-2.49976,-2.39351,-2.32828,-2.31116,-2.34247,-2.416,-2.5201,-2.6395,-2.75751,-2.85839,-2.92941,-2.96247,-2.955,-2.91001,-2.83547,-2.74294,-2.64581,-2.55739,-2.48907,-2.44886,-2.44047,-2.46305,-2.51156,-2.57774,-2.65143,-2.72212,-2.78046,-2.81947,-2.83537,-2.82789,-2.80007,-2.75754,-2.70755,-2.65774,-2.61503,-2.58463,-2.56944,-2.56978,-2.15359,-2.26739,-2.43112,-2.62121,-2.81134,-2.97604,-3.09416,-3.1515,-3.14259,-3.07102,-2.94871,-2.79396,-2.62874,-2.47559,-2.35454,-2.28047,-2.26135,-2.29739,-2.38135,-2.49986,-2.63544,-2.76914,-2.88311,-2.96304,-2.99992,-2.991,-2.9399,-2.85578,-2.75176,-2.64297,-2.54433,-2.4685,-2.42427,-2.41556,-2.44119,-2.49532,-2.56859,-2.64967,-2.72693,-2.79018,-2.83193,-2.84833,-2.83941,-2.80879,-2.76284,-2.70956,-2.6572,-2.61307,-2.58247,-2.56807,-2.56978}
    ,{-3.93555,-4.05842,-4.1416,-4.17514,-4.15609,-4.08863,-3.98334,-3.85553,-3.72305,-3.60379,-3.51328,-3.46263,-3.45724,-3.49621,-3.57271,-3.67511,-3.78873,-3.89792,-3.98823,-4.04833,-4.07139,-4.05587,-4.00548,-3.92849,-3.83638,-3.7422,-3.65864,-3.59643,-3.56285,-3.56099,-3.58948,-3.6429,-3.71275,-3.78873,-3.86028,-3.91803,-3.95505,-3.96764,-3.95565,-3.92233,-3.87363,-3.81724,-3.76134,-3.7135,-3.67958,-3.66307,-3.66474,-3.68275,-3.7131,-3.75039,-3.78873,-3.96224,-4.10719,-4.20507,-4.24428,-4.22145,-4.14168,-4.01756,-3.8672,-3.71165,-3.57191,-3.46612,-3.40722,-3.40134,-3.44728,-3.53688,-3.65643,-3.78873,-3.91553,-4.02008,-4.08932,-4.11552,-4.09707,-4.03849,-3.94948,-3.84344,-3.73541,-3.63998,-3.56931,-3.53159,-3.5301,-3.56303,-3.62399,-3.70314,-3.78873,-3.86881,-3.93296,-3.97357,-3.98676,-3.9727,-3.9353,-3.88141,-3.81968,-3.75917,-3.70805,-3.6725,-3.65596,-3.65886,-3.67878,-3.71111,-3.74985,-3.78873,-3.9887,-4.15555,-4.26802,-4.31284,-4.28626,-4.19429,-4.05149,-3.87878,-3.70034,-3.54029,-3.41936,-3.35226,-3.34591,-3.39875,-3.50135,-3.63791,-3.78873,-3.93299,-4.05166,-4.12997,-4.15928,-4.13794,-4.07122,-3.97031,-3.85044,-3.72868,-3.62147,-3.54242,-3.50059,-3.49946,-3.5368,-3.60524,-3.69362,-3.78873,-3.87728,-3.94777,-3.99193,-4.00572,-3.98961,-3.94816,-3.88912,-3.8221,-3.75702,-3.70265,-3.66549,-3.64892,-3.65302,-3.67485,-3.70913,-3.74932,-3.78873}
    ,{1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227}};

    // 204
    // double path[rows][cols] ={{-2.57653,-2.59588,-2.6252,-2.66055,-2.69714,-2.73004,-2.7548,-2.76807,-2.76807,-2.7548,-2.73004,-2.69714,-2.66055,-2.6252,-2.59588,-2.57653,-2.56978,-2.57653,-2.59588,-2.6252,-2.66055,-2.69714,-2.73004,-2.7548,-2.76807,-2.76807,-2.7548,-2.73004,-2.69714,-2.66055,-2.6252,-2.59588,-2.57653,-2.56978,-2.57653,-2.59588,-2.6252,-2.66055,-2.69714,-2.73004,-2.7548,-2.76807,-2.76807,-2.7548,-2.73004,-2.69714,-2.66055,-2.6252,-2.59588,-2.57653,-2.56978,-2.29077,-2.37393,-2.49407,-2.63397,-2.7743,-2.89625,-2.98409,-3.02714,-3.02112,-2.96845,-2.87779,-2.76256,-2.63906,-2.52412,-2.43282,-2.3765,-2.36139,-2.38793,-2.45093,-2.54051,-2.64358,-2.74579,-2.83346,-2.8955,-2.92472,-2.91869,-2.87987,-2.81499,-2.73404,-2.64867,-2.57056,-2.50982,-2.47366,-2.46558,-2.48509,-2.52793,-2.58696,-2.6532,-2.71727,-2.77067,-2.80691,-2.8223,-2.81627,-2.79128,-2.7522,-2.70553,-2.65829,-2.61701,-2.58682,-2.57081,-2.56978,-2.22189,-2.32044,-2.46246,-2.62756,-2.7929,-2.93632,-3.03935,-3.08958,-3.0821,-3.01995,-2.9134,-2.77833,-2.63388,-2.49976,-2.39351,-2.32828,-2.31116,-2.34247,-2.416,-2.5201,-2.6395,-2.75751,-2.85839,-2.92941,-2.96247,-2.955,-2.91001,-2.83547,-2.74294,-2.64581,-2.55739,-2.48907,-2.44886,-2.44047,-2.46305,-2.51156,-2.57774,-2.65143,-2.72212,-2.78046,-2.81947,-2.83537,-2.82789,-2.80007,-2.75754,-2.70755,-2.65774,-2.61503,-2.58463,-2.56944,-2.56978,-2.15359,-2.26739,-2.43112,-2.62121,-2.81134,-2.97604,-3.09416,-3.1515,-3.14259,-3.07102,-2.94871,-2.79396,-2.62874,-2.47559,-2.35454,-2.28047,-2.26135,-2.29739,-2.38135,-2.49986,-2.63544,-2.76914,-2.88311,-2.96304,-2.99992,-2.991,-2.9399,-2.85578,-2.75176,-2.64297,-2.54433,-2.4685,-2.42427,-2.41556,-2.44119,-2.49532,-2.56859,-2.64967,-2.72693,-2.79018,-2.83193,-2.84833,-2.83941,-2.80879,-2.76284,-2.70956,-2.6572,-2.61307,-2.58247,-2.56807,-2.56978}
    // ,{-3.82485,-3.8561,-3.87824,-3.8883,-3.88491,-3.86853,-3.84137,-3.8071,-3.77035,-3.73608,-3.70892,-3.69254,-3.68915,-3.69921,-3.72136,-3.7526,-3.78873,-3.82485,-3.8561,-3.87824,-3.8883,-3.88491,-3.86853,-3.84137,-3.8071,-3.77035,-3.73608,-3.70892,-3.69254,-3.68915,-3.69921,-3.72136,-3.7526,-3.78873,-3.82485,-3.8561,-3.87824,-3.8883,-3.88491,-3.86853,-3.84137,-3.8071,-3.77035,-3.73608,-3.70892,-3.69254,-3.68915,-3.69921,-3.72136,-3.7526,-3.78873,-3.93555,-4.05842,-4.1416,-4.17514,-4.15609,-4.08863,-3.98334,-3.85553,-3.72305,-3.60379,-3.51328,-3.46263,-3.45724,-3.49621,-3.57271,-3.67511,-3.78873,-3.89792,-3.98823,-4.04833,-4.07139,-4.05587,-4.00548,-3.92849,-3.83638,-3.7422,-3.65864,-3.59643,-3.56285,-3.56099,-3.58948,-3.6429,-3.71275,-3.78873,-3.86028,-3.91803,-3.95505,-3.96764,-3.95565,-3.92233,-3.87363,-3.81724,-3.76134,-3.7135,-3.67958,-3.66307,-3.66474,-3.68275,-3.7131,-3.75039,-3.78873,-3.96224,-4.10719,-4.20507,-4.24428,-4.22145,-4.14168,-4.01756,-3.8672,-3.71165,-3.57191,-3.46612,-3.40722,-3.40134,-3.44728,-3.53688,-3.65643,-3.78873,-3.91553,-4.02008,-4.08932,-4.11552,-4.09707,-4.03849,-3.94948,-3.84344,-3.73541,-3.63998,-3.56931,-3.53159,-3.5301,-3.56303,-3.62399,-3.70314,-3.78873,-3.86881,-3.93296,-3.97357,-3.98676,-3.9727,-3.9353,-3.88141,-3.81968,-3.75917,-3.70805,-3.6725,-3.65596,-3.65886,-3.67878,-3.71111,-3.74985,-3.78873,-3.9887,-4.15555,-4.26802,-4.31284,-4.28626,-4.19429,-4.05149,-3.87878,-3.70034,-3.54029,-3.41936,-3.35226,-3.34591,-3.39875,-3.50135,-3.63791,-3.78873,-3.93299,-4.05166,-4.12997,-4.15928,-4.13794,-4.07122,-3.97031,-3.85044,-3.72868,-3.62147,-3.54242,-3.50059,-3.49946,-3.5368,-3.60524,-3.69362,-3.78873,-3.87728,-3.94777,-3.99193,-4.00572,-3.98961,-3.94816,-3.88912,-3.8221,-3.75702,-3.70265,-3.66549,-3.64892,-3.65302,-3.67485,-3.70913,-3.74932,-3.78873}
    // ,{1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.94227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.69227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.44227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227,1.19227}};

    // 84
    // double path[rows][cols] = {{-2.59212,-2.67672,-2.74457,-2.74457,-2.67672,-2.59212,-2.55447,-2.59212,-2.67672,-2.74457,-2.74457,-2.67672,-2.59212,-2.55447,-2.59212,-2.67672,-2.74457,-2.74457,-2.67672,-2.59212,-2.55447,-2.52671,-2.6989,-2.82964,-2.82491,-2.6954,-2.54306,-2.48103,-2.5496,-2.69073,-2.79655,-2.79183,-2.68723,-2.56595,-2.51775,-2.5725,-2.68256,-2.76347,-2.75874,-2.67906,-2.58885,-2.55447,-2.39285,-2.74428,-3.00372,-2.98932,-2.73361,-2.44267,-2.33075,-2.4626,-2.71939,-2.90294,-2.88854,-2.70872,-2.51241,-2.44261,-2.53234,-2.6945,-2.80215,-2.78776,-2.68383,-2.58216,-2.55447,-2.59212,-2.67672,-2.74457,-2.74457,-2.67672,-2.59212,-2.55447,-2.59212,-2.67672,-2.74457,-2.74457,-2.67672,-2.59212,-2.55447,-2.59212,-2.67672,-2.74457,-2.74457,-2.67672,-2.59212,-2.55447}
    // ,{-13.8403,-13.8596,-13.8055,-13.7187,-13.6646,-13.6839,-13.7621,-13.8403,-13.8596,-13.8055,-13.7187,-13.6646,-13.6839,-13.7621,-13.8403,-13.8596,-13.8055,-13.7187,-13.6646,-13.6839,-13.7621,-13.9223,-13.9567,-13.8464,-13.68,-13.5828,-13.6224,-13.7621,-13.8936,-13.9209,-13.8305,-13.6959,-13.6186,-13.6511,-13.7621,-13.8649,-13.8851,-13.8146,-13.7119,-13.6544,-13.6798,-13.7621,-14.0901,-14.1556,-13.9303,-13.6008,-13.4153,-13.4965,-13.7621,-14.0027,-14.0465,-13.8817,-13.6494,-13.5244,-13.5839,-13.7621,-13.9152,-13.9375,-13.8332,-13.6979,-13.6334,-13.6714,-13.7621,-13.8403,-13.8596,-13.8055,-13.7187,-13.6646,-13.6839,-13.7621,-13.8403,-13.8596,-13.8055,-13.7187,-13.6646,-13.6839,-13.7621,-13.8403,-13.8596,-13.8055,-13.7187,-13.6646,-13.6839,-13.7621}
    // ,{2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.37292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,2.12292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.87292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292,1.62292}};

    // 182
    // double path[rows][cols] = {{3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.12041, 3.22457, 3.29839, 3.33191, 3.32061, 3.26601, 3.17548, 3.06125, 2.93875, 2.82452, 2.73399, 2.67939, 2.66809, 2.70161, 2.77543, 2.87959, 3, 3, 3.06021, 3.11228, 3.14919, 3.16596, 3.1603, 3.133, 3.08774, 3.03062, 2.96938, 2.91226, 2.867, 2.8397, 2.83404, 2.85081, 2.88772, 2.93979, 3, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3, 3, 3.18062, 3.33685, 3.44758, 3.49787, 3.48091, 3.39901, 3.26322, 3.09187, 2.90813, 2.73678, 2.60099, 2.51909, 2.50213, 2.55242, 2.66315, 2.81938, 3}
    // ,{3, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.33333, 3.31082, 3.24634, 3.14858, 3.03076, 2.90878, 2.79912, 2.71659, 2.67234, 2.67234, 2.71659, 2.79912, 2.90878, 3.03076, 3.14858, 3.24634, 3.31082, 3.33333, 3.16667, 3.15541, 3.12317, 3.07429, 3.01538, 2.95439, 2.89956, 2.8583, 2.83617, 2.83617, 2.8583, 2.89956, 2.95439, 3.01538, 3.07429, 3.12317, 3.15541, 3.16667, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5, 3.5, 3.46624, 3.3695, 3.22287, 3.04613, 2.86317, 2.69868, 2.57489, 2.50851, 2.50851, 2.57489, 2.69868, 2.86317, 3.04613, 3.22287, 3.3695, 3.46624, 3.5}
    // ,{3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1.25, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25}};

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
                velocity = direction_to_carrot.normalized() * set_speed_fast;
            }else{
                velocity = direction_to_carrot.normalized() * set_speed; // balls
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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <vector>

using namespace std;

#define PATH_SIZE 50

int nlayers = 1; // store number of layers globally for ease of publishing

// generate a circle based on input parameters
void Cylinder(float r, int n, float h, float cyl[3][PATH_SIZE]){
  // float r = diameter/2;
  float dt = (2*M_PI)/(n-1);

  // generate circle
  for(int i = 0; i < n; i++){
    cyl[0][i] = r*cos(i*dt);
    cyl[1][i] = r*sin(i*dt);
    cyl[2][i] = h;
  }
}

void Lemniscate(float a, int n, float h, float inf[3][PATH_SIZE]){
  float dt = (2*M_PI)/(n-1);

  for(int i = 0; i < n; i++){
    inf[0][i] = (a*cos(i*dt))/(1+sin(i*dt)*sin(i*dt));
    inf[1][i] = (a*cos(i*dt)*sin(i*dt))/(1+sin(i*dt)*sin(i*dt));
    inf[2][i] = h;
  }
}
int GeneratePath(float (&path)[3][PATH_SIZE], string path_style, float path_dim){
    // generate diagonal line from (1,1,1) diagonal to end 
    if(path_style == "line"){
        float div = path_dim/(PATH_SIZE - 1); // distance between points
        float t; // proportion along line segment
        Eigen::Vector3f start(1.0, 1.0, 1.0); // start point of path
        Eigen::Vector3f end(1.0 + path_dim, 1.0 + path_dim, 1.0 + path_dim); // end point of path

        // calculate coordinates of each point
        for(int i = 0; i < PATH_SIZE; i++){
            t = div*i;
            path[0][i] = (1 - t)*start.x() + (t)*end.x();
            path[1][i] = (1 - t)*start.y() + (t)*end.y();
            path[2][i] = (1 - t)*start.z() + (t)*end.z();
        }
    } else if(path_style == "circle"){
      Cylinder(path_dim, PATH_SIZE, 1.0, path);
    } else if(path_style == "inf"){
      Lemniscate(path_dim, PATH_SIZE, 1.0, path);
    }
    return PATH_SIZE;
}

int main (int argc, char *argv[]) {
  // init ros
  ros::init(argc, argv, "test_path");
  ros::NodeHandle nh;
  ros::Rate loop_rate(5);

  // comms
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path_gen/planned_path", 1000);
  ros::Publisher path_layer_pub = nh.advertise<std_msgs::Int32>("/path_gen/layer_count", 10);
  nav_msgs::Path planned;

  // path
  float path[3][PATH_SIZE];
  int size;

  static bool first = true;
  static bool sent_path = false;
  static bool done = false; 

  // check that bounding box has been given
  if (argc < 3) {
      ROS_ERROR("Missing required command line arguments <path style> <dimension>");
      ros::shutdown(); 
      return -1;      
  }

  std::string path_style  = argv[1];
  float path_dim = stof(argv[2]);

  geometry_msgs::PoseStamped p;

  while (ros::ok()){
    
    // generate path first
    if(first){
      size = GeneratePath(path, path_style, path_dim);
      ROS_INFO("Generated path of size: %d!",size);
      first = false;
    }

    // send path after has been generated 
    if(!first && !sent_path){
      for (int i = 0; i < size; i++){
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";
        
        p.pose.position.x = path[0][i];
        p.pose.position.y = path[1][i];
        p.pose.position.z = path[2][i];

        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0; 
        p.pose.orientation.w = 1.0;
        planned.poses.push_back(p);
        sent_path = true;
        }
    }

    planned.header.frame_id = "map";
    planned.header.stamp = ros::Time::now();
    path_pub.publish(planned);

    std_msgs::Int32 nl;
    nl.data = nlayers;
    path_layer_pub.publish(nl);

    // Run callbacks
    ros::spinOnce();
    // Wait
    loop_rate.sleep();
  }

  return 0;
}
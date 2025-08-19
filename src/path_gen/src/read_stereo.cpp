#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include "path_gen/PCHandler.hpp"

using namespace std;

#define PATH_SIZE 1000

// IMG PARAMETERS
#define IMGH 480
#define IMGW 640
#define IMGD 3

// BOUNDING BOX CENTRE POSITION DEFINITION
#define BBX 320 
#define BBY 240

// INPUT PARAMETERS
#define R_AVOID 0.1 // inner radius to avoid cutting [m]
#define H_LAYER 0.25 // distance between cutting layers [m]
#define D_CUT 0.4 // diameter of cutting head [m] 
#define UNCERTAINTY 0.9 // fraction inside bounding box to be used for path generation
#define MIN_SPACE 0.15 // minimum spacing between spiral loops/concentric circles in a layer
#define N_LAYER_POINTS 50 // number of points in a layer 
#define N_SPIRALS 3 // target number of spirals in a layer

#define SF 1 // scaling factor for path to avoid hitting the target with the drone
#define H_OFFSET 0 // z offset for drone position if required
               
PCHandler handler;
nav_msgs::Odometry drone_pos;

int nlayers; // store number of layers globally for ease of publishing

void get_position(const nav_msgs::Odometry& msg){
  drone_pos = msg; 
}

// generate a circle based on input parameters
void Cylinder(float r, int n, float h, float cyl[][3]){
  // float r = diameter/2;
  float dt = (2*M_PI)/(n-1);

  // generate circle
  for(int i = 0; i < n; i++){
    cyl[i][0] = r*cos(i*dt);
    cyl[i][1] = r*sin(i*dt);
    cyl[i][2] = h;
  }
}

// generate a spiral based on input parameters
void Spiral(float r_min, float r_max, int n_points, int n_spiral, float h, float sp[][3]){

  float dt = (2*M_PI) * (n_spiral/(float)n_points); 
  float dr = (r_max - r_min)/n_points; 

  // generate spiral (outside -> in)
  int cnt = 0;
  for(int i = n_points - 1; i >= 0; i--){
    sp[cnt][0] = (r_min+dr*i)*cos(i*dt);
    sp[cnt][1] = (r_min+dr*i)*sin(i*dt);
    sp[cnt][2] = h;
    cnt++;
  }
}

int GeneratePath(float (&path)[3][PATH_SIZE], int bbx, int bby, int bbh, int bbw, float r_avoid, float h_layer, float d_cut, float u){

  float bbwu = bbw * u; // scale bounding box to uncertainty
  float bbhu = bbh * u; 
  float bbru;
  int sz = 0;
  float maxh; 
  float minh;
  float starth;
  float endh;

  // get width and height of point cloud from depth camera
  int width = handler.GetPC()->width;
  int height = handler.GetPC()->height;

  // get centre point of point cloud 
  auto& dcpt = handler.GetPC()->at(bbx,bby);

  // put in terms of absolute position
  float dcx = dcpt.x + drone_pos.pose.pose.position.x;
  float dcy = dcpt.y + drone_pos.pose.pose.position.y;
  float dcz = drone_pos.pose.pose.position.z;

  ROS_INFO("GENERATING PATH CENTRED AT [%f, %f]", dcx, dcy);
  std::ofstream logfile;
  logfile.open("/home/matt/catkin_ws/bag/pathgen_log.txt");
  logfile << "Path centered at: " << dcx << ", " << dcy << endl;

  // fit circle within 
  if(bbhu < bbwu){
    bbru = bbhu / 2;
  } else {
    bbru = bbwu / 2;
  }

  // find max and min depth within area of interest
  for(int i = 0; i < width; i++){
    for(int j = 0; j < height; j++){
      if(pow((i - bbx),2) + pow((j - bby),2) <= pow(bbru, 2)){
        auto& pt = handler.GetPC()->at(i,j);
        if(sz == 0){
          maxh = pt.z;
          minh = pt.z;
        } else{
          if(pt.z < minh){
            minh = pt.z;
          } 
          if(pt.z > maxh){
            maxh = pt.z;
          }
        } 
        sz++;
      }
    }
  }

  // start at min depth
  starth = minh;
  endh = minh + (maxh - minh)*d_cut;

  // get layer depths
  nlayers = ceil((endh-starth)/h_layer);
  if(nlayers > 1){
    ROS_INFO("Generating path with %d layers.", nlayers);
  } 

  float layer_depths[nlayers];
  for(int i = 0; i < nlayers; i++){
    layer_depths[i] = starth + h_layer*i;
  }

  // estimate diameter at each layer
  float h_tol = 0.01;
  float xval;
  float yval;
  float maxr = 0;
  float layer_r[nlayers];
  for (int i = 0; i < nlayers; i++){
    for(int j = 0; j < width; j++){
      for(int k = 0; k < height; k++){
        auto& pt2 = handler.GetPC()->at(j,k);
        // if within fit circle, consider for max radius
        if(pow((j - bbx),2) + pow((k - bby),2) <= pow(bbru, 2) && abs(pt2.z - layer_depths[i]) <= h_tol){
          xval = pt2.x;
          yval = pt2.y;
          if(i == 0){
          }
          if (xval > maxr){
            maxr = xval;
          }
          if(yval > maxr){
            maxr = yval;
          }
        }
      }
    }
    if(i != 0 && maxr == 0){
      layer_r[i] = layer_r[i-1];
    } else{
      layer_r[i] = maxr;
    }
    logfile << "Layer " << i << " radius: " << maxr << endl;
    maxr = 0;
  }

  int n_points = N_LAYER_POINTS; // number of points between layers
  int n_spiral; // maximum (target) number of spirals in spiral layer
  float cyl[n_points][3]; // layer
  float pathx[PATH_SIZE];
  float pathy[PATH_SIZE];
  float pathz[PATH_SIZE];
  float r; 
  int p_size = 0;

  // generate layers
  for(int i = 0; i < nlayers; i++){
    r = layer_r[i];
    n_spiral = N_SPIRALS;
    // saturate to minimum radius to avoid stem
    if(r < R_AVOID){
      r = R_AVOID;
    }

    while((r-R_AVOID)/n_spiral < MIN_SPACE && n_spiral > 1){
      n_spiral--;
    }

    // generate layers, spiral if large enough otherwise single circle
    if(n_spiral == 1){ // circle
      ROS_INFO("Generating Layer %d as a circle with r = %.2fm.", i, r);
      Cylinder(r, n_points, layer_depths[i], cyl);
    } else{
      Spiral(R_AVOID, r, n_points, n_spiral, layer_depths[i], cyl);
      ROS_INFO("Generating Layer %d as a spiral with r = %.2fm and n = %d (/%d).", i, r, n_spiral, N_SPIRALS);
    }
    
    for(int j = 0; j < n_points; j++){
      
      // store in path arrays
      pathx[p_size + j] = (cyl[j][0] * SF) + dcx;
      pathy[p_size + j] = (cyl[j][1] * SF) + dcy;
      pathz[p_size + j] = (dcz - cyl[j][2]) + H_OFFSET;
      logfile << j << " " << "[" << cyl[j][0] + dcx << ", " << cyl[j][1] + dcy << ", " <<  cyl[j][2]<< "]" << endl;
    }
    p_size += n_points;
    
  }

  // store all in one path
  for(int  i = 0; i<p_size;i++){
    path[0][i] = pathx[i];
    path[1][i] = pathy[i];
    path[2][i] = pathz[i];

  }

  // output to logfile
  logfile << "Path size = " << p_size << endl;
  logfile << "{";
  for(int i = 0; i < 3;i++){
    logfile << "{";
    for(int j = 0; j < p_size; j++){
      logfile << path[i][j];
      if(j != p_size - 1){
        logfile << ",";
      }
    }
    logfile << "}";
    if(i != 2){
      logfile << ",";
    }
  }
  logfile << "}";
  logfile.close(); 
  return p_size;
}

int main (int argc, char *argv[]) {
  // init ros
  ros::init(argc, argv, "read_stereo")  ;
  ros::NodeHandle nh;
  ros::Rate loop_rate(5);

  // comms
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 10, &PCHandler::CallbackPC, &handler);
  ros::Subscriber pos = nh.subscribe("/mavros/global_position/local", 10, get_position);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path_gen/planned_path", 1000);
  ros::Publisher path_layer_pub = nh.advertise<std_msgs::Int32>("/path_gen/layer_count", 10);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/path_gen/depth_model", 1000);
  nav_msgs::Path planned;

  // path
  float path[3][PATH_SIZE];
  int size;

  static bool first = true;
  static bool sent_path = false;
  static bool done = false; 

  // check that bounding box has been given
  if (argc < 2) {
      ROS_ERROR("Missing required command line arguments <bounding box height> <bounding box width>");
      ros::shutdown(); 
      return -1;      
  }

  // set bounding box size based on input arguments 
  int bb_height = std::atoi(argv[1]);
  int bb_width = std::atoi(argv[2]);

  geometry_msgs::PoseStamped p;
  while (ros::ok()){
    
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
      cloud_pub.publish(handler.GetModelPC());
    }

    planned.header.frame_id = "map";
    planned.header.stamp = ros::Time::now();
    path_pub.publish(planned);

    std_msgs::Int32 nl;
    nl.data = nlayers;
    path_layer_pub.publish(nl);

    // Run callbacks
    ros::spinOnce();

    // generate path first
    if(first && handler.exists){
      size = GeneratePath(path, BBX, BBY, bb_height, bb_width, R_AVOID, H_LAYER, D_CUT, UNCERTAINTY);
      handler.GenerateModelPC(drone_pos.pose.pose.position.x, drone_pos.pose.pose.position.y, drone_pos.pose.pose.position.z);
      ROS_INFO("Generated path of size: %d!",size);
      first = false;
    }

    // Wait
    loop_rate.sleep();
  }

  return 0;
}

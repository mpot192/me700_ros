#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include "path_gen/PCHandler.hpp"

using namespace std;

#define PATH_SIZE 200

// IMG PARAMETERS
#define IMGH 480
#define IMGW 640
#define IMGD 3

// BOUNDING BOX DEFINITION
#define BBX 321
#define BBY 229
#define BBH 200
#define BBW 200

// INPUT PARAMETERS
#define R_AVOID 0.1 // inner radius to avoid cutting
#define H_LAYER 0.25 // distance between cutting layers 
#define D_CUT 0.4 // diameter of cutting head
#define UNCERTAINTY 0.9 // fraction inside bounding box to be used for path generation

#define SF 1 // scaling factor to avoid hitting the target with the drone
#define H_OFFSET 0 
               
bool created_image = false;
PCHandler handler;
nav_msgs::Odometry drone_pos;

void get_position(const nav_msgs::Odometry& msg){
  drone_pos = msg; 
}

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

  float bbwu = bbw * u;
  float bbhu = bbh * u; 
  float bbru;
  int sz = 0;
  float maxh; 
  float minh;
  float starth;
  float endh;

  int width = handler.GetPC()->width;
  int height = handler.GetPC()->height;

  auto& dcpt = handler.GetPC()->at(bbx,bby);

  float dcx = dcpt.x + drone_pos.pose.pose.position.x;
  float dcy = dcpt.y + drone_pos.pose.pose.position.y;
  float dcz = drone_pos.pose.pose.position.z;
  ROS_INFO("GENERATING PATH CENTRED AT [%f, %f]", dcx, dcy);
  std::ofstream logfile;
  logfile.open("/home/matt/catkin_ws/bag/pathgen_log.txt");
  logfile << "Path centered at: " << dcx << ", " << dcy << endl;
  // cout << "1" << endl;
  ROS_INFO("1");
  if(bbhu < bbwu){
    bbru = bbhu / 2;
  } else {
    bbru = bbwu / 2;
  }
  // cout << "2 " << endl;

  ROS_INFO("2");
  // find max and min height within area of interest
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
  // cout << "Min Depth: " << minh << endl;
  // cout << "Max Depth: " << maxh << endl;

  ROS_INFO("3");
  starth = minh;
  endh = minh + (maxh - minh)*d_cut;

  
  // cout << "4" << endl;

  // get layer depths
  int nlayers = ceil((endh-starth)/h_layer);
  float layer_depths[nlayers];
  for(int i = 0; i < nlayers; i++){
    layer_depths[i] = starth + h_layer*i;
  }


  ROS_INFO("4");
  // cout << "5" << endl;
  // estimate diameter at each layer
  float h_tol = 0.01;
  float xval;
  float yval;
  float maxr = 0;
  float layer_r[nlayers];
  for (int i = 0; i < nlayers; i++){
    for(int j = 0; j < width; j++){
      for(int k = 0; k < height; k++){
        // ROS_INFO("4.1");

        auto& pt2 = handler.GetPC()->at(j,k);
        // ROS_INFO("4.2");
        if(pow((j - bbx),2) + pow((k - bby),2) <= pow(bbru, 2) && abs(pt2.z - layer_depths[i]) <= h_tol){
          // xval = abs(abs(pt2.x) - abs(dcx));
          // yval = abs((pt2.y) - abs(dcy));
          xval = pt2.x;
          yval = pt2.y;
          if(i == 0){
            // ROS_INFO("k: %d j: %d x: %f y: %f", k, j, xval, yval);
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
    layer_r[i] = maxr;
    ROS_INFO("Layer %d radius: %f", i, maxr);
    logfile << "Layer " << i << " radius: " << maxr << endl;
    maxr = 0;
  }
  // cout << "6" << endl;

  ROS_INFO("5");
  int n_points = 20;
  float cyl[n_points+1][3];
  float pathx[PATH_SIZE];
  float pathy[PATH_SIZE];
  float pathz[PATH_SIZE];
  float r; 
  int p_size = 0;
  for(int i = 0; i < nlayers; i++){
    r = layer_r[i];
    if(r < R_AVOID){
      r = R_AVOID;
    }
    // cout << "Radius: " << r << endl; 
    // Cylinder(r, n_points+1, layer_depths[i], cyl);
    Spiral(R_AVOID, r, n_points+1, 3, layer_depths[i], cyl); 
    for(int j = 0; j < n_points+1; j++){
      // cout << "[" << cyl[j][0] + dcx << ", " << cyl[j][1] + dcy << ", " << cyl[j][2] << "]" << endl;
      pathx[p_size + j] = (cyl[j][0] * SF) + dcx;
      pathy[p_size + j] = (cyl[j][1] * SF) + dcy;
      pathz[p_size + j] = (dcz - cyl[j][2]) + H_OFFSET;
      logfile << j << " " << "[" << cyl[j][0] + dcx << ", " << cyl[j][1] + dcy << ", " <<  cyl[j][2]<< "]" << endl;
    }
    p_size += n_points + 1;
    
  }
  logfile.close(); 
  ROS_INFO("7");
  // cout << "_______________" << endl;
  // cout << "7 " << path[1][0] << endl;
  for(int  i = 0; i<p_size;i++){
    path[0][i] = pathx[i];
    path[1][i] = pathy[i];
    path[2][i] = pathz[i];
    // cout << "[" << path[0][i] << ", " << path[1][i]<< ", " << path[2][i] << "]" << endl;
  }

  // cout << "7 " << path[1][0] << endl;
  
  ROS_INFO("8");
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
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 1000);
  nav_msgs::Path planned;

  // path
  float path[3][PATH_SIZE];
  int size;

  static bool first = true;
  static bool done = false; 

  geometry_msgs::PoseStamped p;
  while (ros::ok()){
    
    if(!first){
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
      }
    }

    planned.header.frame_id = "map";
    planned.header.stamp = ros::Time::now();
    path_pub.publish(planned);

    // Run callbacks
    ros::spinOnce();

    if(first && handler.exists){
      size = GeneratePath(path, BBX, BBY, BBH, BBW, R_AVOID, H_LAYER, D_CUT, UNCERTAINTY);
      ROS_INFO("Generated path of size: %d!",size);
      ROS_INFO("PATH [%f, %f, %f]", path[0][0], path[1][0], path[2][0]);
      first = false;
    }

    // Wait
    loop_rate.sleep();
  }

  return 0;
}

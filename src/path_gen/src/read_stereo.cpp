#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
#define R_AVOID 0.1 
#define H_LAYER 0.1 
#define D_CUT 0.4
#define UNCERTAINTY 0.9

bool created_image = false;
PCHandler handler;

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

int GeneratePath(float (&path)[PATH_SIZE][3], int bbx, int bby, int bbh, int bbw, float r_avoid, float h_layer, float d_cut, float u){

  float bbwu = bbw * u;
  float bbhu = bbh * u; 
  float bbru;
  int sz = 0;
  float maxh; 
  float minh;
  float starth;
  float endh;

  auto& dcpt = handler.GetPC()->at(bby,bbx);

  float dcx = dcpt.x;
  float dcy = dcpt.y;

  // cout << "1" << endl;
  if(bbhu < bbwu){
    bbru = bbhu / 2;
  } else {
    bbru = bbwu / 2;
  }
  // cout << "2 " << endl;

  // find max and min height within area of interest
  for(int i = 0; i < IMGW; i++){
    for(int j = 0; j < IMGH; j++){
      if(pow((i - bbx),2) + pow((j - bby),2) <= pow(bbru, 2)){
        auto& pt = handler.GetPC()->at(j,i) ;
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

  starth = minh;
  endh = minh + (maxh - minh)*d_cut;

  
  // cout << "4" << endl;

  // get layer depths
  int nlayers = ceil((endh-starth)/h_layer);
  float layer_depths[nlayers];
  for(int i = 0; i < nlayers; i++){
    layer_depths[i] = starth + h_layer*i;
  }


  // cout << "5" << endl;
  // estimate diameter at each layer
  float h_tol = 0.01;
  float xval;
  float yval;
  float maxr = 0;
  float layer_r[nlayers];
  for (int i = 0; i < nlayers; i++){
    for(int j = 0; j < IMGW; j++){
      for(int k = 0; k < IMGH; k++){
        auto& pt2 = handler.GetPC()->at(k,j);
        if(pow((j - bbx),2) + pow((k - bby),2) <= pow(bbru, 2) && abs(pt2.z - layer_depths[i]) <= h_tol){
          xval = abs(pt2.x - dcx);
          yval = abs(pt2.y - dcy);
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
  }
  // cout << "6" << endl;

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
    Cylinder(r, n_points+1, layer_depths[i], cyl);
    for(int j = 0; j < n_points+1; j++){
      // cout << "[" << cyl[j][0] + dcx << ", " << cyl[j][1] + dcy << ", " << cyl[j][2] << "]" << endl;
      pathx[p_size + j] = cyl[j][0] + dcx;
      pathy[p_size + j] = cyl[j][1] + dcy;
      pathz[p_size + j] = cyl[j][2];

    }
    p_size += n_points + 1;
    
  }

  // cout << "_______________" << endl;
  // cout << "7 " << path[1][0] << endl;
  for(int  i = 0; i<p_size;i++){
    path[0][i] = pathx[i];
    path[1][i] = pathy[i];
    path[2][i] = pathz[i];
    // cout << "[" << path[0][i] << ", " << path[1][i]<< ", " << path[2][i] << "]" << endl;
  }

  // cout << "7 " << path[1][0] << endl;
  
  return p_size;
}

int main (int argc, char *argv[]) {
  // init ros
  ros::init(argc, argv, "read_stereo")  ;
  ros::NodeHandle nh;
  ros::Rate loop_rate(5);

  // comms
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 10, &PCHandler::CallbackPC, &handler);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 1000);
  nav_msgs::Path planned;

  // path
  float path[PATH_SIZE][3];
  int size;

  static bool first = true;

  while (ros::ok()){
    
    if(!first){
      for (int i = 0; i < size; i++){
        geometry_msgs::PoseStamped p;
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
      first = false;
    }

    // Wait
    loop_rate.sleep();
  }

  return 0;
}

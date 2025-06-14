#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>


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
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;

float GetHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int bb_w, int bb_h, int bb_x, int bb_y){
  int pad = 10; // Distance around bounding box input to include in scan
  float grad_thresh = 0.5; // Threshold gradient [m] to count as hard edge of object
  ROS_INFO("Processing bounding box: [%d %d], [%d %d]", bb_w, bb_h, bb_x, bb_y);
  float grad;
  float grads[bb_h];
  int grad_cnt = 0; 

  for(int i = (bb_y - floor(bb_h/2) - pad) + 1; i < (bb_y + floor(bb_h/2) + pad); i++){
    for(int j = (bb_x - floor(bb_w/2) - pad) + 1; j < (bb_x + floor(bb_w/2) + pad); j++){
      grad = (cloud->at(j,i).z) - (cloud->at(j-1,i).z);
      if(abs(grad > grad_thresh)){
        grads[grad_cnt++] = abs(grad);
      }
    }
  }

  float grad_sum = 0;
  for(int i = 0; i < grad_cnt; i++){
    grad_sum += grads[i];
  }
  return grad_sum/grad_cnt;
}


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud){
  pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  // This just saves the first captured stereo reading as img and text file of depth values
  if(!created_image){
    int width = pcl_cloud->width;
    int height = pcl_cloud->height;
  // just depth information
    cv::Mat img(height, width, CV_8UC1);
    std::ofstream out("./img.txt");
    out << "[";
    for(int i = 0; i < height; i++){
      for(int j = 0; j < width; j++){
        auto& pt = pcl_cloud->at(j,i) ;
        img.at<uint8_t>(i,j) = static_cast<uint8_t>((pt.z) * 255/5);
        out << pt.z;
        if (j != width - 1){
          out << ", ";
        }
      }
      out << std::endl;
    }
    out << "]";
    out.close();

    // full positions in matlab format
    std::ofstream outf("./full_info.txt");
    outf << "depthxyz(:,:,1) = [";
    for(int i = 0; i < height; i++){
      for(int j = 0; j < width; j++){
        auto& pt = pcl_cloud->at(j,i) ;
        outf << pt.x;
        if (j != width - 1){
          outf << ", ";
        }
      }
      outf << ";" << std::endl;
    }
    outf << "];" << std::endl;

    outf << "depthxyz(:,:,2) = [";
    for(int i = 0; i < height; i++){
      for(int j = 0; j < width; j++){
        auto& pt = pcl_cloud->at(j,i) ;
        outf << pt.y;
        if (j != width - 1){
          outf << ", ";
        }
      }
      outf << ";" << std::endl;
    }
    outf << "];" << std::endl;

    outf << "depthxyz(:,:,3) = [";
    for(int i = 0; i < height; i++){
      for(int j = 0; j < width; j++){
        auto& pt = pcl_cloud->at(j,i) ;
        outf << pt.z;
        if (j != width - 1){
          outf << ", ";
        }
      }
      outf << ";" << std::endl;
    }
    outf << "];" << std::endl;
    outf.close();

    // show image
    cv::imshow("window", img);
    cv::imwrite("./img.png", img);
    int k = cv::waitKey(0);
    cv::destroyAllWindows();
    created_image = true;

    // Processing data to find objects 
    float w = 200; // bounding box width 
    float h = 200; // height 
    float x = 300; // centre x
    float y = 150; // centre y
                  
    float d = GetHeight(pcl_cloud, w, h, x, y);
    ROS_INFO("OBJECT HEIGHT/DEPTH: %.2f", d);
  }
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

int GeneratePath(float (&path)[PATH_SIZE][3], int bbx, int bby, int bbh, int bbw, float r_avoid, float h_layer, float d_cut, float u){

  float bbwu = bbw * u;
  float bbhu = bbh * u; 
  float bbru;
  int sz = 0;
  float maxh; 
  float minh;
  float starth;
  float endh;

  auto& dcpt = pcl_cloud->at(bby,bbx) ;

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
        auto& pt = pcl_cloud->at(j,i) ;
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
        auto& pt2 = pcl_cloud->at(k,j);
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
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 10, cloud_cb);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 1000);
  nav_msgs::Path planned;
  ros::Rate loop_rate(5);
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

      planned.header.frame_id = "map";
      planned.header.stamp = ros::Time::now();
      path_pub.publish(planned);
    }
    // Run callbacks
    ros::spinOnce();
    if(first){
      size = GeneratePath(path, BBX, BBY, BBH, BBW, R_AVOID, H_LAYER, D_CUT, UNCERTAINTY);
      first = false;
    }
    // Wait
    loop_rate.sleep();
  }

  return 0;
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
bool created_image = false;

float GetHeight(pcl::PointCloud<pcl::PointXYZ>& cloud, int bb_w, int bb_h, int bb_x, int bb_y){
  int pad = 10; // Distance around bounding box input to include in scan
  float grad_thresh = 0.5; // Threshold gradient [m] to count as hard edge of object
  ROS_INFO("Processing bounding box: [%d %d], [%d %d]", bb_w, bb_h, bb_x, bb_y);
  float grad;
  float grads[bb_h];
  int grad_cnt = 0; 

  for(int i = (bb_y - floor(bb_h/2) - pad) + 1; i < (bb_y + floor(bb_h/2) + pad); i++){
    for(int j = (bb_x - floor(bb_w/2) - pad) + 1; j < (bb_x + floor(bb_w/2) + pad); j++){
      grad = (cloud.at(j,i).z) - (cloud.at(j-1,i).z);
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
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*cloud, pcl_cloud);
  // This just saves the first captured stereo reading as img and text file of depth values
  if(!created_image){
    int width = pcl_cloud.width;
    int height = pcl_cloud.height;
  // just depth information
    cv::Mat img(height, width, CV_8UC1);
    std::ofstream out("./img.txt");
    out << "[";
    for(int i = 0; i < height; i++){
      for(int j = 0; j < width; j++){
        auto& pt = pcl_cloud.at(j,i) ;
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
        auto& pt = pcl_cloud.at(j,i) ;
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
        auto& pt = pcl_cloud.at(j,i) ;
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
        auto& pt = pcl_cloud.at(j,i) ;
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

int main (int argc, char *argv[]) {
  // init ros
  ros::init(argc, argv, "read_stereo")  ;
  ros::NodeHandle nh;

  // Subscriber for point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 10, cloud_cb);

  ros::spin();
  return 0;
}

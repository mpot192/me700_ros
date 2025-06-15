#include "path_gen/PCHandler.hpp"

PCHandler::PCHandler() {
  PC_.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

PCHandler::~PCHandler() {}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCHandler::GetPC(){
  return PC_;
}

void PCHandler::CallbackPC(const sensor_msgs::PointCloud2ConstPtr& cloud){
  pcl::fromROSMsg(*cloud, *PC_);
  exists = true;
}

#include "path_gen/PCHandler.hpp"

PCHandler::PCHandler() {
  PC_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

PCHandler::~PCHandler() {}

// return point cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCHandler::GetPC(){
  return PC_;
}

// get point cloud from ROS and convert to PCL point cloud
void PCHandler::CallbackPC(const sensor_msgs::PointCloud2ConstPtr& cloud){
  pcl::fromROSMsg(*cloud, *PC_);
  exists = true;
}


// Create model point cloud for visualisation in rviz
void PCHandler::GenerateModelPC(float x_pos, float y_pos, float z_pos){
  if(!generated){
    // create temporary point cloud and resize to match actual
    pcl::PointCloud<pcl::PointXYZRGB> temp;
    int c = PC_->width;
    int r = PC_->height;
    temp.width = c;
    temp.height = r;
    temp.points.resize(temp.width * temp.height);
    // take current point cloud relative to drone and put in absolute position
    for(int i = 0; i < r; i++){
      for(int j = 0; j < c; j++){
        auto& pt_in = PC_->at(j, i);
        auto& pt_out = temp.at(j, i);
        pt_out.x = pt_in.x + x_pos;
        pt_out.y = pt_in.y + y_pos;
        pt_out.z = z_pos - pt_in.z;
        pt_out.r = pt_in.r;
        pt_out.g = pt_in.g;
        pt_out.b = pt_in.b;
      }
    }
    // convert to ros point cloud for publishing
    model_pc_.reset(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(temp, *model_pc_);
    model_pc_->header.frame_id = "map";
    generated = true;
  }
}

// get model pc if required
sensor_msgs::PointCloud2ConstPtr PCHandler::GetModelPC(){
  return model_pc_;
}

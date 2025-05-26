#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud){
  ROS_INFO("Height: %d", cloud->height);
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

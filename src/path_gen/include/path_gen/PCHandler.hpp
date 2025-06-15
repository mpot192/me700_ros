#ifndef PCHANDLER_HPP
#define PCHANDLER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

class PCHandler {
  public:
    PCHandler();
    ~PCHandler();
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetPC();
    void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& cloud);
    bool exists = false;
  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr PC_;
};
#endif // !PCHANDLER_HPP

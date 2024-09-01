#ifndef PUBLISHERS_H
#define PUBLISHERS_H

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>

#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include "common/read_params.h"

class Publishers
{
 private:
  Publishers();

 public:
  static Publishers &getInstance()
  {
    static Publishers pubers;
    return pubers;
  }
  int updateTrace(std::string new_trace_file = std::string());
  void publishCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud);
  void publishTrace(const pcl::PointCloud<pcl::PointXYZI> &cloud);

 private:
  void parseParams(); // read parameters from configuration file
  std::string frame_id;
  // refer area(csv);
  pcl::PointCloud<pcl::PointXYZI> trace_cloud; // trace
  std::string trace_file;
  int parseTraceCloud(); // parse trace from file
  ros::NodeHandle nh;
  void createTrace(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, nav_msgs::Path &trace); // convert cloud to curve line
  std::vector<ros::Publisher> pub_vec;
  ros::V_Publisher pubs;
  void createPubs(); // create ros::Publisher
};

#endif

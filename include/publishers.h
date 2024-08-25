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
  void parseParams(); // 从参数文件中读取参数
  std::string frame_id;
  // refer area(csv);
  pcl::PointCloud<pcl::PointXYZI> trace_cloud; // trace
  std::string trace_file;
  int parseTraceCloud(); // 从文件中解析出trace的数据
  ros::NodeHandle nh;
  void createTrace(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, nav_msgs::Path &trace); // 把点云转换为线
  std::vector<ros::Publisher> pub_vec;
  ros::V_Publisher pubs;
  void createPubs(); // 创建ros::Publisher
};

#endif

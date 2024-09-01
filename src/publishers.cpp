#include "publishers.h"

#include "utlqr.h"

Publishers::Publishers()
{
  parseParams(); // read parameters from configuration file
  createPubs();  // create ros::Publisher
  LOG(INFO) << "Publishers created";
}

// read parameters from configuration file
void Publishers::parseParams()
{
  std::cout << utlqr::get_root_path() << std::endl;
  std::string config_file = utlqr::get_root_path() + "/config/params.yaml";
  YAML::Node config_node  = YAML::LoadFile(config_file);
  std::string field;

  field = "frame_id";
  readParam(config_node[field], frame_id, field);
}

// create ros::Publisher
void Publishers::createPubs()
{
  // 0: trace
  pub_vec.push_back(nh.advertise<nav_msgs::Path>("/trace", 1, true));
  // 1: cloud
  pub_vec.push_back(nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1, true));
}

int Publishers::updateTrace(std::string new_trace_file)
{
  if (new_trace_file.size() == 0)
  {
    LOG(WARNING) << "Path of trace file is empty!";
    return 1;
  }
  else if (!std::filesystem::exists(new_trace_file))
  {
    LOG(WARNING) << "Trace file doesn't exist: " << new_trace_file;
    return 2;
  }
  else if (!std::filesystem::is_regular_file(new_trace_file))
  {
    LOG(WARNING) << "Path of trace file isn't a file: " << new_trace_file;
    return 3;
  }
  else
  {
    trace_file = new_trace_file;
  }
  LOG(INFO) << "Trace file: " << trace_file;

  parseTraceCloud();         // parse trace from csv file
  publishTrace(trace_cloud); // publish trace
  return 0;
}

// publish cloud corresponding topic
void Publishers::publishCloud(const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub_vec[1].publish(cloud_msg);
}

// publish trace
void Publishers::publishTrace(const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  nav_msgs::Path path;
  createTrace(cloud, path);

  pub_vec[0].publish(path);
}

// convert cloud to curve line
void Publishers::createTrace(const pcl::PointCloud<pcl::PointXYZI> &cloud, nav_msgs::Path &trace)
{
  trace.header.frame_id = frame_id;
  trace.poses.clear();
  geometry_msgs::PoseStamped pose_stamped;
  for (const auto &pt : cloud)
  {
    pose_stamped.pose.position.x = pt.x;
    pose_stamped.pose.position.y = pt.y;
    pose_stamped.pose.position.z = pt.z;

    trace.poses.push_back(pose_stamped);
  }
}

// parse trace from file
int Publishers::parseTraceCloud()
{
  std::ifstream fs;

  fs.open(trace_file);
  if (!fs.is_open())
  {
    LOG(WARNING) << "can't open " << trace_file;
    return 1;
  }
  if (!fs)
  {
    LOG(WARNING) << "error: " << trace_file;
    return 2;
  }
  trace_cloud.clear();

  if (!fs.is_open())
  {
    LOG(ERROR) << "Unable to open file for writing." << std::endl;
    return 5;
  }

  if (fs)
  {
    std::string line;
    while (std::getline(fs, line))
    {
      std::string token;
      std::istringstream tokenStream(line);
      char delimiter = ',';
      std::vector<double> vec;
      while (std::getline(tokenStream, token, delimiter))
      {
        vec.push_back(std::stof(token));
      }
      pcl::PointXYZI pt;

      pt.x = vec[0];
      pt.y = vec[1];
      pt.z = vec[2];
      trace_cloud.push_back(pt);
    }
    fs.close();
  }
  return 1;
}
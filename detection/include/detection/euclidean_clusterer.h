#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include "detection/VectorPointCloud.h"
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class EuclideanClusterer
{

public:
  EuclideanClusterer();
  ~EuclideanClusterer();

  void loop();

private:
  // Callbacks
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);
  void phase_cb(const std_msgs::Int8 phaseMode);

  //
  void params();

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber velodyne_sub, sub_phase;

  // Publishers
  ros::Publisher pub_pointclouds;
  std::vector<ros::Publisher> pub_vec;

  // Variables

  // Params
  int phase; // 1 Docking - 2 Harbor - 3 Sea
  float distanceThreshold, clusterTolerance, minClusterSize, maxClusterSize;
};
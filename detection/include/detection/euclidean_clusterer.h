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

private:
  // Callbacks
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);
  void phaseCallback(const std_msgs::Int8 phaseMode);

  //
  void getParameters();

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_velodyne, sub_phase;

  // Publishers
  ros::Publisher pub_point_clouds;
  std::vector<ros::Publisher> pub_vec_point_clouds;

  // Variables

  // getParameters
  int phase; // 1 Docking - 2 Harbor - 3 Sea
  float distance_threshold, cluster_tolerance, min_cluster_size, max_cluster_size;
};
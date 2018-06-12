#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

class EuclideanClusterer
{

public:
  EuclideanClusterer();
  ~EuclideanClusterer();

  void loop();

private:
  // Callbacks
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);

  //
  void params();

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber cloud_sub;

  // Publishers
  ros::Publisher pub_pointclouds;
  std::vector<ros::Publisher> pub_vec;
  
  // Variables

  // Params
  float distanceThreshold, clusterTolerance, minClusterSize, maxClusterSize, phase;
};
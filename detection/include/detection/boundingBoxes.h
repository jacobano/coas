#include "detection/vectorPointCloud.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <Eigen/Eigen>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

class BoundingBoxes
{

public:
  BoundingBoxes();
  ~BoundingBoxes();

  void loop();

private:
  // Callbacks
  void test_cb(const detection::vectorPointCloud input);
  void resetVariables();
  void calculateCenters();
  void constructBoundingBoxes();
  void postProcess();
  void postProcess1(const detection::vectorPointCloud clusters);
  float dist2Points(float x1, float y1, float z1, float x2, float y2, float z2);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_vector_pointclouds;

  // Publishers
  ros::Publisher pub_bigDist, pub_smallDist, pub_heightDist, pub_boxArray, pub_boxArray1;

  // Variables
  int j;
  int label_box;
  Eigen::Vector4f centroid, polygon_centroid;
  float maxDistX, maxDistY, maxDistZ, xMax, xMin, yMax, yMin, zMax, zMin, centerX, centerY, centerZ, maxDistXpol, maxDistYpol, maxDistZpol;
  jsk_recognition_msgs::BoundingBox box, box1;
  jsk_recognition_msgs::BoundingBoxArray boxes, boxes1;
  std::vector<geometry_msgs::PoseStamped> polygon;
  geometry_msgs::PoseStamped pose;
  pcl::PointCloud<pcl::PointXYZ> polygon_cloud;
  pcl::PointXYZ polygon_pose;
  // std::vector<std::vector<int>> vec_polygons;
  // std::vector<int> polygon_label;
  std::vector<std::map<int, geometry_msgs::PoseStamped>> vec_polygons;
  std::map<int, geometry_msgs::PoseStamped> poligono;
};
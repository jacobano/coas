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
  void clusters_cb(const detection::vectorPointCloud input);
  
  // 
  void params();
  void resetVariables();
  void cleanVariables();
  void calcCenters();
  void constructBoundingBoxes(float x, float y, float z, float dimX, float dimY, float dimZ, bool merge);
  void postProcess();
  void calcVecPolygons();
  void mergeBoundingBoxes();
  void calcMaxDistancesCluster(const pcl::PointCloud<pcl::PointXYZ> cluster);
  float dist2Points(float x1, float y1, float z1, float x2, float y2, float z2);
  Eigen::Vector4f pc2_centroid(const sensor_msgs::PointCloud2 pc2);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_vector_pointclouds;

  // Publishers
  ros::Publisher pub_boxArray, pub_mergeBoxesArray, pub_boxRef;

  // Variables
  int label_box, label_mergeBox;
  float xMax, xMin, yMax, yMin, zMax, zMin, centerX, centerY, centerZ;
  float maxDistX, maxDistY, maxDistZ, maxDistXpol, maxDistYpol, maxDistZpol, maxBoxX, maxBoxY, maxBoxZ;
  Eigen::Vector4f centroid;
  jsk_recognition_msgs::BoundingBox box;
  jsk_recognition_msgs::BoundingBoxArray boxes, mergeBoxes;
  std::vector<int> polygon_labels;
  std::vector<std::vector<int>> vec_polygon_labels;
  int cont_cb = 0;
  bool found = false;
  bool repeat = false;
  detection::vectorPointCloud clusters;
  // Params 
  int phase;
  float close_dist, xyMinPoste, xyMaxPoste, zMinPoste, zMaxPoste;
};
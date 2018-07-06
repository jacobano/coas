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
  nav_msgs::Path constructPath(std::vector<float> x, std::vector<float> y, std::vector<float> z, int length);
  void checkPostes(float xDim, float yDim, float zDim);
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
  ros::Publisher pub_boxArray, pub_mergeBoxesArray, pub_boxesRef, pub_pathPoste1, pub_pathPoste2, pub_pathPoste3, pub_pathPoste12, pub_pathPoste13, pub_pathPoste23;

  // Variables
  int cont_postes = 0;
  int label_box, label_mergeBox;
  float xMax, xMin, yMax, yMin, zMax, zMin, centerX, centerY, centerZ;
  float maxDistX, maxDistY, maxDistZ, maxDistXpol, maxDistYpol, maxDistZpol, maxBoxX, maxBoxY, maxBoxZ;
  Eigen::Vector4f centroid;
  nav_msgs::Path pathPoste1, pathPoste2, pathPoste3, pathPoste12, pathPoste13, pathPoste23;
  jsk_recognition_msgs::BoundingBox box;
  jsk_recognition_msgs::BoundingBoxArray boxes, boxesRef, mergeBoxes;
  std::vector<int> polygon_labels;
  std::vector<std::vector<int>> vec_polygon_labels;
  // Params 
  int phase;
  float close_dist, xyMinPoste, xyMaxPoste, zMinPoste, zMaxPoste, minDistPoste12, maxDistPoste12, minDistPoste13, maxDistPoste13;
};
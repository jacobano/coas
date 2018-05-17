#include "detection/vectorPointCloud.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <Eigen/Eigen>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

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

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_vector_pointclouds;

  // Publishers
  ros::Publisher pub_bigDist, pub_smallDist, pub_heightDist, pub_boxArray;

  // Variables
  int j;
  Eigen::Vector4f centroid;
  float maxDistX, maxDistY, maxDistZ, xMax, xMin, yMax, yMin, zMax, zMin, centerX, centerY, centerZ;
  jsk_recognition_msgs::BoundingBox box;
  jsk_recognition_msgs::BoundingBoxArray boxes;
};
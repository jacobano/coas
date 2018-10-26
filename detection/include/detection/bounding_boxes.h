#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include "detection/VectorPointCloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

class BoundingBoxes
{

public:
  BoundingBoxes();
  ~BoundingBoxes();

private:
  // Callbacks
  void phaseCallback(const std_msgs::Int8 phaseMode);
  //void clustersCallback(const detection::VectorPointCloud input);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);  
  //
  void getParameters();
  void calculateCenters();
  void postProcess();
  void resetVariables();
  void cleanVariables();
  void calculateVectorPolygons();
  void mergeBoundingBoxes();
  void savePose(int nPost, nav_msgs::Path path);
  void checkPostDimension(float xDim, float yDim, float zDim);
  void calculateMaxDistancesCluster(const pcl::PointCloud<pcl::PointXYZ> cluster);
  float calculateDistance2Points(float x1, float y1, float z1, float x2, float y2, float z2);
  void saveDistances(bool b, nav_msgs::Path path1, nav_msgs::Path path2, nav_msgs::Path path3);
  void constructBoundingBoxes(float x, float y, float z, float dimX, float dimY, float dimZ, bool merge);
  nav_msgs::Path constructPath(std::vector<float> x, std::vector<float> y, std::vector<float> z, int length);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_velodyne; // PointCloud subscriber
  ros::Subscriber sub_phase;
  
  // Cluster Publishers
  ros::Publisher pub_point_clouds;
  std::vector<ros::Publisher> pub_vec_point_clouds;
  // Bounding Boxer Publishers
  ros::Publisher pub_boxes, pub_merge_boxes, pub_reference_boxes;
  ros::Publisher pub_path_post_1, pub_path_post_2, pub_path_post_3, pub_path_post_12, pub_path_post_13, pub_path_post_23;

  // Variables
  int counter_posts = 0;
  Eigen::Vector4f centroid;
  int label_box, label_merge_box;
  std::vector<int> vec_label_polygon;
  std::vector<std::vector<int>> vec_vec_label_polygon;
  jsk_recognition_msgs::BoundingBox box;
  jsk_recognition_msgs::BoundingBoxArray boxes, reference_boxes, merge_boxes;
  float x_max, x_min, y_max, y_min, z_max, z_min, x_center, y_center, z_center;
  float max_dist_x, max_dist_y, max_dist_z, max_dist_x_polygon, max_dist_y_polygon, max_dist_z_polygon;
  nav_msgs::Path path_post_1, path_post_2, path_post_3, path_post_12, path_post_13, path_post_23;

  std::string log_output;
  int contTest, contTestPose;
  double time_start, time_pose_start;
  std::ofstream file_distance_to_posts, file_distance_between_posts, file_distance_to_posts_times, file_distance_between_posts_times;
  std::ofstream file_post_1, file_post_2, file_post_3, file_post_1_time, file_post_2_time, file_post_3_time;

  // Phase
  int phase; // 1 Docking - 2 Harbor - 3 Sea
  // Euclidean Clusterer specific parameters
  float distance_threshold, cluster_tolerance, min_cluster_size, max_cluster_size;
  // Bounding Boxes specific parameters
  float close_distance, xy_min_post, xy_max_post, z_min_post, z_max_post, min_distance_post_12, max_distance_post_12, min_distance_post_13, max_distance_post_13;
};
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "visualization_msgs/Marker.h"

class Matching
{

public:
  Matching();
  ~Matching();

  // void loop();
  void toDo();

private:
  //
  void drawPosts();
  void savePose(int nPost, geometry_msgs::PoseStamped waypoint);
  float calculateDistance2Points(float x1, float y1, float z1, float x2, float y2, float z2);

  // Callbacks
  void pathPost1Callback(const nav_msgs::Path path);
  void pathPost2Callback(const nav_msgs::Path path);
  void pathPost3Callback(const nav_msgs::Path path);
  void pathPost12Callback(const nav_msgs::Path path);
  void pathPost13Callback(const nav_msgs::Path path);
  void pathPost23Callback(const nav_msgs::Path path);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_path_post_1, sub_path_post_2, sub_path_post_3, sub_path_post_12, sub_path_post_13, sub_path_post_23;

  // Publishers
  ros::Publisher pub_marker_1, pub_marker_2, pub_marker_3, pub_marker_4, pub_marker_5, pub_marker_6;

  // Variables
  typedef std::pair<int, float> pair;
  int counter;
  std::vector<float> vec_distance;
  std::map<int, float> vec_distance_and_label;
  nav_msgs::Path now_path_post_1, now_path_post_2, now_path_post_3, now_path_post_12, now_path_post_13, now_path_post_23, prev_path_posts, now_path_posts;
  std::ofstream file_post_1, file_post_2, file_post_3, file_post_1_time, file_post_2_time, file_post_3_time;
  std::string logOutput;
  double startTimePose;
  // Markers
  visualization_msgs::Marker marker_post_1, marker_post_2, marker_post_3, marker_post_4, marker_post_5, marker_post_6;
};
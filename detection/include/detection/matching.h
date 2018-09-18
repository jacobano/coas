#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "visualization_msgs/Marker.h"

class Matching
{

public:
  Matching();
  ~Matching();

  void loop();
  void toDo();

private:
  //
  void drawPosts();
  float dist2Points(float x1, float y1, float z1, float x2, float y2, float z2);

  // Callbacks
  void cb_pathPoste1(const nav_msgs::Path path);
  void cb_pathPoste2(const nav_msgs::Path path);
  void cb_pathPoste3(const nav_msgs::Path path);
  void cb_pathPoste12(const nav_msgs::Path path);
  void cb_pathPoste13(const nav_msgs::Path path);
  void cb_pathPoste23(const nav_msgs::Path path);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_pathPoste1, sub_pathPoste2, sub_pathPoste3, sub_pathPoste12, sub_pathPoste13, sub_pathPoste23;

  // Publishers
  ros::Publisher pub_marker1, pub_marker2, pub_marker3, pub_marker4, pub_marker5, pub_marker6;

  // Variables
  typedef std::pair<int, float> pair;
  int cont; 
  std::vector<float> vec_dist;
  std::map<int, float> vec_distAndLabel;
  nav_msgs::Path nowPathPost1, nowPathPost2, nowPathPost3, nowPathPost12, nowPathPost13, nowPathPost23, prevPathPosts, nowPathPosts;

  // Markers
  visualization_msgs::Marker marker_post1, marker_post2, marker_post3, marker_post4, marker_post5, marker_post6;
};
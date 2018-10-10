#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
// --
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>

class VolexFilter
{

public:
  VolexFilter();
  ~VolexFilter();

  void loop();

private:
  // Callbacks
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber cloud_sub;

  // Publishers
  ros::Publisher pub;
  std::vector<ros::Publisher> pub_vec;

  // Variables
};
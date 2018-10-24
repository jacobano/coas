#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class VoxelFilter
{

public:
  VoxelFilter();
  ~VoxelFilter();

  void loop();

private:
  // Callbacks
  void sensorCallback(const sensor_msgs::PointCloud2ConstPtr &input);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers

  // Publishers
  ros::Publisher pub_voxel_filter_points;

  // Variables
};
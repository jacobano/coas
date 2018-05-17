#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>
#include <geodesy/utm.h>
#include <Eigen/Geometry>
#include <geographic_msgs/GeoPoint.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "detection/geographic_to_UTM_to_cartesian.hpp"

class Pcl2RotTrans
{

public:
  Pcl2RotTrans();
  ~Pcl2RotTrans();

  void loop();

private:
  // Callbacks
  void gps_cb(sensor_msgs::NavSatFix msg);
  void velodyne_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input);
  void imu_cb(sensor_msgs::Imu msg);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber velodyne_sub, imu_sub, velocity_sub, gps_sub;     

  ros::Publisher pub_map_imu, pub_gps_vel;

  // Variables
  bool flag = false;
  geographic_msgs::GeoPoint geo_pt, geo_pt_origin;
  float despX, despY, despZ, rotPitch, rotRoll, rotYaw; // Translation Matrix Rotation & Translation
  float lat, lon, alt, originx, originy;
};
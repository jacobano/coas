#include <ros/ros.h>
#include <fstream>
#include <std_msgs/Int8.h>
#include "mapping/vectorInt.h"
#include "mapping/vectorVector.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class MapFilter
{

public:
  MapFilter();
  ~MapFilter();

  void loop();

private:
  // Callbacks
  void exploration(const sensor_msgs::PointCloud2 &cloud2d_usv);
  void receiveSensor(const sensor_msgs::PointCloud2 &cloud);

  typedef std::vector<float> VI;
  typedef std::vector<VI> VVI;
  typedef std::vector<VVI> VVVI;

  //
  int is_in_map(int i, int j);
  int readV(const VVVI &V, int i, int j);
  void save_matrix(char *filename, const VVVI &M);
  void save_matrix3d(char *fileName, const VVVI &m, bool vel);
  void phase_cb(const std_msgs::Int8 phaseMode);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_phase, sub;

  // Publishers
  ros::Publisher pub_filtered_map2, pub_matrix;

  // Variables
  int phase, range_dock, range_sea, rang, rows, columns;
  float cell_div; // number of cell per meter

  // Params
};
#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>

#include "mapping/vectorVector.h"

class ThinMapping
{

public:
  ThinMapping();
  ~ThinMapping();

  void loop();

private:
  // Callbacks
  void matrix_cb(const mapping::vectorVector input);

  //
  void params();
  void save_matrix(char *fileName, const std::vector<std::vector<int>> &M);
  float dist2Points(float x1, float y1, float z1, float x2, float y2, float z2);
  std::vector<Eigen::Vector3i> voxel_traversal(Eigen::Vector3d ray_start, Eigen::Vector3d ray_end);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber matrix_sub;

  // Publishers
  //ros::Publisher pub;

  // Variables
  int cell_div = 2;
  int range_dock, rang, rr, cc;
  double _bin_size = 1;

  // Params
  int phase;
};
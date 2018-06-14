#include <ros/ros.h>
#include <Eigen/Dense>
// fast
#include <Eigen/Core>
#include <cfloat>
#include <vector>
#include <iostream>
// fast
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
  std::vector<Eigen::Vector3i> voxel_traversal(Eigen::Vector3d ray_start, Eigen::Vector3d ray_end);
  float dist2Points(float x1, float y1, float z1, float x2, float y2, float z2);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber matrix_sub;

  // Publishers
  //ros::Publisher pub;

  // Variables
  double _bin_size = 1;
  int rang, rr, cc;

  // Params
  int phase;
};
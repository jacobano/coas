#include <ros/ros.h>
#include "detection/timeNode.h"

class Timers
{

public:
  Timers();
  ~Timers();

  void loop();

private:
  // Callbacks
  void mapp_cb(const detection::timeNode _time);
  void eucl_cb(const detection::timeNode _time);
  void bbxs_cb(const detection::timeNode _time);

  // Node handlers
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_mapp, sub_eucl, sub_bbxs;

  // Publishers

  // Variables
  detection::timeNode timeMapp, timeEucl, timeBbxs, timeTotal;
  bool flagMapp, flagEucl, flagBbxs;
};
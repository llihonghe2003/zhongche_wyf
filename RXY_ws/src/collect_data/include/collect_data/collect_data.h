#ifndef COLLECT_DATA_H
#define COLLECT_DATA_H

#include <nlink_parser/LinktrackNodeframe2.h>
// #include
// </home/xtark/RXY_ws/devel/include/nlink_parser/LinktrackNodeframe2.h>
#include <rosbag/bag.h>
#include <signal.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

namespace CollectData {

struct Pose2D {
  double x;
  double y;
  double h;
  double v;
  double vy;
  double r;
  double ax;
  double ay;
  double w;
};

class Collector {
 private:
  ros::NodeHandle rosNode;
  double rosLoopHz;
  rosbag::Bag bag;
  ros::Subscriber subPoints;

  Pose2D nowPos;

 public:
  Collector();
  Collector(ros::NodeHandle &nh, const double frq);
  virtual ~Collector();

 public:
  void run();
  void DoCollect(const ros::TimerEvent &e);

  void Callback_UpdateLidar(const sensor_msgs::LaserScan &msg);
  void Callback_UpdateUwb(
      const nlink_parser::LinktrackNodeframe2::ConstPtr &msg);
  void Callback_UpdateCamera(const sensor_msgs::LaserScan &msg);
};
}  // namespace CollectData

#endif
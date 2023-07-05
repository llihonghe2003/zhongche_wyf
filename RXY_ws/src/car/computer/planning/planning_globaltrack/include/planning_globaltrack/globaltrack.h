#ifndef _GLOBALTRACK_H
#define _GLOBALTRACK_H

#include <math.h>
#include <ros/ros.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <nav_msgs/Odometry.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include "cta_msgs/WayPoint2d.h"
#include "cta_msgs_perception/SelfPose.h"
#include "cta_msgs_planning/DecisionBehavior.h"
#include "cta_msgs_planning/GlobalRoute2d.h"
#include "cta_msgs_planning/LocalRoute2d.h"
#include "usrlib/usrlib.h"
// extern XmlRpc::XmlRpcValue params456;
namespace PlannerGlobalTrack {

struct Pose2D {
  double x;
  double y;
  double h;
  double v;
};

typedef std::vector<Pose2D> Path;

class Planner {
 private:
  ros::NodeHandle rosNode;
  double rosLoopHz;

  ros::Publisher pubBehDec;
  ros::Publisher pubGlbRoute;
  ros::Publisher pubLocTrj;

  ros::Subscriber subSelfPose;
  ros::Subscriber subSelfTheta;
  ros::Subscriber subSelfPoints;

  bool isRun;
  Path glbRoute;

  Pose2D nowPos;
  cta_msgs_planning::DecisionBehavior msgBeh;
  cta_msgs_planning::GlobalRoute2d msgGlbRoute;
  cta_msgs_planning::LocalRoute2d msgLocTrj;

 public:
  Planner();
  Planner(ros::NodeHandle &nh, const double frq);
  virtual ~Planner();
  void run();

 public:
  void Callback_UpdateSelfPose(
      const cta_msgs_perception::SelfPose::ConstPtr &msg);
  void Callback_UpdateTheta(const nav_msgs::Odometry::ConstPtr &msg);
  void Callback_UpdatePoints(
      const nlink_parser::LinktrackNodeframe2::ConstPtr &msg);

  bool PlanBehavior();
  bool PlanGlobalRoute();
  bool PlanLocalTrajectory();

  bool SetGobalRoute(std::string filename);
  int GetRefernum(const Pose2D pos);

  cv::Mat Plot(const std::string win, const std::vector<double> x,
               const std::vector<double> y, const cv::Scalar color,
               int linewidth);

  double Dis(const Pose2D p1, const Pose2D p2);
  double Ang(const Pose2D p1, const Pose2D p2);
  double Uag(const double a);
};

}  // namespace PlannerGlobalTrack

#endif  //_GLOBALTRACK_H
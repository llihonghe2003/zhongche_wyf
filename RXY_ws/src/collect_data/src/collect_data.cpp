#include "collect_data.h"

namespace CollectData {

Collector::Collector() {}

Collector::Collector(ros::NodeHandle &nh, const double frq) {
  this->rosNode = nh;
  this->rosLoopHz = frq;
  // subscribe uwb msg

  subPoints = rosNode.subscribe("nlink_linktrack_nodeframe2", 1,
                                &Collector::Callback_UpdateUwb, this);
  // bag.open("src/collect_data/scripts/hello.bag", rosbag::BagMode::Write);
  bag.open(ros::package::getPath("collect_data") + "/scripts/hello.bag",
           rosbag::BagMode::Write);
  bag.close();
}
Collector::~Collector() { bag.close(); }
void Collector::run() {
  ros::Timer t = rosNode.createTimer(ros::Duration(1 / Collector::rosLoopHz),
                                     &Collector::DoCollect, this);
  ros::spin();
}
void Collector::DoCollect(const ros::TimerEvent &e) {
  ROS_INFO("hello!");
  // bag.open("src/collect_data/scripts/hello.bag", rosbag::BagMode::Append);
  bag.open(ros::package::getPath("collect_data") + "/scripts/hello.bag",
           rosbag::BagMode::Append);
  std_msgs::String msg;
  msg.data = "Hello 2xxx2";
  // topic time msg
  bag.write("/chatter", ros::Time::now(), msg);
  bag.close();
  bag.open(ros::package::getPath("collect_data") + "/scripts/hello.bag",
           rosbag::BagMode::Append);
  std_msgs::String msg2;
  msg2.data = "Hello 3xxx3";
  // topic time msg
  bag.write("/hehe", ros::Time::now(), msg2);
  bag.close();
}

void Collector::Callback_UpdateLidar(const sensor_msgs::LaserScan &msg){};
void Collector::Callback_UpdateUwb(
    const nlink_parser::LinktrackNodeframe2::ConstPtr &msg) {
  nowPos.x = msg->pos_3d[0];
  nowPos.y = msg->pos_3d[1];
  bag.open(ros::package::getPath("collect_data") + "/scripts/hello.bag",
           rosbag::BagMode::Append);
  // topic time msg
  bag.write("/nlink_linktrack_nodeframe2", ros::Time::now(), msg);
  bag.close();
};
void Collector::Callback_UpdateCamera(const sensor_msgs::LaserScan &msg){};

}  // namespace CollectData
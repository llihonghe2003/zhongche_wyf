#include "../include/planning_globaltrack/globaltrack.h"
#include "signal.h"

void KillSigintHandler(int sig) {
  //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  ROS_INFO("Shutting Down Planner: Global-Route !!!!!!!");
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv) {
  //初始化ROS节点，节点名称请使用与ROS包名相同
  ros::init(argc, argv, "planning_globaltrack", ros::init_options::NoSigintHandler);
  signal(SIGINT, KillSigintHandler);
  setlocale(LC_ALL, "");
  ros::param::get("params", UsrLib::params);
  ROS_ERROR_STREAM("2当前车辆是：" << UsrLib::params);
  //定义类对象指针
  ros::NodeHandle nh;
  double loopHz = 10;
  PlannerGlobalTrack::Planner* exe = new PlannerGlobalTrack::Planner(nh, loopHz);
  //调用对象run函数，外部直接调用的类应当包含至少一个run函数
  exe->run();
  //关闭ROS节点
  ros::shutdown();
  return 0;
}

#include "MPC.h"

int main(int argc, char** argv) {
  //初始化ROS节点，节点名称请使用与ROS包名相同
  ros::init(argc, argv, "MPC_controller");
  //定义类对象指针
  ros::NodeHandle nh;
  double loopHz = 10;
  setlocale(LC_ALL, "");
  ros::param::get("params", UsrLib::params);

  if (UsrLib::params["name"] == "xtark_01") {
    ros::param::set("params/avo_name", "xtark_02");
    ros::param::get("params", UsrLib::params);
    ROS_ERROR_STREAM("当前障碍物是：" << UsrLib::params["avo_name"]);
  } else if (UsrLib::params["name"] == "xtark_02") {
    ros::param::set("params/avo_name", "xtark_01");
    ros::param::get("params", UsrLib::params);
    ROS_ERROR_STREAM("当前障碍物是：" << UsrLib::params["avo_name"]);
  }

  ROS_ERROR_STREAM("当前车辆是：" << UsrLib::params);
  getParam<string>("params/environment", "123");
  MPCtrack::Controller* exe = new MPCtrack::Controller(nh, loopHz);
  // PureTrack::Controller *exe = new PureTrack::Controller(nh, loopHz);
  //调用对象run函数，外部直接调用的类应当包含至少一个run函数
  exe->run();
  //关闭ROS节点
  ros::shutdown();
  return 0;
}
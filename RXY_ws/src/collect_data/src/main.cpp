// #include "MPC.h"
#include "collect_data.h"

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");
  //初始化ROS节点，节点名称请使用与ROS包名相同
  ros::init(argc, argv, "collect_data");

  //定义类对象指针
  ros::NodeHandle nh;
  double loopHz = 10;

  XmlRpc::XmlRpcValue params;
  nh.getParam("one", params);
  // ROS_INFO(typeof(params))
  std::cout << params["three"] << std::endl;
  CollectData::Collector* exe = new CollectData::Collector(nh, loopHz);

  //调用对象run函数，外部直接调用的类应当包含至少一个run函数
  exe->run();
  //关闭ROS节点
  ros::shutdown();
  return 0;
}
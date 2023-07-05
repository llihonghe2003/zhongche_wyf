#include "../include/usrlib/usrlib.h"

#include <yaml-cpp/yaml.h>
#include <fstream>

XmlRpc::XmlRpcValue params123;

// XmlRpc::XmlRpcValue params456;
int main(int argc, char** argv) {
  //初始化ROS节点，节点名称请使用与ROS包名相同
  ros::init(argc, argv, "usrfunction");
  //定义类对象指针
  ros::NodeHandle nh;
  double loopHz = 10;
  setlocale(LC_ALL, "");
  std::string TOPIC_CONTROL_LAMP = "msg_control_lamp";

  ros::param::get("params", UsrLib::params);

  //读取文件
  std::string param_file = "/home/xtark/xtark_01/RXY_ws/src/test_tools/config/config_bak.yaml";
  YAML::Node yamlConfig = YAML::LoadFile(param_file);

  //读取父项和子项
  // YAML::Node parent_param = yamlConfig["params"]["name"];
  // YAML::Node chile_param = parent_param["chile_param"];
  yamlConfig["params"]["name"] = 10;
  //修改
  // parent_param["name"] = "xtark_03";
  // yamlConfig["params/data_num"] = 10;

  //写
  // std::ofstream file;
  // file.open(param_file);
  // file.flush();
  // file << yamlConfig;

  ros::Rate r(1);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  ros::shutdown();
  return 0;
}

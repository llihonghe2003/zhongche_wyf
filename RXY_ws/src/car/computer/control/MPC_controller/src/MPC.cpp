#include "MPC.h"
double Distance = 0.3;

ofstream OutFile;
ofstream paramFile;
// MPC class definition implementation.
//
// 构造函数

cta_msgs::PosAll PosAll_3;
namespace MPCtrack {
MPC::MPC() {}
//析构函数
MPC::~MPC() {}

// 一个内联函数

// 传入参数 : 初始条件 和 参考曲线的三次多项式系数
// 传出参数 : Solution 结构体，包含 六个变量序列 加两个控制量序列
Solution MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd x_avoidance, Eigen::VectorXd X_ref, Eigen::VectorXd Y_ref, Eigen::VectorXd Theta_ref, Eigen::VectorXd V_ref, Eigen::VectorXd control_out) {
  size_t i;

  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = x0[0];
  double y = x0[1];
  double theta = x0[2];

  double x_avo = x_avoidance[0];
  double y_avo = x_avoidance[1];
  double theta_avo = x_avoidance[2];
  double v_avo = x_avoidance[3];

  // 得到误差变量
  double e_x = (cos(theta) * (X_ref[0] - x0[0]) + sin(theta) * (Y_ref[0] - x0[1])) * 1;
  double e_y = (-sin(theta) * (X_ref[0] - x0[0]) + cos(theta) * (Y_ref[0] - x0[1])) * 1;
  double e_theta = Theta_ref[0] - x0[2];

  if (e_theta > 6.0) {
    e_theta -= 2 * pi;
  }
  if (e_theta < -6.0) {
    e_theta += 2 * pi;
  }

  // number of independent variables
  // N timesteps == N - 1 actuations
  // 计算变量个数 状态(e_x, e_y, e_theta) * 预测步长 + 控制量 * (预测步长 - 1)
  // 状态多一个是因为 初始状态也算变量
  // 三个 误差状态 + 两个控制量 + 2个辅助变量状态
  size_t n_vars = (N + 1) * 3 + N * 4 + N * 2 + N * 2;

  // Number of constraints
  size_t n_constraints = (N + 1) * 3 + N * 4 + N * 2;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);

  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;  // 变量清0
  }
  // Set the initial variable values
  // 传入初始状态

  vars[e_x_start] = e_x;
  vars[e_y_start] = e_y;
  vars[e_theta_start] = e_theta;

  Eigen::VectorXd e0(3);
  e0 << e_x, e_y, e_theta;

  // 建立数学模型
  FG_eval fg_eval(e0, x_avoidance, X_ref, Y_ref, Theta_ref, V_ref, control_out);

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  // 把所有状体变量都 置 0
  for (int i = 0; i < v_start; i++) {
    // 正负无穷
    vars_lowerbound[i] = -1000;
    vars_upperbound[i] = 1000;
  }

  // v约束
  for (int i = v_start; i < omega_start; i++) {
    vars_lowerbound[i] = 0;
    vars_upperbound[i] = 5;
  }

  // w约束
  for (int i = omega_start; i < dV_start; i++) {
    vars_lowerbound[i] = -1000;
    vars_upperbound[i] = 1000;
  }

  // dv约束
  // for (int i = dV_start; i < dV_start+1; i++)
  // {
  // 	vars_lowerbound[i] =-100;
  // 	vars_upperbound[i] =  100;
  // }

  // dDelta约束
  for (int i = dDelta_start; i < dDelta_start + 1; i++) {
    vars_lowerbound[i] = -0.1;
    vars_upperbound[i] = 0.1;
  }

  // obstacle constance
  for (int i = x_obstacle; i < y_obstacle; i++) {
    vars_lowerbound[i] = -1000;
    vars_upperbound[i] = 1000;
  }
  for (int i = y_obstacle; i < v_e_start; i++) {
    vars_lowerbound[i] = -1000;
    vars_upperbound[i] = 1000;
  }

  // v_e约束
  for (int i = v_e_start; i < omega_e_start; i++) {
    vars_lowerbound[i] = -1000;
    vars_upperbound[i] = 1000;
  }
  // Acceleration/decceleration upper and lower limits.

  // 注意 这里一定要是 w_e 是最后的向量
  for (int i = omega_e_start; i < n_vars; i++) {
    // w_e约束
    vars_lowerbound[i] = -1000;
    vars_upperbound[i] = 1000;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.

  //状态约束
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (int i = 0; i < n_constraints; i++) {
    // 等式约束的右边都是0
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[e_x_start] = e_x;
  constraints_lowerbound[e_y_start] = e_y;
  constraints_lowerbound[e_theta_start] = e_theta;

  constraints_upperbound[e_x_start] = e_x;
  constraints_upperbound[e_y_start] = e_y;
  constraints_upperbound[e_theta_start] = e_theta;

  // 初始 的 状态约束 赋值 为当前状态

  // Options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // Place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // Solve the problem
  // 求解

  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

  bool ok = true;

  //检查是否求解成功
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  if (ok) {
    // cout << "求解成功 ！ " << solution.status << endl;
  } else {
    // cout << "求解失败 ！ " << solution.status << endl;
    // return 0;
  }

  Solution sol;

  for (int i = 0; i <= N - 1; i++) {
    //  cout << i << ": " << "E_x is : " << solution.x[e_x_start+i] << "  E_y
    //  is: " << solution.x[e_y_start+i]
    //       << "  E_theta is : " << solution.x[e_theta_start+i] << endl;

    cout << i << ". V_e is : " << solution.x[v_e_start + i] << "，  Omega_e is: " << solution.x[omega_e_start + i] << endl;
    sol.E_X.push_back(solution.x[e_x_start + i]);
    sol.E_Y.push_back(solution.x[e_y_start + i]);
    sol.E_Theta.push_back(solution.x[e_theta_start + i]);
    sol.V_e.push_back(solution.x[v_e_start + i]);
    sol.Omega_e.push_back(solution.x[omega_e_start + i]);
  }
  sol.status = solution.status;
  double cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  return sol;
}

Controller::Controller() {}

Controller::Controller(ros::NodeHandle &nh, const double frq) {
  this->rosNode = nh;
  this->rosLoopHz = frq;
  // OutFile.open(string("/home/xtark/") + string(UsrLib::params["name"]) + string("/RXY_ws/src/car/computer/control/MPC_controller/trajectory/ch_data.txt"));
  const char *data_path = nullptr;
  std::string path = string("/home/")+ string(UsrLib::params["type"]) +string("/")+ string(UsrLib::params["name"]) + string("/RXY_ws/src/data/") + to_string(int(UsrLib::params["data_num"]));  //+ string(UsrLib::params["name"]) + string("/RXY_ws/src/data/") + string(UsrLib::params["data_num"]);
  data_path = path.c_str();
  ROS_INFO_STREAM("创建" << UsrLib::params["data_num"] << "数据文件夹:" << mkdir(data_path, 0755));
  string config_path = string("/home/") + string(UsrLib::params["type"]) +string("/")+ string(UsrLib::params["name"]) + string("/RXY_ws/src/test_tools/config/config.yaml");
  YAML::Node yamlConfig = YAML::LoadFile(config_path);
  // int num = yamlConfig["params"]["data_num"].as<int>();
  yamlConfig["params"]["data_num"] = yamlConfig["params"]["data_num"].as<int>() + 1;
  OutFile.open(string("/home/") + string(UsrLib::params["type"]) +string("/")+ string(UsrLib::params["name"]) + string("/RXY_ws/src/data/") + to_string(int(UsrLib::params["data_num"])) + string("/") + string(UsrLib::params["name"]) + string("_data.txt"));

  // ros::param::set("params/data_num", atoi(string(UsrLib::params["data_num"]).c_str()) + 1);

  paramFile.open(config_path);
  paramFile.flush();
  paramFile << yamlConfig;
  paramFile.close();

  //发布器

  pubMode = rosNode.advertise<cta_msgs_control::ControlMode>(UsrLib::TOPIC_CONTROL_MODE, 1);
  pubMotion = rosNode.advertise<cta_msgs_control::MotionControl>(UsrLib::TOPIC_CONTROL_MOTION, 1);
  pubLamp = rosNode.advertise<cta_msgs_control::LampControl>(UsrLib::TOPIC_CONTROL_LAMP, 1);

  // control_ada_ = rosNode.advertise<geometry_msgs::Twist>("cmd_vel", 10);      //发布控制信号
  control_ada_ = rosNode.advertise<geometry_msgs::Twist>("smoother_cmd_vel", 10);      //发布控制信号
  pos_3 = rosNode.advertise<cta_msgs_control::PredictPoint>("poseAll_3", 1);  //发布3车最优位置序列

  //订阅器
  if (UsrLib::params["environment"] == "simulation") {
    ROS_INFO("订阅仿真位置信息");
    std::cout << UsrLib::params["simulation"] << "!" << endl;
    //订阅本车规划的参考位置
    subPlanTrj = rosNode.subscribe(UsrLib::TOPIC_PLAN_LOCALROUTE, 1, &Controller::Callback_UpdatePlanTrj, this);

    //订阅本车的仿真位置
    subSelfPose = rosNode.subscribe(UsrLib::TOPIC_PERCEPTION_MOTIONINFO, 1, &Controller::Callback_UpdateSelfPose, this);

    //订阅障碍的仿真位置
    AvoidancePose = rosNode.subscribe(string("/") + string(UsrLib::params["avo_name"]) + string("/msg_perception_selfpose"), 1, &Controller::Callback_AvoidancePose, this);

    //订阅avoidance的仿真位置
    // AvoidancePose = rosNode.subscribe("msg_pose_avoidance", 1, &Controller::Callback_AvoidancePose, this);

  } else if (UsrLib::params["environment"] == "soloExp") {

    //订阅本车规划的参考位置
    subPlanTrj = rosNode.subscribe(UsrLib::TOPIC_PLAN_LOCALROUTE, 1, &Controller::Callback_UpdatePlanTrj, this);

    ROS_INFO("订阅真实单车位姿信息");
    std::cout << UsrLib::params["environment"] << "!" << endl;

    //订阅本车的真实位姿信息
    // subTheta = rosNode.subscribe("odom", 1, &Controller::xtark_Callback_UpdateTheta, this);
    subTheta = rosNode.subscribe("/RPY", 1, &Controller::yhs_Callback_UpdateTheta, this);
    subPoints = rosNode.subscribe("nlink_linktrack_nodeframe2", 1, &Controller::Callback_UpdatePoints, this);

  } else if (UsrLib::params["environment"] == "multiExp") {
    ROS_INFO("订阅真实多车位姿信息");

    //订阅本车规划的参考位置
    subPlanTrj = rosNode.subscribe(UsrLib::TOPIC_PLAN_LOCALROUTE, 1, &Controller::Callback_UpdatePlanTrj, this);

    std::cout << UsrLib::params["environment"] << "!" << endl;

    //订阅本车真实位姿信息
    subTheta = rosNode.subscribe("odom", 1, &Controller::xtark_Callback_UpdateTheta, this);
    subPoints = rosNode.subscribe("nlink_linktrack_nodeframe2", 1, &Controller::Callback_UpdatePoints, this);

    //订阅xtark_01车真实位姿信息
    subTheta_2 = rosNode.subscribe(string("/") + string(UsrLib::params["avo_name"]) + string("/odom_raw"), 1, &Controller::Callback_UpdateTheta_2, this);
    subPoints_2 = rosNode.subscribe(string("/") + string(UsrLib::params["avo_name"]) + string("/nlink_linktrack_nodeframe2"), 1, &Controller::Callback_UpdatePoints_2, this);

    //订阅xtark_01的仿真避障位置
    AvoidancePose = rosNode.subscribe("/xtark_01/msg_perception_selfpose", 1, &Controller::Callback_AvoidancePose, this);
    /*
    //订阅2车仿真预测信息
    subSelfPose_2 =
        rosNode.subscribe("/r2/msg_perception_selfpose", 1,
                          &Controller::Callback_UpdateSelfPose_2, this);

    //订阅2车仿真位置信息
    subPoseAll_2 = rosNode.subscribe(
        "/r2/poseAll_2", 1, &Controller::Callback_UpdateAllPos_2, this);
    */
  }
}

Controller::~Controller() {}

void Controller::run() {
  ros::Timer t = rosNode.createTimer(ros::Duration(1 / rosLoopHz), &Controller::DoControl, this);
  ros::spin();
}

void Controller::DoControl(const ros::TimerEvent &e) {
  // this->ctrlVal.gear = 1;
  // this->ctrlVal.throttle = 0.0;
  // this->ctrlVal.brake = 0.7;
  // this->ctrlVal.steerangle = 0.0;
  // this->ctrlVal.steerspeed = 0.0;
  // PublishTopics();
  ROS_ERROR_STREAM("1当前车辆是：" << UsrLib::params["name"]);
  ROS_ERROR_STREAM("参考轨迹：" << refTrj.size());

  if (this->refTrj.size() == 0) {
    ROS_INFO("No Reference Route ------");
    return;
  } 
  // else if (!AvoidancePos_flag) {
  //   // ROS_INFO(string("等待") + string(UsrLib::params["name"]) + string("的位置信息"));
  //   ROS_INFO_STREAM("等待" << UsrLib::params["avo_name"] << "的位置信息");
  //   ROS_INFO("AvoidancePos_flag：%d", AvoidancePos_flag);
  //   return;
  // } 
  else {
    GetGoalPose();
    AllController();
    // LampController();
    PublishTopics();

    ROS_INFO("v:%.6f, delta:%.2f\n", msgControl.linear.x, msgControl.angular.z);

    ros::Time current_time;
    current_time = ros::Time::now();

    OutFile << " " << nowPos.x << " " << nowPos.y << " " << nowPos.h << " " << nowPos.v << " " << nowPos.w << " ";  // 0 1 2 3
    OutFile << X_ref(0) << " " << Y_ref(0) << " " << Theta_ref(0) << " " << v_star << " " << omega << " " << Delta_f << " ";
    OutFile << AvoidancePos.x << " " << AvoidancePos.y << " " << AvoidancePos.h << " " << AvoidancePos.v << " ";
    OutFile << ctrlVal.brake << " " << ctrlVal.throttle << " " << endl;
  }
}

void Controller::Callback_UpdateSelfPose(const cta_msgs_perception::SelfPose::ConstPtr &msg) {
  nowPos.x = msg->state.pos.x;
  nowPos.y = msg->state.pos.y;
  nowPos.h = msg->state.rot.z;
  nowPos.v = msg->state.vel.x;
  // ROS_INFO("------xxccxeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" );
  pos3_flag = 1;
  point3_flag = 1;
}

void Controller::Callback_AvoidancePose(const cta_msgs_perception::SelfPose::ConstPtr &msg) {
  AvoidancePos.x = msg->state.pos.x;
  AvoidancePos.y = msg->state.pos.y;
  AvoidancePos.h = msg->state.rot.z;
  AvoidancePos.v = msg->state.vel.x;
  ROS_INFO_STREAM("输出" << UsrLib::params["avo_name"] << "的位置信息");
  AvoidancePos_flag = 1;
}

//订阅2车位置仿真信息
void Controller::Callback_UpdateSelfPose_2(const cta_msgs_perception::SelfPose::ConstPtr &msg) {
  nowPos_2.x = msg->state.pos.x;
  nowPos_2.y = msg->state.pos.y;
  nowPos_2.h = msg->state.rot.z;
  nowPos_2.v = msg->state.vel.x;
  //   ROS_INFO("************************x_2:%.6f,
  //   *********************************88y_2:%.2f\n",  nowPos_2.x, nowPos_2.y
  //   );
  pos2_flag = 1;
  point2_flag = 1;
}

//订阅2车仿真预测位置信息
void Controller::Callback_UpdateAllPos_2(const cta_msgs_control::PredictPoint::ConstPtr &msg) {
  for (int s = 0; s < N - 1; s++) {
    PosAll_2.x[s] = msg->waypoints[s].pos.x;
    PosAll_2.y[s] = msg->waypoints[s].pos.y;
    PosAll_2.theta[s] = msg->waypoints[s].rot;
    linjv2_flag = 1;
  }
  // ROS_INFO("************************x_2:%.6f,
  // *********************************88y_2:%.2f\n",  nowPos_2.x, nowPos_2.y
  // );
}

// fg[1 + v_de_start + i] = dv_cons - v_de;
// fg[1 + omega_e_start + i] = domega_cons - omega_de;
void Controller::xtark_Callback_UpdateTheta(const nav_msgs::Odometry::ConstPtr &msg) {
  nowPos.h = msg->pose.pose.position.z;
  static int runonce = 1;
  // if(runonce)
  // {
  //     angle = nowPos.h;
  //     runonce = 0;
  // }
  nowPos.h = nowPos.h;
  if (nowPos.h >= pi) {
    nowPos.h -= 2 * pi;
  }
  if (nowPos.h <= -pi) {
    nowPos.h += 2 * pi;
  }
  pos3_flag = 1;
}

void Controller::yhs_Callback_UpdateTheta(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  nowPos.h = msg->data[2]/180*3.14;
  static int runonce = 1;
  // if(runonce)
  // {
  //     angle = nowPos.h;
  //     runonce = 0;
  // }
  nowPos.h = nowPos.h;
  if (nowPos.h >= pi) {
    nowPos.h -= 2 * pi;
  }
  if (nowPos.h <= -pi) {
    nowPos.h += 2 * pi;
  }
  pos3_flag = 1;
}

void Controller::Callback_UpdatePoints(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg) {
  nowPos.x = msg->pos_3d[0];
  nowPos.y = msg->pos_3d[1];
  // ROS_INFO("************************x:%.6f,
  // *********************************8y:%.2f\n",  nowPos.x, nowPos.y );
  point3_flag = 1;
}

void Controller::Callback_UpdateTheta_2(const nav_msgs::Odometry::ConstPtr &msg) {
  nowPos_2.h = msg->pose.pose.position.z;
  static int runonce = 1;
  // if(runonce)
  // {
  //     angle = nowPos_2.h;
  //     runonce = 0;
  // }
  nowPos_2.h = nowPos_2.h;
  if (nowPos_2.h >= pi) {
    nowPos_2.h -= 2 * pi;
  }
  if (nowPos_2.h <= -pi) {
    nowPos_2.h += 2 * pi;
  }
  pos2_flag = 1;
}
void Controller::Callback_UpdatePoints_2(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg) {
  nowPos_2.x = msg->pos_3d[0];
  nowPos_2.y = msg->pos_3d[1];
  // ROS_INFO("************************x:%.6f,
  // *********************************8y:%.2f\n",  nowPos.x, nowPos.y );
  point2_flag = 1;
}

void Controller::Callback_UpdatePlanTrj(const cta_msgs_planning::LocalRoute2d::ConstPtr &msg) {
  this->refTrj.clear();
  Pose2D p;
  for (int i = 0; i < msg->waypoints.size(); i++) {
    p.h = msg->waypoints[i].rot;
    p.x = msg->waypoints[i].pos.x;
    p.y = msg->waypoints[i].pos.y;
    p.v = msg->waypoints[i].vel;
    // p.v = 0.26;
    // ROS_INFO("************************x:%.6f,
    // *********************************Y:%.6f\n",  p.x, p.y );
    this->refTrj.push_back(p);    
  }
  // ROS_ERROR_STREAM("参考轨迹回调：" << refTrj.size());
  ref_flag = 1;
  // ROS_INFO("************************ok,
  // *********************************ok.\n");
}

void Controller::PublishTopics() {
  control_ada_.publish(msgControl);

  // control mode message
  msgMode.header.frame_id = "controlmode";
  msgMode.header.stamp = ros::Time::now();
  msgMode.motionmode = UsrLib::CTRLMODE_AUTO;
  // msgMode.motionmode = UsrLib::CTRLMODE_AUTOSPEED;
  msgMode.islampauto = true;

  this->pubMode.publish(msgMode);

  ROS_INFO("PUB MESSAGE ------------------------------");
  // motion control message
  msgMotion.header.frame_id = "motioncontrol";
  msgMotion.header.stamp = ros::Time::now();

  msgMotion.gear = ctrlVal.gear;
  msgMotion.throttle = ctrlVal.throttle;
  msgMotion.brake = ctrlVal.brake;
  msgMotion.steerangle = ctrlVal.steerangle;
  msgMotion.steerspeed = ctrlVal.steerspeed;

  this->pubMotion.publish(msgMotion);

  // lamp control message
  msgMotion.header.frame_id = "lampcontrol";
  msgMotion.header.stamp = ros::Time::now();
  // msgLamp.leftture = false;
  // msgLamp.rightturn = true;

  msgLamp.leftture = ctrlLmp.leftturn;
  msgLamp.rightturn = ctrlLmp.rightturn;
  if (msgLamp.rightturn) {
    delta_ey = 88;
  }

  this->pubLamp.publish(msgLamp);
}

void Controller::GetGoalPose() {
  double mindis = 1e8;
  int refnum = 0;
  //发布最优位置序列
  // if(solve_flag==0&&pos3_flag==1&&point3_flag==1)
  // {
  //  msgPosAll.waypoints.clear();
  // for (int s=0; s<N;s++)
  // {
  //     PosAll_3.rot = nowPos.h ;
  //     PosAll_3.pos.x = nowPos.x ;
  //     PosAll_3.pos.y = nowPos.y  ;
  //     PosAll_3.vel = 0;
  //     PosAll_3.delta = 0;
  //     msgPosAll.waypoints.push_back(PosAll_3);

  // }
  // pos_3.publish(msgPosAll);

  // }
  //找最近的点的位置
  for (int i = 0; i < refTrj.size(); i++) {
    double d = Dis(refTrj[i], nowPos);
    if (d < mindis) {
      mindis = d;
      refnum = i;  //最近点的位置
    }
  }

  //取合适的参考点
  int goalnum = min(refnum, (int)refTrj.size());
  goalnumGolbal = goalnum;
  // int flag =goalnumGolbal  + ym;
  int flag = 0 + ym;
  //真正的参考序列
  for (int i = 0; i <= N; i++) {
    if (flag >= refTrj.size() - N - 1) flag = refTrj.size() - N - 1;
    // if (nowPos.v > 3)
    X_ref(i) = refTrj[flag + DT * (i)].x;
    Y_ref(i) = refTrj[flag + DT * (i)].y;
    Theta_ref(i) = refTrj[flag + DT * (i)].h;
    //  X_ref(i) = 5;
    //  Y_ref(i) =5;
    //  Theta_ref(i) = pi/4;
    // V_ref(i) = 0;
    V_ref(i) = refTrj[flag + DT * (i)].v;
    ROS_INFO(
        "************************X_ref:%.6f, "
        "*********************************Y_ref:%.6f,goalnum:%d\n",
        X_ref(i), Y_ref(i), goalnum);
  }
  DT = 1;
}

double Controller::calcurv() {
  if (nowPos.v < 0.8) return 0;
  int l = 20;
  double x1, y1, x2, y2, h1, h2;
  x1 = refTrj[goalnumGolbal].x;
  y1 = refTrj[goalnumGolbal].y;
  x2 = refTrj[goalnumGolbal + l].x;
  y2 = refTrj[goalnumGolbal + l].y;
  h1 = refTrj[goalnumGolbal].h;
  h2 = refTrj[goalnumGolbal + l].h;

  return (h2 - h1) / sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void Controller::AllController() {
  y_inf = nowPos.y;
  x_inf = nowPos.x;
  theta_inf = nowPos.h;
  //预测步长为采样点间距/速度

  double e_x = cos(theta_inf) * (X_ref[0] - x_inf) + sin(theta_inf) * (Y_ref[0] - y_inf);
  double e_y = -sin(theta_inf) * (X_ref[0] - x_inf) + cos(theta_inf) * (Y_ref[0] - y_inf);
  double e_theta = Theta_ref[0] - theta_inf;
  ROS_ERROR_STREAM("Theta_ref[0]:" << Theta_ref[0]);

  ROS_INFO(
      "ref_flag: %d, pos3_flag: %d, point3_flag: %d,pos2_flag: %d, "
      "point2_flag: %d,linjv2_flag: %d\n",
      ref_flag, pos3_flag, point3_flag, pos2_flag, point2_flag, linjv2_flag);
  count_Time++;
  // 控制器计算
  // 测试用
  // flag = 3;

  //  if(ref_flag==1&&pos3_flag==1&&point3_flag==1)
  //  {
  if (count_Time >= 2) {
    ROS_INFO("e_x = %f", e_x);
    ROS_INFO("e_y = %f", e_y);
    Eigen::VectorXd state_1(3);
    Eigen::VectorXd state_avoidance(4);
    state_1 << x_inf, y_inf, theta_inf;
    state_avoidance << AvoidancePos.x, AvoidancePos.y, AvoidancePos.h, AvoidancePos.v;
    Eigen::VectorXd control_out(2);
    control_out << v_star, Delta_f;

    // 求解
    Solution sol = mpc.Solve(state_1, state_avoidance, X_ref, Y_ref, Theta_ref, V_ref, control_out);
    ref_flag = 0;
    pos3_flag = 0;
    AvoidancePos_flag = 0;
    point3_flag = 0;
    pos2_flag = 0;
    point2_flag = 0;
    linjv2_flag = 0;
    // 得到误差控制量
    vee = sol.V_e.at(0);
    omegaee = sol.Omega_e.at(0);
    for (int x = 0; x < N; x++) {
      ROS_INFO(
          "************************X_e:%.6f, "
          "*********************************Y_e:%.6f\n",
          sol.E_X.at(x), sol.E_Y.at(x));
    }
    ROS_ERROR_STREAM("Theta_ref(0):" << Theta_ref(0));
    double te = Theta_ref(0) - theta_inf;
    if (sol.status != 1) {
      cout << "求解失败 ！ " << endl;
      solve_flag = 0;
    } else {
      cout << "求解成功 ！ " << endl;
      v_star = -vee + V_ref(0) * cos(te);
      omega = (-omegaee + V_ref(0) * sin(te)) / row;
      solve_flag = 1;
      msgPosAll.waypoints.clear();
      for (int s = 0; s < N; s++) {
        PosAll_3.rot = Theta_ref[s + 1] - sol.E_Theta.at(s);
        PosAll_3.pos.x = X_ref[s + 1] - cos(PosAll_3.rot) * sol.E_X.at(s) + sin(PosAll_3.rot) * sol.E_Y.at(s);
        PosAll_3.pos.y = Y_ref[s + 1] - cos(PosAll_3.rot) * sol.E_Y.at(s) - sin(PosAll_3.rot) * sol.E_X.at(s);
        PosAll_3.vel = 0;
        PosAll_3.delta = 0;
        msgPosAll.waypoints.push_back(PosAll_3);
      }
      pos_3.publish(msgPosAll);
    }
    ROS_INFO("Solve By MPC!");
    double Tan_f_star = omega * row / v_star;
    Delta_f = atan(Tan_f_star);
    if (Delta_f > 0.5235987666666666) {
      Delta_f = 0.5235987666666666;
    } else if (Delta_f < -0.5235987666666666) {
      Delta_f = -0.5235987666666666;
    }
    // Delta_f = Delta_f/pi *180;
    msgControl.linear.x = v_star;
    // msgControl.angular.z = Delta_f;
    msgControl.angular.z = omega;

    //   msgControl.linear.x = 0.2;
    //  msgControl.angular.z = 0 ;
    // msgControl.angular.z = 0.5;
  }
  // }
}
/*void Controller::LampController(){
    bool leftture = true;
    bool rightturn = false;
}*/
void Controller::LampController() {
  if (this->now_behavior == UsrLib::BEHAVIOR_LEFTTURN || this->now_behavior == UsrLib::BEHAVIOR_LEFTMERGE || now_behavior == UsrLib::BEHAVIOR_LEFTPARK) {
    ctrlLmp.leftturn = true;
    ctrlLmp.rightturn = false;
    // ROS_INFO("111111111111111111111");
  } else if (this->now_behavior == UsrLib::BEHAVIOR_RIGHTTURN || this->now_behavior == UsrLib::BEHAVIOR_RIGHTMERGE || now_behavior == UsrLib::BEHAVIOR_RIGHTPARK) {
    ctrlLmp.leftturn = false;
    ctrlLmp.rightturn = true;
    // ROS_INFO("22222222222222222222");
  } else {
    ctrlLmp.leftturn = false;
    ctrlLmp.rightturn = false;
    // ROS_INFO("333333333333333333333");
  }
}

double Controller::Dis(const Pose2D p1, const Pose2D p2) { return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); }

double Controller::Ang(const Pose2D p1, const Pose2D p2) { return atan2(p2.y - p1.y, p2.x - p1.x); }

double Controller::Uag(const double a) {
  double u = a;
  while (u > M_PI) {
    u -= 2 * M_PI;
  }
  while (u <= -M_PI) {
    u += 2 * M_PI;
  }
  return u;
}

int Controller::sign(double x) {
  if (x < 0) {
    return -1;
  } else if (x > 0) {
    return 1;
  } else {
    return 0;
  }
}

double Controller::fal(double e, double alpha, double delta) {
  if (abs(e) <= delta) {
    return e / pow(delta, (1 - alpha));
  } else {
    return pow(abs(e), alpha) * sign(e);
  }
}

double Controller::fsg(double x, double d) { return (sign(x + d) - sign(x - d)) / 2; }

double Controller::fhan(double x1, double x2, double r, double h) {
  double c = 1;
  h = c * h;
  double d = r * h * h;
  double a0 = h * x2;
  double y = x1 + a0;
  double a1 = sqrt(d * (d + 8 * abs(y)));
  double a2 = a0 + sign(y) * (a1 - d) / 2;
  double a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
  double u = -r * (a / d) * fsg(a, d) - r * sign(a) * (1 - fsg(a, d));
  return u;
}
}  // namespace MPCtrack
#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

#include <math.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include "cta_msgs/PosAll.h"
#include "cta_msgs_control/ControlMode.h"
#include "cta_msgs_control/LampControl.h"
#include "cta_msgs_control/MotionControl.h"
#include "cta_msgs_control/PredictPoint.h"
#include "cta_msgs_perception/SelfPose.h"
#include "cta_msgs_planning/DecisionBehavior.h"
#include "cta_msgs_planning/GlobalRoute2d.h"
#include "cta_msgs_planning/LocalRoute2d.h"
#include "cta_msgs_sensor/Imu.h"
#include "usrlib/usrlib.h"

using namespace std;
using CppAD::AD;

// XmlRpc::XmlRpcValue paramss;
// ros::param::get("cam/FRAME_WIDTH", paramss);
// cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@" << paramss << endl;

template <typename T>
T getParam(const std::string &name,
           const T &defaultValue)  // This name must be namespace+parameter_name
{
  T v;
  if (ros::param::get(name, v))  // get parameter by name depend on ROS.
  {
    ROS_INFO_STREAM("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Found parameter: "
                    << name << ",\tvalue: " << v);
    return v;
  } else
    ROS_WARN_STREAM("Cannot find value for parameter: "
                    << name << ",\tassigning default: " << defaultValue);
  return defaultValue;  // if the parameter haven't been set,it's value will
                        // return defaultValue.
}

// ros::init(argc, argv, "test_tf_node");
// ros::NodeHandle nh;
// int FRAME_WIDTH = 0;
// getParam<int>("cam/FRAME_WIDTH", 0);
namespace MPCtrack {
#define N 5
const double dt = 0.1;  // 采样周期
int ym = 0;             /// 5;//global_tracking
int DT = 1;

// 构造一个向量表达各个变量
// (N + 1) * 3 个状态量 虽然 e_theta 不在代价函数里，但是在约束中有所体现
size_t e_x_start = 0;
size_t e_y_start = e_x_start + N + 1;
size_t e_theta_start = e_y_start + N + 1;

size_t v_start = e_theta_start + N + 1;
size_t omega_start = v_start + N;
size_t dV_start = omega_start + N;
size_t dDelta_start = dV_start + N;
size_t x_obstacle = dDelta_start + N;
size_t y_obstacle = x_obstacle + N;
// N * 2 个控制量
size_t v_e_start = y_obstacle + N;
size_t omega_e_start = v_e_start + N;

int ref_flag = 0;
int pos3_flag = 0;
int AvoidancePos_flag = 0;
int point3_flag = 0;
int pos2_flag = 0;
int point2_flag = 0;
int linjv2_flag = 0;
int solve_flag = 0;
double Delta_f = 0;

/*******************gai dong qian***********************/
// double Q_1 = 0.2;
// // double Q_2 = 1.856;
// double Q_2 = 0.865;  //(越大跟的越快)

// double P_1 = 0.2;
// // double P_2 = 0.4;//(越大越缓)
// double P_2 = 0.2;

/*******************gai dong houn***********************/
double Q_1 = 0.5;
// double Q_2 = 1.856;
double Q_2 = 0.5;  //(越大跟的越快)

double P_1 = 0.2;
// double P_2 = 0.4;//(越大越缓)
double P_2 = 0.2;

double R_1 = 0.2;
double R_2 = 0.2;

double row = 0.18;
double V_max = 17;

double x_inf;
double y_inf;
double theta_inf;
double ey_last = 1000;

double omega = 0;
double v_star = 0;
double vee = 0;
double omegaee = 0;
int count_Time = 0;
double delta_ey = 0;
double angle = 0;

double posx_ob = 2.6;
double posy_ob = 3.0;

double a_ = 0.5;
double b_ = 0.2;

double lamuda_ob = 0.5;
double k_ob = 2.5;
double R_ob = 0.5;

struct Solution {
  vector<double> E_X;
  vector<double> E_Y;
  vector<double> E_Theta;
  vector<double> Ctrl;
  vector<double> V_e;
  vector<double> Omega_e;
  vector<double> V_de;
  vector<double> Omega_de;

  int status;
};
struct ControlLamp {
  bool leftturn;
  bool rightturn;
};

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
struct vector2d {
  double x;
  double y;
};

struct ControlVal {
  int gear;
  double throttle;
  double brake;
  double steerangle;
  double steerspeed;
};

struct PosAll {
  double x[N];
  double y[N];
  double theta[N];
};
typedef std::vector<Pose2D> Path;

class Controller {
 private:
  ros::NodeHandle rosNode;
  double rosLoopHz;

  ros::Publisher pubMode;
  ros::Publisher pubMotion;
  ros::Publisher pubLamp;
  ros::Publisher control_ada_;
  ros::Publisher pos_3;

  ros::Subscriber subSelfPose;
  ros::Subscriber subSelfPose_2;
  ros::Subscriber subPoseAll_2;
  ros::Subscriber subPlanTrj;
  ros::Subscriber AvoidancePose;
  ros::Subscriber subTheta;
  ros::Subscriber subPoints;
  ros::Subscriber subTheta_2;
  ros::Subscriber subPoints_2;

  cta_msgs_control::ControlMode msgMode;
  cta_msgs_control::MotionControl msgMotion;
  cta_msgs_control::LampControl msgLamp;
  geometry_msgs::Twist msgControl;
  cta_msgs_control::PredictPoint msgPosAll;

  Pose2D nowPos;
  Pose2D AvoidancePos;
  Pose2D nowPos_2;
  Pose2D goalPos;
  PosAll PosAll_2;
  PosAll SelfPos;
  double refSpeed;
  Path refTrj;
  ControlVal ctrlVal;
  ControlLamp ctrlLmp;
  unsigned int now_behavior;
  double stop_point_x;
  double stop_point_y;

  double log_beta_angle = 0;
  double log_x2d = 0;
  double log_x2f = 0;
  double log_vx = 0;
  double log_z2 = 0;
  double log_a = 0;
  double log_a1 = 0;
  double log_a2 = 0;
  double log_a3 = 0;
  double log_D2 = 0;
  double log_ob = 0;
  double obs_x = 0;
  double obs_y = 0;
  int goalnumGolbal = 0;

 public:
  Controller();
  Controller(ros::NodeHandle &nh, const double frq);
  virtual ~Controller();
  void run();

 public:
  void DoControl(const ros::TimerEvent &e);

  void Callback_UpdateSelfPose(
      const cta_msgs_perception::SelfPose::ConstPtr &msg);
  void Callback_AvoidancePose(
      const cta_msgs_perception::SelfPose::ConstPtr &msg);
  void Callback_UpdateSelfPose_2(
      const cta_msgs_perception::SelfPose::ConstPtr &msg);
  void Callback_UpdateAllPos_2(
      const cta_msgs_control::PredictPoint::ConstPtr &msg);
  void Callback_UpdatePlanTrj(
      const cta_msgs_planning::LocalRoute2d::ConstPtr &msg);
  void Callback_UpdateTheta(const nav_msgs::Odometry::ConstPtr &msg);
  void Callback_UpdatePoints(
      const nlink_parser::LinktrackNodeframe2::ConstPtr &msg);
  void Callback_UpdateTheta_2(const nav_msgs::Odometry::ConstPtr &msg);
  void Callback_UpdatePoints_2(
      const nlink_parser::LinktrackNodeframe2::ConstPtr &msg);
  void PublishTopics();

  void GetGoalPose();

  void AllController();
  void LampController();
  double calcurv();

  double Dis(const Pose2D p1, const Pose2D p2);
  double Ang(const Pose2D p1, const Pose2D p2);
  double Uag(const double a);

  int sign(double x);
  double fal(double e, double alpha, double delta);
  double fsg(double x, double d);
  double fhan(double x1, double x2, double r, const double frq);
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  Solution Solve(Eigen::VectorXd x0, Eigen::VectorXd x_avoidance,
                 Eigen::VectorXd X_ref, Eigen::VectorXd Y_ref,
                 Eigen::VectorXd Theta_ref, Eigen::VectorXd V_ref,
                 Eigen::VectorXd control_out);
};

class FG_eval {
 public:
  Eigen::VectorXd X_ref;
  Eigen::VectorXd Y_ref;
  Eigen::VectorXd Theta_ref;
  Eigen::VectorXd V_ref;

  AD<double> X_avo;
  AD<double> Y_avo;
  AD<double> Theta_avo;
  AD<double> V_avo;

  Eigen::VectorXd control_out;

  // Coefficients of the fitted polynomial.
  // 先取出多项式系数，和 预测的动作
  FG_eval(Eigen::VectorXd x0, Eigen::VectorXd x_avoidance,
          Eigen::VectorXd X_ref, Eigen::VectorXd Y_ref,
          Eigen::VectorXd Theta_ref, Eigen::VectorXd V_ref,
          Eigen::VectorXd control_out)  // 构造函数
  {
    // 传入参考轨迹
    this->X_ref = X_ref;
    this->Y_ref = Y_ref;
    this->Theta_ref = Theta_ref;
    this->V_ref = V_ref;

    this->X_avo = x_avoidance[0];
    this->Y_avo = x_avoidance[1];
    this->Theta_avo = x_avoidance[2];
    this->V_avo = x_avoidance[3];
    this->control_out = control_out;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector &fg, const ADvector &vars) {
    // The cost is stored is the first element of `fg`.
    // 代价存储在 fg的第一个
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // 计算误差

    // The part of the cost based on the reference state.
    // 代价函数填充
    for (int i = 1; i <= N; i++) {
      // e_x ^ 2
      fg[0] += Q_1 * CppAD::pow(vars[e_x_start + i], 2);
      // e_y ^ 2
      fg[0] += Q_2 * CppAD::pow(vars[e_y_start + i], 2);
      // fg[0] += Q_2 * CppAD::pow(vars[e_theta_start + i], 2);
      // cout << " e_x " << vars[e_x_start + i] << endl;
    }

    // 终端状态项
    fg[0] += R_1 * CppAD::pow(vars[e_x_start + N], 2);
    fg[0] += R_2 * CppAD::pow(vars[e_y_start + N], 2);

    //代价函数中，加入控制量 u_e 和 w_e
    for (int i = 0; i <= N - 1; i++) {
      // ve
      fg[0] += P_1 * CppAD::pow(vars[v_e_start + i], 2);
      // we
      fg[0] += P_2 * CppAD::pow(vars[omega_e_start + i], 2);

      // AD<double> lamda = 0.7;
      // AD<double> R = 0.3;
      // AD<double> dab = 1;
      // AD<double> Ex_k = vars[e_x_start + i];
      // AD<double> Ey_k = vars[e_y_start + i];
      // AD<double> Etheta_k = vars[e_theta_start + i];
      // AD<double> theta_k = Theta_ref[i] - Etheta_k;
      // AD<double> x_k =
      //     X_ref[i] - CppAD::cos(theta_k) * Ex_k + CppAD::sin(theta_k) * Ey_k;
      // AD<double> y_k =
      //     Y_ref[i] - CppAD::cos(theta_k) * Ey_k - CppAD::sin(theta_k) * Ex_k;
      // AD<double> xe_ob = x_k - X_avo;
      // AD<double> ye_ob = y_k - Y_avo;
      // AD<double> e_ob =
      // CppAD::pow(CppAD::pow(xe_ob, 2) + CppAD::pow(ye_ob, 2), 0.5);

      //   fg[0] += lamda * (V_ref[i] - vars[v_e_start + i]) * dt /
      //            (1 + CppAD::exp(e_ob-2*R));
      // fg[0] += lamda * (V_ref[i] - vars[v_e_start + i]) * dt /
      //          (1 + CppAD::exp((CppAD::pow(xe_ob, 2) + CppAD::pow(ye_ob, 2))
      //          -
      //                          CppAD::pow(0.5, 2)));

      AD<double> Ex_k = vars[e_x_start + i];
      AD<double> Ey_k = vars[e_y_start + i];
      AD<double> Etheta_k = vars[e_theta_start + i];
      AD<double> theta_k = Theta_ref[i] - Etheta_k;
      AD<double> x_k =
          X_ref[i] - CppAD::cos(theta_k) * Ex_k + CppAD::sin(theta_k) * Ey_k;
      AD<double> y_k =
          Y_ref[i] - CppAD::cos(theta_k) * Ey_k - CppAD::sin(theta_k) * Ex_k;
      //
      AD<double> xe_ob = x_k - X_avo;
      AD<double> ye_ob = y_k - Y_avo;
      AD<double> e_ob =
          CppAD::pow(CppAD::pow(xe_ob, 2) + CppAD::pow(ye_ob, 2), 0.5);

      AD<double> beita_x =
          (x_k - X_avo) * cos(Theta_avo) + (y_k - Y_avo) * sin(Theta_avo);
      AD<double> beita_y =
          (y_k - Y_avo) * cos(Theta_avo) + (x_k - X_avo) * sin(Theta_avo);
      AD<double> beita_oa = atan(beita_y / beita_x);
      AD<double> D = a_ * b_ /
                     CppAD::pow(CppAD::pow(a_ * cos(beita_oa), 2) +
                                    CppAD::pow(b_ * sin(beita_oa), 2),
                                0.5);
      // fg[0] += 1/D * ( e_ob - D);
      // fg[0] += lamuda_ob / (1+CppAD::exp(-k_ob * ( CppAD::pow(D, 2)-(
      // CppAD::pow(xe_ob, 2)+ CppAD::pow(ye_ob, 2)))));
      // fg[0] += lamuda_ob / (1 + CppAD::exp(-k_ob * (CppAD::pow(0.5, 2) -
      //                                               (CppAD::pow(xe_ob, 2) +
      //                                                CppAD::pow(ye_ob,
      //                                                2)))));
      fg[0] += lamuda_ob / (1 + CppAD::exp(-3 * (CppAD::pow(0.5 * D, 2) -
                                                 (CppAD::pow(xe_ob, 2) +
                                                  CppAD::pow(ye_ob, 2)))));
    }

    // 初始约束
    // x(0) = x(0);
    fg[1 + e_x_start] = vars[e_x_start];
    fg[1 + e_y_start] = vars[e_y_start];
    fg[1 + e_theta_start] = vars[e_theta_start];

    for (int i = 0; i <= N - 1; i++) {
      // The state at time k+1 .
      // k + 1 时刻的变量
      AD<double> Ex_k_1 = vars[e_x_start + i + 1];
      AD<double> Ey_k_1 = vars[e_y_start + i + 1];
      AD<double> Etheta_k_1 = vars[e_theta_start + i + 1];

      // The state at time k.
      // k 时刻的变量
      AD<double> Ex_k = vars[e_x_start + i];
      AD<double> Ey_k = vars[e_y_start + i];
      AD<double> Etheta_k = vars[e_theta_start + i];

      AD<double> theta_k = Theta_ref[i] - Etheta_k;
      AD<double> x_k =
          X_ref[i] - CppAD::cos(theta_k) * Ex_k + CppAD::sin(theta_k) * Ey_k;
      AD<double> y_k =
          Y_ref[i] - CppAD::cos(theta_k) * Ey_k - CppAD::sin(theta_k) * Ex_k;
      AD<double> xe_ob = x_k - posx_ob;
      AD<double> ye_ob = y_k - posy_ob;

      // Only consider the actuation at time t.

      AD<double> v_cons = vars[v_start + i];
      AD<double> omega_cons = vars[omega_start + i];
      AD<double> dV_cons = vars[dV_start + i];
      AD<double> dDelta_cons = vars[dDelta_start + i];
      AD<double> xb_cons = vars[x_obstacle + i];
      AD<double> yb_cons = vars[y_obstacle + i];
      AD<double> V_e_k = vars[v_e_start + i];
      AD<double> W_e_k = vars[omega_e_start + i];

      AD<double> V_k = -V_e_k + V_ref[i] * CppAD::cos(Etheta_k);
      AD<double> W_k = (-W_e_k + V_ref[i] * CppAD::sin(Etheta_k)) / row;
      AD<double> Delta_k = atan(W_k * row / V_k);

      fg[1 + v_start + i] = v_cons - V_k;
      fg[1 + omega_start + i] = omega_cons - W_k;

      fg[1 + x_obstacle + i] =
          xb_cons - CppAD::pow(xe_ob, 2);  //- CppAD::pow(ye_ob, 2);  //???
      fg[1 + y_obstacle + i] = yb_cons - CppAD::pow(ye_ob, 2);

      //
      if (i == 0) {
        // AD<double>v_de = V_k - control_out[0];
        //    AD<double>delta_de = Delta_k - control_out[1];
        // fg[1 + dV_start + i] = dV_cons - v_de;
        // fg[1 + dDelta_start + i] = dDelta_cons - (Delta_k - control_out[1]);
      }

      // else
      // {
      //     AD<double> V_pk = vars[v_start + i - 1];
      //     AD<double> W_pk = vars[omega_start + i - 1];
      //     AD<double> Delta_pk =atan(W_pk*row/V_pk);
      //     AD<double>v_de = V_k - V_pk;
      //     AD<double>delta_de = Delta_k - Delta_pk;
      //     fg[1 + dV_start + i] = dV_cons - v_de;
      //     fg[1 + dDelta_start + i] = dDelta_cons - delta_de;
      // }

      // fg[1 + v_de_start + i] = dv_cons - v_de;
      // fg[1 + omega_e_start + i] = domega_cons - omega_de;

      // Recall the equations for the model:
      // E_theta[k+1] =  E_theta[k] + dt * (Omega_ref[k] - (-W_e_k + V_ref[k] *
      // sin(e_theta[k]))/row) E_x[k+1] = E_x[k] + dt * ((-w_e_k + V_ref[k] *
      // CppAD::sin(E_theta[k]))/row * E_y_k + v_e_k) E_y[k+1] == E_y{k} + T *
      // (-(-we{k} + V_r{k} * sin(E_theta{k}))/row * E_x{k} + we{k})];

      // 等式约束 将变量都写在同一侧 另一侧 都是 0

      fg[2 + e_x_start + i] = Ex_k_1 - (Ex_k + (W_k * Ey_k + V_e_k) * dt);
      fg[2 + e_y_start + i] = Ey_k_1 - (Ey_k + (-W_k * Ex_k + W_e_k) * dt);
      fg[2 + e_theta_start + i] =
          Etheta_k_1 -
          (Etheta_k + ((Theta_ref[i + 1] - Theta_ref[i]) / dt - W_k) * dt);
    }
  }
};

int FindNearDouble(double x, double *X_ref, int crow);
int FindNearCircle(double x, double y, double theta, double *X_ref,
                   double *Y_ref, double *Theta_ref, int crow, int Init_index);
int FindOuD(double x, double y, double theta, double *X_ref, double *Y_ref,
            double *Theta_ref, int crow, int Init_index);

// Struct for the car information
// 记录车辆的位姿信息结构体
struct Car_Info {
  double x;
  double y;
  double yaw;
  double v_now;
  double acc_x;
  double acc_y;
  double a_now;
};

// Load Rference Data
double *REF_X;
double *REF_Y;
double *REF_Theta;
double *REF_V;
// double *REF_Omega;

// 参考轨迹向量
Eigen::VectorXd X_ref(N + 1);
Eigen::VectorXd Y_ref(N + 1);
Eigen::VectorXd Theta_ref(N + 1);
Eigen::VectorXd V_ref(N + 1);

MPC mpc;

#define pi 3.1415926
}  // namespace MPCtrack
#endif

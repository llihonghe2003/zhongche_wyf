#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "ros/ros.h"

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <fstream>

#include "usrlib/usrlib.h"
#include "cta_msgs_perception/SelfPose.h"
#include "cta_msgs_sensor/Imu.h"
#include "std_msgs/Float64MultiArray.h"
#include "cta_msgs_planning/DecisionBehavior.h"
#include "cta_msgs_planning/GlobalRoute2d.h"
#include "cta_msgs_planning/LocalRoute2d.h"
#include "cta_msgs_control/ControlMode.h"
#include "cta_msgs_control/MotionControl.h"
#include "cta_msgs_control/LampControl.h"
#include "cta_msgs_control/PredictPoint.h"
#include <sensor_msgs/Imu.h>
#include "cta_msgs/PosAll.h"
#include <geometry_msgs/Twist.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nav_msgs/Odometry.h>

#include <chrono>
using namespace std;
using CppAD::AD;

namespace MPCtrack
{
#define N 6 // 6
#define SERVO_MAX (45) * pi / 180
    const double dt = 0.02; // 0.02; // 采样周期
    int ym = 0;             /// 5;//global_tracking
    int DT = 1;
    //double solve_t;
    // 构造一个向量表达各个变量
    // (N + 1) * 3 个状态量 虽然 e_theta 不在代价函数里，但是在约束中有所体现
    size_t e_x_start = 0;
    size_t e_y_start = e_x_start + N + 1;
    size_t e_theta_start = e_y_start + N + 1;

    size_t v_start = e_theta_start + N + 1;
    size_t omega_start = v_start + N;
    size_t dV_start = omega_start + N;
    size_t delta_start = dV_start + N;

    // 在 MPCtrack 命名空间中添加
    // size_t d_14_start = delta_start + N;//d_14
    // size_t longitudinal_error_start = d_14_start + N;
    // N * 2 个控制量
    // size_t v_e_start = d_14_start + N;
    size_t v_e_start = delta_start + N;
    size_t omega_e_start = v_e_start + N;

    int ref_flag = 0;
    int solve_flag = 0;
    double Delta_f = 0;
    int point_flag = 0;
    int ref4_flag = 0;
    int input4_flag = 0;
    int pos4_flag = 0;
    int point4_flag = 0;

    float e_x = 0;
    float e_y = 0;
    float e_theta = 0;

    float dx = -0.6; // -0.8
    float dy = -0.6; // -0.8
    float d = sqrt(dx * dx + dy * dy);

    double Q_1 = 0.45;  // ex0.55
    double Q_2 = 0.865; // 0.765ey那个大调大0.865 0.965
    // double Q_2 =0.865;//(越大跟的越快)

    double P_1 = 0.25; // ve0.25/0.45
    // double P_2 = 0.4;//(越大越缓)
    double P_2 = 0.35; // we0.35/0.45

    // double v_1 = 0.2;

    double R_1 = 0.5; // 终端ex0.2/0.35
    double R_2 = 0.75; // 终端ey0.2/0.35

    // double k_1 =0.2;
    // double k_2 = 0.15;//0.8肯定不行

    double h = 2.2; // 调2.2/2.8
    double min_safe_distance = 0.8;//0.8

    double row = 0.4;
    double V_max = 1.0;
    double P_max = 0.9; // 预测视界0.9

    double lamuda_ob = 0.5;
    double k_ob = 2.5;
    double R_ob = 0.5;

    double a = 0.95;//0.95/1.05/1.15
    double b = 0.7;//0.7/0.65/0.85
    double h_t = 1.15; //1.25

    double x_inf;
    double y_inf;
    double theta_inf;

    double omega = 0;
    double v_star = 0;
    double vee = 0;
    double COST = 0;
    double omegaee = 0;
    int count_Time = 0;
    double delta_ey = 0;
    double angle = 0;
    double cost;

    // 计算apf参数
    //  double a_ =0.5;
    //  double b_=0.2;

    // double lamuda_ob =0.5;//0.9
    // double k_ob =1.5;//2.5
    // double lamuda_3 =0.15;
    // double k_3 =1.7;
    // double R_ob =0.3;//APF函数

    struct Solution
    {
        vector<double> E_X;
        vector<double> E_Y;
        vector<double> E_Theta;
        vector<double> Ctrl;
        vector<double> V_e;
        vector<double> Omega_e;
        vector<double> V_de;
        vector<double> Omega_de;
        vector<double> delta;
        double Cost;

        int status;
    };
    struct ControlLamp
    {
        bool leftturn;
        bool rightturn;
    };

    struct Pose2D
    {
        double x;
        double y;
        double h;
        double v;
        double w;

        double vy;
        double r;
        double ax;
        double ay;
    };
    struct vector2d
    {
        double x;
        double y;
    };

    struct ControlVal
    {
        int gear;
        double throttle;
        double brake;
        double steerangle;
        double steerspeed;
    };

    struct PosAll
    {
        double x[N];
        double y[N];
        double theta[N];
    };
    typedef std::vector<Pose2D> Path;

    class Controller
    {
    private:
        ros::NodeHandle rosNode;
        double rosLoopHz;

        ros::Publisher pubMode;
        ros::Publisher pubMotion;
        ros::Publisher pubLamp;
        ros::Publisher control_ada_;
        ros::Subscriber subPlanTrj;
        ros::Subscriber subTheta;
        ros::Subscriber subPoints;
        ros::Publisher pos_3;

        ros::Subscriber subSelfPose_4;
        ros::Subscriber subTheta_4;
        ros::Subscriber subPoints_4;
        ros::Subscriber subPlanTrj_4;
        ros::Subscriber cmd_sub_4;

        cta_msgs_control::ControlMode msgMode;
        cta_msgs_control::MotionControl msgMotion;
        cta_msgs_control::LampControl msgLamp;
        geometry_msgs::Twist msgControl;
        cta_msgs_control::PredictPoint msgPosAll;
        geometry_msgs::Twist current_twist_;

        Pose2D nowPos;
        Pose2D goalPos;
        PosAll PosAll_2;
        PosAll SelfPos;

        Pose2D Input_4;
        PosAll PosAll_4;
        Pose2D nowPos_4;
        Path refTrj_4;

        double refSpeed;
        Path refTrj;
        Path refTask;
        ControlVal ctrlVal;
        ControlLamp ctrlLmp;
        unsigned int now_behavior;
        double stop_point_x;
        double stop_point_y;

        int goalnumGolbal = 0;

    public:
        Controller();
        Controller(ros::NodeHandle &nh, const double frq);
        virtual ~Controller();
        void run();

    public:
        void DoControl(const ros::TimerEvent &e);
        void yhs_Callback_UpdateTheta(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void Callback_UpdateSelfPose(const cta_msgs_perception::SelfPose::ConstPtr &msg);
        void Callback_AvoidancePose(const cta_msgs_perception::SelfPose::ConstPtr &msg);
        void Callback_UpdateSelfPose_2(const cta_msgs_perception::SelfPose::ConstPtr &msg);
        void Callback_UpdateAllPos_2(const cta_msgs_control::PredictPoint::ConstPtr &msg);
        void Callback_UpdatePlanTrj(const cta_msgs_planning::LocalRoute2d::ConstPtr &msg);
        void Callback_UpdateTheta(const nav_msgs::Odometry::ConstPtr &msg);
        void Callback_UpdatesTheta(const sensor_msgs::Imu::ConstPtr &msg);
        void Callback_UpdatePoints(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg);
        void Callback_UpdateTheta_2(const nav_msgs::Odometry::ConstPtr &msg);
        void Callback_UpdatePoints_2(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg);

        void Callback_UpdateSelfPose_4(const cta_msgs_perception::SelfPose::ConstPtr &msg);
        void Callback_UpdateAllPos_4(const cta_msgs_control::PredictPoint::ConstPtr &msg);
        void Callback_UpdatePlanTrj_4(const cta_msgs_planning::LocalRoute2d::ConstPtr &msg);
        void Callback_Ctrl_4(const geometry_msgs::Twist::ConstPtr &msg);
        void Callback_UpdatePoints_4(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg);
        // void Callback_UpdateTheta_4(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void Callback_UpdateTheta_4(const nav_msgs::Odometry::ConstPtr &msg);

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

    class MPC
    {
    public:
        MPC();

        virtual ~MPC();
        double solve_t; 
        Solution Solve(Eigen::VectorXd x0, Eigen::VectorXd X_ref, Eigen::VectorXd Y_ref, Eigen::VectorXd Theta_ref,
                       Eigen::VectorXd V_ref, Eigen::VectorXd W_ref, Eigen::VectorXd control_out, Eigen::VectorXd x_point_4);
    };

    class FG_eval
    {
    public:
        Eigen::VectorXd X_ref;
        Eigen::VectorXd Y_ref;
        Eigen::VectorXd Theta_ref;
        Eigen::VectorXd V_ref;
        Eigen::VectorXd W_ref;
        Eigen::VectorXd control_out;

        // 4che
        AD<double> X_4;
        AD<double> Y_4;
        AD<double> Theta_4;
        // Coefficients of the fitted polynomial.
        // 先取出多项式系数，和预测的动作
        FG_eval(Eigen::VectorXd x0, Eigen::VectorXd X_ref, Eigen::VectorXd Y_ref, Eigen::VectorXd Theta_ref,
                Eigen::VectorXd V_ref, Eigen::VectorXd W_ref, Eigen::VectorXd control_out, Eigen::VectorXd x_point_4) // 构造函数
        {
            // 传入参考轨迹
            this->X_ref = X_ref;
            this->Y_ref = Y_ref;
            this->Theta_ref = Theta_ref;
            this->V_ref = V_ref;
            this->W_ref = W_ref;
            this->control_out = control_out;
            // 初始化避障位置和预测动作
            this->X_4 = x_point_4[0];
            this->Y_4 = x_point_4[1];
            this->Theta_4 = x_point_4[2];
            // ROS_INFO("Received x_point4 position -> x: %.6f, y: %.2f", X_4.Value(), Y_4.Value());
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        // 这是该类的核心部分，它重载了 operator() 运算符，用于定义模型的代价函数和约束条件。该函数接收两个参数：
        // fg：这是一个向量，用来存储代价函数和约束条件的值。
        // vars：这是优化问题的决策变量，包括车辆的状态、控制输入等。
        void operator()(ADvector &fg, const ADvector &vars)
        {
            // The cost is stored is the first element of `fg`.
            // 代价存储在 fg的第一个
            // Any additions to the cost should be added to `fg[0]`.
            fg[0] = 0;
            // 计算误差
            // The part of the cost based on the reference state.
            // 代价函数填充
            for (int i = 1; i <= N; i++)
            {
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

            // 代价函数中，加入控制量 v_e 和 w_e
            for (int i = 0; i <= N - 1; i++)
            {
                // ve
                fg[0] += P_1 * CppAD::pow(vars[v_e_start + i], 2);
                // we
                fg[0] += P_2 * CppAD::pow(vars[omega_e_start + i], 2);

                // fg[0] += v_1 * CppAD::pow(vars[v_start + i]-0.2, 2);
                // 提取当前误差变量
                AD<double> Ex_k = vars[e_x_start + i];
                AD<double> Ey_k = vars[e_y_start + i];
                AD<double> Etheta_k = vars[e_theta_start + i];
                // 将当前参考角度 Theta_ref[i] 减去角度误差 Etheta_k，得到车辆实际行驶方向的调整角度。
                AD<double> theta_k = Theta_ref[i] - Etheta_k;
                // 计算在 i 时刻车辆的实际 x 和 y 位置
                AD<double> x_k = X_ref[i] - CppAD::cos(theta_k) * Ex_k + CppAD::sin(theta_k) * Ey_k;
                AD<double> y_k = Y_ref[i] - CppAD::cos(theta_k) * Ey_k - CppAD::sin(theta_k) * Ex_k;
                // 计算与四车的距离
                AD<double> xe_14 = x_k - X_4;
                AD<double> ye_14 = y_k - Y_4;
                //AD<double> Pe_14 = CppAD::pow(CppAD::pow(xe_14, 2) + CppAD::pow(ye_14, 2), 0.5);
                // me
                //if (Pe_14 <= min_safe_distance)
                //{
                    //fg[0] += h * CppAD::exp(-Pe_14) * (P_max - Pe_14);
                    // fg[0] += lamuda_ob / (1 + CppAD::exp(-1.5 * (CppAD::pow(0.5 * D, 2) - Pe_14)));
                //}
                AD<double> Pe_14 = CppAD::pow(xe_14, 2) / CppAD::pow(a, 2) + CppAD::pow(ye_14, 2) / CppAD::pow(b, 2);
                if (Pe_14 <= 1)
                {
                     fg[0] += h_t * CppAD::exp(- Pe_14);//+a_l * CppAD::exp(-CppAD::pow((y_k-4.4), 2)/CppAD::pow(b_l, 2))
                }
            }
            // 初始约束
            // x(0) = x(0);
            fg[1 + e_x_start] = vars[e_x_start];
            fg[1 + e_y_start] = vars[e_y_start];
            fg[1 + e_theta_start] = vars[e_theta_start];
            //  等式约束 右侧为0,0~n-1时刻,状态变量在初始时刻满足特定的条件
            for (int i = 0; i <= N - 1; i++)
            {
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
                // 将参考轨迹的角度转换为车辆坐标系。
                AD<double> theta_k = Theta_ref[i] - Etheta_k;
                AD<double> x_k = X_ref[i] - CppAD::cos(theta_k) * Ex_k + CppAD::sin(theta_k) * Ey_k;
                AD<double> y_k = Y_ref[i] - CppAD::cos(theta_k) * Ey_k - CppAD::sin(theta_k) * Ex_k;

                AD<double> xe_14 = x_k - X_4;
                AD<double> ye_14 = y_k - Y_4;
                AD<double> Pe_14 = CppAD::pow(CppAD::pow(xe_14, 2) + CppAD::pow(ye_14, 2), 0.5);
                // Only consider the actuation at time t.
                AD<double> v_cons = vars[v_start + i];
                AD<double> omega_cons = vars[omega_start + i];
                AD<double> delta_cons = vars[delta_start + i];
                AD<double> V_e_k = vars[v_e_start + i];
                AD<double> W_e_k = vars[omega_e_start + i];

                // AD<double> d_14_k = vars[d_14_start + i];
                // 终端控制器
                //  AD<double> V_k = k_1*Ex_k + V_ref[i] * CppAD::cos(Etheta_k)-row*W_ref[i]*sin(theta_k);
                //  AD<double> W_k = (k_2*Ey_k + V_ref[i] * CppAD::sin(Etheta_k))/row ;
                //  AD<double> V_k = k_1*Ex_k + V_ref[i] * CppAD::cos(Etheta_k);
                //  AD<double> W_k = (k_2*Ey_k + V_ref[i] * CppAD::sin(Etheta_k))/row ;
                AD<double> V_k = -V_e_k + V_ref[i] * CppAD::cos(Etheta_k);
                AD<double> W_k = (-W_e_k + V_ref[i] * CppAD::sin(Etheta_k)) / row;
                AD<double> Delta_k = atan(W_k * row / V_k);
                // 控制量更新约束
                // fg[1 + v_start + i ] = v_cons*CppAD::tan(delta_cons) - V_max + v_cons;
                fg[1 + omega_start + i] = omega_cons - W_k;
                fg[1 + v_start + i] = v_cons - V_k;
                // fg[1 + delta_start + i ] = -v_cons*CppAD::tan(delta_cons) - V_max + v_cons;
                //  设置纵向误差的范围约束，确保 -0.6 <= e_y <= 0
                // fg[1 + e_y_start + i] = vars[e_y_start + i] - (-0.6);  // 确保 e_y >= -0.6
                fg[1 + e_y_start + i] = vars[e_y_start + i] - 0; // 确保 e_y <= 0
                // e_y = (-sin(theta) * (X_ref[0] - x0[0]) + cos(theta) * (Y_ref[0] - x0[1])) ;
                // fg[1 + e_y_start + i ] = Y_e_k-e_y;//
                //  约束：确保14车距离不小于安全距离
                // fg[1 + d_14_start + i] = d_14_k -Pe_14 ;  // 即 d >= d_safe

                // 状态方程约束，描述了系统的状态（如误差、航向等）如何随时间变化，确保动态方程在优化过程中成立
                //  fg[2 + d_14 + i] = Pe_14 - 0.5;
                //  fg[2 + longitudinal_error_start + i] = vars[e_y_start + i] - 0;

                fg[2 + e_x_start + i] = Ex_k_1 - (Ex_k + (W_k * Ey_k + V_e_k) * dt);
                fg[2 + e_y_start + i] = Ey_k_1 - (Ey_k + (-W_k * Ex_k + W_e_k) * dt);
                fg[2 + e_theta_start + i] = Etheta_k_1 - (Etheta_k + ((Theta_ref[i + 1] - Theta_ref[i]) / dt - W_k) * dt);
            }
        }
    };

    int FindNearDouble(double x, double *X_ref, int crow);
    int FindNearCircle(double x, double y, double theta, double *X_ref, double *Y_ref, double *Theta_ref, int crow, int Init_index);
    int FindOuD(double x, double y, double theta, double *X_ref, double *Y_ref, double *Theta_ref, int crow, int Init_index);

    // Struct for the car information
    // 记录车辆的位姿信息结构体
    struct Car_Info
    {
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
    Eigen::VectorXd W_ref(N + 1);

    MPC mpc;

#define pi 3.1415926
}
#endif

// 避障2
// 动态参数
// AD<double> beita_x= (x_k - X_4) * cos(Theta_4) + (y_k - Y_4) * sin(Theta_4);
// AD<double> beita_y= (y_k - Y_4) * cos(Theta_4) + (x_k - X_4) * sin(Theta_4);
// AD<double> beita_oa= atan(beita_y/beita_x);
// AD<double> D= a_ * b_ / CppAD::pow(CppAD::pow(a_ * cos(beita_oa),2) + CppAD::pow(b_ * sin(beita_oa),2),0.5);
// fg[0] += lamuda_ob / (1 + CppAD::exp(-1.5 * (CppAD::pow(0.5, 2) -
//                                          (CppAD::pow(xe_14, 2) +
//                                           CppAD::pow(ye_14, 2)))));
// AD<double> D =0;
//  fg[0] += 1/D * ( e_ob - D);
//  fg[0] += lamuda_ob / (1+CppAD::exp(-k_ob * ( CppAD::pow(D, 2)-(
//  CppAD::pow(xe_ob, 2)+ CppAD::pow(ye_ob, 2)))));
//  fg[0] += lamuda_ob / (1 + CppAD::exp(-k_ob * (CppAD::pow(0.5, 2) - (CppAD::pow(xe_ob, 2) + CppAD::pow(ye_ob,2)))));
// 第一篇避障参数
//  fg[0] += lamuda_ob / (1 + CppAD::exp(-1.5 * (CppAD::pow(0.5 * D, 2) - (CppAD::pow(xe_ob, 2) + CppAD::pow(ye_ob, 2)))));
// 编队避障参数
//  fg[0] += lamuda_ob / (1 + CppAD::exp(-1.5 * (CppAD::pow(0.5 * 1, 2) - (CppAD::pow(xe_ob, 2) - CppAD::pow(ye_ob, 2)))));
// fg[0] += lamuda_ob / (1+CppAD::exp(-k_ob * ( CppAD::pow(0.5, 2)-( CppAD::pow(xe_4, 2)+ CppAD::pow(ye_4, 2)))));
// fg[0] += lamuda_3 / (1+CppAD::exp(-k_3 * ( CppAD::pow(0.5, 2)-( CppAD::pow(xe_3, 2)+ CppAD::pow(ye_3, 2)))));
#include "MPC.h"
// ofstream OutFile("/home/yhs/wyf_ws/src/car/computer/control/MPC_controller/trajectory/diyici.txt");
string filename = "/home/yhs/wyf_ws/src/car/computer/control/MPC_controller/trajectory/ch_data.txt";
// MPC class definition implementation.
// 构造函数

cta_msgs::PosAll PosAll_3;
namespace MPCtrack
{
    MPC::MPC() {}
    // 析构函数
    MPC::~MPC() {}
    // 一个内联函数

    // 传入参数 : 初始条件 和 参考路径信息
    // 传出参数 : Solution 结构体，包含 六个变量序列 加两个控制量序列
    Solution MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd X_ref, Eigen::VectorXd Y_ref, Eigen::VectorXd Theta_ref,
                        Eigen::VectorXd V_ref, Eigen::VectorXd W_ref, Eigen::VectorXd control_out, Eigen::VectorXd x_point_4)
    {
        size_t i;
        
        typedef CPPAD_TESTVECTOR(double) Dvector;

        double x = x0[0];
        double y = x0[1];
        double theta = x0[2];

        double X_4 = x_point_4[0];
        double Y_4 = x_point_4[1];
        double Theta_4 = x_point_4[2];
        // ROS_INFO("Received x_point4 position -> x: %.6f, y: %.2f", CppAD::Value(X_4), CppAD::Value(Y_4));
        ROS_INFO("Received point4 position -> x: %.6f, y: %.2f", X_4, Y_4);
        // int safe_distance;
        //  得到误差变量
        double e_x = (cos(theta) * (X_ref[0] - x0[0]) + sin(theta) * (Y_ref[0] - x0[1]));
        double e_y = (-sin(theta) * (X_ref[0] - x0[0]) + cos(theta) * (Y_ref[0] - x0[1]));
        double e_theta = Theta_ref[0] - x0[2];

        if (e_theta > 6.0)
        {
            e_theta -= 2 * pi;
        }
        if (e_theta < -6.0)
        {
            e_theta += 2 * pi;
        }

        // number of independent variables
        // N timesteps == N - 1 actuations
        // 计算变量个数 状态(e_x, e_y, e_theta) * 预测步长 + 控制量 * (预测步长 - 1)
        // 状态多一个是因为 初始状态也算变量
        // 三个 误差状态 + 两个控制量 + 2个辅助变量状态
        size_t n_vars = (N + 1) * 3 + N * 4 + N * 2;

        // Number of constraints,除了控制量
        size_t n_constraints = (N + 1) * 3 + N * 4;

        // Initial value of the independent variables.
        // Should be 0 except for the initial values.
        Dvector vars(n_vars);

        for (int i = 0; i < n_vars; i++)
        {
            vars[i] = 0.0; // 变量清0
        }
        // Set the initial variable values
        // 传入初始状态

        vars[e_x_start] = e_x;
        vars[e_y_start] = e_y;
        vars[e_theta_start] = e_theta;
        Eigen::VectorXd e0(3);
        e0 << e_x, e_y, e_theta;

        // 建立数学模型,x_point_2
        FG_eval fg_eval(e0, X_ref, Y_ref, Theta_ref, W_ref, V_ref, control_out, x_point_4);

        // Lower and upper limits for x
        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);

        // Set all non-actuators upper and lowerlimits
        // to the max negative and positive values.
        // 把所有状体变量都 置 0
        for (int i = 0; i < v_start; i++)
        {
            // 正负无穷
            vars_lowerbound[i] = -1000;
            vars_upperbound[i] = 1000;
        }
        // v约束
        for (int i = v_start; i < omega_start; i++)
        {
            vars_lowerbound[i] = 0;
            vars_upperbound[i] = V_max;
        }
        // w约束
        for (int i = omega_start; i < dV_start; i++)
        {
            vars_lowerbound[i] = -1000;
            vars_upperbound[i] = 1000;
        }
        // dv约束
        //  for (int i = dV_start; i < dV_start+1; i++)
        // {
        // 	vars_lowerbound[i] =-100;
        // 	vars_upperbound[i] =  100;
        // }
        // delta约束
        for (int i = delta_start; i < v_e_start; i++)
        {
            vars_lowerbound[i] = -SERVO_MAX;
            vars_upperbound[i] = SERVO_MAX;
        }
        // d_14约束
        //  for (int i = d_14_start; i < v_e_start; i++) {
        //  	vars_lowerbound[i] = 0.5;
        //      vars_upperbound[i] = 1000;
        //  }
        //  v_e约束
        for (int i = v_e_start; i < omega_e_start; i++)
        {
            vars_lowerbound[i] = -1000;
            vars_upperbound[i] = 1000;
        }
        // Acceleration/decceleration upper and lower limits.
        // 注意 这里一定要是 w_e 是最后的向量
        for (int i = omega_e_start; i < n_vars; i++)
        {
            // w_e约束
            vars_lowerbound[i] = -1000;
            vars_upperbound[i] = 1000;
        }

        // Lower and upper limits for constraints
        // All of these should be 0 except the initial
        // state indices.

        // 状态约束
        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);

        // 初始化所有约束为等式约束 0
        for (int i = 0; i < n_constraints; i++)
        {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
        }
        // 设置安全距离约束界限
        // for (int i = 0; i < N; i++) {
        //     constraints_lowerbound[safe_distance_start + i] = min_safe_distance;
        //     constraints_upperbound[safe_distance_start + i] = 1000;  // 可以设置为一个较大的值
        // }

        // // 设置纵向误差约束界限
        // for (int i = 0; i < N; i++) {
        //     constraints_lowerbound[longitudinal_error_start + i] = -1;  // 确保 e_y >= -1
        //     constraints_upperbound[longitudinal_error_start + i] = 0;  // 确保 e_y <= 0
        // }
        // 设置横向误差 e_x 为等式约束
        constraints_lowerbound[e_x_start] = e_x;
        constraints_upperbound[e_x_start] = e_x;

        // 设置纵向误差 e_y 的范围约束：-0.6 <= e_y <= 0
        constraints_lowerbound[e_y_start] = -1;
        constraints_upperbound[e_y_start] = 0;

        // 设置航向误差 e_theta 为等式约束
        constraints_lowerbound[e_theta_start] = e_theta;
        constraints_upperbound[e_theta_start] = e_theta;
        // 设置安全距离约束
        //     for (int i = 0; i < N; i++) {
        //     AD<double> x_k = X_ref[i] - CppAD::cos(Theta_ref[i]) * e_x + CppAD::sin(Theta_ref[i]) * e_y;
        //     AD<double> y_k = Y_ref[i] - CppAD::cos(Theta_ref[i]) * e_y - CppAD::sin(Theta_ref[i]) * e_x;
        //     AD<double> xe_4 =x_k- X_4;
        //     AD<double> ye_4= y_k- Y_4;
        //     AD<double> Pe_4 =CppAD::pow(CppAD::pow(xe_4, 2) + CppAD::pow(ye_4, 2), 0.5);
        //     // 确保与障碍物的距离大于最小安全距离
        //     constraints_lowerbound[distance14_start + i] = min_safe_distance; // 安全距离下限
        //     constraints_upperbound[distance14_start + i] = 1000; // 可以设置为一个较大的值
        // }

        // 初始的状态约束赋值为当前状态
        // Options
        std::string options;
        options += "Integer print_level  0\n";
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";

        // Place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;
        auto start_time = std::chrono::high_resolution_clock::now(); // 记录开始时间
        // Solve the problem
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options,                // 求解器的选项，例如容差、最大迭代次数
            vars,                   // 决策变量（优化变量）的向量
            vars_lowerbound,        // 决策变量的下界
            vars_upperbound,        // 决策变量的上界
            constraints_lowerbound, // 约束条件的下界
            constraints_upperbound, // 约束条件的上界
            fg_eval,                // 计算目标函数和约束条件的函数
            solution                // 求解器返回的解
        );

        auto end_time = std::chrono::high_resolution_clock::now(); // 记录结束时间
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double solve_time = duration.count() / 1e6; // 转换为秒，double类型
        
        // 可以将 solve_time 保存到成员变量中，以便在 Controller::DoControl 中写入文件
        this->solve_t = solve_time; // 假设您在 MPC 类中添加了 solve_t 成员变量

        std::cout << "Ipopt求解耗时: " << this->solve_t << " 秒" << std::endl;
        std::cout << "原始微秒数: " << duration.count() << " μs" << std::endl;
        
        // 检查是否求解成功
        bool ok = true;
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
        Solution sol;

        for (int i = 0; i <= N - 1; i++)
        {
            cout << i << ": " << "E_x is : " << solution.x[e_x_start + i] << "  E_y is: " << solution.x[e_y_start + i]
                 << "  E_theta is : " << solution.x[e_theta_start + i] << endl;

            cout << i << ": " << "V_e is : " << solution.x[v_e_start + i] << "  Omega_e is: " << solution.x[omega_e_start + i]
                 << endl;
            // 将路径跟踪误差的x分量、y分量和角度分量分别添加到 sol 对象的相应向量中
            sol.E_X.push_back(solution.x[e_x_start + i]);
            sol.E_Y.push_back(solution.x[e_y_start + i]);
            sol.E_Theta.push_back(solution.x[e_theta_start + i]);
            sol.V_e.push_back(solution.x[v_e_start + i]);
            sol.Omega_e.push_back(solution.x[omega_e_start + i]);
            sol.delta.push_back(solution.x[delta_start + i]);
        }
        sol.status = solution.status;
        cost = solution.obj_value;
        std::cout << "Cost " << cost << std::endl;

        return sol;
    }

    Controller::Controller() {}

    Controller::Controller(ros::NodeHandle &nh, const double frq)
    {
        this->rosNode = nh;
        this->rosLoopHz = frq;
        // 清空文件
        fstream file(filename.c_str(), ios::out);
        // 发布器
        // pubMode = rosNode.advertise<cta_msgs_control::ControlMode>(UsrLib::TOPIC_CONTROL_MODE, 1);
        // pubMotion = rosNode.advertise<cta_msgs_control::MotionControl>(UsrLib::TOPIC_CONTROL_MOTION, 1);
        // pubLamp = rosNode.advertise<cta_msgs_control::LampControl>(UsrLib::TOPIC_CONTROL_LAMP, 1);

        control_ada_ = rosNode.advertise<geometry_msgs::Twist>("/wyf_ws_1/smoother_cmd_vel", 10); // 发布控制信号
        // 订阅器
        // 订阅本车的真实位置
        subTheta = rosNode.subscribe("/wyf_ws_1/RPY", 1, &Controller::yhs_Callback_UpdateTheta, this);
        subPoints = rosNode.subscribe("/wyf_ws_1/nlink_linktrack_nodeframe2", 1, &Controller::Callback_UpdatePoints, this);
        // 订阅本车规划的参考位置
        // subPlanTrj = rosNode.subscribe(UsrLib::TOPIC_PLAN_LOCALROUTE, 1, &Controller::Callback_UpdatePlanTrj, this);
        subPlanTrj = rosNode.subscribe("/msg_plan_local", 1, &Controller::Callback_UpdatePlanTrj, this);

        // 订阅本车的仿真位置
        //  subSelfPose = rosNode.subscribe(UsrLib::TOPIC_PERCEPTION_MOTIONINFO, 1, &Controller::Callback_UpdateSelfPose, this);
        pos_3 = rosNode.advertise<cta_msgs_control::PredictPoint>("poseAll_3", 1); // 发布本车最优位置序列

        // subSelfPose_4 = rosNode.subscribe("/r4/msg_perception_selfpose", 1, &Controller::Callback_UpdateSelfPose_4, this);//订阅4车仿真位置信息
        // 订阅4车真实位姿
        subTheta_4 = rosNode.subscribe("/wyf_ws_4/odom", 1, &Controller::Callback_UpdateTheta_4, this);
        subPoints_4 = rosNode.subscribe("/wyf_ws_4/nlink_linktrack_nodeframe2", 1, &Controller::Callback_UpdatePoints_4, this);
        // 订阅4车参考轨迹
        // subPlanTrj_4= rosNode.subscribe("/wyf_ws_4/msg_plan_local", 1, &Controller::Callback_UpdatePlanTrj_4, this);
        // 订阅4车输入
        //  cmd_sub_4= rosNode.subscribe("/r4/smoother_cmd_vel", 1, &Controller::Callback_Ctrl_4, this);
    }

    Controller::~Controller() {}

    void Controller::run()
    {
        ros::Timer t = rosNode.createTimer(ros::Duration(1 / rosLoopHz), &Controller::DoControl, this);
        ros::spin();
    }

    void Controller::DoControl(const ros::TimerEvent &e)
    {
        if (this->refTrj.size() == 0)
        {
            ROS_INFO("No Reference Route ------");
            return;
        }
        else if (point4_flag == 0)
        {
            ROS_INFO(" point4_flag=%d", point4_flag);
            return;
        }
        else
        {
            // if(point4_flag==1)
            // {
            GetGoalPose();
            AllController();
            PublishTopics();

            ROS_INFO("MPC:: desired liner speed : %.3f, desired steer angle : %.3f", msgControl.linear.x, msgControl.angular.z);

            ros::Time current_time;
            current_time = ros::Time::now();

            ofstream OutFile(filename.c_str(), ios::app); // 以追加模式打开文件
            if (!OutFile)
            {
                cout << "打开文件失败，路径: " << filename << endl;
                perror("错误原因");
            }
            OutFile << nowPos.x << " " << nowPos.y << " " << nowPos.h << " " << nowPos.v << " " << nowPos.w << " ";     // 0 1 2 3 4
            OutFile << X_ref(0) << " " << Y_ref(0) << " " << Theta_ref(0) << " " << V_ref(0) << " " << W_ref(0) << " "; // 5-9
            OutFile << v_star << " " << omega << " " << Delta_f << " " << vee << " " << omegaee << " ";                 // 10-14
            OutFile << nowPos_4.x << " " << nowPos_4.y << " " << nowPos_4.h << " ";                                     // 15-17
            OutFile << mpc.solve_t << " " ;//18
            OutFile << cost << " " << endl; // 19
            OutFile.close();
            // }
        }
    }

    void Controller::Callback_UpdateSelfPose(const cta_msgs_perception::SelfPose::ConstPtr &msg)
    {
        nowPos.x = msg->state.pos.x;
        nowPos.y = msg->state.pos.y;
        nowPos.h = msg->state.rot.z; // msg 对象中获取 state 结构体的 rot 联合体（或结构体）的 z 成员，并将这个值赋给 nowPos.h
        nowPos.v = msg->state.vel.x;
        point_flag = 1;
        // ROS_INFO("------xxccxeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" );
    }

    void Controller::yhs_Callback_UpdateTheta(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        nowPos.h = msg->data[2] / 180 * 3.14;
        static int runonce = 1;
        nowPos.h = nowPos.h;
        if (nowPos.h >= pi)
        {
            nowPos.h -= 2 * pi;
        }
        if (nowPos.h <= -pi)
        {
            nowPos.h += 2 * pi;
        }
    }

    void Controller::Callback_UpdatePoints(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
    {
        nowPos.x = msg->pos_3d[0];
        nowPos.y = msg->pos_3d[1];

        point_flag = 1;
    }

    // 清空参考轨迹，写入新的规划轨迹
    void Controller::Callback_UpdatePlanTrj(const cta_msgs_planning::LocalRoute2d::ConstPtr &msg)
    {
        this->refTrj.clear();
        Pose2D p;
        for (int i = 0; i < msg->waypoints.size(); i++)
        {
            p.h = msg->waypoints[i].rot;
            p.x = msg->waypoints[i].pos.x;
            p.y = msg->waypoints[i].pos.y;
            p.v = msg->waypoints[i].vel;
            p.w = msg->waypoints[i].pla;
            // p.v = 0.26;
            // ROS_INFO("************************x:%.6f, *********************************Y:%.6f\n",  p.x, p.y );
            this->refTrj.push_back(p);
        }
        ref_flag = 1;
        // ROS_INFO("************************ok, *********************************ok.\n");
    }

    // 订阅四车
    // 邻居4车的真实位置信息
    void Controller::Callback_UpdateTheta_4(const nav_msgs::Odometry::ConstPtr &msg)
    {
        nowPos_4.h = msg->pose.pose.position.z;

        if (nowPos_4.h >= pi)
        {
            nowPos_4.h -= 2 * pi;
        }
        if (nowPos_4.h <= -pi)
        {
            nowPos_4.h += 2 * pi;
        }
        ROS_INFO("Received point4 h -> h: %.2f", nowPos_4.h);
        pos4_flag = 1;
    }
    // void Controller::Callback_UpdateTheta_4(const std_msgs::Float64MultiArray::ConstPtr &msg){
    //     nowPos_4.h = msg->data[2]/180*3.14;;
    //     if(nowPos_4.h >= pi)
    //     {
    //         nowPos_4.h -= 2*pi;
    //     }
    //     if(nowPos_4.h <= -pi)
    //     {
    //         nowPos_4.h += 2*pi;
    //     }
    //     ROS_INFO("Received point4 h -> h: %.2f", nowPos_4.h);
    //     pos4_flag =1;

    // }
    // 邻居4车的真实位置信息

    void Controller::Callback_UpdatePoints_4(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
    {

        nowPos_4.x = msg->pos_3d[0];
        nowPos_4.y = msg->pos_3d[1];

        // nowPos_4.x = 2.6;
        // nowPos_4.y = 3.6; // 固定 y 为 3.6
        ROS_INFO("Received point4 position -> x: %.6f, y: %.2f", nowPos_4.x, nowPos_4.y);
        point4_flag = 1;
    }

    // 发布速度控制消息msgControl到cmd_vel话题，这是ROS中用于控制机器人或车辆速度的标准话题
    void Controller::PublishTopics()
    {

        control_ada_.publish(msgControl);
    }
    // 更新参考轨迹
    void Controller::GetGoalPose()
    {
        double mindis = 1e8;                  // 初始值设为一个非常大的数 1e8，用于存储当前车辆与参考轨迹点的最短距离。
        int refnum = 0;                       // 用于记录当前车辆与轨迹点之间最短距离对应的参考轨迹点索引
        int refsize = (int)refTrj.size() - N; // 计算有效参考轨迹点的数量（refTrj.size() - N），表示在轨迹中可用于参考的点的数目
        // 找最近的点的位置
        //  for (int i = 0; i < refTrj.size(); i++) {
        //      double d = Dis(refTrj[i], nowPos);
        //      if (d < mindis) {
        //      mindis = d;
        //      refnum = i;  //最近点的位置
        //      }
        //  }
        // 生成参考轨迹
        int nowpos_num = min((refnum + 1), refsize);
        for (int i = 0; i < N; i++)
        {
            X_ref(i) = refTrj[nowpos_num + DT * (i)].x;
            Y_ref(i) = refTrj[nowpos_num + DT * (i)].y;
            Theta_ref(i) = refTrj[nowpos_num + DT * (i)].h;
            V_ref(i) = refTrj[nowpos_num + DT * (i)].v;
            W_ref(i) = refTrj[nowpos_num + DT * (i)].w;
        }
        ROS_INFO("M:: nowpos_num = %d, X_ref(num) = %.3f, Y_ref(num) = %.3f", nowpos_num, X_ref(N - 1), Y_ref(N - 1));
    }

    // 解决动态障碍物避障问题并生成控制指令
    void Controller::AllController()
    {

        y_inf = nowPos.y;
        x_inf = nowPos.x;
        theta_inf = nowPos.h;
        // 预测步长为采样点间距/速度
        double e_x = cos(theta_inf) * (X_ref[0] - x_inf) + sin(theta_inf) * (Y_ref[0] - y_inf);
        double e_y = -sin(theta_inf) * (X_ref[0] - x_inf) + cos(theta_inf) * (Y_ref[0] - y_inf);
        double e_theta = Theta_ref[0] - theta_inf;
        // e_x += cos(e_theta) * dx - sin(e_theta) * dy;
        // e_y += sin(e_theta) * dx + cos(e_theta) * dy;
        ROS_INFO("ref_flag: %d,point_flag: %d,point4_flag: %d", ref_flag, point_flag, point4_flag);
        count_Time++;
        // 控制器计算
        if (ref_flag == 1)
        {
            if (count_Time >= 2)
            {
                ROS_INFO("e_x = %f", e_x);
                ROS_INFO("e_y = %f", e_y);
                // 初始化当前状态和障碍物状态向量
                Eigen::VectorXd state_1(3);
                state_1 << x_inf, y_inf, theta_inf;
                Eigen::VectorXd control_out(2);
                control_out << v_star, Delta_f;
                Eigen::VectorXd point_4(3);
                point_4 << nowPos_4.x, nowPos_4.y, nowPos_4.h;
                // ROS（Robot Operating System）的日志系统来输出错误信息
                ROS_INFO("begain solve!");
                ROS_INFO("nowpoint:: (%.3f, %.3f, %.3f)", x_inf, y_inf, theta_inf);
                ROS_INFO("refpoint:: (%.3f, %.3f, %.3f, %.3f)", X_ref(0), Y_ref(0), Theta_ref(0), V_ref(0));
                // 求解
                Solution sol = mpc.Solve(state_1, X_ref, Y_ref, Theta_ref, V_ref, W_ref, control_out, point_4);
                ref_flag = 0;
                point_flag = 0;
                pos4_flag = 0;
                point4_flag = 0;
                // 得到误差控制量
                vee = sol.V_e.at(0);
                COST = sol.Cost;
                omegaee = sol.Omega_e.at(0);
                // Delta_f = sol.D.at(0);
                //  输出每一步的误差信息
                for (int x = 0; x < N; x++)
                {
                    ROS_INFO("************************X_e:%.6f, *********************************Y_e:%.6f\n", sol.E_X.at(x), sol.E_Y.at(x));
                }
                if (sol.status != 1)
                {
                    cout << "求解失败 ！ " << endl;
                    solve_flag = 0;
                }
                else
                {
                    cout << "求解成功 ！ " << endl;
                    // 根据误差调整当前控制输出
                    double te = Theta_ref(0) - theta_inf;
                    // v_star = -vee + V_ref(0) * cos(te)-row*W_ref(0)*sin(te);
                    // omega = (-omegaee + V_ref(0) * sin(te)+row*W_ref(0)*cos(te)) / row;
                    v_star = -vee + V_ref(0) * cos(te);
                    omega = (-omegaee + V_ref(0) * sin(te)) / row;
                    // v_star = vee ;
                    // Delta_f = sol.D.at(0);
                    // omega = v_star*tan(Delta_f)/row;
                    solve_flag = 1;
                    // 更新预测路径
                    msgPosAll.waypoints.clear();
                    for (int s = 0; s < N; s++)
                    {
                        PosAll_3.rot = Theta_ref[s + 1] - sol.E_Theta.at(s); // sol.E_Theta 是 Solution 对象中的一个成员变量,at(s) 是 Eigen 向量的访问方法
                        PosAll_3.pos.x = X_ref[s + 1] - cos(PosAll_3.rot) * sol.E_X.at(s) + sin(PosAll_3.rot) * sol.E_Y.at(s);
                        PosAll_3.pos.y = Y_ref[s + 1] - cos(PosAll_3.rot) * sol.E_Y.at(s) - sin(PosAll_3.rot) * sol.E_X.at(s);
                        PosAll_3.vel = 0;
                        PosAll_3.delta = 0;
                        msgPosAll.waypoints.push_back(PosAll_3); // 发布预测路径
                    }
                    pos_3.publish(msgPosAll);
                }
                ROS_INFO("Solve By MPC!");
                // 计算目标前轮转角
                double Tan_f_star = omega * row / v_star;
                Delta_f = atan(Tan_f_star);
                if (Delta_f > SERVO_MAX)
                {
                    Delta_f = SERVO_MAX;
                }
                else if (Delta_f < -SERVO_MAX)
                {
                    Delta_f = -SERVO_MAX;
                }
                // Delta_f = Delta_f/pi *180;
                msgControl.linear.x = v_star;
                // msgControl.linear.x = -v_star;
                msgControl.angular.z = omega;
                nowPos.v = v_star;
                nowPos.w = omega;
                //   msgControl.linear.x = 0.2;
                //  msgControl.angular.z = 0 ;
                // msgControl.angular.z = 0.5;
            }
        }
    }
    double Controller::Dis(const Pose2D p1, const Pose2D p2) { return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); }

}

// double mindis = 1e8; // 用于存储与障碍物的最短距离
// int refsize = (int)refTrj.size() - N; // 参考轨迹的有效范围
// int refnum = 0;

//     //找最近的点的位置
// for (int i = 0; i < refTrj.size(); i++) {
//     double d = Dis(refTrj[i], nowPos);
//     if (d < mindis) {
//     mindis = d;
//     refnum = i;  //最近点的位置
//     }
// }
// // 假设障碍物4的位置和状态
// double obstacle_x = nowPos_4.x;    // 障碍物4的x坐标
// double obstacle_y = nowPos_4.y;    // 障碍物4的y坐标
// double obstacle_v = 0.15;    // 障碍物4的速度
// double obstacle_h = nowPos_4.h;    // 障碍物4的航向角（方向）

// // 预测障碍物未来的位置
// double dt = 0.1; // 假设预测的时间步长为0.1秒
// double predicted_obstacle_x = obstacle_x + obstacle_v * dt * cos(obstacle_h); // 预测的x坐标
// double predicted_obstacle_y = obstacle_y + obstacle_v * dt * sin(obstacle_h); // 预测的y坐标

// refTask.clear();
// // 生成参考轨迹
// int nowpos_num = min((refnum + 1), refsize);

// for (int i = 0; i < N; i++) {
//     int index = nowpos_num + DT * i;
//     if (index < refTrj.size()) {
//         // 获取当前轨迹点的位置
//         double ref_x = refTrj[index].x;
//         double ref_y = refTrj[index].y;

//         // 计算参考轨迹点与障碍物未来位置之间的距离
//         double dist_to_obstacle = sqrt(pow(ref_x - predicted_obstacle_x, 2) + pow(ref_y - predicted_obstacle_y, 2));
//            //double dist_to_obstacle = sqrt(pow(ref_x - obstacle_x, 2) + pow(ref_y - obstacle_y, 2));
//         // 如果距离小于安全距离，进行避障
//         if (dist_to_obstacle < min_safe_distance) {
//             // 计算避障方向，推离障碍物
//             double angle_to_obstacle = atan2(ref_y - predicted_obstacle_y, ref_x - predicted_obstacle_x);
//             double avoidance_angle = angle_to_obstacle + M_PI / 4; // 绕障碍物的90度避让

//             // 根据避让角度调整参考轨迹
//             X_ref(i) = ref_x + cos(avoidance_angle) * (min_safe_distance - dist_to_obstacle);
//             Y_ref(i) = ref_y + sin(avoidance_angle) * (min_safe_distance - dist_to_obstacle);
//             Theta_ref(i) = atan2(Y_ref(i) - nowPos.y, X_ref(i) - nowPos.x); // 更新航向角
//             V_ref(i) = refTrj[index].v;
//             W_ref(i) = refTrj[index].w;
//         } else {
//             // 如果距离足够远，保持原有轨迹
//             X_ref(i) = ref_x;
//             Y_ref(i) = ref_y;
//             Theta_ref(i) = refTrj[index].h;
//             V_ref(i) = refTrj[index].v;
//             W_ref(i) = refTrj[index].w;
//           }
//       }
//  }

// //输出参考轨迹的最后一个点
// ROS_INFO("M:: nowpos_num = %d, X_ref(num) = %.3f, Y_ref(num) = %.3f", nowpos_num, X_ref(N-1), Y_ref(N-1));

// void Controller::GetGoalPose() {
//   double mindis = 1e8;
//   int refnum = 0;
//   //发布最优位置序列
//   // if(solve_flag==0&&pos3_flag==1&&point3_flag==1)
//   // {
//   //  msgPosAll.waypoints.clear();
//   // for (int s=0; s<N;s++)
//   // {
//   //     PosAll_3.rot = nowPos.h ;
//   //     PosAll_3.pos.x = nowPos.x ;
//   //     PosAll_3.pos.y = nowPos.y  ;
//   //     PosAll_3.vel = 0;
//   //     PosAll_3.delta = 0;
//   //     msgPosAll.waypoints.push_back(PosAll_3);

//   // }
//   // pos_3.publish(msgPosAll);

//   // }
//   //找最近的点的位置
//   for (int i = 0; i < refTrj.size(); i++) {
//     double d = Dis(refTrj[i], nowPos);
//     if (d < mindis) {
//       mindis = d;
//       refnum = i;  //最近点的位置
//     }
//   }

//   //取合适的参考点
//   int goalnum = min(refnum, (int)refTrj.size());
//   goalnumGolbal = goalnum;
//   // int flag =goalnumGolbal  + ym;
//   int flag = 0 + ym;
//   //真正的参考序列
//   for (int i = 0; i <= N; i++) {
//     if (flag >= refTrj.size() - N - 1) flag = refTrj.size() - N - 1;
//     // if (nowPos.v > 3)
//     X_ref(i) = refTrj[flag + DT * (i)].x;
//     Y_ref(i) = refTrj[flag + DT * (i)].y;
//     Theta_ref(i) = refTrj[flag + DT * (i)].h;
//     //  X_ref(i) = 5;
//     //  Y_ref(i) =5;
//     //  Theta_ref(i) = pi/4;
//     // V_ref(i) = 0;
//     V_ref(i) = refTrj[flag + DT * (i)].v;
//     ROS_INFO(
//         "************************X_ref:%.6f, "
//         "*********************************Y_ref:%.6f,goalnum:%d\n",
//         X_ref(i), Y_ref(i), goalnum);
//   }
//   DT = 1;
// }
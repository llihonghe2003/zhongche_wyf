#include "../include/planning_globaltrack/globaltrack.h"

using namespace std;

namespace PlannerGlobalTrack
{

    Planner::Planner() {}

    Planner::Planner(ros::NodeHandle &nh, const double frq)
    {
        this->rosNode = nh;
        this->rosLoopHz = frq;
        this->isRun = false;

        // 订阅器     仿真位置                // 话题/msg_perception_selfpose
        // this->subSelfPose = rosNode.subscribe(UsrLib::TOPIC_PERCEPTION_MOTIONINFO, 1, &Planner::Callback_UpdateSelfPose, this);
        // 订阅器     实际位置
        // this->subSelfTheta = rosNode.subscribe("odom_raw", 1, &Planner::Callback_UpdateTheta, this);
        // this->subSelfTheta = rosNode.subscribe("odom", 1, &Planner::Callback_UpdateTheta, this);
        this->subSelfPoints_1 = rosNode.subscribe("/wyf_ws_1/nlink_linktrack_nodeframe2", 1, &Planner::Callback_UpdatePoints, this);
        //    this->subSelfPoints = rosNode.subscribe("nlink_linktrack_nodeframe2", 1, &Planner::Callback_UpdatePoints, this);
        //    this->subSelfTheta =rosNode.subscribe("RPY", 1,  &Planner::yhs_Callback_UpdateTheta, this);
        // 发布器  参考
        // this->pubLocTrj_1 = rosNode.advertise<cta_msgs_planning::LocalRoute2d>(UsrLib::TOPIC_PLAN_LOCALROUTE, 1);
        this->pubLocTrj_1 = rosNode.advertise<cta_msgs_planning::LocalRoute2d>("/msg_plan_local", 1);
    }

    Planner::~Planner() {}

    void Planner::run()
    {
        ros::Rate loop(rosLoopHz);

        // load global route
        string filename = "/home/yhs/wyf_ws/src/route/lineping-0.5,3.6.v0.25.txt";
        //string filename = "/home/yhs/wyf_ws/src/route/line-5.0,0.5.v0.25.txt";
        if (this->SetGobalRoute(filename))
        {
            ROS_INFO("Success to Load Global Route !!!!!");
        }
        else
        {
            ROS_INFO("Fail to Load Global Route !!!!!");
            return;
        }

        // main loop
        while (ros::ok)
        {
            ros::spinOnce();
            loop.sleep();

            if (!this->isRun)
            {
                ROS_INFO("G:: No isRun State Info ......");
                continue;
            }

            if (this->PlanLocalTrajectory())
            {
                // 不用
                // PlanBehavior();
                // this->pubBehDec.publish(msgBeh);

                // 发布参考
                this->pubLocTrj_1.publish(msgLocTrj);
                ROS_INFO("G:: Publish the Plan Local Trajectory--------");
            }
        }
    }

    // 订阅仿真位置
    void Planner::Callback_UpdateSelfPose(const cta_msgs_perception::SelfPose::ConstPtr &msg)
    {
        nowPos.x = msg->state.pos.x;
        nowPos.y = msg->state.pos.y;
        nowPos.h = msg->state.rot.z;
        nowPos.v = msg->state.vel.x;

        this->isRun = true;
    }

    // 订阅真实位姿
    void Planner::Callback_UpdateTheta(const nav_msgs::Odometry::ConstPtr &msg)
    {
        nowPos.h = msg->pose.pose.position.z;
        this->isRun = true;
    }
    // 订阅真实位置
    void Planner::Callback_UpdatePoints(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg)
    {
        nowPos.x = msg->pos_3d[0];
        nowPos.y = msg->pos_3d[1];
        this->isRun = true;
        //    ROS_INFO("************************x:%.6f, *********************************8y:%.2f\n",  nowPos.x, nowPos.y );
    }
    // 订阅真实位姿
    //  void Planner::yhs_Callback_UpdateTheta(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    //  nowPos.h = msg->data[2]/180*3.14;
    //  this->isRun = true;
    //  ROS_ERROR_STREAM("nowPos.h：" << msg->data[2]/180*3.14);
    //  }
    //  //订阅真实位置
    //  void Planner::Callback_UpdatePoints(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg){
    //       nowPos.x = msg->pos_3d[0];
    //       nowPos.y = msg->pos_3d[1];
    //       this->isRun = true;
    //   //    ROS_INFO("************************x:%.6f, *********************************8y:%.2f\n",  nowPos.x, nowPos.y );

    // }

    bool Planner::PlanBehavior()
    {
        this->msgBeh.header.frame_id = "planbehavior";
        this->msgBeh.header.stamp = ros::Time::now();

        this->msgBeh.scenario = UsrLib::SCENARIO_FREE; // free scenario
        this->msgBeh.behavior = UsrLib::BEHAVIOR_FOWARDRUN;

        this->msgBeh.speed = 3.0;

        // BEHAVIOR_RIGHTMERGE;
        // BEHAVIOR_LEFTMERGE;
        // BEHAVIOR_LEFTTURN;
        // BEHAVIOR_LEFTMERGE
        // BEHAVIOR_FOWARDRUN; //go forward

        return true;
    }

    bool Planner::PlanGlobalRoute()
    {

        // message header
        msgGlbRoute.header.frame_id = "globalroute";
        msgGlbRoute.header.stamp = ros::Time::now();

        // global route information
        msgGlbRoute.startpos.x = this->glbRoute[0].x;
        msgGlbRoute.startpos.y = this->glbRoute[0].y;
        msgGlbRoute.waypoints.clear();

        int ptcount = this->glbRoute.size();
        cta_msgs::WayPoint2d wayPt;
        for (int i = 0; i < ptcount; i++)
        {
            wayPt.roadid = 0;
            wayPt.laneid = 0;
            wayPt.pos.x = this->glbRoute[i].x;
            wayPt.pos.y = this->glbRoute[i].y;
            wayPt.rot = this->glbRoute[i].h;
            wayPt.vel = 5;
            wayPt.curv = 0.0;

            msgGlbRoute.waypoints.push_back(wayPt);
        }

        msgGlbRoute.goalpos.x = msgGlbRoute.waypoints.back().pos.x;
        msgGlbRoute.goalpos.y = msgGlbRoute.waypoints.back().pos.y;

        return true;
    }
    // 跟据位置计算参考
    bool Planner::PlanLocalTrajectory()
    {

        double sstep = 0.1;
        int Nx = 20;

        ROS_INFO("G:: Search ---------");
        // 计算距离当前点最近的点
        int refNum = this->GetRefernum(this->nowPos);
        if (refNum < 0)
        {
            ROS_INFO("G:: refNum error, No Reference Pos !!!!");
            return false;
        }
        else
        {
            Pose2D pos = this->glbRoute[refNum];
            ROS_INFO("G:: Refnum = %d, x = %.3f, y = %.3f, h = %.3f", refNum, pos.x, pos.y, pos.h);
        }

        int stNum = refNum;
        stNum = max(0, stNum);

        // local route message: message header
        msgLocTrj.header.frame_id = "localroute";
        msgLocTrj.header.stamp = ros::Time::now();

        // local route message: sample parameters
        msgLocTrj.sampletype = 1; // distanse-equal sample
        msgLocTrj.samplestep = sstep;

        cta_msgs::WayPoint2d wp;
        msgLocTrj.waypoints.clear();
        for (int i = stNum; i < stNum + Nx; i++)
        {
            if (i >= this->glbRoute.size())
            {
                wp.roadid = 0;
                wp.laneid = 0; // back() 取数组最后一个元素
                wp.pos.x = this->glbRoute.back().x;
                wp.pos.y = this->glbRoute.back().y;
                wp.rot = this->glbRoute.back().h;
                wp.vel = this->glbRoute.back().v;
                wp.curv = 0.0;
                wp.pla = this->glbRoute.back().w;
            }
            else
            {

                // ?
                for (int j = 0; j < 5; j++)
                {
                    wp.roadid = 0;
                    wp.laneid = 0;
                    wp.pos.x = glbRoute[i].x + j / 5 * (glbRoute[i + 1].x - glbRoute[i].x);
                    wp.pos.y = glbRoute[i].y + j / 5 * (glbRoute[i + 1].y - glbRoute[i].y);
                    wp.rot = this->Ang(glbRoute[i], glbRoute[i + 1]);
                    wp.vel = this->glbRoute[i].v;
                    wp.curv = 0.0;
                    wp.pla = this->glbRoute[i].w;
                }
                // double dx = glbRoute[i+1].x - glbRoute[i].x;
                // double dy = glbRoute[i+1].y - glbRoute[i].y;
                // ROS_INFO("plan wp h = %.3f, x1 = %.3f, y1 = %.3f, x2 = %.3f, y2 = %.3f",
                // wp.rot, glbRoute[i].x, glbRoute[i].y, glbRoute[i+1].x, glbRoute[i+1].y);
            }

            msgLocTrj.waypoints.push_back(wp);
            if (msgLocTrj.waypoints.size() >= Nx)
            {
                break;
            }
        }
        // ROS_INFO("G:: x = %.3f, y = %.3f, h = %.3f", wp.pos.x, wp.pos.y, wp.rot);
        // ROS_INFO("G:: Plan over, Trj size = %d", msgLocTrj.waypoints.size());//
        ROS_INFO("G:: Plan over, Trj size = %d", static_cast<int>(msgLocTrj.waypoints.size()));

        return true;
    }

    // 参考文件数据导入序列
    bool Planner::SetGobalRoute(string filename)
    {
        ifstream infile;
        infile.open(filename.c_str());
        if (!infile.is_open())
        {
            ROS_INFO("G:: File Open Failed !!!!!!");
            return false;
        }

        double lat, lon, x, y, h, v, w;
        this->glbRoute.clear();
        // 文件结束
        while (!infile.eof())
        {
            infile >> x >> y >> h >> v >> w;
            Pose2D p_ = {x, y, h, v, w};
            // 文件尾端插入
            this->glbRoute.push_back(p_);
        }
        // 文件尾端删除 此时仅仅是取消数组地址最后一个元素的地址映射
        this->glbRoute.pop_back();

        infile.close();
        return true;
    }

    // 计算距离实际位置最近的点
    int Planner::GetRefernum(const Pose2D pos)
    {
        int ptcount = this->glbRoute.size();
        ROS_INFO("G:: nowPos.x is %f, nowPos.y is %f, nowPos.h is %f", pos.x, pos.y, pos.h);
        // get refer point on global route
        double mindis = 300;
        int refnum = -1;
        Pose2D p;
        for (int i = 0; i < ptcount; i++)
        {
            p = this->glbRoute[i];

            double d = Dis(p, pos);
            if (d < mindis)
            {
                mindis = d;
                refnum = i;
            }
        }
        return refnum;
    }

    cv::Mat Planner::Plot(const string win, const vector<double> x, const vector<double> y, const cv::Scalar color, int linewidth)
    {

        double minX = floor(*min_element(x.begin(), x.end()));
        double maxX = ceil(*max_element(x.begin(), x.end()));
        double minY = floor(*min_element(y.begin(), y.end()));
        double maxY = ceil(*max_element(y.begin(), y.end()));

        double rtX = (maxX - minX) / 300.0;
        double rtY = (maxY - minY) / 300.0;

        cv::Point pt1, pt2;
        for (int i = 0; i < x.size() - 1; i++)
        {
            pt1.x = (int)(x[i] - minX) / rtX;
            pt1.y = (int)(y[i] - minY) / rtY;
            pt2.x = (int)(x[i + 1] - minX) / rtX;
            pt2.y = (int)(y[i + 1] - minY) / rtY;
        }

        cv::Mat outimg;

        cv::namedWindow(win);
        cv::imshow(win, outimg);
        return outimg;
    }

    double Planner::Dis(const Pose2D p1, const Pose2D p2)
    {
        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    }

    double Planner::Ang(const Pose2D p1, const Pose2D p2)
    {
        return atan2(p2.y - p1.y, p2.x - p1.x);
    }

    double Planner::Uag(const double a)
    {
        double u = a;
        while (u > M_PI)
        {
            u -= 2 * M_PI;
        }
        while (u <= -M_PI)
        {
            u += 2 * M_PI;
        }
        return u;
    }

}
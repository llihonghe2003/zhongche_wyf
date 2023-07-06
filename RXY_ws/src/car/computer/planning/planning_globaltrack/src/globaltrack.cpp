#include "../include/planning_globaltrack/globaltrack.h"

using namespace std;

namespace PlannerGlobalTrack {

Planner::Planner() {}

Planner::Planner(ros::NodeHandle &nh, const double frq) {
  this->rosNode = nh;
  this->rosLoopHz = frq;
  this->isRun = false;

  //订阅器     仿真位置
  // this->subSelfPose = rosNode.subscribe(UsrLib::TOPIC_PERCEPTION_MOTIONINFO, 1, &Planner::Callback_UpdateSelfPose, this);
 
  //订阅器     实际位置
  // this->subSelfTheta = rosNode.subscribe("odom_raw",
  // 1,&Planner::Callback_UpdateTheta, this);
  // this->subSelfTheta =rosNode.subscribe("odom", 1,  &Planner::Callback_UpdateTheta, this); 
  this->subSelfTheta =rosNode.subscribe("/RPY", 1,  &Planner::yhs_Callback_UpdateTheta, this); 
  this->subSelfPoints =rosNode.subscribe( "nlink_linktrack_nodeframe2", 1, &Planner::Callback_UpdatePoints, this);

  //发布器  不用
  this->pubBehDec = rosNode.advertise<cta_msgs_planning::DecisionBehavior>(UsrLib::TOPIC_PLAN_BEHAVIOR, 1);
  this->pubGlbRoute = rosNode.advertise<cta_msgs_planning::GlobalRoute2d>(UsrLib::TOPIC_PLAN_GLOBALROUTE, 1);

  //发布器  参考
  this->pubLocTrj = rosNode.advertise<cta_msgs_planning::LocalRoute2d>(UsrLib::TOPIC_PLAN_LOCALROUTE, 1);
}

Planner::~Planner() {}

void Planner::run() {
  ros::Rate loop(rosLoopHz);
  // XmlRpc::XmlRpcValue UsrLib::params;
  // ros::param::get("params", UsrLib::params);
  // UsrLib::paramss = 1;
  // extern int UsrLib::paramss;
  // std::cout << UsrLib::paramss << endl;
  // load global route

  ROS_ERROR_STREAM("当前车辆是：" << UsrLib::params);
  string filename = string("/home/") + string(UsrLib::params["type"]) +string("/")+ string(UsrLib::params["name"]) + string("/RXY_ws/src/route/") + string(UsrLib::params["reference"]) + string(".txt");
  ROS_ERROR_STREAM(filename);
  if (this->SetGobalRoute(filename)) {
    ROS_INFO("Success to Load Global Route !!!!!");
  } else {
    ROS_ERROR("Fail to Load Global Route !!!!!");
    return;
  }

  // main loop
  while (ros::ok) {
    ros::spinOnce();
    loop.sleep();

    if (!this->isRun) {
      ROS_INFO("No State Info ......");
      continue;
    }

    if (this->PlanLocalTrajectory()) {
      //不用
      PlanBehavior();
      this->pubBehDec.publish(msgBeh);

      //发布参考
      this->pubLocTrj.publish(msgLocTrj);
      ROS_INFO("Plan Local Trajectory --------");
    }
  }
}

//订阅仿真位置
void Planner::Callback_UpdateSelfPose(const cta_msgs_perception::SelfPose::ConstPtr &msg) {
  nowPos.x = msg->state.pos.x;
  nowPos.y = msg->state.pos.y;
  nowPos.h = msg->state.rot.z;
  nowPos.v = msg->state.vel.x;

  this->isRun = true;
}

//订阅真实位姿
void Planner::yhs_Callback_UpdateTheta(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  nowPos.h = msg->data[2]/180*3.14;
  this->isRun = true;
  ROS_ERROR_STREAM("nowPos.h：" << msg->data[2]/180*3.14;);
}

//订阅真实位置
void Planner::Callback_UpdatePoints(const nlink_parser::LinktrackNodeframe2::ConstPtr &msg) {
  nowPos.x = msg->pos_3d[0];
  nowPos.y = msg->pos_3d[1];
  //    ROS_INFO("************************x:%.6f,
  //    *********************************8y:%.2f\n",  nowPos.x, nowPos.y );
  ROS_ERROR_STREAM("当前位置：（" << msg->pos_3d[0] << "," << msg->pos_3d[1] << ")");
}

bool Planner::PlanBehavior() {
  this->msgBeh.header.frame_id = "planbehavior";
  this->msgBeh.header.stamp = ros::Time::now();

  this->msgBeh.scenario = UsrLib::SCENARIO_FREE;  // free scenario
  this->msgBeh.behavior = UsrLib::BEHAVIOR_FOWARDRUN;

  this->msgBeh.speed = 3.0;

  // BEHAVIOR_RIGHTMERGE;
  // BEHAVIOR_LEFTMERGE;
  // BEHAVIOR_LEFTTURN;
  // BEHAVIOR_LEFTMERGE
  // BEHAVIOR_FOWARDRUN; //go forward

  return true;
}

bool Planner::PlanGlobalRoute() {
  // message header
  msgGlbRoute.header.frame_id = "globalroute";
  msgGlbRoute.header.stamp = ros::Time::now();

  // global route information
  msgGlbRoute.startpos.x = this->glbRoute[0].x;
  msgGlbRoute.startpos.y = this->glbRoute[0].y;
  msgGlbRoute.waypoints.clear();

  int ptcount = this->glbRoute.size();
  cta_msgs::WayPoint2d wayPt;
  for (int i = 0; i < ptcount; i++) {
    wayPt.roadid = 0;
    wayPt.laneid = 0;
    wayPt.pos.x = this->glbRoute[i].x;
    wayPt.pos.y = this->glbRoute[i].y;
    wayPt.rot = this->glbRoute[i].h;
    wayPt.vel = 5.0;
    wayPt.curv = 0.0;

    msgGlbRoute.waypoints.push_back(wayPt);
  }

  msgGlbRoute.goalpos.x = msgGlbRoute.waypoints.back().pos.x;
  msgGlbRoute.goalpos.y = msgGlbRoute.waypoints.back().pos.y;

  return true;
}
//跟据位置计算参考
bool Planner::PlanLocalTrajectory() {
  double sstep = 0.1;
  int Nx = 10;

  ROS_INFO("Search ---------");
  //计算距离当前点最近的点
  int refNum = this->GetRefernum(this->nowPos);
  if (refNum < 0) {
    ROS_INFO("No Reference Pos !!!!");
    return false;
  } else {
    Pose2D pos = this->glbRoute[refNum];
    ROS_INFO("Ref = %d, x = %.3f, y = %.3f, h = %.3f", refNum, pos.x, pos.y, pos.h);
  }

  int stNum = refNum;
  stNum = max(0, stNum);

  // local route message: message header
  msgLocTrj.header.frame_id = "localroute";
  msgLocTrj.header.stamp = ros::Time::now();

  // local route message: sample parameters
  msgLocTrj.sampletype = 1;  // distanse-equal sample
  msgLocTrj.samplestep = sstep;

  cta_msgs::WayPoint2d wp;
  msgLocTrj.waypoints.clear();
  for (int i = stNum; i < stNum + Nx; i++) {
    if (i >= this->glbRoute.size()) {
      wp.roadid = 0;
      wp.laneid = 0;
      wp.pos.x = this->glbRoute.back().x;
      wp.pos.y = this->glbRoute.back().y;
      wp.rot = this->glbRoute.back().h;
      wp.vel = 0.2;
      wp.curv = 0.0;
    } else {
      for (int j = 0; j < 5; j++) {
        wp.roadid = 0;
        wp.laneid = 0;
        wp.pos.x = glbRoute[i].x + j / 5 * (glbRoute[i + 1].x - glbRoute[i].x);
        wp.pos.y = glbRoute[i].y + j / 5 * (glbRoute[i + 1].y - glbRoute[i].y);
        // wp.rot = this->Ang(glbRoute[i], glbRoute[i + 1]);
        wp.rot = glbRoute[i].h;
        wp.vel = 0.2;
        wp.curv = 0.0;
      }
      // double dx = glbRoute[i+1].x - glbRoute[i].x;
      // double dy = glbRoute[i+1].y - glbRoute[i].y;
      // ROS_INFO("plan wp h = %.3f, x1 = %.3f, y1 = %.3f, x2 = %.3f, y2 =
      // %.3f", wp.rot, glbRoute[i].x, glbRoute[i].y, glbRoute[i+1].x,
      // glbRoute[i+1].y);
    }
    // ROS_ERROR_STREAM("msgLocTrj.waypoints：" << msgLocTrj.waypoints.size());
    msgLocTrj.waypoints.push_back(wp);
    if (msgLocTrj.waypoints.size() >= Nx) {
      break;
    }
  }

  ROS_INFO("Trj size = %d", msgLocTrj.waypoints.size());

  return true;
}

//参考文件数据导入序列
bool Planner::SetGobalRoute(string filename) {
  ifstream infile;
  infile.open(filename.c_str());
  if (!infile.is_open()) {
    ROS_ERROR("File Open Failed !!!!!!");
    return false;
  }

  double lat, lon, x, y, h, v;
  this->glbRoute.clear();
  while (!infile.eof()) {
    infile >> x >> y >> h >> v;
    Pose2D p_ = {x, y, h, 0};
    this->glbRoute.push_back(p_);
  }
  this->glbRoute.pop_back();

  infile.close();
  return true;
}

//计算距离实际位置最近的点
// ptcount=all points num

int Planner::GetRefernum(const Pose2D pos) {
  int ptcount = this->glbRoute.size();

  // get refer point on global route
  double mindis = 300;
  int refnum = -1;
  Pose2D p;
  for (int i = 0; i < ptcount; i++) {
    p = this->glbRoute[i];

    double d = Dis(p, pos);
    if (d < mindis) {
      mindis = d;
      refnum = i;
    }
  }
  return refnum;
}

cv::Mat Planner::Plot(const string win, const vector<double> x, const vector<double> y, const cv::Scalar color, int linewidth) {
  double minX = floor(*min_element(x.begin(), x.end()));
  double maxX = ceil(*max_element(x.begin(), x.end()));
  double minY = floor(*min_element(y.begin(), y.end()));
  double maxY = ceil(*max_element(y.begin(), y.end()));

  double rtX = (maxX - minX) / 300.0;
  double rtY = (maxY - minY) / 300.0;

  cv::Point pt1, pt2;
  for (int i = 0; i < x.size() - 1; i++) {
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

double Planner::Dis(const Pose2D p1, const Pose2D p2) { return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); }

double Planner::Ang(const Pose2D p1, const Pose2D p2) { return atan2(p2.y - p1.y, p2.x - p1.x); }

double Planner::Uag(const double a) {
  double u = a;
  while (u > M_PI) {
    u -= 2 * M_PI;
  }
  while (u <= -M_PI) {
    u += 2 * M_PI;
  }
  return u;
}

}  // namespace PlannerGlobalTrack
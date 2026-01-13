#include "../include/simuctrl/simuctrl_avoidance.h"


using namespace std;
//namespace SimuCtrl{

double acc;
double brake;
double delta=0;
double x;
double y;
double theta;
double v=0;

geometry_msgs::Twist current_twist_;

cta_msgs_perception::SelfPose msgLoc;
ros::Publisher pubLoc_avoidance;







# define pi 3.1415926
int main(int argc, char **argv)
{
	//初始化节点
	ros::init(argc, argv, "simuctrl_avoidance");
	ros::NodeHandle nh;
	 pubLoc_avoidance  = nh.advertise<cta_msgs_perception::SelfPose>("msg_pose_avoidance",1);//发布控制信号



	//直线障碍物
	 x =3;	 	
     y =  2.6;
	//  x =0;	 	
    //  y =  0;


	// x =4.5;	 	
	// y =  4.9;
	theta =pi/4*5;


	v = 0;

	// 订阅控制量信息
	//ros::Subscriber subctrl = nh.subscribe(UsrLib::TOPIC_CONTROL_MOTION, 0, Callback_Ctrl);
	
	// 订阅控制量信息
	//ros::Subscriber  cmd_sub_     = nh.subscribe<geometry_msgs::Twist>("cmd_vel",10,Callback_Ctrl);
	int iters = 4000;
	size_t i = 0;
	double dt = 0.1;
	double l = 0.18;

	while(ros::ok() && i < iters)
	{
		//x += dt * (v * cos(theta) - row * omega * sin(theta));
		//y += dt * (v * sin(theta) + row * omega * cos(theta));
		x += dt * v * cos(theta);
		y += dt * v * sin(theta);
		if(2/pi < theta < pi)
		theta += dt * (tan(delta) * v / l);
		if(theta >  pi + 0.005)
		{
			theta -= 2*pi;
		}
		else if(theta < -pi)
		{
			theta += 2*pi;
		}
		//  if(acc == 0)
		//  {
		// 		v -=  dt * brake;//

		//  }
		// else
		// {
		// 	v += dt * acc;
		// }
		
		//msgLoc.tssec = ros::Time::now();
		msgLoc.state.pos.x = x;
		msgLoc.state.pos.y = y;
		msgLoc.state.rot.z = theta;
		msgLoc.state.vel.x = v;
	    pubLoc_avoidance.publish(msgLoc);
		//ROS_INFO("------xxccxeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee" );
		ros::Rate loop_rate(10);

		i += 1;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

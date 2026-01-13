#include "../include/simuctrl/simuctrl.h"


using namespace std;
//namespace SimuCtrl{

double acc;
double brake;
double delta;
double x;
double y;
double theta;
double v;
geometry_msgs::Twist current_twist_;

cta_msgs_perception::SelfPose msgLoc;
ros::Publisher pubLoc;



void Callback_Ctrl(const geometry_msgs::Twist::ConstPtr& msg)
{

        
        current_twist_ = *msg.get();
		v = current_twist_.linear.x;
        delta = current_twist_.angular.z;
   // ROS_INFO("**************************************8");

}

// void Callback_Ctrl(const cta_msgs_control::MotionControl::ConstPtr& msg)
// {

        
// 	acc = msg->throttle;
// 	brake = msg -> brake;
// 	delta = msg -> steerangle;

// }


//}
# define pi 3.1415926
int main(int argc, char **argv)
{
	//初始化节点
	ros::init(argc, argv, "simuctrl");
	ros::NodeHandle nh;
	pubLoc = nh.advertise<cta_msgs_perception::SelfPose>(UsrLib::TOPIC_PERCEPTION_MOTIONINFO, 1);



	


     x =1.5;	 	
	 y = 0.5;

    //  x =6;	 	
	//  y = 0.1;
	
	theta =1.538796;


	v = 0.2;

	// 订阅控制量信息
	//ros::Subscriber subctrl = nh.subscribe(UsrLib::TOPIC_CONTROL_MOTION, 0, Callback_Ctrl);
	
	// 订阅控制量信息
	ros::Subscriber  cmd_sub_     = nh.subscribe<geometry_msgs::Twist>("cmd_vel",10,Callback_Ctrl);
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
		// 		v -=  dt * brake;

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
	    pubLoc.publish(msgLoc);
		ros::Rate loop_rate(10);

		ROS_INFO("x is : %f", msgLoc.state.pos.x);
		ROS_INFO("y is : %f", msgLoc.state.pos.y);
		ROS_INFO("theta is : %f", msgLoc.state.rot.z);
		ROS_INFO("v is : %f", msgLoc.state.vel.x);
		ROS_INFO("-----------------------");
		i += 1;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

#include<ros/ros.h>
#include<motor_system/velocity.h>
#include<can_msgs/Frame.h>
#include<sstream>
#include<string>

class MotorSystem{
	ros::NodeHandle nh;
	ros::Subscriber vel_topic;
	ros::Publisher can_bus;
public:
	MotorSystem();
	void SetVelocity(const motor_system::velocity);
};

MotorSystem::MotorSystem()
{
	vel_topic = nh.subscribe("SetVelocity",1000,&MotorSystem::SetVelocity,this);
	can_bus = nh.advertise<can_msgs::Frame>("/can0/sent",1000);
	ROS_INFO_STREAM("MotorSystem Constract");
}

void MotorSystem::SetVelocity(const motor_system::velocity vel)
{
	std::stringstream ss;
	can_msgs::Frame msg;
	msg.id=0x00;
	msg.is_rtr=false;
	msg.is_extended = false;
	msg.dlc=4;
	union{
		struct{
			float f;
			float dumy;
		};
		struct{
			unsigned char c[8];
		};
	}data;
	data.f = vel.velocity;
	msg.data[0] = data.c[0];
	msg.data[1] = data.c[1];
	msg.data[2] = data.c[2];
	msg.data[3] = data.c[3];
	ss<<"velocity "<<vel.velocity;
	can_bus.publish(msg);
	ROS_INFO_STREAM(ss.str());
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"motor_system");
	MotorSystem motor;
	ros::spin();
	return 0;
}

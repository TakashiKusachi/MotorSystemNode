#include<ros/ros.h>
#include<motor_system/velocity.h>
#include<can_msgs/Frame.h>
#include<sstream>
#include<string>

class MotorSystem{

/*	MotorSystem menver	*/
private:
	unsigned char id;
	float velocity;

	enum{
		INIT,
		BEGIN_WAIT,
		WAIT,
		RUNNING,
	}state;

/*	ROS menber		*/
private:
	ros::NodeHandle nh;
	ros::Subscriber vel_topic;
	ros::Publisher sent_can_bus;
	ros::Subscriber receive_can_bus;
	
	ros::Timer timer;
private:
	bool is_my_message(can_msgs::Frame msg){
		return (msg.id & 0x0f) == id;
	}
	//void _send_velocity(float vel);
	//void _send_begin();

public:
	MotorSystem(unsigned char);
	void SetVelocity(const motor_system::velocity);
	void CatchMsg(const can_msgs::Frame);

	void Process(const ros::TimerEvent&);
};

MotorSystem::MotorSystem(unsigned char _id=0)
{
	state = INIT;
	this->id = _id;
	this->velocity = 0;
	vel_topic = nh.subscribe("SetVelocity",1000,&MotorSystem::SetVelocity,this);
	sent_can_bus = nh.advertise<can_msgs::Frame>("/can0/sent",1000);
	receive_can_bus = nh.subscribe("/can0/receive",1000,&MotorSystem::CatchMsg,this);
	this->timer = nh.createTimer(ros::Duration(0.01),&MotorSystem::Process,this);
	ROS_INFO_STREAM("MotorSystem Constracter");
	ROS_INFO_STREAM("Send Begin Message");
}

void MotorSystem::SetVelocity(const motor_system::velocity vel)
{
	this->velocity = vel.velocity;
}

void MotorSystem::CatchMsg(const can_msgs::Frame msg)
{
	if(msg.id & 0x0f)return ;
}

void MotorSystem::Process(const ros::TimerEvent& e)
{
	//ROS_INFO_STREAM("TIMER");
	can_msgs::Frame msg;
	msg.id = this->id;
	msg.is_rtr = false;
	msg.is_extended=false;
	msg.dlc=4;
	msg.data[0] = 0;
	msg.data[1] = 0;
	msg.data[2] = 0;
	msg.data[3] = 0;
	sent_can_bus.publish(msg);
	
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"motor_system");
	MotorSystem motor(0x08);
	ros::spin();
	return 0;
}

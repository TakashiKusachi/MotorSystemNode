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
	
	//コマンドコード　7bit
	//[6] - [5] 優先度(00b:通常、01b:予約、10b:デバッグ・調整、11b:監視
	//      [4] R/W (R = 1,W = 0)
	//[3] - [0] 識別子
	typedef enum{
		SET_VELOCITY	= 0x00,
		GET_VELOCITY	= 0x70,
		SET_TORQUE	= 0x01,
		GET_TORQUE	= 0x71,
		SET_DUTY	= 0x02,
		GET_DUTY	= 0x72,
		GET_CURRENT	= 0x73,
		
		SET_MODE	= 0x04,
		GET_MODE	= 0x14,
		GET_STATE	= 0x74,
		
		SET_VCC		= 0x45,
		SET_PPR		= 0x46,
		SET_KT		= 0x47,
		
		SET_VGAIN_K	= 0x48,
		GET_V_K		= 0x78,
		SET_VGAIN_TI	= 0x49,
		GET_V_TI	= 0x79,
		SET_VGAIN_TD	= 0x4a,
		GET_V_TD	= 0x7a,
		
		SET_CGAIN_K	= 0x4c,
		GET_C_K		= 0x7c,
		SET_CGAIN_TI	= 0x4d,
		GET_C_TI	= 0x7d,
		SET_CGAIN_TD	= 0x4e,
		GET_C_TD	= 0x7e,
		
		BEGIN		= 0x44,
	}MotorSystem_CMD;//IDの上位7bit分
	
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
	void _send_velocity(float vel);
	void _send_begin();
	void _send_msg(unsigned short,bool,unsigned char,unsigned char*);
public:
	MotorSystem(unsigned char);

	//subscriber callhandle関数
	void SetVelocity(const motor_system::velocity);
	void CatchMsg(const can_msgs::Frame);

	//メインタイマーループ
	void Process(const ros::TimerEvent&);
	void RunningProcess(void);
};

MotorSystem::MotorSystem(unsigned char _id=0)
{
	//内部変数初期化
	state = INIT;
	this->id = _id % 0x0f;
	this->velocity = 0;

	//ros変数初期化
	vel_topic = nh.subscribe("SetVelocity",1000,&MotorSystem::SetVelocity,this);
	sent_can_bus = nh.advertise<can_msgs::Frame>("/can0/sent",1000);
	receive_can_bus = nh.subscribe("/can0/receive",1000,&MotorSystem::CatchMsg,this);
	this->timer = nh.createTimer(ros::Duration(0.01),&MotorSystem::Process,this);

	//log出力
	ROS_INFO_STREAM("MotorSystem Constracter");
	ROS_INFO_STREAM("Send Begin Message");

	state = BEGIN_WAIT;
	this->_send_begin();
}

void MotorSystem::SetVelocity(const motor_system::velocity vel)
{
	this->velocity = vel.velocity;
}

void MotorSystem::CatchMsg(const can_msgs::Frame msg)
{
	if(!this->is_my_message(msg))return ;
	switch(msg.id >> 4){
	case BEGIN:
		this->state = RUNNING;
		break;
	}
}

void MotorSystem::Process(const ros::TimerEvent& e)
{
	switch(this->state){
	case BEGIN_WAIT:
		break;
	case RUNNING:
		this->RunningProcess();
		break;
	}
}

void MotorSystem::RunningProcess(void)
{
	_send_velocity(this->velocity);
}

void MotorSystem::_send_begin(void)
{
	this->_send_msg(this->id | (MotorSystem::BEGIN  << 4),true,0,NULL);
}

void MotorSystem::_send_velocity(float velocity)
{
	union{
		struct{
			float f;
			float dumy;
		};
		unsigned char c[8];
	}Convter;
	Convter.f = velocity;
	this->_send_msg(this->id | (MotorSystem::SET_VELOCITY << 4),false,4,Convter.c);
}

void MotorSystem::_send_msg(unsigned short id,bool rtr,unsigned char dlc,unsigned char *data)
{
	can_msgs::Frame msg;
	msg.id = id;
	msg.is_rtr = rtr;
	msg.is_extended = false;
	msg,dlc = dlc;
	for (int i=0;i < dlc;i++){
		msg.data[i] = data[i];
	}
	sent_can_bus.publish(msg);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"motor_system");
	ros::NodeHandle nh("~");
	int id;
	if(!nh.getParam("id",id)){
		ROS_ERROR_STREAM("ID is not setting.");
		return -1;
	}
	MotorSystem motor(id);
	ros::spin();
	return 0;
}

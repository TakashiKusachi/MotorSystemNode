#include<ros/ros.h>
#include<std_msgs/Float32.h>
#include<can_msgs/Frame.h>
#include<sstream>
#include<string>

class MotorSystem{

/*	MotorSystem menver	*/
private:
	unsigned char id;

	float velocity;
	float now_velocity;

	struct{
		bool _send_begin:1;
		bool Begined:1;
	}Flags;

	enum{
		INIT,
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

	typedef union{
		struct{
			float f;
			float dumy;
		};
		unsigned char c[8];
	}FloatConv;
	
/*	ROS menber		*/
private:
	ros::NodeHandle nh;
	ros::Subscriber vel_topic;
	ros::Publisher sent_can_bus;
	ros::Subscriber receive_can_bus;
	
private:
	bool is_my_message(can_msgs::Frame msg){
		return (msg.id & 0x0f) == id;
	}
	void _send_velocity(float vel);
	void _send_begin();
	void _send_msg(unsigned short,bool,unsigned char,unsigned char*);
public:
	MotorSystem(unsigned char);
	void begin(void);

	//subscriber callhandle関数
	void SetVelocity(const std_msgs::Float32);
	void CatchMsg(const can_msgs::Frame);

	//各状態のメイン処理
	void InitProcess(void);
	void RunningProcess(void);
	
	bool IsBegined(void);
};

MotorSystem::MotorSystem(unsigned char _id=0)
{
	//内部変数初期化
	state = INIT;
	this->id = _id % 0x0f;
	this->velocity = 0;
	Flags.Begined = false;
	Flags._send_begin = false;

	//ros変数初期化
	vel_topic = nh.subscribe("SetVelocity",1000,&MotorSystem::SetVelocity,this);
	sent_can_bus = nh.advertise<can_msgs::Frame>("/can0/sent",1000);
	receive_can_bus = nh.subscribe("/can0/receive",1000,&MotorSystem::CatchMsg,this);

}

void MotorSystem::begin(void)
{
	//ループ周期を決定
	ros::Rate loop_rate(50);

	while(ros::ok()){

		//subscribeを呼び出すのに必要な関数。決して自動でハンドラは呼ばれない。
		ros::spinOnce();

		switch(this->state){

		//初期化状態
		case INIT:
			this->InitProcess();
			if(this->IsBegined())this->state = RUNNING;
			break;

		//ランニング状態
		case RUNNING:
			this->RunningProcess();
			break;
		}
		loop_rate.sleep();
	}
}

/*
//	速度受信ハンドラ。メンバ変数に格納されます。
*/
void MotorSystem::SetVelocity(const std_msgs::Float32 vel)
{
	this->velocity = vel.data;
}

/*
// CANハンドラからの受信。
// 自分でIDをみてフィルティングしてるのが勿体ないかな？
// 各命令にしたがい処理をおこなう。
*/
void MotorSystem::CatchMsg(const can_msgs::Frame msg)
{
	FloatConv Convter;
	float vel = ((FloatConv *)&msg.data[0])->f;
	if(!this->is_my_message(msg))return ;
	switch(msg.id >> 4){
	case BEGIN:
		Flags.Begined=true;
		break;
	case GET_VELOCITY:
		this->now_velocity = vel;
	}
}

void MotorSystem::InitProcess(void)
{
	if(!Flags._send_begin){
		this->_send_begin();
		Flags._send_begin = true;
	}
	return;
}

void MotorSystem::RunningProcess(void)
{
	_send_velocity(this->velocity);
	return;
}

bool MotorSystem::IsBegined(void)
{
	return Flags.Begined;
}

void MotorSystem::_send_begin(void)
{
	this->_send_msg(this->id | (MotorSystem::BEGIN  << 4),true,0,NULL);
	return;
}

void MotorSystem::_send_velocity(float velocity)
{
	/*
	union{
		struct{
			float f;
			float dumy;
		};
		unsigned char c[8];
	}Convter;
*/
	FloatConv Convter;
	Convter.f = velocity;
	this->_send_msg(this->id | (MotorSystem::SET_VELOCITY << 4),false,4,Convter.c);
}

void MotorSystem::_send_msg(unsigned short id,bool rtr,unsigned char dlc,unsigned char *data)
{
	can_msgs::Frame msg;
	msg.id = id;
	msg.is_rtr = rtr;
	msg.is_extended = false;
	msg.dlc = dlc;
	for (int i=0;i < dlc;i++){
		msg.data[i] = data[i];
	}
	//ROS_INFO_STREAM("send_id "<<id);
	//ROS_INFO_STREAM("send_dlc" <<dlc);
	sent_can_bus.publish(msg);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"motor_system");
	ros::NodeHandle nh("~");
	
	int id;
	
	//設定パラメータを取得と判定
	if(!nh.getParam("id",id)){
		ROS_ERROR_STREAM("ID is not setting.");
		return -1;
	}
	
	MotorSystem motor(id);
	
	//socketcan_bridgeが起動するのがすこし遅いので待ち。（これ絶対いらないどうにかして）
	ros::Duration(1).sleep();
	
	motor.begin();
	return 0;
}
